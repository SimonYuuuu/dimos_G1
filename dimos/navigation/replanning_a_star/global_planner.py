# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
from threading import Event, RLock, Thread, current_thread
import time
from typing import Any

from dimos_lcm.std_msgs import Bool
import numpy as np
from reactivex import Subject
from reactivex.disposable import CompositeDisposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.global_config import GlobalConfig
from dimos.core.resource import Resource
from dimos.mapping.occupancy.path_resampling import simple_resample_path, smooth_resample_path
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.base import NavigationState
from dimos.navigation.replanning_a_star.goal_validator import find_safe_goal
from dimos.navigation.replanning_a_star.local_planner import LocalPlanner, StopMessage
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.navigation.replanning_a_star.navigation_map import NavigationMap
from dimos.navigation.replanning_a_star.path_shortcutting import shortcut_path
from dimos.navigation.replanning_a_star.position_tracker import PositionTracker
from dimos.navigation.replanning_a_star.replan_limiter import ReplanLimiter
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

logger = setup_logger()

# Matches the occupancy grid cell size, so a shortcut path carries the same
# pose density as the raw A* output the downstream smoothing was tuned for.
_COSTMAP_SPACING = 0.05


class GlobalPlanner(Resource):
    path: Subject[Path]
    goal_reached: Subject[Bool]

    _current_odom: PoseStamped | None = None
    _current_goal: PoseStamped | None = None
    _goal_reached: bool = False
    _thread: Thread | None = None

    _global_config: GlobalConfig
    _navigation_map: NavigationMap
    _navigation_map_near: NavigationMap
    _local_planner: LocalPlanner
    _position_tracker: PositionTracker
    _replan_limiter: ReplanLimiter
    _disposables: CompositeDisposable
    _stop_planner: Event
    _replan_event: Event
    _replan_reason: StopMessage | None
    _lock: RLock
    _safe_goal_clearance: float
    _last_failure_reason: str | None
    _last_event: str

    _safe_goal_tolerance: float = 4.0
    _goal_tolerance: float = 0.2
    _rotation_tolerance: float = math.radians(15)
    _replan_goal_tolerance: float = 0.5
    _stuck_time_window: float = 8.0
    _stuck_threshold: float = 0.4
    # A robot rotating in place has deliberately zero position progress.  It
    # must therefore be monitored by yaw progress, not PositionTracker.
    _rotation_stuck_time_window: float = 15.0
    _rotation_progress_threshold: float = math.radians(10)
    _max_path_deviation: float = 0.9
    _replanning_enabled: bool = True

    def __init__(self, global_config: GlobalConfig) -> None:
        self.path = Subject()
        self.goal_reached = Subject()

        self._global_config = global_config
        self._navigation_map = NavigationMap(self._global_config, "voronoi")
        self._navigation_map_near = NavigationMap(self._global_config, "gradient")
        self._local_planner = LocalPlanner(
            self._global_config, self._navigation_map, self._goal_tolerance
        )

        stuck_threshold = self._stuck_threshold
        if global_config.simulation:
            stuck_threshold = 1.0

        self._position_tracker = PositionTracker(self._stuck_time_window, stuck_threshold)
        self._replan_limiter = ReplanLimiter()
        self._disposables = CompositeDisposable()
        self._stop_planner = Event()
        self._replan_event = Event()
        self._replan_reason = None
        self._lock = RLock()
        self._last_failure_reason = None
        self._last_event = "idle"
        self._reset_safe_goal_clearance()

    def start(self) -> None:
        self._local_planner.start()
        self._disposables.add(
            self._local_planner.stopped_navigating.subscribe(self._on_stopped_navigating)
        )
        self._stop_planner.clear()
        self._thread = Thread(target=self._thread_entrypoint, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.cancel_goal()
        self._local_planner.stop()
        self._disposables.dispose()
        self._stop_planner.set()
        self._replan_event.set()

        if self._thread is not None and self._thread is not current_thread():
            self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
            if self._thread.is_alive():
                logger.error("GlobalPlanner thread did not stop in time.")
            self._thread = None

    def handle_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._current_odom = msg

        self._local_planner.handle_odom(msg)
        self._position_tracker.add_position(msg)

    def handle_global_costmap(self, msg: OccupancyGrid) -> None:
        self._navigation_map.update(msg)
        self._navigation_map_near.update(msg)

    def handle_goal_request(self, goal: PoseStamped) -> None:
        logger.info("Got new goal", goal=str(goal))
        with self._lock:
            self._current_goal = goal
            self._goal_reached = False
            self._last_failure_reason = None
            self._last_event = "goal_received"
        self._replan_limiter.reset()
        self._plan_path()

    def set_safe_goal_clearance(self, clearance: float) -> None:
        with self._lock:
            self._safe_goal_clearance = clearance

    def reset_safe_goal_clearance(self) -> None:
        self._reset_safe_goal_clearance()

    def cancel_goal(self, *, but_will_try_again: bool = False, arrived: bool = False) -> None:
        # return silently so we don't flood the logs.
        with self._lock:
            no_goal = self._current_goal is None
        if no_goal and self._local_planner.get_state() == NavigationState.IDLE:
            return

        logger.info("Cancelling goal.", but_will_try_again=but_will_try_again, arrived=arrived)

        with self._lock:
            self._position_tracker.reset_data()

            if not but_will_try_again:
                self._current_goal = None
                self._goal_reached = arrived
                self._replan_limiter.reset()
                if arrived:
                    self._last_failure_reason = None
                    self._last_event = "arrived"
                elif self._last_failure_reason is None:
                    self._last_event = "cancelled"

        self.path.on_next(Path())
        self._local_planner.stop_planning()

        if not but_will_try_again:
            self.goal_reached.on_next(Bool(arrived))

    def set_replanning_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._replanning_enabled = enabled

    def get_state(self) -> NavigationState:
        return self._local_planner.get_state()

    def is_goal_reached(self) -> bool:
        with self._lock:
            return self._goal_reached

    def diagnostics(self) -> dict[str, Any]:
        """Return persistent facts after a path has been cleared."""
        with self._lock:
            goal = self._current_goal
            return {
                "state": self._local_planner.get_state().value,
                "goal_reached": self._goal_reached,
                "goal": None
                if goal is None
                else {"x": round(goal.position.x, 2), "y": round(goal.position.y, 2)},
                "replan_attempt": self._replan_limiter.get_attempt(),
                "last_event": self._last_event,
                "last_failure_reason": self._last_failure_reason,
            }

    @property
    def cmd_vel(self) -> Subject[Twist]:
        return self._local_planner.cmd_vel

    @property
    def navigation_costmap(self) -> Subject[OccupancyGrid]:
        return self._local_planner.navigation_costmap

    def _thread_entrypoint(self) -> None:
        """Monitor if the robot is stuck, veers off track, or stopped navigating."""

        last_id = -1
        last_stuck_check = time.perf_counter()
        rotation_reference_yaw: float | None = None
        rotation_last_progress = last_stuck_check

        while not self._stop_planner.is_set():
            # Wait for either timeout or replan signal from local planner.
            replanning_wanted = self._replan_event.wait(timeout=0.1)

            if self._stop_planner.is_set():
                break

            # Handle stop message from local planner (priority)
            if replanning_wanted:
                self._replan_event.clear()
                with self._lock:
                    reason = self._replan_reason
                    self._replan_reason = None

                if reason is not None:
                    self._handle_stop_message(reason)
                    last_stuck_check = time.perf_counter()
                    continue

            with self._lock:
                current_goal = self._current_goal
                current_odom = self._current_odom

            if not current_goal or not current_odom:
                continue

            if (
                current_goal.position.distance(current_odom.position) < self._goal_tolerance
                and abs(
                    angle_diff(current_goal.orientation.euler[2], current_odom.orientation.euler[2])
                )
                < self._rotation_tolerance
            ):
                logger.info("Close enough to goal. Accepting as arrived.")
                self.cancel_goal(arrived=True)
                continue

            # Check if robot has veered too far off the path
            deviation = self._local_planner.get_distance_to_path()
            if deviation is not None and deviation > self._max_path_deviation:
                logger.info(
                    "Robot veered off track. Replanning.",
                    deviation=round(deviation, 2),
                    threshold=self._max_path_deviation,
                )
                self._replan_path()
                last_stuck_check = time.perf_counter()
                continue

            state, new_id = self._local_planner.get_unique_state()

            if new_id != last_id:
                last_id = new_id
                last_stuck_check = time.perf_counter()
                rotation_reference_yaw = current_odom.orientation.euler[2]
                rotation_last_progress = last_stuck_check
                continue

            # ``PositionTracker`` intentionally sees no progress while the
            # local planner performs an initial/final turn.  Previously that
            # made every turn longer than eight seconds look like a blocked
            # path: the planner cancelled it, rebuilt the same route, and
            # eventually hit ``stuck_retry_limit``.  Track angular progress
            # for those states instead.
            if state in ("initial_rotation", "final_rotation"):
                current_yaw = current_odom.orientation.euler[2]
                if rotation_reference_yaw is None:
                    rotation_reference_yaw = current_yaw
                    rotation_last_progress = time.perf_counter()
                    continue
                yaw_progress = abs(angle_diff(current_yaw, rotation_reference_yaw))
                if yaw_progress >= self._rotation_progress_threshold:
                    rotation_reference_yaw = current_yaw
                    rotation_last_progress = time.perf_counter()
                    continue
                if time.perf_counter() - rotation_last_progress > self._rotation_stuck_time_window:
                    logger.warning(
                        "Robot rotation made no progress. Replanning.",
                        state=state,
                        yaw=round(current_yaw, 2),
                        progress=round(yaw_progress, 2),
                    )
                    with self._lock:
                        self._last_event = "rotation_stalled"
                    self._replan_path()
                    rotation_reference_yaw = current_yaw
                    rotation_last_progress = time.perf_counter()
                continue

            if (
                time.perf_counter() - last_stuck_check > self._stuck_time_window
                and self._position_tracker.is_stuck()
            ):
                logger.info("Robot is stuck. Replanning.")
                with self._lock:
                    self._last_event = "stuck_detected"
                self._replan_path()
                last_stuck_check = time.perf_counter()

    def _on_stopped_navigating(self, stop_message: StopMessage) -> None:
        with self._lock:
            self._replan_reason = stop_message
        # Signal the monitoring thread to do the replanning. This is so we don't have two
        # threads which could be replanning at the same time.
        self._replan_event.set()

    def _handle_stop_message(self, stop_message: StopMessage) -> None:
        # Note, this runs in the monitoring thread.

        self.path.on_next(Path())

        if stop_message == "arrived":
            logger.info("Arrived at goal.")
            self.cancel_goal(arrived=True)
        elif stop_message == "obstacle_found":
            logger.info("Replanning path due to obstacle found.")
            with self._lock:
                self._last_event = "obstacle_found_replanning"
            self._replan_path()
        elif stop_message == "error":
            logger.info("Failure in navigation.")
            with self._lock:
                self._last_event = "local_planner_error_replanning"
            self._replan_path()
        else:
            logger.error(f"No code to handle '{stop_message}'.")
            self.cancel_goal()

    def _replan_path(self) -> None:
        with self._lock:
            current_odom = self._current_odom
            current_goal = self._current_goal

        logger.info("Replanning.", attempt=self._replan_limiter.get_attempt())

        assert current_odom is not None
        assert current_goal is not None

        if current_goal.position.distance(current_odom.position) < self._replan_goal_tolerance:
            self.cancel_goal(arrived=True)
            return

        if not self._replanning_enabled:
            with self._lock:
                self._last_failure_reason = "replanning_disabled"
                self._last_event = "navigation_cancelled"
            self.cancel_goal()
            return

        if not self._replan_limiter.can_retry(current_odom.position):
            with self._lock:
                self._last_failure_reason = "stuck_retry_limit"
                self._last_event = "navigation_cancelled"
            self.cancel_goal()
            return

        self._replan_limiter.will_retry()

        self._plan_path()

    def _plan_path(self) -> None:
        self.cancel_goal(but_will_try_again=True)

        with self._lock:
            current_odom = self._current_odom
            current_goal = self._current_goal

        assert current_goal is not None

        if current_odom is None:
            logger.warning("Cannot handle goal request: missing odometry.")
            with self._lock:
                self._last_failure_reason = "missing_odometry"
                self._last_event = "planning_failed"
            return

        safe_goal = self._find_safe_goal(current_goal.position)

        if not safe_goal:
            logger.warning(
                "No safe goal found.", x=round(current_goal.x, 3), y=round(current_goal.y, 3)
            )
            with self._lock:
                self._last_failure_reason = "no_safe_goal"
                self._last_event = "planning_failed"
            self.cancel_goal()
            return

        path = self._find_wide_path(safe_goal, current_odom.position)

        if not path:
            logger.warning(
                "No path found to the goal.", x=round(safe_goal.x, 3), y=round(safe_goal.y, 3)
            )
            with self._lock:
                self._last_failure_reason = "no_path"
                self._last_event = "planning_failed"
            self.cancel_goal()
            return

        # Shortcutting leaves few, long segments, and ``smooth_resample_path``
        # smooths proportionally to pose count. Restore the grid-like density
        # it expects so the smoothing window stays a fraction of a metre
        # instead of spanning the whole path.
        dense_path = simple_resample_path(path, current_goal, _COSTMAP_SPACING)
        resampled_path = smooth_resample_path(dense_path, current_goal, 0.1)

        self.path.on_next(resampled_path)
        with self._lock:
            self._last_event = "following_path"

        self._local_planner.start_planning(resampled_path)

    def _find_wide_path(self, goal: Vector3, robot_pos: Vector3) -> Path | None:
        #        sizes_to_try: list[float] = [2.2, 1.7, 1.3, 1]
        sizes_to_try: list[float] = [1.1]

        for size in sizes_to_try:
            distance = robot_pos.distance(goal)
            navigation_map = self._navigation_map if distance > 1.5 else self._navigation_map_near
            costmap = navigation_map.make_gradient_costmap(size)
            self._clear_robot_footprint(costmap, navigation_map.binary_costmap, robot_pos)
            path = min_cost_astar(costmap, goal, robot_pos)
            if path and path.poses:
                shortcut = shortcut_path(path, costmap)
                logger.info(
                    f"Found path {size}x robot width.",
                    waypoints=len(path.poses),
                    shortcut_waypoints=len(shortcut.poses),
                )
                return shortcut

        return None

    def _clear_robot_footprint(
        self, costmap: OccupancyGrid, binary: OccupancyGrid, robot_pos: Vector3
    ) -> None:
        """Make the cells under the robot passable for planning.

        A new obstacle can be observed for the first time when the robot is
        already inside its inflation envelope (it drove there before the
        costmap caught up). Inflation then covers the robot's cell, A* has no
        passable start neighbors, and every replan fails no matter where the
        goal is. The space the robot stands on is traversable by definition,
        so drop inflated costs there to high-but-passable. Cells occupied in
        the raw binary costmap (actually observed obstacle points) stay
        blocked.
        """
        if binary.grid.shape != costmap.grid.shape or binary.origin != costmap.origin:
            # A newer map update raced in between building the two grids;
            # skip clearing rather than misalign cells.
            return

        center = costmap.world_to_grid(robot_pos)
        center_x, center_y = int(center.x), int(center.y)
        cells = int(self._global_config.robot_rotation_diameter / 2 / costmap.resolution) + 1

        height, width = costmap.grid.shape
        y0, y1 = max(0, center_y - cells), min(height, center_y + cells + 1)
        x0, x1 = max(0, center_x - cells), min(width, center_x + cells + 1)
        if y0 >= y1 or x0 >= x1:
            return

        region = costmap.grid[y0:y1, x0:x1]
        binary_region = binary.grid[y0:y1, x0:x1]
        rows, columns = np.ogrid[y0:y1, x0:x1]
        disc = (rows - center_y) ** 2 + (columns - center_x) ** 2 <= cells**2
        clearable = disc & (region >= CostValues.OCCUPIED) & (binary_region < CostValues.OCCUPIED)
        region[clearable] = CostValues.OCCUPIED - 1

    def _find_safe_goal(self, goal: Vector3) -> Vector3 | None:
        costmap = self._navigation_map.binary_costmap

        if costmap.cell_value(goal) == CostValues.UNKNOWN:
            return goal

        safe_goal = find_safe_goal(
            costmap,
            goal,
            algorithm="bfs_contiguous",
            cost_threshold=CostValues.OCCUPIED,
            min_clearance=self._safe_goal_clearance,
            max_search_distance=self._safe_goal_tolerance,
        )

        if safe_goal is None:
            logger.warning("No safe goal found near requested target.")
            return None

        goals_distance = safe_goal.distance(goal)
        if goals_distance > 0.2:
            logger.warning(f"Travelling to goal {goals_distance}m away from requested goal.")

        logger.info("Found safe goal.", x=round(safe_goal.x, 2), y=round(safe_goal.y, 2))

        return safe_goal

    def _reset_safe_goal_clearance(self) -> None:
        with self._lock:
            self._safe_goal_clearance = self._global_config.robot_rotation_diameter / 2
