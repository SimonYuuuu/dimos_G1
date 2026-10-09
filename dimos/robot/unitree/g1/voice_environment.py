"""Read-only environment skills for the first G1 voice-agent milestone."""

from __future__ import annotations

import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.replanning_a_star.module_spec import ReplanningAStarPlannerSpec


class G1VoiceEnvironmentSkill(Module):
    """Expose the live nav-simple map state to an agent without motion authority."""

    odometry: In[Odometry]
    global_costmap: In[OccupancyGrid]
    path: In[Path]

    _navigation: ReplanningAStarPlannerSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._latest_odometry: Odometry | None = None
        self._latest_costmap: OccupancyGrid | None = None
        self._latest_path: Path | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.global_costmap.subscribe(self._on_costmap)))
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))

    def _on_odometry(self, odometry: Odometry) -> None:
        with self._lock:
            self._latest_odometry = odometry

    def _on_costmap(self, costmap: OccupancyGrid) -> None:
        with self._lock:
            self._latest_costmap = costmap

    def _on_path(self, path: Path) -> None:
        with self._lock:
            self._latest_path = path

    @skill
    def describe_environment(self) -> str:
        """Describe the currently observed lidar map and robot pose.

        This is read-only.  It reports only quantities available from the
        running Point-LIO/costmap pipeline; it does not claim semantic object
        understanding before a stable camera pipeline is added.
        """
        with self._lock:
            odometry = self._latest_odometry
            costmap = self._latest_costmap
            path = self._latest_path

        details: list[str] = []
        if odometry is None:
            details.append("No live odometry has been received yet.")
        else:
            age_sec = max(0.0, time.time() - odometry.ts)
            freshness = "STALE" if age_sec > 1.5 else "live"
            details.append(
                "Robot pose in "
                f"{odometry.frame_id or 'the navigation frame'}: "
                f"x={odometry.x:.2f} m, y={odometry.y:.2f} m, yaw={odometry.yaw:.2f} rad; "
                f"linear velocity=({odometry.vx:.2f}, {odometry.vy:.2f}) m/s; "
                f"odometry is {freshness} (age={age_sec:.2f} s)."
            )

        if costmap is None or costmap.total_cells == 0:
            details.append("No navigation costmap has been received yet.")
        else:
            known = costmap.free_cells + costmap.occupied_cells
            extent_x = costmap.width * costmap.resolution
            extent_y = costmap.height * costmap.resolution
            details.append(
                f"Costmap covers about {extent_x:.1f} by {extent_y:.1f} m: "
                f"{known} known cells, {costmap.free_cells} free, and "
                f"{costmap.occupied_cells} occupied."
            )

        if path is not None and path.poses:
            details.append(f"A navigation path is active with {len(path.poses)} waypoints.")
        else:
            details.append("No active navigation path is displayed.")
        return " ".join(details)

    @skill
    def navigation_status(self) -> str:
        """Report navigation state without changing the robot's movement."""
        try:
            state = self._navigation.get_state()
            reached = self._navigation.is_goal_reached()
            diagnostics = self._navigation.get_diagnostics()
        except Exception as exc:
            return f"Navigation status is unavailable: {exc}"

        with self._lock:
            path = self._latest_path
        path_description = (
            f"{len(path.poses)} waypoints are currently displayed"
            if path is not None and path.poses
            else "no path is currently displayed"
        )
        goal = diagnostics.get("goal")
        goal_description = "no retained goal"
        if isinstance(goal, dict):
            goal_description = f"retained goal at x={goal.get('x')}, y={goal.get('y')}"
        failure = diagnostics.get("last_failure_reason")
        failure_description = (
            f"Last navigation failure: {failure}." if failure else "No recorded navigation failure."
        )
        return (
            f"Navigation state: {state.value}. Goal reached: {reached}. {path_description}; "
            f"{goal_description}. Last event: {diagnostics.get('last_event')}; "
            f"replan attempts: {diagnostics.get('replan_attempt')}. {failure_description}"
        )
