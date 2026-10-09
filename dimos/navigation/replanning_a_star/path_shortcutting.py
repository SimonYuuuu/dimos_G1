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

"""Line-of-sight shortcutting ("string pulling") for grid A* paths.

An 8-connected A* path is only optimal among grid moves: a straight run that
is not a multiple of 45 degrees is expressed as a staircase, and among the
many staircases of identical cost the tie-break is arbitrary. The result
looks like a sequence of pointless turns even in an empty room. Replacing
maximal sub-paths with straight segments that keep at least the clearance A*
already accepted removes those turns and shortens the path.
"""

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path

# Matches the A* treatment of unexplored cells: passable, but expensive.
_UNKNOWN_PENALTY = 0.8

# Cells are sampled twice per cell width so a segment cannot tunnel through a
# one-cell-thick obstacle between samples.
_SAMPLES_PER_CELL = 2.0


def _cell_costs(grid: NDArray[np.int8], cost_threshold: int) -> NDArray[np.float64]:
    costs = grid.astype(np.float64)
    np.copyto(costs, cost_threshold * _UNKNOWN_PENALTY, where=grid == CostValues.UNKNOWN)
    return costs


def _segment_max_cost(
    costs: NDArray[np.float64],
    start: NDArray[np.float64],
    end: NDArray[np.float64],
    cost_threshold: int,
) -> float | None:
    """Maximum cell cost on the straight segment, or None if it is blocked."""

    delta = end - start
    samples = int(np.ceil(float(np.max(np.abs(delta))) * _SAMPLES_PER_CELL)) + 1
    t = np.linspace(0.0, 1.0, samples)
    xs = np.rint(start[0] + t * delta[0]).astype(np.int64)
    ys = np.rint(start[1] + t * delta[1]).astype(np.int64)

    height, width = costs.shape
    if xs.min() < 0 or ys.min() < 0 or xs.max() >= width or ys.max() >= height:
        return None

    values = costs[ys, xs]
    maximum = float(values.max())
    if maximum >= cost_threshold:
        return None
    return maximum


def _segment_is_known_passable(
    grid: NDArray[np.int8],
    start: NDArray[np.float64],
    end: NDArray[np.float64],
    cost_threshold: int,
) -> bool:
    """Return whether a segment stays inside known, inflated free space."""

    delta = end - start
    samples = int(np.ceil(float(np.max(np.abs(delta))) * _SAMPLES_PER_CELL)) + 1
    t = np.linspace(0.0, 1.0, samples)
    xs = np.rint(start[0] + t * delta[0]).astype(np.int64)
    ys = np.rint(start[1] + t * delta[1]).astype(np.int64)

    height, width = grid.shape
    if xs.min() < 0 or ys.min() < 0 or xs.max() >= width or ys.max() >= height:
        return False

    values = grid[ys, xs]
    return bool(np.all(values != CostValues.UNKNOWN) and np.all(values < cost_threshold))


def shortcut_path(
    path: Path,
    costmap: OccupancyGrid,
    cost_threshold: int = int(CostValues.OCCUPIED),
) -> Path:
    """Remove grid-staircase detours from ``path``.

    A completely known, collision-free direct segment is preferred first.
    The input map has already been inflated for the robot footprint, so this
    keeps a clear forward/backward command straight without sacrificing body
    clearance. If the direct corridor is blocked or unknown, intermediate
    shortcuts retain the original clearance-preserving cost rule.
    """

    if len(path.poses) < 3:
        return path

    cells = np.array(
        [(lambda g: (g.x, g.y))(costmap.world_to_grid(pose.position)) for pose in path.poses],
        dtype=np.float64,
    )

    # A* optimizes the gradient cost as well as distance and can therefore
    # draw a wide arc through an otherwise empty corridor. For an entirely
    # known direct corridor, prefer the geometrically straight route. Never
    # do this through unknown or inflated occupied cells.
    if _segment_is_known_passable(costmap.grid, cells[0], cells[-1], cost_threshold):
        return Path(
            frame_id=path.frame_id,
            poses=[
                PoseStamped(
                    frame_id=path.frame_id,
                    position=[path.poses[i].position.x, path.poses[i].position.y, 0.0],
                    orientation=Quaternion(0, 0, 0, 1),
                )
                for i in (0, len(path.poses) - 1)
            ],
        )

    costs = _cell_costs(costmap.grid, cost_threshold)

    indices = np.rint(cells).astype(np.int64)
    height, width = costs.shape
    np.clip(indices[:, 0], 0, width - 1, out=indices[:, 0])
    np.clip(indices[:, 1], 0, height - 1, out=indices[:, 1])
    pose_costs = costs[indices[:, 1], indices[:, 0]]

    last = len(path.poses) - 1
    kept = [0]
    index = 0

    while index < last:

        def reachable(candidate: int, origin: int = index) -> bool:
            segment_cost = _segment_max_cost(costs, cells[origin], cells[candidate], cost_threshold)
            if segment_cost is None:
                return False
            return segment_cost <= float(pose_costs[origin : candidate + 1].max())

        # Grow the candidate exponentially, then binary search the boundary:
        # a linear scan is quadratic on the long paths this is most useful on.
        furthest = index + 1
        step = 2
        while index + step <= last and reachable(index + step):
            furthest = index + step
            step *= 2

        low, high = furthest, min(index + step, last)
        while low < high:
            middle = (low + high + 1) // 2
            if reachable(middle):
                low = middle
            else:
                high = middle - 1

        index = low
        kept.append(index)

    if len(kept) == len(path.poses):
        return path

    return Path(
        frame_id=path.frame_id,
        poses=[
            PoseStamped(
                frame_id=path.frame_id,
                position=[path.poses[i].position.x, path.poses[i].position.y, 0.0],
                orientation=Quaternion(0, 0, 0, 1),
            )
            for i in kept
        ],
    )
