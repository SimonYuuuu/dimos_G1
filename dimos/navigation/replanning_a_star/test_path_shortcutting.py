from itertools import pairwise

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.replanning_a_star.path_shortcutting import shortcut_path


def _costmap(grid: np.ndarray) -> OccupancyGrid:
    return OccupancyGrid(
        grid=grid.astype(np.int8),
        resolution=0.1,
        origin=Vector3(0.0, 0.0, 0.0),
        frame_id="map",
    )


def _path(costmap: OccupancyGrid, cells: list[tuple[int, int]]) -> Path:
    poses = []
    for cell in cells:
        point = costmap.grid_to_world(cell)
        poses.append(PoseStamped(frame_id="map", position=[point.x, point.y, 0.0]))
    return Path(frame_id="map", poses=poses)


def _cells(costmap: OccupancyGrid, path: Path) -> list[tuple[int, int]]:
    out = []
    for pose in path.poses:
        grid = costmap.world_to_grid(pose.position)
        out.append((round(grid.x), round(grid.y)))
    return out


def test_staircase_in_free_space_collapses_to_a_straight_line() -> None:
    costmap = _costmap(np.zeros((20, 20)))
    staircase = [(i // 2, i - i // 2) for i in range(17)]
    path = _path(costmap, staircase)

    result = shortcut_path(path, costmap)

    assert _cells(costmap, result) == [staircase[0], staircase[-1]]


def test_shortcut_does_not_cross_an_obstacle() -> None:
    grid = np.zeros((20, 20))
    grid[0:15, 10] = CostValues.OCCUPIED
    costmap = _costmap(grid)
    detour = [(0, 0), (0, 18), (19, 18), (19, 0)]
    path = _path(costmap, detour)

    result = shortcut_path(path, costmap)
    cells = _cells(costmap, result)

    assert cells[0] == detour[0]
    assert cells[-1] == detour[-1]
    for start, end in pairwise(cells):
        samples = np.linspace(0, 1, 100)
        xs = np.rint(start[0] + samples * (end[0] - start[0])).astype(int)
        ys = np.rint(start[1] + samples * (end[1] - start[1])).astype(int)
        assert grid[ys, xs].max() < CostValues.OCCUPIED


def test_known_passable_direct_route_wins_over_gradient_detour() -> None:
    grid = np.zeros((20, 20))
    grid[8:13, 2:18] = 50
    costmap = _costmap(grid)
    detour = [(1, 10), (1, 5), (18, 5), (18, 10)]

    result = shortcut_path(_path(costmap, detour), costmap)

    assert _cells(costmap, result) == [detour[0], detour[-1]]


def test_direct_route_does_not_cut_through_unknown_space() -> None:
    grid = np.zeros((20, 20))
    grid[8:13, 2:18] = CostValues.UNKNOWN
    costmap = _costmap(grid)
    detour = [(1, 10), (1, 5), (18, 5), (18, 10)]

    result = shortcut_path(_path(costmap, detour), costmap)

    assert len(result.poses) > 2


def test_short_paths_are_returned_unchanged() -> None:
    costmap = _costmap(np.zeros((20, 20)))
    path = _path(costmap, [(0, 0), (1, 1)])

    assert shortcut_path(path, costmap) is path
