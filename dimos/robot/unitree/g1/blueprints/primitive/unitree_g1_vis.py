#!/usr/bin/env python3
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

from __future__ import annotations

import math
from typing import Any

import numpy as np

from dimos.core.global_config import global_config
from dimos.msgs.nav_msgs.Path import Path
from dimos.robot.unitree.g1.g1_rerun import g1_costmap, g1_odometry_tf_override, g1_static_robot
from dimos.visualization.vis_module import vis_module

_PATH_Z_LIFT = 0.3
_PATH_COLOR_RGBA = (0, 255, 128, 255)
_PATH_RADIUS_METERS = 0.05
_PROPOSED_PATH_COLOR_RGBA = (255, 210, 0, 255)

# Small lift prevents z-fighting with the floor plane.
_VIS_LIFT = 0.3

# Per-entity render cap; keeps Rerun stable at robot publish rates.
_MAX_HZ = 30.0


def _default_rerun_blueprint() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(origin="world", name="3D"),
    )


def _g1_path_colors(path: Path) -> Any:
    # Return an explicit entity path so the generic bridge does not attach
    # this geometry to ``tf#/odom``.  The G1 nav view already renders its map
    # coordinates below ``world`` and has no standalone tf#/odom entity.
    import rerun as rr

    if not path.poses:
        return [("world/path", rr.LineStrips3D([]))]

    points = [[pose.x, pose.y, pose.z + _PATH_Z_LIFT] for pose in path.poses]
    return [
        (
            "world/path",
            rr.LineStrips3D(
                [points], colors=[_PATH_COLOR_RGBA], radii=_PATH_RADIUS_METERS
            ),
        )
    ]


def _g1_proposed_path_colors(path: Path) -> Any:
    """Render an unconfirmed voice plan separately from the active route."""
    import rerun as rr

    if not path.poses:
        return [("world/proposed_path", rr.LineStrips3D([]))]
    points = [[pose.x, pose.y, pose.z + _PATH_Z_LIFT + 0.02] for pose in path.poses]
    return [
        (
            "world/proposed_path",
            rr.LineStrips3D(
                [points],
                colors=[_PROPOSED_PATH_COLOR_RGBA],
                radii=_PATH_RADIUS_METERS,
            ),
        )
    ]


def _global_map_colors(cloud: Any) -> Any:
    import rerun as rr

    points, _ = cloud.as_numpy()
    if len(points) == 0:
        return None

    z = points[:, 2]
    z_min, z_max = z.min(), z.max()
    z_norm = (z - z_min) / (z_max - z_min + 1e-8)

    # Low z  = deep blue  (30, 80, 200)
    # High z = vivid green (60, 220, 100)
    colors = np.zeros((len(points), 3), dtype=np.uint8)
    colors[:, 0] = (30 + z_norm * 30).astype(np.uint8)
    colors[:, 1] = (80 + z_norm * 140).astype(np.uint8)
    colors[:, 2] = (200 - z_norm * 100).astype(np.uint8)

    return rr.Points3D(positions=points[:, :3], colors=colors, radii=0.03)


def _waypoint_colors(waypoint: Any) -> Any:
    import rerun as rr

    if not all(math.isfinite(value) for value in (waypoint.x, waypoint.y, waypoint.z)):
        return None

    return [
        (
            "world/way_point",
            rr.Points3D(
                positions=[[waypoint.x, waypoint.y, waypoint.z + _VIS_LIFT]],
                colors=[(255, 140, 0)],
                radii=0.22,
            ),
        )
    ]


def _goal_colors(goal: Any) -> Any:
    import rerun as rr

    if not all(math.isfinite(value) for value in (goal.x, goal.y, goal.z)):
        return None

    return [
        (
            "world/goal",
            rr.Points3D(
                positions=[[goal.x, goal.y, goal.z + _VIS_LIFT]],
                colors=[(180, 60, 220)],
                radii=0.3,
            ),
        )
    ]


def _static_floor(rerun_module: Any) -> list[Any]:
    half_size = 50.0
    z_below_ground = -0.2
    floor_color_rgba = [40, 40, 40, 120]  # dark grey, semi-transparent
    return [
        rerun_module.Mesh3D(
            vertex_positions=[
                [-half_size, -half_size, z_below_ground],
                [half_size, -half_size, z_below_ground],
                [half_size, half_size, z_below_ground],
                [-half_size, half_size, z_below_ground],
            ],
            triangle_indices=[[0, 1, 2], [0, 2, 3]],
            vertex_colors=[floor_color_rgba] * 4,
        )
    ]


_visual_override: dict[str, Any] = {
    "world/odometry": g1_odometry_tf_override,
    "world/lidar": None,
    # Show the recent odom-frame map so nearby/moving obstacles are visible
    # without attaching a raw sensor-frame scan to the robot.  Navigation
    # already consumes this stream; this override changes visualization only.
    "world/local_map": _global_map_colors,
    "world/global_costmap": g1_costmap,
    "world/global_map": _global_map_colors,
    "world/path": _g1_path_colors,
    "world/proposed_path": _g1_proposed_path_colors,
    "world/way_point": _waypoint_colors,
    "world/goal": _goal_colors,
}

unitree_g1_vis = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config={
        "blueprint": _default_rerun_blueprint,
        "visual_override": _visual_override,
        "static": {"world/tf/robot": g1_static_robot, "world/floor": _static_floor},
        "memory_limit": "1GB",
        # Keep the full point cloud/FOV, but do not resend the ever-growing
        # accumulated map at 30 Hz.  That saturates the Rerun stream and makes
        # live odometry look seconds or minutes behind the physical robot.
        # These limits affect visualization only; mapping and planning still
        # consume their original full-rate streams.
        "max_hz": {
            "world/odometry": 15.0,
            "world/tf": 15.0,
            "world/global_costmap": 1.0,
            "world/global_map": 0.2,
            "world/local_map": 2.0,
            # The planner clears the previous route and publishes the new
            # route immediately afterwards.  Rate limiting this entity can
            # keep the clear while dropping the replacement, making only the
            # first route visible in Rerun.
            "world/path": 0,
            "world/proposed_path": 0,
            "world/way_point": 10.0,
            "world/goal": 10.0,
        },
    },
)
