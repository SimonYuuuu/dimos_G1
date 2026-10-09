#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
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

"""Passive G1 premap relocalization.

Runs Point-LIO and aligns its live accumulated point cloud to a saved
``.pc2.lcm`` premap.  It intentionally has no planner, velocity command, or
motion-control module.  A successful alignment publishes ``world -> map`` TF;
use it as the acceptance gate before adding autonomous navigation.

Example::

    dimos run unitree-g1-relocalization --map-file=g1_map
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.relocalization.module import RelocalizationModule
from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_vis import unitree_g1_vis


unitree_g1_relocalization = autoconnect(
    # A new Point-LIO session starts its local frame at the robot's current
    # position.  Calling that frame world matches RelocalizationModule's
    # published world -> map transform.
    PointLio.blueprint(frame_id="world"),
    RayTracingVoxelMap.blueprint(world_frame="world", global_emit_every=4),
    # MID-360 scans on a G1 indoor walk commonly remain below the generic
    # 50k-point Go2 gate.  ICP fitness remains the acceptance criterion; this
    # lower gate merely starts attempts once there is a useful local submap.
    RelocalizationModule.blueprint(publish_loaded_map=True, min_local_points=8_000),
    unitree_g1_vis,
).global_config(n_workers=8, robot_model="unitree_g1")
