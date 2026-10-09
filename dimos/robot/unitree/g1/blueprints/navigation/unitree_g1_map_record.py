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

"""Passive G1 LiDAR mapping recorder.

This blueprint deliberately has no motion-control, planning, or navigation
modules.  Move the G1 with the Unitree handheld controller while this process
records Point-LIO's LiDAR, odometry, and TF streams.  The resulting database is
the input for ``dimos map global <db> --export`` and later relocalization.

The database path is unique for every launch::

    recordings/<timestamp>/g1_map.db

Point-LIO reads the site-specific network addresses from
``DIMOS_POINTLIO_HOST_IP`` and ``DIMOS_POINTLIO_LIDAR_IP``.  Keep those values
in the launcher rather than hard-coding a particular robot network here.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.recorder import PointlioRecorder
from dimos.memory.module import default_recording_dir
from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_vis import unitree_g1_vis


_RECORDING_DB = default_recording_dir() / "g1_map.db"


unitree_g1_map_record = autoconnect(
    PointLio.blueprint().remappings(
        [
            (PointLio, "lidar", "pointlio_lidar"),
            (PointLio, "odometry", "pointlio_odometry"),
        ]
    ),
    # Keep PointlioRecorder's dedicated pose setters, but write the canonical
    # stream names expected by ``dimos map global`` into the database.
    PointlioRecorder.blueprint(
        db_path=str(_RECORDING_DB),
        stream_remapping={"pointlio_lidar": "lidar", "pointlio_odometry": "odometry"},
    ),
    unitree_g1_vis,
).global_config(n_workers=6, robot_model="unitree_g1")
