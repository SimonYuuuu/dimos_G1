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

"""Inspire RH56DFTP dexterous hand (Modbus TCP) for the Unitree G1."""

from dimos.robot.unitree.g1.effectors.inspire_hand.modbus import (
    DEFAULT_IP,
    DEFAULT_PORT,
    InspireModbusClient,
)
from dimos.robot.unitree.g1.effectors.inspire_hand.module import (
    JOINT_NAMES,
    InspireHandModule,
    InspireHandModuleConfig,
    joint_names,
)

__all__ = [
    "DEFAULT_IP",
    "DEFAULT_PORT",
    "JOINT_NAMES",
    "InspireHandModule",
    "InspireHandModuleConfig",
    "InspireModbusClient",
    "joint_names",
]
