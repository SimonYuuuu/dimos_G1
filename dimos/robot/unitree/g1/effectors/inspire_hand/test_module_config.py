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

"""Hardware-free tests for InspireHandModule config / helpers."""

import pytest

from dimos.robot.unitree.g1.effectors.inspire_hand.module import (
    InspireHandModuleConfig,
    joint_names,
)


def test_joint_names_side_validation() -> None:
    assert joint_names("r")[0] == "right_hand/pinky"
    assert joint_names("l")[0] == "left_hand/pinky"
    with pytest.raises(ValueError):
        joint_names("x")


def test_config_ip_from_env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("INSPIRE_HAND_IP", "10.1.2.3")
    assert InspireHandModuleConfig().ip == "10.1.2.3"
    assert InspireHandModuleConfig(ip="1.1.1.1").ip == "1.1.1.1"


def test_config_validation() -> None:
    with pytest.raises(ValueError):
        InspireHandModuleConfig(side="left")
    with pytest.raises(ValueError):
        InspireHandModuleConfig(skill_min_opening=1.5)
