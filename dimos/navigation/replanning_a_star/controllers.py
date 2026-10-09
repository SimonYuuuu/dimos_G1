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
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.trigonometry import angle_diff


class Controller(Protocol):
    def advance(self, lookahead_point: NDArray[np.float64], current_odom: PoseStamped) -> Twist: ...

    def translate(
        self, lookahead_point: NDArray[np.float64], current_odom: PoseStamped
    ) -> Twist: ...

    def rotate(self, yaw_error: float) -> Twist: ...

    def arc_rotate(self, yaw_error: float) -> Twist: ...

    def reset_errors(self) -> None: ...

    def reset_yaw_error(self, value: float) -> None: ...


class PController:
    _global_config: GlobalConfig
    _speed: float
    _control_frequency: float
    _arc_reverse: bool

    _min_linear_velocity: float = 0.2
    _min_angular_velocity: float = 0.2
    _k_angular: float = 0.5
    _max_angular_accel: float = 2.0
    _rotation_threshold: float = 90 * (math.pi / 180)
    # Below this heading error no yaw correction is issued at all. Without it
    # the minimum-velocity floors here and in the G1 SDK turned every
    # sub-degree error into a full 0.4 rad/s turn, so the robot wove left and
    # right along an otherwise straight path instead of tracking it.
    _yaw_deadband: float = 0.12
    # Some physical bases (including the current tethered G1 setup) accept a
    # zero-linear-velocity turn command but do not reliably take a turning
    # step.  This must be at least the verified physical walking minimum;
    # smaller commands (0.08 m/s) were accepted by the SDK but did not move
    # the robot into path-following.
    _arc_alignment_speed: float = 0.2
    # Separate thresholds for entering and leaving reverse creep: a single
    # threshold at 90 degrees made the robot alternate between stepping
    # forward and backward while the error hovered around it.
    _arc_reverse_threshold: float = 110 * (math.pi / 180)
    _arc_forward_threshold: float = 70 * (math.pi / 180)
    # Short translation-only paths used to send only 0.25 m/s forward. The
    # SDK accepted that command, but the physical G1 sometimes never entered
    # its walking gait. Keep lateral motion conservative while giving forward
    # and backward translation a modest, independently tuned start speed.
    _translation_forward_speed: float = 0.35
    # The G1 walks sideways more slowly and less stably than forwards.
    _lateral_speed: float = 0.25

    def __init__(self, global_config: GlobalConfig, speed: float, control_frequency: float) -> None:
        self._global_config = global_config
        self._speed = speed
        self._control_frequency = control_frequency
        self._arc_reverse = False

    def advance(self, lookahead_point: NDArray[np.float64], current_odom: PoseStamped) -> Twist:
        current_pos = np.array([current_odom.position.x, current_odom.position.y])
        direction = lookahead_point - current_pos
        distance = np.linalg.norm(direction)

        if distance < 1e-6:
            # Robot is coincidentally at the lookahead point; skip this cycle.
            return Twist()

        robot_yaw = current_odom.orientation.euler[2]
        desired_yaw = np.arctan2(direction[1], direction[0])
        yaw_error = angle_diff(desired_yaw, robot_yaw)

        # A large heading error used to command a pure in-place turn, which
        # this base does not reliably perform, so path following stalled.
        if abs(yaw_error) > self._rotation_threshold:
            return self.arc_rotate(yaw_error)

        angular_velocity = self._compute_angular_velocity(yaw_error)

        # When aligned, drive forward with proportional angular correction
        linear_velocity = self._speed * (1.0 - abs(yaw_error) / self._rotation_threshold)
        linear_velocity = self._apply_min_velocity(linear_velocity, self._min_linear_velocity)

        return Twist(
            linear=Vector3(linear_velocity, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )

    def translate(self, lookahead_point: NDArray[np.float64], current_odom: PoseStamped) -> Twist:
        """Drive straight at the target without changing heading.

        Turning to face a target costs over a metre of travel on this base
        (0.36 m turn radius at the creep speed), which is more than a short
        repositioning move is long. Walking there sideways or backwards keeps
        the robot on the planned path instead.
        """

        current_pos = np.array([current_odom.position.x, current_odom.position.y])
        direction = lookahead_point - current_pos
        distance = float(np.linalg.norm(direction))

        if distance < 1e-6:
            return Twist()

        robot_yaw = current_odom.orientation.euler[2]
        cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
        forward = (direction[0] * cos_yaw + direction[1] * sin_yaw) / distance
        lateral = (-direction[0] * sin_yaw + direction[1] * cos_yaw) / distance

        return Twist(
            linear=Vector3(
                forward * self._translation_forward_speed,
                lateral * self._lateral_speed,
                0.0,
            ),
            angular=Vector3(0.0, 0.0, 0.0),
        )

    def rotate(self, yaw_error: float) -> Twist:
        angular_velocity = self._compute_angular_velocity(yaw_error)
        return self._angular_twist(angular_velocity)

    def arc_rotate(self, yaw_error: float) -> Twist:
        """Turn while creeping along the planned direction.

        A path behind the robot is approached in reverse rather than asking
        the base to perform an unreliable in-place 180 degree turn.  This
        preserves the path direction: forward for a point in the front
        hemisphere and reverse for one in the rear hemisphere.
        """
        angular_velocity = self._compute_angular_velocity(yaw_error)

        magnitude = abs(yaw_error)
        if self._arc_reverse:
            if magnitude < self._arc_forward_threshold:
                self._arc_reverse = False
        elif magnitude > self._arc_reverse_threshold:
            self._arc_reverse = True

        linear_velocity = (
            -self._arc_alignment_speed if self._arc_reverse else self._arc_alignment_speed
        )
        return Twist(
            linear=Vector3(linear_velocity, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )

    def _compute_angular_velocity(self, yaw_error: float) -> float:
        if abs(yaw_error) < self._yaw_deadband:
            return 0.0
        angular_velocity = self._k_angular * yaw_error
        angular_velocity = np.clip(angular_velocity, -self._speed, self._speed)
        angular_velocity = self._apply_min_velocity(angular_velocity, self._min_angular_velocity)
        return float(angular_velocity)

    def reset_errors(self) -> None:
        self._arc_reverse = False

    def reset_yaw_error(self, value: float) -> None:
        pass

    def _apply_min_velocity(self, velocity: float, min_velocity: float) -> float:
        """Apply minimum velocity threshold, preserving sign. Returns 0 if velocity is 0."""
        if velocity == 0.0:
            return 0.0
        if abs(velocity) < min_velocity:
            return min_velocity if velocity > 0 else -min_velocity
        return velocity

    def _angular_twist(self, angular_velocity: float) -> Twist:
        # In simulation, we need stroger values
        if self._global_config.simulation and abs(angular_velocity) < 0.8:
            angular_velocity = 0.8 * np.sign(angular_velocity)

        return Twist(
            linear=Vector3(0.0, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )


class PdController(PController):
    _k_derivative: float = 0.15

    _prev_yaw_error: float
    _prev_angular_velocity: float

    def __init__(self, global_config: GlobalConfig, speed: float, control_frequency: float) -> None:
        super().__init__(global_config, speed, control_frequency)

        self._prev_yaw_error = 0.0
        self._prev_angular_velocity = 0.0

    def reset_errors(self) -> None:
        super().reset_errors()
        self._prev_yaw_error = 0.0
        self._prev_angular_velocity = 0.0

    def reset_yaw_error(self, value: float) -> None:
        self._prev_yaw_error = value

    def _compute_angular_velocity(self, yaw_error: float) -> float:
        if abs(yaw_error) < self._yaw_deadband:
            self._prev_yaw_error = yaw_error
            self._prev_angular_velocity = 0.0
            return 0.0

        dt = 1.0 / self._control_frequency

        # PD control: proportional + derivative damping
        yaw_error_derivative = (yaw_error - self._prev_yaw_error) / dt
        angular_velocity = self._k_angular * yaw_error - self._k_derivative * yaw_error_derivative

        # Rate limiting: limit angular acceleration to prevent jerky corrections
        max_delta = self._max_angular_accel * dt
        angular_velocity = np.clip(
            angular_velocity,
            self._prev_angular_velocity - max_delta,
            self._prev_angular_velocity + max_delta,
        )

        angular_velocity = np.clip(angular_velocity, -self._speed, self._speed)
        angular_velocity = self._apply_min_velocity(angular_velocity, self._min_angular_velocity)

        self._prev_yaw_error = yaw_error
        self._prev_angular_velocity = angular_velocity

        return float(angular_velocity)
