import math

import numpy as np

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.replanning_a_star.controllers import PController
from dimos.utils.transform_utils import euler_to_quaternion


def _controller() -> PController:
    return PController(GlobalConfig(), speed=0.6, control_frequency=10.0)


def _odom(x: float, y: float, yaw: float) -> PoseStamped:
    return PoseStamped(
        frame_id="map",
        position=[x, y, 0.0],
        orientation=euler_to_quaternion(Vector3(0.0, 0.0, yaw)),
    )


def test_arc_rotate_creeps_forward_for_front_side_goal() -> None:
    command = _controller().arc_rotate(math.pi / 2)

    assert command.linear.x > 0.0
    assert command.angular.z > 0.0


def test_arc_rotate_creeps_backward_for_goal_behind_robot() -> None:
    command = _controller().arc_rotate(math.pi)

    assert command.linear.x < 0.0
    assert command.angular.z > 0.0


def test_arc_rotate_preserves_turn_direction() -> None:
    command = _controller().arc_rotate(-math.pi / 2)

    assert command.linear.x > 0.0
    assert command.angular.z < 0.0
    assert np.isclose(abs(command.linear.x), 0.2)


def test_arc_rotate_does_not_flip_direction_around_ninety_degrees() -> None:
    controller = _controller()
    controller.arc_rotate(math.pi)

    assert controller.arc_rotate(math.radians(95)).linear.x < 0.0
    assert controller.arc_rotate(math.radians(60)).linear.x > 0.0


def test_small_heading_errors_command_no_turn() -> None:
    controller = _controller()

    assert controller.advance(np.array([1.0, 0.02]), _odom(0.0, 0.0, 0.0)).angular.z == 0.0
    assert controller.advance(np.array([1.0, 0.5]), _odom(0.0, 0.0, 0.0)).angular.z > 0.0


def test_advance_keeps_walking_when_the_target_is_behind() -> None:
    command = _controller().advance(np.array([-1.0, 0.0]), _odom(0.0, 0.0, 0.0))

    assert command.linear.x != 0.0


def test_translate_walks_sideways_without_turning() -> None:
    command = _controller().translate(np.array([0.0, 1.0]), _odom(0.0, 0.0, 0.0))

    assert command.angular.z == 0.0
    assert np.isclose(command.linear.x, 0.0, atol=1e-9)
    assert command.linear.y > 0.0


def test_translate_walks_backwards_for_a_target_behind() -> None:
    command = _controller().translate(np.array([-1.0, 0.0]), _odom(0.0, 0.0, 0.0))

    assert command.angular.z == 0.0
    assert command.linear.x < 0.0


def test_translate_uses_physical_gait_start_speed_for_forward_motion() -> None:
    command = _controller().translate(np.array([1.0, 0.0]), _odom(0.0, 0.0, 0.0))

    assert np.isclose(command.linear.x, 0.35)
    assert np.isclose(command.linear.y, 0.0)
