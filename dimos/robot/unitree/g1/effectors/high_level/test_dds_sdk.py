import threading
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from dimos_lcm.std_msgs import String
import pytest

pytest.importorskip("unitree_sdk2py")

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.effectors.high_level.dds_sdk import G1HighLevelDdsSdk


def _sdk() -> G1HighLevelDdsSdk:
    sdk = object.__new__(G1HighLevelDdsSdk)
    sdk.config = SimpleNamespace(
        min_effective_linear_velocity=0.05,
        min_effective_angular_velocity=0.4,
        cmd_vel_command_duration=0.3,
        arm_action_hold_s=2.0,
    )
    sdk.loco_client = MagicMock()
    sdk._last_angular_command_log = 0.0
    sdk._last_velocity_command_log = 0.0
    sdk._arm_release_timer = None
    sdk._arm_release_lock = threading.Lock()
    return sdk


def test_move_uses_finite_acknowledged_velocity_command() -> None:
    sdk = _sdk()
    sdk.loco_client.SetVelocity.return_value = 0

    accepted = sdk.move(Twist(angular=Vector3(z=0.4)))

    assert accepted is True
    sdk.loco_client.SetVelocity.assert_called_once_with(0.0, 0.0, 0.4, 0.3)
    sdk.loco_client.Move.assert_not_called()


def test_move_boosts_small_turn_command_above_g1_turn_deadzone() -> None:
    sdk = _sdk()
    sdk.loco_client.SetVelocity.return_value = 0

    sdk.move(Twist(angular=Vector3(z=0.2)))

    sdk.loco_client.SetVelocity.assert_called_once_with(0.0, 0.0, 0.4, 0.3)


def test_move_stops_explicitly_for_zero_twist() -> None:
    sdk = _sdk()

    accepted = sdk.move(Twist())

    assert accepted is True
    sdk.loco_client.StopMove.assert_called_once_with()
    sdk.loco_client.SetVelocity.assert_not_called()


def test_move_reports_rejected_velocity_command() -> None:
    sdk = _sdk()
    sdk.loco_client.SetVelocity.return_value = 7

    accepted = sdk.move(Twist(linear=Vector3(x=0.2)))

    assert accepted is False


def test_holding_arm_action_is_automatically_released() -> None:
    sdk = _sdk()

    with (
        patch(
            "dimos.robot.unitree.g1.effectors.high_level.dds_sdk.execute_g1_command",
            return_value="'RightHeart' command executed successfully.",
        ),
        patch("dimos.robot.unitree.g1.effectors.high_level.dds_sdk.threading.Timer") as timer,
    ):
        sdk._on_arm_command(String(data="RightHeart"))

    timer.assert_called_once_with(2.0, sdk._release_arm_action)
    timer.return_value.start.assert_called_once_with()
