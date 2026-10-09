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

"""G1 high-level control via native Unitree SDK2 (DDS)."""

from enum import IntEnum
import json
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import String
from reactivex.disposable import Disposable
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (  # type: ignore[import-not-found]
    MotionSwitcherClient,
)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # type: ignore[import-not-found]
from unitree_sdk2py.g1.arm.g1_arm_action_client import (  # type: ignore[import-not-found]
    G1ArmActionClient,
)
from unitree_sdk2py.g1.loco.g1_loco_api import (  # type: ignore[import-not-found]
    ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
)
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient  # type: ignore[import-not-found]

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.effectors.high_level.commands import (
    ARM_API_ID,
    ARM_COMMANDS,
    ARM_COMMANDS_DOC,
    ARM_TOPIC,
    MODE_API_ID,
    MODE_COMMANDS,
    MODE_COMMANDS_DOC,
    MODE_TOPIC,
    execute_g1_command,
)
from dimos.robot.unitree.g1.effectors.high_level.high_level_spec import HighLevelG1Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_LOCO_API_IDS = {
    "GET_FSM_ID": ROBOT_API_ID_LOCO_GET_FSM_ID,
    "GET_FSM_MODE": ROBOT_API_ID_LOCO_GET_FSM_MODE,
    "GET_BALANCE_MODE": ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
}


class FsmState(IntEnum):
    ZERO_TORQUE = 0
    DAMP = 1
    SIT = 3
    AI_MODE = 200
    LIE_TO_STANDUP = 702
    SQUAT_STANDUP_TOGGLE = 706


class G1HighLevelDdsSdkConfig(ModuleConfig):
    ip: str | None = None
    network_interface: str = "eth0"
    connection_mode: str = "ai"
    ai_standup: bool = True
    motion_switcher_timeout: float = 5.0
    loco_client_timeout: float = 10.0
    cmd_vel_timeout: float = 0.2
    # Each command must expire by itself if the navigation process dies.  It
    # is slightly longer than the 10 Hz local-planner interval.
    cmd_vel_command_duration: float = 0.3
    # deadzone compensation
    min_effective_linear_velocity: float = 0.05  # m/s
    # G1 accepts 0.2 rad/s requests but, on this EDU unit, they do not
    # reliably overcome the standing-turn response threshold.  0.4 remains
    # below Unitree's documented 0.6 rad/s high-level maximum.
    min_effective_angular_velocity: float = 0.4  # radians/s
    # Unitree's own examples release holding arm actions after about 2 s.
    # Without this, actions such as heart can hold a high center-of-mass pose
    # until the firmware's much longer timeout and destabilize the robot.
    arm_action_hold_s: float = 2.0


def _boost_above_deadzone(value: float, min_effective_magnitude: float) -> float:
    if value == 0.0 or min_effective_magnitude <= 0.0:
        return value
    if abs(value) >= min_effective_magnitude:
        return value
    return min_effective_magnitude if value > 0 else -min_effective_magnitude


class G1HighLevelDdsSdk(Module, HighLevelG1Spec):
    cmd_vel: In[Twist]
    arm_command: In[String]
    config: G1HighLevelDdsSdkConfig

    # Primary timing knob — individual delays in methods are fractions of this.
    _standup_step_delay: float = 3.0

    def __init__(self, *args: Any, g: GlobalConfig = global_config, **kwargs: Any) -> None:
        super().__init__(*args, g=g, **kwargs)
        self._global_config = g
        self._stop_timer: threading.Timer | None = None
        self._arm_release_timer: threading.Timer | None = None
        self._arm_release_lock = threading.Lock()
        self._last_angular_command_log = 0.0
        self._last_velocity_command_log = 0.0
        self._running = False
        self._mode_selected = False
        self.motion_switcher: Any = None
        self.loco_client: Any = None
        self.arm_action_client: Any = None

    @rpc
    def start(self) -> None:
        super().start()

        network_interface = self.config.network_interface

        # Initialise DDS channel factory
        logger.info(f"Initializing DDS on interface: {network_interface}")
        ChannelFactoryInitialize(0, network_interface)

        # Motion switcher (required before LocoClient commands work)
        self.motion_switcher = MotionSwitcherClient()
        self.motion_switcher.SetTimeout(self.config.motion_switcher_timeout)
        self.motion_switcher.Init()
        logger.info("Motion switcher initialized")

        # Locomotion client
        self.loco_client = LocoClient()
        self.loco_client.SetTimeout(self.config.loco_client_timeout)
        self.loco_client.Init()

        self.loco_client._RegistApi(_LOCO_API_IDS["GET_FSM_ID"], 0)
        self.loco_client._RegistApi(_LOCO_API_IDS["GET_FSM_MODE"], 0)
        self.loco_client._RegistApi(_LOCO_API_IDS["GET_BALANCE_MODE"], 0)

        # Arm gestures are served by the separate ``arm`` RPC service.  They
        # are not LocoClient FSM commands, even though both APIs use 7xxx IDs.
        # Initialising this SDK client here also makes the operator CLI use the
        # same DDS participant/interface as the running navigation stack.
        self.arm_action_client = G1ArmActionClient()
        self.arm_action_client.SetTimeout(self.config.loco_client_timeout)
        self.arm_action_client.Init()

        self._select_motion_mode()
        self._running = True

        # Stream._transport is the only way to check if a port is wired;
        # there is no public API for this yet (see dimos/core/stream.py).
        if self.cmd_vel._transport is not None:
            self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))
        if self.arm_command._transport is not None:
            self.register_disposable(Disposable(self.arm_command.subscribe(self._on_arm_command)))
        logger.info("G1 DDS SDK connection started")

    def _on_arm_command(self, message: String) -> None:
        command_name = message.data.strip()
        if not command_name:
            return
        self._cancel_arm_release_timer()
        result = execute_g1_command(
            self.publish_request,
            ARM_COMMANDS,
            ARM_API_ID,
            ARM_TOPIC,
            command_name,
            logger=logger,
        )
        logger.info("G1 voice arm command handled", command=command_name, result=result)
        holding_actions = {
            "Handshake",
            "HighFive",
            "Hug",
            "ArmHeart",
            "RightHeart",
            "HandsUp",
            "XRay",
            "RightHandUp",
            "RightHandOnHeart",
            "Reject",
        }
        if command_name in holding_actions and "successfully" in result:
            timer = threading.Timer(self.config.arm_action_hold_s, self._release_arm_action)
            timer.daemon = True
            with self._arm_release_lock:
                self._arm_release_timer = timer
            timer.start()

    def _cancel_arm_release_timer(self) -> None:
        with self._arm_release_lock:
            timer, self._arm_release_timer = self._arm_release_timer, None
        if timer is not None:
            timer.cancel()

    def _release_arm_action(self) -> None:
        with self._arm_release_lock:
            self._arm_release_timer = None
        result = execute_g1_command(
            self.publish_request,
            ARM_COMMANDS,
            ARM_API_ID,
            ARM_TOPIC,
            "CancelAction",
            logger=logger,
        )
        logger.info("G1 arm action automatically released", result=result)

    @rpc
    def stop(self) -> None:
        self._cancel_arm_release_timer()
        if self._stop_timer:
            self._stop_timer.cancel()
            self._stop_timer = None

        if self.arm_action_client is not None:
            try:
                self.arm_action_client.ExecuteAction(ARM_COMMANDS["CancelAction"][0])
            except Exception as exc:
                logger.warning("Failed to release G1 arms during shutdown", error=str(exc))

        if self.loco_client is not None:
            try:
                self.loco_client.StopMove()
            except Exception as e:
                logger.error(f"Error stopping robot: {e}")

        self._running = False
        logger.info("G1 DDS SDK connection stopped")
        super().stop()

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        assert self.loco_client is not None
        raw_vx = twist.linear.x
        raw_vy = twist.linear.y
        raw_vyaw = twist.angular.z

        vx = _boost_above_deadzone(raw_vx, self.config.min_effective_linear_velocity)
        vy = _boost_above_deadzone(raw_vy, self.config.min_effective_linear_velocity)
        vyaw = _boost_above_deadzone(raw_vyaw, self.config.min_effective_angular_velocity)

        try:
            if vx == 0.0 and vy == 0.0 and vyaw == 0.0:
                self.loco_client.StopMove()
                return True

            # ``LocoClient.Move(..., continous_move=True)`` discards the
            # firmware result and creates a 10-day command.  That concealed
            # failed angular requests and made a dropped planner heartbeat
            # unsafe.  Use the underlying finite SetVelocity API directly so
            # every command is acknowledged and automatically expires.
            command_duration = duration if duration > 0 else self.config.cmd_vel_command_duration
            code = self.loco_client.SetVelocity(vx, vy, vyaw, command_duration)
            if code != 0:
                logger.warning(
                    "G1 SetVelocity rejected command",
                    code=code,
                    vx=round(vx, 3),
                    vy=round(vy, 3),
                    vyaw=round(vyaw, 3),
                    duration=command_duration,
                )
                return False
            # Log the complete accepted command at most once a second.  In
            # particular, this distinguishes a base that ignored a command
            # from an initial alignment that was intentionally creeping in
            # reverse or forward while turning.
            if time.monotonic() - self._last_velocity_command_log >= 1.0:
                logger.info(
                    "G1 velocity command accepted",
                    vx=round(vx, 3),
                    vy=round(vy, 3),
                    vyaw=round(vyaw, 3),
                    duration=command_duration,
                )
                self._last_velocity_command_log = time.monotonic()

            return True
        except Exception as e:
            logger.error(f"Failed to send movement command: {e}")
            return False

    @rpc
    def get_state(self) -> str:
        fsm_id = self._get_fsm_id()
        if fsm_id is None:
            return "Unknown (query failed)"
        try:
            return FsmState(fsm_id).name
        except ValueError:
            return f"UNKNOWN_{fsm_id}"

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[str, Any]:
        logger.info(f"Publishing request to topic: {topic} with data: {data}")
        assert self.loco_client is not None

        api_id = data.get("api_id")
        parameter = data.get("parameter", {})

        try:
            API_SET_FSM_ID = 7101
            API_SET_VELOCITY = 7105
            API_EXECUTE_ARM_ACTION = 7106
            if api_id == API_SET_FSM_ID:
                fsm_id = parameter.get("data", 0)
                code = self.loco_client.SetFsmId(fsm_id)
                return {"code": code}
            elif api_id == API_SET_VELOCITY:
                velocity = parameter.get("velocity", [0, 0, 0])
                dur = parameter.get("duration", 1.0)
                code = self.loco_client.SetVelocity(velocity[0], velocity[1], velocity[2], dur)
                return {"code": code}
            elif api_id == API_EXECUTE_ARM_ACTION:
                if self.arm_action_client is None:
                    raise RuntimeError("G1 arm-action client is not initialized")
                action_id = parameter.get("data")
                if not isinstance(action_id, int):
                    raise ValueError("G1 arm action requires an integer action ID")
                code = self.arm_action_client.ExecuteAction(action_id)
                return {"code": code}
            else:
                logger.warning(f"Unsupported API ID: {api_id}")
                return {"code": -1, "error": "unsupported_api"}
        except Exception as e:
            logger.error(f"publish_request failed: {e}")
            return {"code": -1, "error": str(e)}

    @rpc
    def stand_up(self) -> bool:
        assert self.loco_client is not None
        try:
            logger.info(f"Current state before stand_up: {self.get_state()}")

            if self.config.ai_standup:
                fsm_id = self._get_fsm_id()
                if fsm_id is None:
                    logger.warning(
                        "Could not read FSM ID; aborting stand_up to avoid unsafe state transition"
                    )
                    return False
                if fsm_id == FsmState.ZERO_TORQUE:
                    logger.info("Robot in zero torque, enabling damp mode...")
                    self.loco_client.SetFsmId(FsmState.DAMP)
                    time.sleep(self._standup_step_delay / 3)
                    # Default to DAMP if the re-query fails — we just commanded
                    # the transition, so DAMP is the most likely current state.
                    fsm_id = self._get_fsm_id() or FsmState.DAMP
                if fsm_id != FsmState.AI_MODE:
                    logger.info("Starting AI mode...")
                    self.loco_client.SetFsmId(FsmState.AI_MODE)
                    time.sleep(self._standup_step_delay / 2)
            else:
                logger.info("Enabling damp mode...")
                self.loco_client.SetFsmId(FsmState.DAMP)
                time.sleep(self._standup_step_delay / 3)

            logger.info("Executing Squat2StandUp...")
            self.loco_client.SetFsmId(FsmState.SQUAT_STANDUP_TOGGLE)
            time.sleep(self._standup_step_delay)
            logger.info(f"Final state: {self.get_state()}")
            return True
        except Exception as e:
            logger.error(f"Standup failed: {e}")
            return False

    @rpc
    def lie_down(self) -> bool:
        assert self.loco_client is not None
        try:
            self.loco_client.StandUp2Squat()
            time.sleep(self._standup_step_delay / 3)
            self.loco_client.Damp()
            return True
        except Exception as e:
            logger.error(f"Lie down failed: {e}")
            return False

    def disconnect(self) -> None:
        self.stop()

    @skill
    def move_velocity(
        self, x: float, y: float = 0.0, yaw: float = 0.0, duration: float = 0.0
    ) -> str:
        """Move the robot at the given velocity for ``duration`` seconds."""
        twist = Twist(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw))
        self.move(twist, duration=duration)
        return f"Started moving with velocity=({x}, {y}, {yaw}) for {duration} seconds"

    @skill
    def execute_arm_command(self, command_name: str) -> str:
        """Execute a Unitree G1 arm command."""
        return execute_g1_command(
            self.publish_request, ARM_COMMANDS, ARM_API_ID, ARM_TOPIC, command_name, logger=logger
        )

    execute_arm_command.__doc__ = f"""Execute a Unitree G1 arm command.

        Example usage:

            execute_arm_command("ArmHeart")

        Here are all the command names and what they do.

        {ARM_COMMANDS_DOC}
        """

    @skill
    def execute_mode_command(self, command_name: str) -> str:
        """Execute a Unitree G1 mode command."""
        return execute_g1_command(
            self.publish_request,
            MODE_COMMANDS,
            MODE_API_ID,
            MODE_TOPIC,
            command_name,
            logger=logger,
        )

    execute_mode_command.__doc__ = f"""Execute a Unitree G1 mode command.

        Example usage:

            execute_mode_command("RunMode")

        Here are all the command names and what they do.

        {MODE_COMMANDS_DOC}
        """

    def _select_motion_mode(self) -> None:
        if not self.motion_switcher or self._mode_selected:
            return

        try:
            code, result = self.motion_switcher.CheckMode()
            if code == 0 and result:
                current_mode = result.get("name", "none")
                logger.info(f"Current motion mode: {current_mode}")
                if current_mode and current_mode != "none":
                    logger.warning(
                        f"Robot is in '{current_mode}' mode. "
                        "If SDK commands don't work, you may need to activate "
                        "via controller: L1+A then L1+UP "
                        "(for chinese L2+B then L2+up then R2+A)"
                    )
        except Exception as e:
            logger.debug(f"Could not check current mode: {e}")

        mode = self.config.connection_mode
        logger.info(f"Selecting motion mode: {mode}")
        code, _ = self.motion_switcher.SelectMode(mode)
        if code == 0:
            logger.info(f"Motion mode '{mode}' selected successfully")
            self._mode_selected = True
            time.sleep(self._standup_step_delay / 6)
        else:
            logger.error(
                f"Failed to select mode '{mode}': code={code}\n"
                "  The robot may need to be activated via controller first:\n"
                "  1. Press L1 + A on the controller\n"
                "  2. Then press L1 + UP\n"
                "  This enables the AI Sport client required for SDK control."
            )

    def _get_fsm_id(self) -> int | None:
        try:
            code, data = self.loco_client._Call(_LOCO_API_IDS["GET_FSM_ID"], "{}")
            if code == 0 and data:
                result = json.loads(data) if isinstance(data, str) else data
                fsm_id = result.get("data") if isinstance(result, dict) else result
                logger.debug(f"Current FSM ID: {fsm_id}")
                return fsm_id
            else:
                logger.warning(f"Failed to get FSM ID: code={code}, data={data}")
                return None
        except Exception as e:
            logger.error(f"Error getting FSM ID: {e}")
            return None
