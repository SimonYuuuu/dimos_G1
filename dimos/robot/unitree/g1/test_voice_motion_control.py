# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

from collections import deque
from threading import RLock
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.robot.unitree.g1.voice_motion_control import G1VoiceMotionControl


def test_parse_chinese_relative_motion_with_arabic_number() -> None:
    request = G1VoiceMotionControl._parse_relative_motion("请向前3米", "zh")

    assert request is not None
    assert request.direction == "forward"
    assert request.distance_m == 3.0


def test_parse_chinese_relative_motion_with_chinese_decimal() -> None:
    request = G1VoiceMotionControl._parse_relative_motion("向左移动一点五米", "zh")

    assert request is not None
    assert request.direction == "left"
    assert request.distance_m == 1.5


def test_parse_english_relative_motion() -> None:
    request = G1VoiceMotionControl._parse_relative_motion("backward two meters", "en")

    assert request is not None
    assert request.direction == "backward"
    assert request.distance_m == 2.0


def test_parse_english_motion_when_asr_joins_direction_and_number() -> None:
    request = G1VoiceMotionControl._parse_relative_motion("Go forward2 meters", "en")

    assert request is not None
    assert request.direction == "forward"
    assert request.distance_m == 2.0


def test_stop_and_continue_grammar_is_explicit() -> None:
    assert G1VoiceMotionControl._is_stop("停")
    assert G1VoiceMotionControl._is_stop("小诺请停一下")
    assert G1VoiceMotionControl._is_stop("Stop now")
    assert G1VoiceMotionControl._is_stop("Nova stop")
    assert G1VoiceMotionControl._is_continue("继续")
    assert G1VoiceMotionControl._is_continue("resume")
    assert not G1VoiceMotionControl._is_stop("停车场在哪里")
    assert G1VoiceMotionControl._is_rejection("不执行")


def test_gesture_grammar_is_allowlisted_and_honours_negation() -> None:
    assert G1VoiceMotionControl._parse_gesture("请和我握手") == "Handshake"
    assert G1VoiceMotionControl._parse_gesture("please wave") == "FaceWave"
    assert G1VoiceMotionControl._parse_gesture("请鼓掌") == "Clap"
    assert G1VoiceMotionControl._parse_gesture("举右手") == "RightHandUp"
    assert G1VoiceMotionControl._parse_gesture("右手放在胸前") == "RightHandOnHeart"
    assert G1VoiceMotionControl._parse_gesture("请飞吻") == "RightKiss"
    assert G1VoiceMotionControl._parse_gesture("不要握手") is None


def test_unsafe_voice_gestures_are_explicitly_blocked() -> None:
    assert G1VoiceMotionControl._blocked_gesture("请比心") == "heart"
    assert G1VoiceMotionControl._blocked_gesture("make a heart") == "heart"
    assert G1VoiceMotionControl._blocked_gesture("请举起双手") == "two_arm_overhead"
    assert G1VoiceMotionControl._parse_gesture("请比心") is None


def test_chinese_distance_format_survives_strict_tts_filter() -> None:
    assert G1VoiceMotionControl._format_distance_zh(3.0) == "三"
    assert G1VoiceMotionControl._format_distance_zh(0.25) == "零点二五"


def _preview_controller() -> G1VoiceMotionControl:
    controller = object.__new__(G1VoiceMotionControl)
    controller.config = SimpleNamespace(robot_rotation_diameter_m=0.8)
    return controller


def test_preview_accepts_free_route_and_rejects_outside_goal() -> None:
    controller = _preview_controller()
    costmap = OccupancyGrid(np.zeros((100, 100), dtype=np.int8), resolution=0.1, frame_id="odom")

    path, reason = controller._preview_path(
        costmap,
        Vector3(1.0, 1.0, 0.0),
        PoseStamped(frame_id="odom", position=Vector3(3.0, 1.0, 0.0)),
    )
    outside, outside_reason = controller._preview_path(
        costmap,
        Vector3(1.0, 1.0, 0.0),
        PoseStamped(frame_id="odom", position=Vector3(30.0, 1.0, 0.0)),
    )

    assert reason == "ok"
    assert path is not None and len(path.poses) >= 2
    assert outside is None
    assert outside_reason == "outside"


def test_preview_rejects_unknown_and_occupied_goals() -> None:
    controller = _preview_controller()
    cells = np.zeros((100, 100), dtype=np.int8)
    costmap = OccupancyGrid(cells, resolution=0.1, frame_id="odom")
    costmap.grid[10, 30] = CostValues.UNKNOWN

    path, reason = controller._preview_path(
        costmap,
        Vector3(1.0, 1.0, 0.0),
        PoseStamped(frame_id="odom", position=Vector3(3.0, 1.0, 0.0)),
    )
    costmap.grid[10, 30] = CostValues.OCCUPIED
    occupied, occupied_reason = controller._preview_path(
        costmap,
        Vector3(1.0, 1.0, 0.0),
        PoseStamped(frame_id="odom", position=Vector3(3.0, 1.0, 0.0)),
    )

    assert path is None and reason == "unknown"
    assert occupied is None and occupied_reason == "collision"


def test_emergency_stop_latches_and_cancels_pending_action() -> None:
    controller = object.__new__(G1VoiceMotionControl)
    controller._lock = RLock()
    controller._paused = False
    controller._pending_goal = MagicMock()
    controller._pending_path = MagicMock()
    controller.cmd_vel = MagicMock()
    controller.arm_command = MagicMock()
    controller.proposed_path = MagicMock()
    controller.control_speech = MagicMock()

    controller._emergency_stop(language="en")

    assert controller._paused
    assert controller._pending_goal is None
    assert controller._pending_path is None
    assert controller.cmd_vel.publish.call_args.args[0].linear.x == 0.0
    assert controller.arm_command.publish.call_args.args[0].data == "CancelAction"
    controller.control_speech.publish.assert_called_once_with(
        "Stopped. Say continue to replan the previous goal."
    )


def test_mapping_frame_roll_does_not_trigger_false_fall_detection() -> None:
    controller = object.__new__(G1VoiceMotionControl)
    controller.config = SimpleNamespace(
        gesture_max_linear_speed_mps=0.03,
        gesture_max_angular_speed_rps=0.08,
    )
    controller._lock = RLock()
    controller._paused = False
    controller._odometry = None
    controller._motion_samples = deque(maxlen=120)
    odometry = Odometry(
        pose=Pose(orientation=Quaternion.from_euler(Vector3(3.08, 0.0, 0.0)))
    )

    controller._on_odometry(odometry)

    assert not controller._paused
    assert len(controller._motion_samples[0]) == 3


def test_stationary_median_gate_tolerates_one_odometry_outlier() -> None:
    controller = object.__new__(G1VoiceMotionControl)
    controller.config = SimpleNamespace(
        gesture_stationary_s=1.0,
        gesture_max_linear_speed_mps=0.08,
        gesture_max_angular_speed_rps=0.20,
        odometry_timeout_s=1.0,
    )
    controller._lock = RLock()
    controller._paused = False
    controller._odometry = Odometry()
    controller._last_motion_at = 0.0
    now = time.monotonic()
    controller._motion_samples = deque(
        [(now - 0.9 + index * 0.1, 0.2 if index == 4 else 0.01, 0.02) for index in range(10)],
        maxlen=120,
    )
    controller.arm_command = MagicMock()
    controller.control_speech = MagicMock()

    controller._execute_gesture("FaceWave", "zh")

    assert controller.arm_command.publish.call_args.args[0].data == "FaceWave"
    controller.control_speech.publish.assert_called_once_with("好的。")


def test_identity_and_location_queries_are_deterministic() -> None:
    assert G1VoiceMotionControl._is_identity_query("请介绍一下自己")
    assert G1VoiceMotionControl._is_identity_query("你的名字是什么")
    assert G1VoiceMotionControl._is_identity_query("Who are you?")
    assert G1VoiceMotionControl._is_location_query("我在哪里?")
    assert G1VoiceMotionControl._is_location_query("你在哪里?")
    assert G1VoiceMotionControl._is_location_query("Where am I?")
    assert not G1VoiceMotionControl._is_location_query("你来自哪里?")


def test_location_answer_uses_fresh_pose_map_and_navigation_state() -> None:
    controller = object.__new__(G1VoiceMotionControl)
    controller.config = SimpleNamespace(odometry_timeout_s=1.0)
    controller._lock = RLock()
    controller._odometry = Odometry(
        ts=time.time(),
        pose=Pose(
            position=Vector3(-1.41, 1.12, 0.0),
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 1.87)),
        ),
    )
    controller._costmap = OccupancyGrid(
        np.zeros((150, 140), dtype=np.int8), resolution=0.1, frame_id="odom"
    )
    controller._paused = False
    controller._pending_goal = None
    controller._active_goal = PoseStamped()
    controller._path = Path(poses=[PoseStamped(), PoseStamped()])
    controller.control_speech = MagicMock()

    controller._answer_location("zh")

    spoken = controller.control_speech.publish.call_args.args[0]
    assert "横坐标负一点四一米" in spoken
    assert "纵坐标一点一二米" in spoken
    assert "地图范围宽十四米高十五米" in spoken
    assert "当前导航状态为正在导航" in spoken


def test_identity_answer_matches_requested_lab_profile() -> None:
    controller = object.__new__(G1VoiceMotionControl)
    controller.control_speech = MagicMock()

    controller._answer_identity("zh")
    controller.control_speech.publish.assert_called_once_with(
        "我是采埃孚人工智能实验室的机器人小诺，是一台宇树机器人。"  # noqa: RUF001
    )
