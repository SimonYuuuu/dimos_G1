# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Deterministic, confirmation-gated voice control for the physical G1."""

# ruff: noqa: RUF001

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import re
from threading import RLock
import time
from typing import Any

from dimos_lcm.std_msgs import Bool, String
import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.occupancy.path_map import make_navigation_map
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.navigation.replanning_a_star.path_shortcutting import shortcut_path
from dimos.robot.unitree.g1.config import G1
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass(frozen=True)
class RelativeMotionRequest:
    direction: str
    distance_m: float
    language: str


class G1VoiceMotionControlConfig(ModuleConfig):
    maximum_distance_m: float = Field(default=5.0, gt=0.0)
    minimum_distance_m: float = Field(default=0.25, gt=0.0)
    confirmation_timeout_s: float = Field(default=20.0, gt=0.0)
    odometry_timeout_s: float = Field(default=1.0, gt=0.0)
    robot_rotation_diameter_m: float = Field(default=0.8, gt=0.0)
    gesture_stationary_s: float = Field(default=1.0, gt=0.0)
    gesture_max_linear_speed_mps: float = Field(default=0.08, gt=0.0)
    gesture_max_angular_speed_rps: float = Field(default=0.20, gt=0.0)


class G1VoiceMotionControl(Module):
    """Gate base motion and handle a small, auditable voice-command grammar."""

    config: G1VoiceMotionControlConfig

    voice_control_input: In[str]
    unsafe_cmd_vel: In[Twist]
    odometry: In[Odometry]
    global_costmap: In[OccupancyGrid]
    goal_reached: In[Bool]
    path: In[Path]

    human_input: Out[str]
    control_speech: Out[str]
    cmd_vel: Out[Twist]
    goal_request: Out[PoseStamped]
    arm_command: Out[String]
    proposed_path: Out[Path]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._odometry: Odometry | None = None
        self._costmap: OccupancyGrid | None = None
        self._path: Path | None = None
        self._paused = False
        self._pending_goal: PoseStamped | None = None
        self._pending_path: Path | None = None
        self._pending_language = "zh"
        self._pending_created_at = 0.0
        self._active_goal: PoseStamped | None = None
        self._last_motion_at = 0.0
        self._motion_samples: deque[tuple[float, float, float]] = deque(maxlen=120)

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.voice_control_input.subscribe(self._on_voice)))
        self.register_disposable(Disposable(self.unsafe_cmd_vel.subscribe(self._on_motion)))
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.global_costmap.subscribe(self._on_costmap)))
        self.register_disposable(Disposable(self.goal_reached.subscribe(self._on_goal_reached)))
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))

    @rpc
    def stop(self) -> None:
        self._emergency_stop(speak=False)
        super().stop()

    def _on_odometry(self, message: Odometry) -> None:
        now = time.monotonic()
        linear_speed = math.hypot(message.vx, message.vy)
        angular_speed = abs(message.wz)
        with self._lock:
            self._odometry = message
            # FastLIO's odometry orientation is expressed in the mapping
            # frame.  On this G1 that frame reports roll close to pi while the
            # robot is physically upright, so it must not be used as a body
            # lean/fall signal.  Only measured motion is used here.  A future
            # fall detector must consume the Unitree low-state body IMU.
            self._motion_samples.append((now, linear_speed, angular_speed))

    def _on_costmap(self, message: OccupancyGrid) -> None:
        with self._lock:
            self._costmap = message

    def _on_goal_reached(self, message: Bool) -> None:
        if message.data:
            with self._lock:
                self._active_goal = None

    def _on_path(self, message: Path) -> None:
        with self._lock:
            self._path = message

    def _on_motion(self, message: Twist) -> None:
        moving = any(
            abs(value) > 1e-6 for value in (message.linear.x, message.linear.y, message.angular.z)
        )
        with self._lock:
            paused = self._paused
            if moving:
                self._last_motion_at = time.monotonic()
        self.cmd_vel.publish(Twist() if paused else message)

    def _on_voice(self, agent_text: str) -> None:
        language, text = self._unwrap_agent_text(agent_text)
        if not text:
            return

        if self._is_stop(text):
            self._emergency_stop(language=language)
            return
        if self._is_continue(text):
            self._resume(language)
            return
        if self._is_rejection(text) and self._pending_is_live():
            self._clear_pending()
            self._say("已取消，不会移动。" if language == "zh" else "Cancelled. I will not move.")
            return
        if self._is_confirmation(text) and self._pending_is_live():
            self._execute_pending(language)
            return

        # These high-frequency factual questions are answered from current
        # controller state instead of asking the language model to guess or
        # reuse an older conversation turn.
        if self._is_identity_query(text):
            self._answer_identity(language)
            return
        if self._is_location_query(text):
            self._answer_location(language)
            return

        blocked_gesture = self._blocked_gesture(text)
        if blocked_gesture is not None:
            if blocked_gesture == "heart":
                self._say(
                    "比心动作已因稳定性风险停用。"
                    if language == "zh"
                    else "Heart gestures are disabled because of a stability risk."
                )
            else:
                self._say(
                    "双臂高举动作暂时停用，请换一个单臂动作。"
                    if language == "zh"
                    else "Two-arm overhead gestures are disabled. Please choose a single-arm gesture."
                )
            return

        gesture = self._parse_gesture(text)
        if gesture is not None:
            self._execute_gesture(gesture, language)
            return

        request = self._parse_relative_motion(text, language)
        if request is not None:
            self._prepare_relative_motion(request)
            return

        self.human_input.publish(agent_text)

    def _emergency_stop(self, language: str = "zh", *, speak: bool = True) -> None:
        with self._lock:
            self._paused = True
            self._pending_goal = None
            self._pending_path = None
        # Publish a direct zero through the final velocity gate. Cancel any
        # upper-body action and unconfirmed plan as extra safety measures.
        self.cmd_vel.publish(Twist())
        self.arm_command.publish(String(data="CancelAction"))
        self.proposed_path.publish(Path())
        if speak:
            self._say(
                "已停止。说继续可以重新规划刚才的目标。"
                if language == "zh"
                else "Stopped. Say continue to replan the previous goal."
            )

    def _resume(self, language: str) -> None:
        with self._lock:
            if not self._paused:
                goal = None
                was_paused = False
            else:
                self._paused = False
                goal = self._active_goal
                was_paused = True
        if not was_paused:
            self._say("当前没有暂停的任务。" if language == "zh" else "There is no paused task.")
            return
        if goal is None:
            self._say("已解除停止状态。" if language == "zh" else "The stop latch is released.")
            return
        self.goal_request.publish(goal)
        self._say(
            "继续执行，正在重新规划。" if language == "zh" else "Continuing. I am replanning now."
        )

    def _prepare_relative_motion(self, request: RelativeMotionRequest) -> None:
        if (
            not self.config.minimum_distance_m
            <= request.distance_m
            <= self.config.maximum_distance_m
        ):
            self._say(
                "距离必须在"
                f"{self._format_distance_zh(self.config.minimum_distance_m)}米到"
                f"{self._format_distance_zh(self.config.maximum_distance_m)}米之间。"
                if request.language == "zh"
                else f"Distance must be between {self.config.minimum_distance_m:g} and {self.config.maximum_distance_m:g} meters."
            )
            return
        with self._lock:
            odometry, costmap = self._odometry, self._costmap
        if odometry is None or time.time() - odometry.ts > self.config.odometry_timeout_s:
            self._say(
                "定位信息不可用，无法规划。"
                if request.language == "zh"
                else "Localization is unavailable, so I cannot plan."
            )
            return
        if costmap is None:
            self._say(
                "地图尚未准备好，无法规划。"
                if request.language == "zh"
                else "The map is not ready, so I cannot plan."
            )
            return

        yaw_offsets = {
            "forward": 0.0,
            "backward": math.pi,
            "left": math.pi / 2,
            "right": -math.pi / 2,
        }
        bearing = odometry.yaw + yaw_offsets[request.direction]
        goal = PoseStamped(
            frame_id=costmap.frame_id,
            position=Vector3(
                odometry.x + request.distance_m * math.cos(bearing),
                odometry.y + request.distance_m * math.sin(bearing),
                0.0,
            ),
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, odometry.yaw)),
        )
        path, reason = self._preview_path(costmap, odometry.position, goal)
        if path is None:
            messages = {
                "outside": (
                    "目标超出已知地图边界，无法规划。",
                    "The target is outside the mapped boundary.",
                ),
                "unknown": (
                    "目标区域尚未建图，无法安全规划。",
                    "The target area is not mapped, so I cannot plan safely.",
                ),
                "collision": (
                    "目标位置或路径可能发生碰撞，无法规划。",
                    "The target or path may collide with an obstacle.",
                ),
            }
            self._say(
                messages.get(reason, messages["collision"])[0 if request.language == "zh" else 1]
            )
            return

        with self._lock:
            self._pending_goal = goal
            self._pending_path = path
            self._pending_language = request.language
            self._pending_created_at = time.monotonic()
        self.proposed_path.publish(path)
        direction_zh = {"forward": "前方", "backward": "后方", "left": "左侧", "right": "右侧"}[
            request.direction
        ]
        self._say(
            f"已规划到{direction_zh}{self._format_distance_zh(request.distance_m)}米处的安全路径。是否执行？"
            if request.language == "zh"
            else f"I found a safe path {request.distance_m:g} meters {request.direction}. Should I execute it?"
        )

    def _preview_path(
        self, costmap: OccupancyGrid, start: Vector3, goal: PoseStamped
    ) -> tuple[Path | None, str]:
        grid_goal = costmap.world_to_grid(goal.position)
        gx, gy = int(grid_goal.x), int(grid_goal.y)
        if not (0 <= gx < costmap.width and 0 <= gy < costmap.height):
            return None, "outside"
        if costmap.grid[gy, gx] == CostValues.UNKNOWN:
            return None, "unknown"

        nav_map = make_navigation_map(
            costmap,
            G1.width_clearance * 1.1,
            strategy="simple",
            gradient_strategy="gradient",
        )
        self._clear_start_footprint(nav_map, costmap, start)
        if nav_map.grid[gy, gx] >= CostValues.OCCUPIED:
            return None, "collision"
        path = min_cost_astar(nav_map, goal.position, start, use_cpp=False)
        if path is None or not path.poses:
            return None, "collision"
        return shortcut_path(path, nav_map), "ok"

    def _clear_start_footprint(
        self, nav_map: OccupancyGrid, binary: OccupancyGrid, start: Vector3
    ) -> None:
        center = nav_map.world_to_grid(start)
        cx, cy = int(center.x), int(center.y)
        cells = int(self.config.robot_rotation_diameter_m / 2 / nav_map.resolution) + 1
        height, width = nav_map.grid.shape
        y0, y1 = max(0, cy - cells), min(height, cy + cells + 1)
        x0, x1 = max(0, cx - cells), min(width, cx + cells + 1)
        if y0 >= y1 or x0 >= x1:
            return
        region = nav_map.grid[y0:y1, x0:x1]
        raw = binary.grid[y0:y1, x0:x1]
        rows, columns = np.ogrid[y0:y1, x0:x1]
        disc = (rows - cy) ** 2 + (columns - cx) ** 2 <= cells**2
        clearable = disc & (region >= CostValues.OCCUPIED) & (raw < CostValues.OCCUPIED)
        region[clearable] = CostValues.OCCUPIED - 1

    def _execute_pending(self, language: str) -> None:
        with self._lock:
            if self._paused:
                goal = None
            else:
                goal = self._pending_goal
                self._active_goal = goal
                self._pending_goal = None
                self._pending_path = None
        self.proposed_path.publish(Path())
        if goal is None:
            self._say(
                "当前处于停止状态，请先说继续。"
                if language == "zh"
                else "The stop latch is active. Say continue first."
            )
            return
        self.goal_request.publish(goal)
        self._say("确认执行，开始导航。" if language == "zh" else "Confirmed. Starting navigation.")

    def _execute_gesture(self, command: str, language: str) -> None:
        now = time.monotonic()
        with self._lock:
            odometry = self._odometry
            samples = [
                sample
                for sample in self._motion_samples
                if sample[0] >= now - self.config.gesture_stationary_s
            ]
            no_recent_motion_command = (
                now - self._last_motion_at >= self.config.gesture_stationary_s
            )
            paused = self._paused
        if odometry is None or time.time() - odometry.ts > self.config.odometry_timeout_s:
            self._say(
                "定位姿态不可用，不能安全执行动作。"
                if language == "zh"
                else "Pose data is unavailable, so I cannot perform the gesture safely."
            )
            return
        enough_history = (
            len(samples) >= 5
            and samples[-1][0] - samples[0][0] >= self.config.gesture_stationary_s * 0.6
        )
        median_linear = float(np.median([sample[1] for sample in samples])) if samples else math.inf
        median_angular = (
            float(np.median([sample[2] for sample in samples])) if samples else math.inf
        )
        stable = (
            enough_history
            and no_recent_motion_command
            and median_linear <= self.config.gesture_max_linear_speed_mps
            and median_angular <= self.config.gesture_max_angular_speed_rps
        )
        if not stable:
            logger.warning(
                "G1 gesture rejected by stability gate",
                enough_history=enough_history,
                no_recent_motion_command=no_recent_motion_command,
                samples=len(samples),
                median_linear_mps=round(median_linear, 3),
                median_angular_rps=round(median_angular, 3),
            )
            self._say(
                "机器人尚未稳定站好，请保持静止后再试。"
                if language == "zh"
                else "The robot is not standing steadily yet. Keep it still and try again."
            )
            return
        if paused:
            self._say(
                "当前处于安全停止状态，请先说继续。"
                if language == "zh"
                else "The safety stop is active. Say continue first."
            )
            return
        self.arm_command.publish(String(data=command))
        self._say("好的。" if language == "zh" else "Okay.")

    def _pending_is_live(self) -> bool:
        with self._lock:
            live = (
                self._pending_goal is not None
                and time.monotonic() - self._pending_created_at
                <= self.config.confirmation_timeout_s
            )
        if not live:
            self._clear_pending()
        return live

    def _clear_pending(self) -> None:
        with self._lock:
            self._pending_goal = None
            self._pending_path = None
        self.proposed_path.publish(Path())

    def _say(self, text: str) -> None:
        logger.info("G1 deterministic voice-control response", text=text)
        self.control_speech.publish(text)

    def _answer_identity(self, language: str) -> None:
        self._say(
            "我是采埃孚人工智能实验室的机器人小诺，是一台宇树机器人。"
            if language == "zh"
            else "I am Nova, from the ZF Artificial Intelligence Lab, a Unitree G1 robot."
        )

    def _answer_location(self, language: str) -> None:
        with self._lock:
            odometry = self._odometry
            costmap = self._costmap
            paused = self._paused
            pending = self._pending_goal is not None
            active = self._active_goal is not None
            has_path = self._path is not None and bool(self._path.poses)
        if odometry is None or time.time() - odometry.ts > self.config.odometry_timeout_s:
            self._say(
                "当前定位信息不可用。" if language == "zh" else "Localization is currently unavailable."
            )
            return

        heading_deg = math.degrees(odometry.yaw) % 360.0
        if paused:
            state_zh, state_en = "安全停止", "safety-stopped"
        elif pending:
            state_zh, state_en = "等待确认", "waiting for confirmation"
        elif has_path:
            state_zh, state_en = "正在导航", "navigating"
        elif active:
            state_zh, state_en = "等待路径规划", "waiting for a path"
        else:
            state_zh, state_en = "空闲", "idle"

        if costmap is None:
            map_zh = "地图信息尚未就绪"
            map_en = "The map is not ready"
        else:
            width = costmap.width * costmap.resolution
            height = costmap.height * costmap.resolution
            map_zh = (
                f"地图范围宽{self._format_number_zh(width)}米高"
                f"{self._format_number_zh(height)}米"
            )
            map_en = f"The map covers {width:.1f} by {height:.1f} meters"

        if language == "zh":
            self._say(
                "我当前在地图坐标横坐标"
                f"{self._format_number_zh(odometry.x)}米纵坐标"
                f"{self._format_number_zh(odometry.y)}米朝向约"
                f"{self._format_number_zh(heading_deg, decimals=0)}度"
                f"{map_zh}当前导航状态为{state_zh}。"
            )
        else:
            self._say(
                f"My map position is x {odometry.x:.2f} meters, y {odometry.y:.2f} meters, "
                f"heading {heading_deg:.0f} degrees. {map_en}; navigation is {state_en}."
            )

    @staticmethod
    def _unwrap_agent_text(agent_text: str) -> tuple[str, str]:
        chinese_prefix = "请只使用中文回答用户说"
        english_prefix = "Reply only in English. User said:"
        if agent_text.startswith(chinese_prefix):
            return "zh", agent_text[len(chinese_prefix) :].strip()
        if agent_text.startswith(english_prefix):
            return "en", agent_text[len(english_prefix) :].strip()
        return ("zh" if re.search(r"[\u4e00-\u9fff]", agent_text) else "en", agent_text.strip())

    @staticmethod
    def _is_stop(text: str) -> bool:
        normalized = G1VoiceMotionControl._normalized(text)
        return normalized in {
            "停",
            "停止",
            "停下",
            "别动",
            "紧急停止",
            "stop",
            "stopnow",
            "halt",
        } or bool(
            re.search(r"(?:马上|立即|立刻)?(?:停止|停下)", text)
            or re.fullmatch(r"停(?:一下|住)?(?:吧)?", normalized)
        )

    @staticmethod
    def _is_continue(text: str) -> bool:
        return G1VoiceMotionControl._normalized(text) in {
            "继续",
            "继续任务",
            "继续导航",
            "恢复",
            "resume",
            "continue",
            "goon",
        }

    @staticmethod
    def _is_confirmation(text: str) -> bool:
        return G1VoiceMotionControl._normalized(text) in {
            "是",
            "好的",
            "好",
            "确认",
            "执行",
            "开始",
            "可以",
            "yes",
            "confirm",
            "execute",
            "goahead",
        }

    @staticmethod
    def _is_rejection(text: str) -> bool:
        return G1VoiceMotionControl._normalized(text) in {
            "不",
            "不要",
            "取消",
            "否",
            "算了",
            "不执行",
            "别执行",
            "no",
            "cancel",
            "donot",
            "donotexecute",
        }

    @staticmethod
    def _is_identity_query(text: str) -> bool:
        normalized = G1VoiceMotionControl._normalized(text)
        return bool(
            re.search(
                r"你是谁|自我介绍|介绍一下(?:你)?自己|叫什么|名字|你是什么机器人|来自哪里",
                normalized,
            )
            or re.search(
                r"\b(?:who are you|introduce yourself|what(?:'s| is) your name|where are you from)\b",
                text.lower(),
            )
        )

    @staticmethod
    def _is_location_query(text: str) -> bool:
        normalized = G1VoiceMotionControl._normalized(text)
        return bool(
            re.search(
                r"(?:我|你|机器人)在哪|(?:我|你|机器人)在哪里|当前位置|我的位置|定位信息|地图坐标",
                normalized,
            )
            or re.search(
                r"\b(?:where (?:am i|are you)|my location|current (?:position|location)|map (?:position|coordinates))\b",
                text.lower(),
            )
        )

    @staticmethod
    def _normalized(text: str) -> str:
        normalized = re.sub(r"[\s，。！？,.!?]+", "", text).lower()
        return re.sub(r"^(?:小诺|nova)?(?:请|please)?", "", normalized)

    @staticmethod
    def _parse_gesture(text: str) -> str | None:
        lowered = text.lower()
        if re.search(r"(?:不要|别|取消|don'?t|do not|not)\s*", lowered):
            return None
        gestures = (
            (r"握手|shake\s*hands?", "Handshake"),
            (r"打招呼|挥手|wave|say\s+hello", "FaceWave"),
            (r"击掌|high\s*five", "HighFive"),
            (r"鼓掌|clap|applause", "Clap"),
            (r"拒绝手势|表示拒绝|say\s+no|reject\s+gesture", "Reject"),
            (
                r"举起?右手|(?:raise|lift|li)\s+(?:up\s+)?(?:your\s+)?right\s+hand",
                "RightHandUp",
            ),
            (r"右手放(?:在)?胸前|致意|hand\s+on\s+(?:your\s+)?heart", "RightHandOnHeart"),
            (r"飞吻|blow\s+(?:a\s+)?kiss", "RightKiss"),
        )
        return next((command for pattern, command in gestures if re.search(pattern, lowered)), None)

    @staticmethod
    def _blocked_gesture(text: str) -> str | None:
        lowered = text.lower()
        if re.search(r"(?:不要|别|取消|don'?t|do not|not)\s*", lowered):
            return None
        if re.search(r"比心|爱心|\bheart\b", lowered):
            return "heart"
        if re.search(r"摊开?双手|举起?双手|both\s+hands\s+up", lowered):
            return "two_arm_overhead"
        return None

    @classmethod
    def _parse_relative_motion(cls, text: str, language: str) -> RelativeMotionRequest | None:
        lowered = text.lower()
        if language == "zh":
            directions = (
                ("forward", r"(?:向|往)?前(?:方)?"),
                ("backward", r"(?:向|往)?后(?:方)?|后退"),
                ("left", r"(?:向|往)?左(?:侧)?"),
                ("right", r"(?:向|往)?右(?:侧)?"),
            )
            match = re.search(r"([零〇一二两三四五六七八九十百点\d.]+)\s*米", text)
            distance = cls._number(match.group(1)) if match else None
        else:
            directions = (
                ("forward", r"\b(?:forward|ahead)(?=\b|\d)"),
                ("backward", r"\b(?:backward|back)(?=\b|\d)"),
                ("left", r"\bleft(?=\b|\d)"),
                ("right", r"\bright(?=\b|\d)"),
            )
            words = {"one": 1.0, "two": 2.0, "three": 3.0, "four": 4.0, "five": 5.0}
            match = re.search(
                r"(\d+(?:\.\d+)?|one|two|three|four|five)\s*(?:m|meter|meters)\b", lowered
            )
            distance = (
                words.get(match.group(1), None)
                if match and not match.group(1)[0].isdigit()
                else (float(match.group(1)) if match else None)
            )
        direction = next(
            (name for name, pattern in directions if re.search(pattern, lowered)), None
        )
        if direction is None or distance is None:
            return None
        return RelativeMotionRequest(direction, distance, language)

    @staticmethod
    def _number(value: str) -> float | None:
        try:
            return float(value)
        except ValueError:
            pass
        digits = {
            "零": 0,
            "〇": 0,
            "一": 1,
            "二": 2,
            "两": 2,
            "三": 3,
            "四": 4,
            "五": 5,
            "六": 6,
            "七": 7,
            "八": 8,
            "九": 9,
        }
        if "点" in value:
            whole, fraction = value.split("点", 1)
            whole_value = G1VoiceMotionControl._number(whole) or 0.0
            if not fraction or any(char not in digits for char in fraction):
                return None
            return whole_value + float("0." + "".join(str(digits[char]) for char in fraction))
        if "十" in value:
            left, right = value.split("十", 1)
            return float((digits.get(left, 1) * 10) + digits.get(right, 0))
        if value and all(char in digits for char in value):
            return float("".join(str(digits[char]) for char in value))
        return None

    @staticmethod
    def _format_distance_zh(value: float) -> str:
        return G1VoiceMotionControl._format_number_zh(value)

    @staticmethod
    def _format_number_zh(value: float, *, decimals: int = 2) -> str:
        sign = "负" if value < 0 else ""
        rendered = f"{abs(value):.{decimals}f}".rstrip("0").rstrip(".")
        whole, separator, fraction = rendered.partition(".")
        number = int(whole)
        digits = "零一二三四五六七八九"
        if number < 10:
            whole_zh = digits[number]
        elif number < 100:
            tens, ones = divmod(number, 10)
            whole_zh = (digits[tens] if tens > 1 else "") + "十"
            if ones:
                whole_zh += digits[ones]
        elif number < 1000:
            hundreds, remainder = divmod(number, 100)
            whole_zh = digits[hundreds] + "百"
            if remainder:
                if remainder < 10:
                    whole_zh += "零"
                tens, ones = divmod(remainder, 10)
                if tens:
                    whole_zh += digits[tens] + "十"
                if ones:
                    whole_zh += digits[ones]
        else:
            whole_zh = whole.translate(str.maketrans("0123456789", "零一二三四五六七八九"))
        fraction_zh = fraction.translate(str.maketrans("0123456789", "零一二三四五六七八九"))
        return sign + whole_zh + ("点" + fraction_zh if separator else "")
