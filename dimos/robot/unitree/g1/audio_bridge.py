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

"""Bridge the G1's native audio service to the read-only DimOS dialogue agent.

The G1 controller already performs microphone recognition and publishes the
recognized text on ``rt/audio_msg``.  This module forwards that text to the
same ``/human_input`` stream used by ``humancli``.  Final agent replies are
sent to the onboard TTS service, and the head light strip signals listening,
thinking, and speaking.  It deliberately defines no agent skill, so this is
not an extra physical-control authority for the language model.
"""

# ruff: noqa: RUF001

from __future__ import annotations

from collections import deque
import json
import re
import threading
import time
from typing import TYPE_CHECKING, Any

from langchain_core.messages import AIMessage
from langchain_core.messages.base import BaseMessage
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.robot.unitree.g1.wake_word import WakeMatch, WakeMatcherSettings, match_wake
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

logger = setup_logger()

_CHINESE_ONLY_NOTICE = "目前只支持中文和英文，其他语言正在开发中。"


class G1AudioBridgeConfig(ModuleConfig):
    """Configuration for the onboard G1 audio bridge."""

    # Do not call this ``network_interface``: the existing G1 high-level DDS
    # module already owns that CLI shorthand.  A duplicate makes the normal
    # ``--network-interface eth0`` launch option ambiguous.
    audio_interface: str = Field(default="eth0")
    asr_topic: str = Field(default="rt/audio_msg")
    chinese_speaker_id: int = Field(default=0, ge=0)
    english_speaker_id: int = Field(default=1, ge=0)
    # The G1 firmware exposes Chinese and English speakers.  German is routed
    # to the English voice until a dedicated local multilingual TTS is added.
    german_speaker_id: int = Field(default=1, ge=0)
    maximum_text_length: int = Field(default=400, ge=1)
    enable_microphone_stream: bool = Field(default=True)
    # In the motion-control blueprint emergency stop words must pass through
    # even while TTS is active. Other speech is still suppressed as echo.
    allow_stop_while_speaking: bool = Field(default=False)
    # The normal dialogue mode strips digits from Chinese utterances.  The
    # motion-control mode must retain numbers in commands such as ``向前3米``.
    preserve_motion_command_numbers: bool = Field(default=False)
    # Wake-word gating is enabled only in the advanced voice-control stack.
    # A short active window avoids requiring the wake word on every sentence.
    wake_word_enabled: bool = Field(default=False)
    wake_session_s: float = Field(default=15.0, gt=0.0)
    wake_retry_count: int = Field(default=3, ge=2)
    wake_retry_window_s: float = Field(default=25.0, gt=0.0)
    chinese_wake_aliases: tuple[str, ...] = (
        "小诺",
        "小糯",
        "小懦",
        "小挪",
        "晓诺",
        "晓糯",
        "小若",
        "小洛",
        "小落",
        "小罗",
        "小娜",
        "小喏",
        "晓落",
        "小末",
        "机器人",
        "唤醒模式",
        "开始对话",
    )
    english_wake_aliases: tuple[str, ...] = ("nova", "nover")
    # Wake matching (see wake_word.py).  The doubled phrase "小诺小诺" /
    # "nova nova" is the recommended wake and is accepted anywhere in a
    # sentence.  Single "小诺"/"Nova" still works at sentence start.
    wake_fuzzy_enabled: bool = Field(default=True)
    wake_fuzzy_threshold: float = Field(default=0.8, ge=0.0, le=1.0)
    wake_strong_threshold: float = Field(default=0.95, ge=0.0, le=1.0)
    wake_require_double: bool = Field(default=False)
    wake_double_anywhere: bool = Field(default=True)
    # Off by default: a single "小诺" mid-sentence is common when people talk
    # about the robot; use the doubled phrase for mid-sentence wake instead.
    wake_strong_anywhere: bool = Field(default=False)


class G1AudioBridge(Module):
    """Forward onboard ASR to the agent and speak final agent responses."""

    config: G1AudioBridgeConfig

    agent: In[BaseMessage]
    control_speech: In[str]
    human_input: Out[str]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._audio_client: AudioClient | None = None
        self._asr_subscriber: ChannelSubscriber | None = None
        self._lock = threading.Lock()
        self._speaking_until = 0.0
        self._led_timer: threading.Timer | None = None
        # Deliberately keep this in the bridge rather than asking the model to
        # infer a language afresh on every turn.  Native ASR occasionally
        # produces a short utterance in a wrong language; it must not switch a
        # conversation that the user explicitly locked to Chinese or English.
        self._conversation_language: str | None = None
        self._awake_until = 0.0
        self._short_wake_attempts: deque[float] = deque(maxlen=8)

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.agent.subscribe(self._on_agent_message)))
        if self.control_speech.transport is not None:
            self.register_disposable(Disposable(self.control_speech.subscribe(self._speak)))

        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
            from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

            if self.config.audio_interface:
                ChannelFactoryInitialize(0, self.config.audio_interface)
            else:
                ChannelFactoryInitialize(0)

            audio_client = AudioClient()
            audio_client.Init()
            if self.config.enable_microphone_stream:
                self._set_microphone_stream_mode(audio_client, 1)
            subscriber = ChannelSubscriber(self.config.asr_topic, String_)
            subscriber.Init(self._on_asr_message, 10)
        except (ImportError, OSError, RuntimeError, ValueError) as exc:
            logger.warning(
                "G1 onboard audio is unavailable; text dialogue remains available", error=str(exc)
            )
            return

        self._audio_client = audio_client
        self._asr_subscriber = subscriber
        self._set_led(0, 0, 0)
        logger.info(
            "G1 onboard audio bridge ready",
            asr_topic=self.config.asr_topic,
            microphone_streaming=self.config.enable_microphone_stream,
        )

    @rpc
    def stop(self) -> None:
        with self._lock:
            if self._led_timer is not None:
                self._led_timer.cancel()
                self._led_timer = None

        if self._asr_subscriber is not None:
            try:
                self._asr_subscriber.Close()
            except (OSError, RuntimeError) as exc:
                logger.warning("G1 audio ASR subscriber close failed", error=str(exc))
        self._asr_subscriber = None
        if self.config.enable_microphone_stream:
            self._set_microphone_stream_mode(self._audio_client, 2)
        self._set_led(0, 0, 0)
        self._audio_client = None
        super().stop()

    @staticmethod
    def _set_microphone_stream_mode(client: AudioClient | None, mode: int) -> None:
        """Enable or disable the G1 microphone multicast/ASR service.

        API 1008 is present in the G1 voice service but is not wrapped by the
        current Python SDK.  ``mode=1`` enables microphone multicast plus
        native ASR; ``mode=2`` restores the firmware default and avoids
        leaving the head cabin streaming after this module exits.
        """
        if client is None:
            return
        try:
            client._RegistApi(1008, 0)
            code, _ = client._Call(1008, json.dumps({"mode": mode}))
        except (OSError, RuntimeError, ValueError) as exc:
            logger.warning("G1 microphone stream mode change failed", mode=mode, error=str(exc))
            return
        if code != 0:
            logger.warning("G1 microphone stream service returned an error", mode=mode, code=code)
            return
        logger.info("G1 microphone stream mode changed", mode=mode)

    def _on_asr_message(self, message: Any) -> None:
        """Receive a completed native ASR utterance from ``rt/audio_msg``."""
        text = self._asr_text(str(getattr(message, "data", "")))
        if not text:
            return
        logger.info("Raw native G1 ASR transcript", text=text)
        if not any(character.isalnum() for character in text):
            logger.debug("Ignoring punctuation-only G1 ASR result", text=text)
            return
        if len(text) > self.config.maximum_text_length:
            logger.warning("Ignoring overlong G1 ASR utterance", length=len(text))
            return
        # Language switching is a local state transition, not dialogue.  It
        # must be recognized before the TTS echo gate and before enforcing the
        # current language lock, otherwise a Chinese request cannot escape an
        # English lock (and vice versa).
        requested_language = self._requested_language(text)
        with self._lock:
            if time.monotonic() < self._speaking_until and not (
                requested_language is not None
                or (self.config.allow_stop_while_speaking and self._looks_like_emergency_stop(text))
            ):
                logger.debug("Ignoring ASR while onboard TTS is speaking")
                return

        if self.config.wake_word_enabled and not self._looks_like_emergency_stop(text):
            wake_language, remainder = self._extract_wake_word(text)
            now = time.monotonic()
            if wake_language is not None:
                self._awake_until = now + self.config.wake_session_s
                # The selected name is also an immediate language switch:
                # 小诺 selects Chinese and Nova selects English.
                self._conversation_language = wake_language
                acknowledgement_language = wake_language
                logger.info(
                    "Voice wake word accepted",
                    language=acknowledgement_language,
                    remainder=remainder,
                )
                self._speak("我在，请说。" if acknowledgement_language == "zh" else "I am here.")
                if not remainder:
                    return
                text = remainder
            elif now >= self._awake_until:
                if self._record_short_wake_attempt(text, now):
                    self._awake_until = now + self.config.wake_session_s
                    if self._conversation_language is None:
                        self._conversation_language = "zh"
                    self._short_wake_attempts.clear()
                    logger.info(
                        "Voice wake fallback accepted after repeated short ASR results",
                        language=self._conversation_language,
                    )
                    self._speak(
                        "我在，请说。" if self._conversation_language == "zh" else "I am here."
                    )
                    return
                logger.info("Ignoring ASR while voice assistant is asleep", text=text)
                return
            else:
                self._awake_until = now + self.config.wake_session_s

        # Re-evaluate after removing a wake word from a combined utterance.
        requested_language = self._requested_language(text)
        detected_language = self._classify_asr_language(text)
        if requested_language is not None:
            self._conversation_language = requested_language
            logger.info("Voice conversation language switched", language=requested_language)
            self._speak("好的。" if requested_language == "zh" else "Okay.")
            return
        else:
            language = detected_language

        # Only Chinese and English are supported dialogue languages.  German
        # and French are classified separately so they cannot be mistaken for
        # English by an ASCII-only ASR transcript, but they intentionally do
        # not reach the model in this stage.
        if language not in {"zh", "en"}:
            logger.info("Ignoring unsupported ASR language", language=language, text=text)
            self._speak(_CHINESE_ONLY_NOTICE, speaker_id=self.config.chinese_speaker_id)
            return

        locked_language = self._conversation_language
        if (
            requested_language is None
            and locked_language is not None
            and language != locked_language
        ):
            logger.info(
                "Ignoring ASR utterance outside locked conversation language",
                detected=language,
                locked=locked_language,
                text=text,
            )
            return

        if language == "zh":
            text = self._normalize_chinese_asr(text)
            text = (
                self._chinese_motion_command_text(text)
                if self.config.preserve_motion_command_numbers
                and self._looks_like_relative_motion_command(text)
                else self._chinese_only(text)
            )
            if not text:
                logger.info("Ignoring Chinese ASR utterance without Chinese characters")
                return
            agent_text = f"请只使用中文回答用户说{text}"
        else:
            agent_text = f"Reply only in English. User said: {text}"

        logger.info("Native G1 ASR transcript", text=text, language=language)
        logger.info("Forwarding native G1 ASR utterance to dialogue agent", text=agent_text)
        self._set_led(255, 180, 0)
        self.human_input.publish(agent_text)

    def _wake_settings(self) -> WakeMatcherSettings:
        config = self.config
        return WakeMatcherSettings(
            chinese_aliases=tuple(config.chinese_wake_aliases),
            english_aliases=tuple(config.english_wake_aliases),
            fuzzy_enabled=getattr(config, "wake_fuzzy_enabled", True),
            fuzzy_threshold=getattr(config, "wake_fuzzy_threshold", 0.8),
            strong_threshold=getattr(config, "wake_strong_threshold", 0.95),
            require_double=getattr(config, "wake_require_double", False),
            double_anywhere=getattr(config, "wake_double_anywhere", True),
            strong_anywhere=getattr(config, "wake_strong_anywhere", False),
        )

    def _match_wake(self, text: str) -> WakeMatch | None:
        return match_wake(text, self._wake_settings())

    def _extract_wake_word(self, text: str) -> tuple[str | None, str]:
        """Return the wake language and the remaining command text."""
        match = self._match_wake(text)
        if match is None:
            return None, text
        logger.info(
            "Voice wake word matched",
            reason=match.reason,
            phrase=match.phrase,
            score=match.score,
            language=match.language,
            text=text,
        )
        return match.language, match.remainder

    def _record_short_wake_attempt(self, text: str, now: float) -> bool:
        """Wake only after repeated short non-Latin results; never execute them."""
        compact = re.sub(r"[\s，。！？,.!?]+", "", text)
        if not compact or len(compact) > 4 or re.search(r"[A-Za-z0-9]", compact):
            self._short_wake_attempts.clear()
            return False
        cutoff = now - self.config.wake_retry_window_s
        while self._short_wake_attempts and self._short_wake_attempts[0] < cutoff:
            self._short_wake_attempts.popleft()
        self._short_wake_attempts.append(now)
        return len(self._short_wake_attempts) >= self.config.wake_retry_count

    @staticmethod
    def _looks_like_emergency_stop(text: str) -> bool:
        """Recognize only conservative stop phrases; false stops are safe."""
        normalized = re.sub(r"[\s，。！？,.!?]+", "", text).lower()
        normalized = re.sub(r"^(?:小诺|nova)?(?:请|please)?", "", normalized)
        return bool(
            re.search(r"(?:停止|停下|别动|紧急停止)", normalized)
            or re.fullmatch(r"停(?:一下|住)?(?:吧)?", normalized)
            or re.fullmatch(r"(?:stop|halt)(?:now|please)?", normalized)
        )

    @staticmethod
    def _chinese_only(text: str) -> str:
        """Keep spoken Chinese free of Latin letters, digits, and symbols."""
        return "".join(re.findall(r"[\u4e00-\u9fff]", text))

    @staticmethod
    def _looks_like_relative_motion_command(text: str) -> bool:
        return bool(
            re.search(
                r"(?:(?:向|往)?前(?:方)?|(?:向|往)?后(?:方)?|后退|"
                r"(?:向|往)?左(?:侧)?|(?:向|往)?右(?:侧)?)",
                text,
            )
            and re.search(r"(?:\d+(?:\.\d+)?|[零〇一二两三四五六七八九十百点]+)\s*米", text)
        )

    @staticmethod
    def _chinese_motion_command_text(text: str) -> str:
        """Keep Chinese and distance notation, but discard Latin/noise."""
        return "".join(re.findall(r"[\u4e00-\u9fff\d.]", text))

    @staticmethod
    def _normalize_chinese_asr(text: str) -> str:
        """Repair a few observed, narrow G1 ASR errors before intent parsing."""
        compact = re.sub(r"[\s，。！？,.!?]+", "", text)
        if compact == "快手":
            return "握手"
        normalized = text.replace("像前", "向前")
        if re.search(r"(?:向前|往前|后退|向后|往后|向左|往左|向右|往右)", normalized):
            normalized = re.sub(
                r"([零〇一二两三四五六七八九十百\d]+)(?:笔|迷|明)",
                r"\1米",
                normalized,
            )
        return normalized

    @staticmethod
    def _requested_language(text: str) -> str | None:
        """Return an explicit user-requested language, never an inference."""
        lowered = re.sub(r"[,.!?，。！？]+", " ", text.lower()).strip()
        compact = re.sub(r"[\s,.!?，。！？]+", "", text.lower())
        if (
            compact in {"中文", "汉语", "普通话", "中文模式"}
            or re.search(
                r"(?:切换|换成|改成|改用|使用|用|说|讲|回答).{0,6}(?:中文|汉语|普通话)", text
            )
            or re.search(
                r"(?:switch|change|speak|use|go)?\s*(?:back\s+)?to\s+(?:chinese|mandarin)\b|"
                r"\b(?:speak|use)\s+(?:chinese|mandarin)\b|"
                r"\b(?:answer|reply)\s+in\s+(?:chinese|mandarin)\b|"
                r"\b(?:chinese|mandarin)(?:\s+(?:mode|please))?\b",
                lowered,
            )
        ):
            return "zh"
        if (
            compact in {"英文", "英语", "英文模式", "英语模式"}
            or re.search(r"(?:切换|换成|改成|改用|使用|用|说|讲|回答).{0,6}(?:英文|英语)", text)
            or re.search(
                r"(?:switch|change|wish|speak|use|go)?\s*(?:back\s+)?to\s+english\b|"
                r"\b(?:switch|change|speak|use)\b.{0,20}\benglish\b|"
                r"\b(?:answer|reply)\s+in\s+english\b|"
                r"\benglish(?:\s+(?:mode|please))?\b",
                lowered,
            )
        ):
            return "en"
        return None

    @staticmethod
    def _classify_asr_language(text: str) -> str:
        """Classify native ASR conservatively without downloading a model."""
        if re.search(r"[\u3040-\u30ff\uac00-\ud7af\u0400-\u052f\u0600-\u06ff]", text):
            return "unsupported"
        if re.search(r"[\u4e00-\u9fff]", text):
            return "zh"
        lowered = text.lower()
        if re.search(r"[äöüß]", lowered) or re.search(
            r"\b(der|die|das|und|ist|nicht|ich|wir|bitte|danke)\b", lowered
        ):
            return "de"
        if re.search(r"[àâçéèêëîïôûùüÿœ]", lowered) or re.search(
            r"\b(le|la|les|bonjour|merci|je|vous|avec|est)\b", lowered
        ):
            return "fr"
        english_words = (
            "a|ahead|an|and|applause|are|back|backward|cancel|can|chinese|clap|confirm|"
            "continue|do|english|execute|forward|go|good|halt|handshake|heart|hello|"
            "hi|how|i|is|language|left|map|me|meter|meters|move|my|navigate|navigation|"
            "no|nova|plan|please|position|resume|right|robot|speak|status|stop|switch|"
            "thank|thanks|the|walk|wave|what|where|who|yes|you|your"
        )
        if re.search(rf"\b({english_words})\b", lowered):
            return "en"
        return "unsupported"

    @staticmethod
    def _asr_text(payload: str) -> str:
        """Extract user text and discard audio-service status notifications."""
        text = payload.strip()
        if not text:
            return ""
        if not text.startswith("{"):
            return text
        try:
            event = json.loads(text)
        except json.JSONDecodeError:
            return text
        if not isinstance(event, dict):
            return ""
        # The native service emits messages such as ``{"play_state": 0}``
        # around TTS playback.  They are control events, not spoken language.
        for key in ("text", "asr", "content"):
            value = event.get(key)
            if isinstance(value, str):
                return value.strip()
        logger.debug("Ignoring non-ASR G1 audio service event", payload=event)
        return ""

    def _on_agent_message(self, message: BaseMessage) -> None:
        """Speak only a final natural-language assistant reply, never tool traffic."""
        if not isinstance(message, AIMessage) or message.tool_calls:
            return
        text = self._message_text(message)
        if not text:
            return
        self._speak(text)

    @staticmethod
    def _message_text(message: AIMessage) -> str:
        content = message.content
        if isinstance(content, str):
            return content.strip()
        if isinstance(content, list):
            parts = [
                item.get("text", "")
                for item in content
                if isinstance(item, dict) and isinstance(item.get("text"), str)
            ]
            return " ".join(parts).strip()
        return ""

    def _speak(self, text: str, speaker_id: int | None = None) -> None:
        client = self._audio_client
        if client is None:
            return
        language = self._conversation_language or self._detect_language(text)
        text = self._spoken_text(text, language)[: self.config.maximum_text_length]
        if not text:
            return
        if speaker_id is None:
            speaker_id = self._speaker_id_for_language(language)
        self._set_led(0, 180, 0)  # speaking
        # Native ASR can hear the speaker.  Ignore it for a conservative
        # duration, preventing a robot reply from becoming a new user prompt.
        duration = max(1.5, min(30.0, 1.0 + len(text) * 0.20))
        with self._lock:
            self._speaking_until = time.monotonic() + duration
            if self.config.wake_word_enabled:
                # Keep the session open for a full window after playback, not
                # merely after the user started the previous utterance.
                self._awake_until = max(
                    self._awake_until,
                    self._speaking_until + self.config.wake_session_s,
                )
        try:
            code = client.TtsMaker(text, speaker_id)
        except (OSError, RuntimeError, ValueError) as exc:
            logger.warning("G1 onboard TTS failed", error=str(exc))
            self._set_led(255, 0, 0)
            return
        if code != 0:
            logger.warning("G1 onboard TTS service returned an error", code=code)
            self._set_led(255, 0, 0)
            return
        logger.info("Spoke dialogue-agent reply through G1 onboard speaker", chars=len(text))
        self._reset_led_after(duration)

    def _speaker_id_for_language(self, language: str) -> int:
        if language == "en":
            return self.config.english_speaker_id
        if language == "de":
            return self.config.german_speaker_id
        return self.config.chinese_speaker_id

    @staticmethod
    def _detect_language(text: str) -> str:
        """Choose an onboard voice from the reply itself without extra models."""
        if re.search(r"[\u4e00-\u9fff]", text):
            return "zh"
        lowered = text.lower()
        german_words = r"\b(der|die|das|und|ist|nicht|ich|wir|bitte|danke|robot|position)\b"
        if re.search(r"[äöüß]", lowered) or re.search(german_words, lowered):
            return "de"
        return "en"

    @staticmethod
    def _spoken_text(text: str, language: str) -> str:
        """Turn model formatting into plain, predictable speech text.

        The G1 firmware TTS reads LaTex delimiters and punctuation literally.
        Keep variable names and numbers, but remove presentation markup and
        replace the few mathematical symbols commonly emitted in pose replies.
        """
        text = text.strip()
        # LaTex inline/display delimiters and escaped punctuation are display
        # syntax, not words users should hear.
        text = re.sub(r"\\\\([()\[\]{}])", r"\1", text)
        text = re.sub(r"[`*_#]", " ", text)
        text = text.replace("\\", " ")
        text = re.sub(r"[()\[\]{}<>]", " ", text)
        text = text.replace("_", " ")
        if language == "zh":
            replacements = {"=": " 等于 ", "+": " 正 ", "-": " 负 ", "/": " 每 ", "%": " 百分之 "}
            text = re.sub(r"(?<=\d)\s*m\b", " 米", text)
            text = re.sub(r"\brad\b", "弧度", text)
        elif language == "de":
            replacements = {"=": " gleich ", "+": " plus ", "/": " pro ", "%": " Prozent "}
            text = re.sub(r"(?<!\w)-(?=\d)", " minus ", text)
            text = re.sub(r"(?<=\d)\s*m\b", " Meter", text)
            text = re.sub(r"\brad\b", "Radiant", text)
        else:
            replacements = {"=": " equals ", "+": " plus ", "/": " per ", "%": " percent "}
            text = re.sub(r"(?<!\w)-(?=\d)", " minus ", text)
            text = re.sub(r"(?<=\d)\s*m\b", " meters", text)
            text = re.sub(r"\brad\b", "radians", text)
        for symbol, spoken in replacements.items():
            text = text.replace(symbol, spoken)
        if language == "zh":
            # The Chinese interaction mode is intentionally strict: never let
            # model labels such as G1, x, yaw, Markdown, or punctuation reach
            # either the speaker or the user-visible ASR/TTS monitor.
            return G1AudioBridge._chinese_only(text)
        # Do not let uncommon control/math symbols reach the firmware TTS.
        # Keep letters, numbers, whitespace, and sentence punctuation only.
        text = re.sub(r"[^\w\s\u4e00-\u9fffäöüßÄÖÜ,.!?;:，。！？；：'-]", " ", text)
        text = text.replace("-", " ")
        text = re.sub(r"\s+", " ", text)
        text = re.sub(r"\s+([,.!?;:。！？；：])", r"\1", text)
        return text.strip(" ,;:")

    def _set_led(self, red: int, green: int, blue: int) -> None:
        client = self._audio_client
        if client is None:
            return
        try:
            code = client.LedControl(red, green, blue)
            if code != 0:
                logger.warning("G1 LED service returned an error", code=code)
        except (OSError, RuntimeError, ValueError) as exc:
            logger.warning("G1 LED control failed", error=str(exc))

    def _reset_led_after(self, seconds: float) -> None:
        with self._lock:
            if self._led_timer is not None:
                self._led_timer.cancel()
            self._led_timer = threading.Timer(seconds, self._set_led, args=(0, 0, 0))
            self._led_timer.daemon = True
            self._led_timer.start()
