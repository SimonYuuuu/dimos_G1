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

# ruff: noqa: RUF001

from types import SimpleNamespace
from typing import Any

import pytest

from dimos.robot.unitree.g1.audio_bridge import G1AudioBridgeConfig
from dimos.robot.unitree.g1.test_audio_bridge import _bridge
from dimos.robot.unitree.g1.wake_word import (
    WakeMatcherSettings,
    chinese_score,
    english_score,
    match_wake,
)

_CONFIG = G1AudioBridgeConfig()
SETTINGS = WakeMatcherSettings(
    chinese_aliases=_CONFIG.chinese_wake_aliases,
    english_aliases=_CONFIG.english_wake_aliases,
)


@pytest.mark.parametrize(
    ("text", "language", "remainder", "reason"),
    [
        ("小诺小诺", "zh", "", "double"),
        ("小诺小诺。", "zh", "", "double"),
        ("角落小诺", "zh", "", "double"),
        ("小诺角落", "zh", "", "double"),
        ("小诺，小诺，向前走", "zh", "向前走", "double"),
        ("你好小诺小诺向前走4米", "zh", "向前走4米", "double"),
        ("那个我说小诺小诺握手", "zh", "握手", "double"),
        ("nova nova move forward", "en", "move forward", "double"),
        ("Nova, Nova. Move forward.", "en", "Move forward.", "double"),
        ("ok so nova noah stop", "en", "stop", "double"),
        ("小诺", "zh", "", "strong_start"),
        ("小诺握手", "zh", "握手", "strong_start"),
        ("你好，小诺", "zh", "", "strong_start"),
        ("小罗向前走", "zh", "向前走", "alias_start"),
        ("机器人", "zh", "", "alias_start"),
        ("Nova", "en", "", "alias_start"),
        ("hey nova, wave", "en", "wave", "alias_start"),
        ("Noah", "en", "", "fuzzy_start"),
        ("角落。", "zh", "", "fuzzy_standalone"),
        ("角落啊。", "zh", "", "fuzzy_standalone"),
        ("行模式。", "zh", "", "fuzzy_standalone"),
    ],
)
def test_wake_matches(text: str, language: str, remainder: str, reason: str) -> None:
    match = match_wake(text, SETTINGS)
    assert match is not None
    assert (match.language, match.remainder, match.reason) == (language, remainder, reason)


@pytest.mark.parametrize(
    "text",
    [
        "把杯子放在角落",
        "角落里有什么",
        "角落角落",  # two weak matches without a strong half
        "今天小诺表现不错",  # single mid-sentence is off by default
        "我在哪里",
        "已进入走跑模式。",
        "move forward",
        "I love the novel",
        "角末",
    ],
)
def test_wake_negatives(text: str) -> None:
    assert match_wake(text, SETTINGS) is None


def test_require_double_disables_single_wake() -> None:
    settings = WakeMatcherSettings(
        chinese_aliases=SETTINGS.chinese_aliases,
        english_aliases=SETTINGS.english_aliases,
        require_double=True,
    )
    assert match_wake("小诺", settings) is None
    assert match_wake("Nova", settings) is None
    assert match_wake("小诺小诺", settings) is not None


def test_fuzzy_disabled_keeps_exact_behavior() -> None:
    settings = WakeMatcherSettings(
        chinese_aliases=SETTINGS.chinese_aliases,
        english_aliases=SETTINGS.english_aliases,
        fuzzy_enabled=False,
    )
    assert match_wake("角落", settings) is None
    assert match_wake("角落小诺", settings) is None
    assert match_wake("小诺小诺", settings) is not None
    assert match_wake("小诺", settings) is not None


def test_strong_anywhere_option() -> None:
    settings = WakeMatcherSettings(
        chinese_aliases=SETTINGS.chinese_aliases,
        english_aliases=SETTINGS.english_aliases,
        strong_anywhere=True,
    )
    match = match_wake("那个小诺向前走", settings)
    assert match is not None and match.remainder == "向前走"


def test_scores() -> None:
    assert chinese_score("小诺") == 1.0
    assert chinese_score("晓糯") == 1.0
    assert 0.8 <= chinese_score("角落") < 0.95
    assert chinese_score("杯子") == 0.0
    assert english_score("NOVA") == 1.0
    assert english_score("nava") >= 0.8
    assert english_score("move") == 0.0


def test_bridge_double_wake_forwards_command_and_sets_language() -> None:
    bridge: Any = _bridge()
    bridge.config.wake_word_enabled = True
    bridge.config.preserve_motion_command_numbers = True
    bridge._conversation_language = "en"

    bridge._on_asr_message(SimpleNamespace(data="你好小诺小诺向前走4米"))

    assert bridge._conversation_language == "zh"
    bridge.human_input.publish.assert_called_once_with("请只使用中文回答用户说向前走4米")
    if bridge._led_timer is not None:
        bridge._led_timer.cancel()


def test_bridge_ordinary_corner_sentence_does_not_wake() -> None:
    bridge: Any = _bridge()
    bridge.config.wake_word_enabled = True

    bridge._on_asr_message(SimpleNamespace(data="把杯子放在角落"))

    bridge._audio_client.TtsMaker.assert_not_called()
    bridge.human_input.publish.assert_not_called()


def test_bridge_emergency_stop_still_bypasses_wake() -> None:
    bridge: Any = _bridge()
    bridge.config.wake_word_enabled = True

    bridge._on_asr_message(SimpleNamespace(data="停下"))

    bridge.human_input.publish.assert_called_once_with("请只使用中文回答用户说停下")
    if bridge._led_timer is not None:
        bridge._led_timer.cancel()
