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

"""Wake-word matching for the G1 native ASR text stream.

The onboard ASR only returns final text, so the wake word has to be
recovered from whatever characters it produced.  Field logs show that
"小诺" is frequently transcribed as near-homophones such as "角落" or "缴落".
This module scores Chinese candidates by pinyin (initial + final) with a
small built-in syllable table, because ``pypinyin`` is not installed in the
offline Orin environment.  Unknown characters simply never match.

Rules, from most to least permissive:

1. Doubled phrase ("小诺小诺", "角落小诺", "nova nova"): accepted anywhere in
   the sentence.  One half must be a strong match (exact xiao+nuo pinyin /
   "nova"), the other at least a fuzzy match.  Two independent near-misses
   in a row are very unlikely in ordinary speech.
2. Strong single ("小诺", "晓诺" ...): accepted at sentence start, or anywhere
   when ``strong_anywhere`` is enabled.
3. Configured single aliases (old behavior, e.g. "小罗", "机器人"): accepted
   only at sentence start.
4. Fuzzy-only single ("角落", "脚落"): accepted only when it is the whole
   utterance, so "把杯子放在角落" never wakes the robot.
"""

# ruff: noqa: RUF001

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise
import re

# Built-in pinyin table for characters seen (or plausible) in G1 ASR output
# around the wake word.  Value: (initial, final).
_PINYIN: dict[str, tuple[str, str]] = {}
for _chars, _syllable in (
    ("小晓筱孝效笑校肖消销萧箫硝宵", ("x", "iao")),
    ("角脚缴饺搅叫教较交胶焦娇骄浇", ("j", "iao")),
    ("巧桥乔敲俏翘悄", ("q", "iao")),
    ("诺糯懦挪喏娜", ("n", "uo")),
    ("落洛罗萝骆络螺锣逻裸烙", ("l", "uo")),
    ("若弱", ("r", "uo")),
    ("末莫默墨摸磨魔", ("m", "o")),
    ("老劳", ("l", "ao")),
    ("闹恼脑", ("n", "ao")),
):
    for _c in _chars:
        _PINYIN[_c] = _syllable
_PINYIN["娜"] = ("n", "a")

_INITIAL_GROUPS = ({"x", "j", "q"}, {"n", "l", "r"})
_FINAL_GROUPS = ({"uo", "o", "ao"}, {"iao", "ao"})
_TARGET: tuple[tuple[str, str], ...] = (("x", "iao"), ("n", "uo"))

_PUNCT = r"\s，。！？、,.!?~…：:；;\"'“”"
_GREETING_RE = re.compile(rf"^(?:你好|您好|嗨|嘿|喂|哎|hey|hello|hi)[{_PUNCT}]*", re.IGNORECASE)
_TRAILING_PARTICLES = "啊呀吧呢哦嘛"

ENGLISH_STRONG = "nova"
ENGLISH_FUZZY_ALIASES = ("nover", "noah", "nava", "novah", "novo", "nowa", "noba")


def _part_sim(a: str, b: str, groups: tuple[set[str], ...]) -> float:
    if a == b:
        return 1.0
    return 0.7 if any(a in g and b in g for g in groups) else 0.0


def syllable_score(char: str, target: tuple[str, str]) -> float:
    py = _PINYIN.get(char)
    if py is None:
        return 0.0
    return 0.5 * _part_sim(py[0], target[0], _INITIAL_GROUPS) + 0.5 * _part_sim(
        py[1], target[1], _FINAL_GROUPS
    )


def chinese_score(pair: str) -> float:
    """Pinyin similarity of a 2-character string to "xiao nuo" (0..1)."""
    if len(pair) != 2:
        return 0.0
    first, second = (syllable_score(c, t) for c, t in zip(pair, _TARGET, strict=True))
    # Both syllables must be at least plausible.
    if min(first, second) < 0.5:
        return 0.0
    return (first + second) / 2


def edit_distance(a: str, b: str) -> int:
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1]


def english_score(token: str) -> float:
    token = token.lower()
    if token == ENGLISH_STRONG:
        return 1.0
    if token in ENGLISH_FUZZY_ALIASES:
        return 0.85
    if len(token) >= 4 and edit_distance(token, ENGLISH_STRONG) <= 1:
        return 0.8
    return 0.0


@dataclass(frozen=True)
class WakeMatch:
    language: str
    remainder: str
    reason: str
    phrase: str
    score: float


@dataclass(frozen=True)
class WakeMatcherSettings:
    chinese_aliases: tuple[str, ...]
    english_aliases: tuple[str, ...]
    fuzzy_enabled: bool = True
    fuzzy_threshold: float = 0.8
    strong_threshold: float = 0.95
    require_double: bool = False
    double_anywhere: bool = True
    strong_anywhere: bool = False
    standalone_aliases: tuple[str, ...] = ("行模式", "醒模式")


def _clean_remainder(text: str) -> str:
    return re.sub(rf"^[{_PUNCT}{_TRAILING_PARTICLES}]+", "", text).strip()


def _strip_prefix(text: str) -> str:
    stripped = re.sub(rf"^[{_PUNCT}]+", "", text).strip()
    return _GREETING_RE.sub("", stripped).strip()


def _is_standalone(text: str) -> bool:
    compact = re.sub(rf"[{_PUNCT}]+", "", text)
    return compact.rstrip(_TRAILING_PARTICLES) == ""


def _han_positions(text: str) -> list[int]:
    return [i for i, c in enumerate(text) if "\u4e00" <= c <= "\u9fff"]


def _chinese_score(pair: str, s: WakeMatcherSettings) -> float:
    if not s.fuzzy_enabled:
        # Exact xiao+nuo pinyin still counts as strong (covers 晓糯 etc.).
        score = chinese_score(pair)
        return score if score >= s.strong_threshold else 0.0
    return chinese_score(pair)


def match_wake(text: str, s: WakeMatcherSettings) -> WakeMatch | None:
    """Return the wake match for ``text`` or ``None``."""
    body = _strip_prefix(text)
    if not body:
        return None

    # --- Chinese doubled phrase (anywhere) and strong singles -------------
    pos = _han_positions(body)
    han = "".join(body[i] for i in pos)
    scores = [_chinese_score(han[k : k + 2], s) for k in range(max(len(han) - 1, 0))]
    for k in range(len(scores) - 2):
        a, b = scores[k], scores[k + 2]
        if max(a, b) >= s.strong_threshold and min(a, b) >= s.fuzzy_threshold:
            if not s.double_anywhere and k != 0:
                continue
            end = pos[k + 3] + 1
            return WakeMatch(
                "zh",
                _clean_remainder(body[end:]),
                "double",
                han[k : k + 4],
                round((a + b) / 2, 3),
            )

    # --- English doubled phrase ------------------------------------------
    tokens = list(re.finditer(r"[A-Za-z]+", body))
    for t1, t2 in pairwise(tokens):
        a, b = english_score(t1.group()), english_score(t2.group())
        between = body[t1.end() : t2.start()]
        if re.fullmatch(rf"[{_PUNCT}]*", between) and max(a, b) >= 1.0 and min(a, b) >= 0.8:
            if not s.double_anywhere and t1.start() != 0:
                continue
            return WakeMatch(
                "en",
                _clean_remainder(body[t2.end() :]),
                "double",
                f"{t1.group()} {t2.group()}",
                (a + b) / 2,
            )

    if s.require_double:
        return None

    # --- Strong single Chinese (start, or anywhere if enabled) -----------
    for k, score in enumerate(scores):
        if score >= s.strong_threshold and (s.strong_anywhere or pos[k] == 0):
            reason = "strong_start" if pos[k] == 0 else "strong_anywhere"
            return WakeMatch(
                "zh", _clean_remainder(body[pos[k + 1] + 1 :]), reason, han[k : k + 2], score
            )

    # --- Configured aliases at sentence start (legacy behavior) ----------
    lowered = body.lower()
    for alias in sorted(s.chinese_aliases, key=len, reverse=True):
        if body.startswith(alias):
            return WakeMatch("zh", _clean_remainder(body[len(alias) :]), "alias_start", alias, 1.0)
    for alias in sorted(s.english_aliases, key=len, reverse=True):
        if lowered.startswith(alias) and (
            len(lowered) == len(alias) or not lowered[len(alias)].isalpha()
        ):
            return WakeMatch("en", _clean_remainder(body[len(alias) :]), "alias_start", alias, 1.0)

    if not s.fuzzy_enabled:
        return None

    # --- English fuzzy single at sentence start ---------------------------
    if tokens and tokens[0].start() == 0:
        score = english_score(tokens[0].group())
        if score >= 0.8:
            return WakeMatch(
                "en",
                _clean_remainder(body[tokens[0].end() :]),
                "fuzzy_start",
                tokens[0].group(),
                score,
            )

    # --- Fuzzy-only Chinese single: whole utterance only ------------------
    if len(han) >= 2 and pos[0] == 0 and _is_standalone(body[pos[1] + 1 :]):
        score = scores[0]
        if score >= s.fuzzy_threshold:
            return WakeMatch("zh", "", "fuzzy_standalone", han[:2], score)
    for alias in s.standalone_aliases:
        if body.startswith(alias) and _is_standalone(body[len(alias) :]):
            return WakeMatch("zh", "", "fuzzy_standalone", alias, 0.8)
    return None
