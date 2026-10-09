#!/usr/bin/env python3
"""Render selected DimOS JSON log events as a compact voice transcript."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
import json
import sys

LOCAL_TIMEZONE = timezone(timedelta(hours=8))


def local_time(timestamp: str) -> str:
    try:
        return datetime.fromisoformat(timestamp.replace("Z", "+00:00")).astimezone(
            LOCAL_TIMEZONE
        ).strftime("%H:%M:%S")
    except (TypeError, ValueError):
        return "--:--:--"


def clean_user_text(text: str) -> str:
    chinese_prefix = "请只使用中文回答用户说"
    english_prefix = "Reply only in English. User said:"
    if text.startswith(chinese_prefix):
        return text[len(chinese_prefix) :].strip()
    if text.startswith(english_prefix):
        return text[len(english_prefix) :].strip()
    return text.strip()


def render(record: dict[str, object]) -> tuple[str, str] | None:
    event = str(record.get("event", ""))
    text = str(record.get("text", "")).strip()

    if event == "Raw native G1 ASR transcript":
        return "识别", text
    if event == "Voice wake word accepted":
        language = "中文" if record.get("language") == "zh" else "英文"
        return "状态", f"唤醒成功  {language}"
    if event == "Voice wake fallback accepted after repeated short ASR results":
        return "状态", "连续短语音容错唤醒成功"
    if event == "Ignoring ASR while voice assistant is asleep":
        return "休眠", f"未匹配唤醒词  {text}"
    if event == "Voice conversation language switched":
        language = "中文" if record.get("language") == "zh" else "英文"
        return "状态", f"已切换到{language}"
    if event == "Treating unsupported ASR result as failed Chinese recognition":
        return "重试", f"疑似中文误识别  {text}"
    if event == "Ignoring unsupported ASR language":
        return "忽略", f"无法识别语言  {text}"
    if event == "Ignoring ASR utterance outside locked conversation language":
        return "忽略", f"与当前语言不一致  {text}"
    if event == "Agent message" and record.get("msg_type") == "ai":
        content = str(record.get("content", "")).strip()
        return ("小诺", content) if content else None
    if event == "G1 deterministic voice-control response":
        return "小诺", text
    if event == "G1 gesture rejected by stability gate":
        return "安全", "动作被稳定性检查拒绝"
    if event == "G1 voice arm command handled":
        return "动作", str(record.get("command", ""))
    if event == "G1 onboard audio bridge ready":
        return "系统", "麦克风与本体语音识别已连接"
    return None


for line in sys.stdin:
    try:
        record = json.loads(line)
    except (json.JSONDecodeError, TypeError):
        continue
    rendered = render(record)
    if rendered is None:
        continue
    role, content = rendered
    print(f"{local_time(str(record.get('timestamp', '')))}  {role:<4}  {content}", flush=True)
