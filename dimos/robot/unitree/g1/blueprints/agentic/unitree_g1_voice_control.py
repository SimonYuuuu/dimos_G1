#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Safety-gated voice navigation and upper-body gestures for Unitree G1."""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.ollama_agent import ollama_installed
from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.g1.audio_bridge import G1AudioBridge
from dimos.robot.unitree.g1.blueprints.agentic.unitree_g1_voice_agentic import (
    _VOICE_ENVIRONMENT_PROMPT,
)
from dimos.robot.unitree.g1.blueprints.navigation.unitree_g1_nav_simple import (
    unitree_g1_nav_simple,
)
from dimos.robot.unitree.g1.voice_environment import G1VoiceEnvironmentSkill
from dimos.robot.unitree.g1.voice_motion_control import G1VoiceMotionControl

_VOICE_CONTROL_PROMPT = (
    _VOICE_ENVIRONMENT_PROMPT
    + """
Base navigation, confirmation, emergency stopping, resuming, and upper-body
gestures are handled by a deterministic safety controller before messages
reach you. Never claim that you executed, stopped, resumed, or planned motion.
If asked about supported control, briefly explain that relative navigation
requires confirmation and that stop commands latch motion until continue.
"""
)

# The old nav/voice blueprints are immutable values. Remap only this composed
# variant so the safety controller is the final authority before G1 cmd_vel.
_safety_gated_nav = unitree_g1_nav_simple.remappings(
    [(MovementManager, "cmd_vel", "unsafe_cmd_vel")]
)

unitree_g1_voice_control = autoconnect(
    _safety_gated_nav,
    McpServer.blueprint(
        # Physical commands never go through the LLM. The deterministic module
        # below owns the small allowlisted command grammar and confirmation.
        allowed_skills=["describe_environment", "navigation_status"],
    ),
    McpClient.blueprint(
        model="ollama:qwen2.5:3b-instruct",
        system_prompt=_VOICE_CONTROL_PROMPT,
        realtime_context_keywords=[
            "现在",
            "当前",
            "我在哪里",
            "哪里",
            "在哪",
            "位置",
            "坐标",
            "导航",
            "路径",
            "障碍",
            "地图",
            "状态",
            "where",
            "location",
            "position",
        ],
        realtime_context_tools=["describe_environment", "navigation_status"],
    ),
    G1VoiceEnvironmentSkill.blueprint(),
    G1VoiceMotionControl.blueprint(),
    G1AudioBridge.blueprint(
        allow_stop_while_speaking=True,
        preserve_motion_command_numbers=True,
        wake_word_enabled=True,
        wake_session_s=30.0,
    ).remappings([(G1AudioBridge, "human_input", "voice_control_input")]),
).requirements(ollama_installed)
