"""Safe first-stage G1 environment dialogue on top of the nav-simple stack.

This blueprint exposes no movement, gesture, or camera skill.  It uses the
G1's native ASR/TTS/LED service as an input/output bridge, while the local
model remains restricted to describing live lidar navigation state.
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.ollama_agent import ollama_installed
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.audio_bridge import G1AudioBridge
from dimos.robot.unitree.g1.blueprints.navigation.unitree_g1_nav_simple import (
    unitree_g1_nav_simple,
)
from dimos.robot.unitree.g1.voice_environment import G1VoiceEnvironmentSkill

_VOICE_ENVIRONMENT_PROMPT = """
You are a read-only environment dialogue assistant. The audio bridge prefixes
every accepted user turn with a mandatory output language. Obey that prefix
exactly and keep using that language until a later user turn explicitly asks
to switch language. Never infer a switch from a short or noisy ASR transcript.
In Chinese mode your name is 小诺. You are a 宇树机器人 from the 采埃孚人工智能实验室.
Use Chinese characters only: no Latin letters, digits, or punctuation. In
English mode your name is Nova. You are a Unitree G1 robot from the ZF
Artificial Intelligence Lab. Use at most two short spoken sentences.
The reply is played directly by a robot speaker, so output plain speech only:
no Markdown, LaTex, backslashes, parentheses, equations, code, lists, or
symbols. Do not add invitations, explanations, or repeated map statistics
unless explicitly requested. For questions about the robot's current pose,
mapped free space, obstacles, or path, use the fresh real-time context
supplied for the current user turn. Never reuse an older pose or navigation
state from conversation history as if it were current.
Do not claim to see people, objects, rooms, or camera imagery: this first
stage has lidar-map awareness only. You have no authority to move the robot,
change its mode, send gestures, or cancel navigation.
"""

unitree_g1_voice_agentic = autoconnect(
    unitree_g1_nav_simple,
    McpServer.blueprint(
        # Prompt instructions alone are insufficient to enforce read-only
        # authority.  Keep the navigation stack's motion/gesture skills out
        # of the MCP registry entirely during this first milestone.
        # ``humancli`` publishes directly to /human_input, so do not expose
        # agent_send to the model.  Otherwise it can recursively send itself a
        # textual "set goal" request and falsely claim that navigation started.
        allowed_skills=["describe_environment", "navigation_status"],
    ),
    McpClient.blueprint(
        model="ollama:qwen2.5:3b-instruct",
        system_prompt=_VOICE_ENVIRONMENT_PROMPT,
        realtime_context_keywords=[
            "现在",
            "当前",
            "目前",
            "我在哪里",
            "哪里",
            "在哪",
            "坐标",
            "位置",
            "定位",
            "导航",
            "路径",
            "目标",
            "障碍",
            "周围",
            "地图",
            "状态",
            "where",
            "location",
            "position",
        ],
        realtime_context_tools=["describe_environment", "navigation_status"],
    ),
    G1VoiceEnvironmentSkill.blueprint(),
    G1AudioBridge.blueprint(),
).requirements(ollama_installed)
