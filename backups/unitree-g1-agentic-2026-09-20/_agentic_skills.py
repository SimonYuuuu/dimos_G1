"""Backup of the pre-voice-agent shared G1 agent skill composition."""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.agents.skills.observe_skill import ObserveSkill
from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.skill_container import UnitreeG1SkillContainer
from dimos.robot.unitree.g1.system_prompt import G1_SYSTEM_PROMPT

_agentic_skills = autoconnect(
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=G1_SYSTEM_PROMPT),
    NavigationSkillContainer.blueprint(),
    ObserveSkill.blueprint(),
    SpeakSkill.blueprint(),
    UnitreeG1SkillContainer.blueprint(),
)
