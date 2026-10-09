"""Backup of the pre-voice-agent G1 hardware agentic blueprint."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.blueprints.agentic._agentic_skills import _agentic_skills
from dimos.robot.unitree.g1.blueprints.perceptive.unitree_g1 import unitree_g1

unitree_g1_agentic = autoconnect(
    unitree_g1,
    _agentic_skills,
)
