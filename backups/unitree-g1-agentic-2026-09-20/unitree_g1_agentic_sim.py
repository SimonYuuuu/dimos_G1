"""Backup of the pre-voice-agent G1 simulation blueprint."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.blueprints.agentic._agentic_skills import _agentic_skills
from dimos.robot.unitree.g1.blueprints.perceptive.unitree_g1_sim import unitree_g1_sim

unitree_g1_agentic_sim = autoconnect(
    unitree_g1_sim,
    _agentic_skills,
)
