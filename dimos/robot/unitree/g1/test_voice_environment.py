import threading
from unittest.mock import MagicMock

import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.robot.unitree.g1.voice_environment import G1VoiceEnvironmentSkill


def test_describe_environment_reports_only_live_map_facts() -> None:
    # Do not call Module.__init__: it opens the process-wide transport session,
    # which is unrelated to this pure formatting/state test.
    skill = object.__new__(G1VoiceEnvironmentSkill)
    skill._lock = threading.Lock()
    skill._latest_odometry = None
    skill._latest_costmap = None
    skill._latest_path = None
    skill._on_odometry(Odometry(frame_id="map", pose=Pose(position=[1.25, -0.5, 0.0])))
    skill._on_costmap(
        OccupancyGrid(grid=np.array([[0, -1], [100, 0]], dtype=np.int8), resolution=0.5)
    )
    skill._on_path(Path(poses=[PoseStamped(position=[1.0, 0.0, 0.0])]))

    description = skill.describe_environment()

    assert "x=1.25 m, y=-0.50 m" in description
    assert "odometry is live" in description
    assert "3 known cells, 2 free, and 1 occupied" in description
    assert "1 waypoints" in description
    assert "camera" not in description.lower()


def test_navigation_status_reports_terminal_failure() -> None:
    skill = object.__new__(G1VoiceEnvironmentSkill)
    skill._lock = threading.Lock()
    skill._latest_path = Path()
    skill._navigation = MagicMock()
    skill._navigation.get_state.return_value.value = "idle"
    skill._navigation.is_goal_reached.return_value = False
    skill._navigation.get_diagnostics.return_value = {
        "goal": None,
        "last_event": "navigation_cancelled",
        "replan_attempt": 6,
        "last_failure_reason": "stuck_retry_limit",
    }

    status = skill.navigation_status()

    assert "stuck_retry_limit" in status
    assert "replan attempts: 6" in status
