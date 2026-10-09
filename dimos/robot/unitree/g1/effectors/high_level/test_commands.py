from dimos.robot.unitree.g1.effectors.high_level.commands import (
    ARM_API_ID,
    ARM_COMMANDS,
    ARM_TOPIC,
    execute_g1_command,
)


def test_arm_command_forwards_action_id() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    def publish(topic: str, data: dict[str, object]) -> dict[str, int]:
        published.append((topic, data))
        return {"code": 0}

    result = execute_g1_command(publish, ARM_COMMANDS, ARM_API_ID, ARM_TOPIC, "FaceWave")

    assert result == "'FaceWave' command executed successfully."
    assert published == [(ARM_TOPIC, {"api_id": 7106, "parameter": {"data": 25}})]


def test_arm_command_reports_hardware_rejection() -> None:
    result = execute_g1_command(
        lambda *_args: {"code": -1, "error": "unsupported_api"},
        ARM_COMMANDS,
        ARM_API_ID,
        ARM_TOPIC,
        "FaceWave",
    )

    assert result == "Failed to execute 'FaceWave' command (code=-1, unsupported_api)."


def test_heart_actions_are_disabled_before_hardware_dispatch() -> None:
    published: list[tuple[str, dict[str, object]]] = []

    result = execute_g1_command(
        lambda topic, data: published.append((topic, data)) or {"code": 0},
        ARM_COMMANDS,
        ARM_API_ID,
        ARM_TOPIC,
        "ArmHeart",
    )

    assert result == "'ArmHeart' is disabled by the G1 safety policy."
    assert published == []
