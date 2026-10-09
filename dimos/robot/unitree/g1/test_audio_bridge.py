from collections import deque
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

from langchain_core.messages import AIMessage, HumanMessage

from dimos.robot.unitree.g1.audio_bridge import G1AudioBridge


def _bridge() -> G1AudioBridge:
    bridge = object.__new__(G1AudioBridge)
    bridge.config = SimpleNamespace(
        maximum_text_length=400,
        chinese_speaker_id=0,
        english_speaker_id=1,
        german_speaker_id=1,
        allow_stop_while_speaking=False,
        preserve_motion_command_numbers=False,
        wake_word_enabled=False,
        wake_session_s=15.0,
        wake_retry_count=3,
        wake_retry_window_s=25.0,
        chinese_wake_aliases=(
            "小诺",
            "小糯",
            "小懦",
            "小挪",
            "晓诺",
            "晓糯",
            "小若",
            "小洛",
            "小落",
            "小罗",
            "小娜",
        ),
        english_wake_aliases=("nova", "nover"),
    )
    bridge._audio_client = MagicMock()
    bridge._asr_subscriber = None
    bridge._lock = threading.Lock()
    bridge._speaking_until = 0.0
    bridge._led_timer = None
    bridge._conversation_language = None
    bridge._awake_until = 0.0
    bridge._short_wake_attempts = deque(maxlen=8)
    bridge.human_input = MagicMock()
    return bridge


def test_asr_forwards_native_text_to_agent_input() -> None:
    bridge = _bridge()

    bridge._on_asr_message(SimpleNamespace(data="我在哪里"))

    bridge.human_input.publish.assert_called_once_with("请只使用中文回答用户说我在哪里")
    bridge._audio_client.LedControl.assert_called_with(255, 180, 0)


def test_audio_service_playback_status_is_not_treated_as_speech() -> None:
    bridge = _bridge()

    bridge._on_asr_message(SimpleNamespace(data='{"play_state": 0}'))

    bridge.human_input.publish.assert_not_called()


def test_punctuation_only_asr_result_is_not_treated_as_speech() -> None:
    bridge = _bridge()

    bridge._on_asr_message(SimpleNamespace(data="..."))

    bridge.human_input.publish.assert_not_called()


def test_microphone_stream_uses_voice_api_1008() -> None:
    client = MagicMock()
    client._Call.return_value = (0, "")

    G1AudioBridge._set_microphone_stream_mode(client, 1)

    client._RegistApi.assert_called_once_with(1008, 0)
    client._Call.assert_called_once_with(1008, '{"mode": 1}')


def test_tts_speaks_only_final_ai_reply_and_ignores_its_echo() -> None:
    bridge = _bridge()

    bridge._on_agent_message(HumanMessage(content="不要念这个"))
    bridge._on_agent_message(AIMessage(content="我在这里。", tool_calls=[]))
    bridge._on_asr_message(SimpleNamespace(data="我在这里"))

    bridge._audio_client.TtsMaker.assert_called_once_with("我在这里", 0)
    bridge.human_input.publish.assert_not_called()
    assert bridge._speaking_until > time.monotonic()
    if bridge._led_timer is not None:
        bridge._led_timer.cancel()


def test_tool_call_message_is_not_spoken() -> None:
    bridge = _bridge()

    bridge._on_agent_message(
        AIMessage(
            content="",
            tool_calls=[{"name": "describe_environment", "args": {}, "id": "call-1"}],
        )
    )

    bridge._audio_client.TtsMaker.assert_not_called()


def test_tts_strips_latex_and_uses_english_voice() -> None:
    bridge = _bridge()

    bridge._speak(r"The pose is \\(x = -1 m\\).")

    bridge._audio_client.TtsMaker.assert_called_once_with("The pose is x equals minus 1 meters.", 1)
    if bridge._led_timer is not None:
        bridge._led_timer.cancel()


def test_tts_uses_chinese_voice_and_speaks_symbols_as_words() -> None:
    bridge = _bridge()

    bridge._speak(r"位置是 \\(x=-1\\)。")

    bridge._audio_client.TtsMaker.assert_called_once_with("位置是等于负", 0)
    if bridge._led_timer is not None:
        bridge._led_timer.cancel()


def test_tts_removes_unrecognised_symbols() -> None:
    bridge = _bridge()

    bridge._speak("位置 @ x=1 < 2")

    bridge._audio_client.TtsMaker.assert_called_once_with("位置等于", 0)
    if bridge._led_timer is not None:
        bridge._led_timer.cancel()


def test_unsupported_asr_never_reaches_language_model() -> None:
    bridge = _bridge()

    bridge._on_asr_message(SimpleNamespace(data="hola amigo"))

    bridge.human_input.publish.assert_not_called()
    bridge._audio_client.TtsMaker.assert_called_once_with(
        "目前只支持中文和英文其他语言正在开发中", 0
    )
    if bridge._led_timer is not None:
        bridge._led_timer.cancel()


def test_explicit_language_switch_locks_following_turns() -> None:
    bridge = _bridge()

    bridge._on_asr_message(SimpleNamespace(data="请切换为英文"))
    # Simulate the short acknowledgement finishing before the next command.
    bridge._speaking_until = 0.0
    bridge._on_asr_message(SimpleNamespace(data="Where am I"))

    assert bridge._conversation_language == "en"
    assert bridge.human_input.publish.call_args_list[0].args == (
        "Reply only in English. User said: Where am I",
    )


def test_language_switch_variants_bypass_lock_and_tts_gate() -> None:
    bridge = _bridge()
    bridge._conversation_language = "en"
    bridge._speaking_until = time.monotonic() + 30.0

    bridge._on_asr_message(SimpleNamespace(data="to Chinese"))

    assert bridge._conversation_language == "zh"
    bridge._audio_client.TtsMaker.assert_called_once_with("好的", 0)
    bridge.human_input.publish.assert_not_called()

    bridge._speaking_until = 0.0
    bridge._on_asr_message(SimpleNamespace(data="Wish to English"))
    assert bridge._conversation_language == "en"


def test_language_switch_phrase_variants_are_recognized() -> None:
    assert G1AudioBridge._requested_language("中文") == "zh"
    assert G1AudioBridge._requested_language("English") == "en"
    assert G1AudioBridge._requested_language("说中文") == "zh"
    assert G1AudioBridge._requested_language("to Chinese") == "zh"
    assert G1AudioBridge._requested_language("Wish to English") == "en"
    assert G1AudioBridge._requested_language("Can you speak English") == "en"


def test_wake_name_switches_conversation_language_immediately() -> None:
    bridge = _bridge()
    bridge.config.wake_word_enabled = True
    bridge._conversation_language = "en"

    bridge._on_asr_message(SimpleNamespace(data="小诺"))

    assert bridge._conversation_language == "zh"
    bridge._audio_client.TtsMaker.assert_called_once_with("我在请说", 0)

    bridge._speaking_until = 0.0
    bridge._on_asr_message(SimpleNamespace(data="Nova"))
    assert bridge._conversation_language == "en"


def test_repeated_short_wrong_language_results_trigger_safe_wake_only() -> None:
    bridge = _bridge()
    bridge.config.wake_word_enabled = True

    bridge._on_asr_message(SimpleNamespace(data="あの"))
    bridge._on_asr_message(SimpleNamespace(data="場しょ"))
    bridge._on_asr_message(SimpleNamespace(data="まあ"))

    assert bridge._conversation_language == "zh"
    assert bridge._awake_until > time.monotonic()
    bridge._audio_client.TtsMaker.assert_called_once_with("我在请说", 0)
    bridge.human_input.publish.assert_not_called()


def test_wrong_language_result_in_chinese_mode_states_supported_languages() -> None:
    bridge = _bridge()
    bridge._conversation_language = "zh"

    bridge._on_asr_message(SimpleNamespace(data="うん"))

    bridge._audio_client.TtsMaker.assert_called_once_with(
        "目前只支持中文和英文其他语言正在开发中", 0
    )
    bridge.human_input.publish.assert_not_called()


def test_single_language_name_switches_without_calling_model() -> None:
    bridge = _bridge()

    bridge._on_asr_message(SimpleNamespace(data="English"))
    assert bridge._conversation_language == "en"
    bridge.human_input.publish.assert_not_called()

    bridge._speaking_until = 0.0
    bridge._on_asr_message(SimpleNamespace(data="中文"))
    assert bridge._conversation_language == "zh"
    bridge.human_input.publish.assert_not_called()


def test_observed_chinese_asr_command_errors_are_repaired() -> None:
    assert G1AudioBridge._normalize_chinese_asr("快手?") == "握手"
    assert G1AudioBridge._normalize_chinese_asr("向前走两笔。") == "向前走两米。"


def test_chinese_lock_ignores_incidental_english_asr() -> None:
    bridge = _bridge()
    bridge._conversation_language = "zh"

    bridge._on_asr_message(SimpleNamespace(data="hello robot"))

    bridge.human_input.publish.assert_not_called()


def test_chinese_tts_removes_latin_letters_digits_and_symbols() -> None:
    bridge = _bridge()
    bridge._conversation_language = "zh"

    bridge._speak("我是小诺 G1，位置 x=2。")  # noqa: RUF001

    bridge._audio_client.TtsMaker.assert_called_once_with("我是小诺位置等于", 0)
    if bridge._led_timer is not None:
        bridge._led_timer.cancel()


def test_motion_mode_preserves_distance_digits_only_for_navigation() -> None:
    bridge = _bridge()
    bridge.config.preserve_motion_command_numbers = True

    bridge._on_asr_message(SimpleNamespace(data="请向前走3.5米"))

    bridge.human_input.publish.assert_called_once_with("请只使用中文回答用户说请向前走3.5米")


def test_english_safety_commands_are_classified_as_english() -> None:
    assert G1AudioBridge._classify_asr_language("stop") == "en"
    assert G1AudioBridge._classify_asr_language("move forward 3 meters") == "en"


def test_near_homophone_wakes_then_opens_conversation_window() -> None:
    bridge = _bridge()
    bridge.config.wake_word_enabled = True

    bridge._on_asr_message(SimpleNamespace(data="小糯"))

    bridge._audio_client.TtsMaker.assert_called_once_with("我在请说", 0)
    bridge.human_input.publish.assert_not_called()

    # Simulate the short acknowledgement completing before the next turn.
    bridge._speaking_until = 0.0
    bridge._on_asr_message(SimpleNamespace(data="我在哪里"))
    bridge.human_input.publish.assert_called_once_with("请只使用中文回答用户说我在哪里")
    if bridge._led_timer is not None:
        bridge._led_timer.cancel()


def test_greeting_before_wake_word_is_accepted() -> None:
    bridge = _bridge()
    bridge.config.wake_word_enabled = True

    bridge._on_asr_message(SimpleNamespace(data="你好，小诺"))  # noqa: RUF001

    bridge._audio_client.TtsMaker.assert_called_once_with("我在请说", 0)
    if bridge._led_timer is not None:
        bridge._led_timer.cancel()


def test_sleeping_assistant_ignores_dialogue_but_never_blocks_stop() -> None:
    bridge = _bridge()
    bridge.config.wake_word_enabled = True

    bridge._on_asr_message(SimpleNamespace(data="我在哪里"))
    bridge._on_asr_message(SimpleNamespace(data="停下"))

    bridge.human_input.publish.assert_called_once_with("请只使用中文回答用户说停下")
