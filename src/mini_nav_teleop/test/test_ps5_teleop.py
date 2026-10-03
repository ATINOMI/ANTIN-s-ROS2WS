import importlib.util
from pathlib import Path

import pytest

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('ps5_teleop', root / 'scripts/ps5_teleop.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def state(left_y=128, right_x=128, pressed=False):
    return {'sticks': {'left': {'x': 255, 'y': left_y}, 'right': {'x': right_x, 'y': 0}},
            'buttons': {'l1': pressed}}


def test_sticks_have_independent_translation_and_rotation_at_requested_limits():
    assert module.command_from_state(state(0, 128), enabled=True) == (2.0, 0.0, True)
    assert module.command_from_state(state(255, 128), enabled=True) == (-2.0, 0.0, True)
    assert module.command_from_state(state(128, 0), enabled=True) == (0.0, 2.0, True)
    assert module.command_from_state(state(128, 255), enabled=True) == (0.0, -2.0, True)
    assert module.command_from_state(state(0, 255), enabled=True) == (2.0, -2.0, True)
    assert module.command_from_state(state(), enabled=True) == (0.0, 0.0, True)


def test_deadzone_and_disabled_mode_stop_both_axes():
    assert module.command_from_state(state(116, 140), enabled=True) == (0.0, 0.0, True)
    assert module.command_from_state(state(0, 0), enabled=False) == (0.0, 0.0, False)


@pytest.mark.parametrize('value', [-1, 256, float('nan'), True, '0'])
def test_invalid_stick_values_cannot_enable_motion(value):
    assert module.command_from_state(state(value, 0), enabled=True) == (0.0, 0.0, False)


def test_malformed_state_does_not_reuse_an_old_command():
    for value in [None, {}, {'buttons': {'l1': True}}]:
        assert module.command_from_state(value, enabled=True) == (0.0, 0.0, False)


def buffer():
    result = module.InputBuffer(0.25, 2.0, 2.0, 0.12)
    result.connection(True)
    result.receive(state(), 1.0)
    return result


def test_l1_rising_edges_toggle_and_release_keeps_control_enabled():
    inputs = buffer()
    inputs.receive(state(0, 0, True), 1.01)
    assert inputs.snapshot(1.01) == (2.0, 2.0, True)
    inputs.receive(state(0, 0, True), 1.02)
    assert inputs.snapshot(1.02) == (2.0, 2.0, True)
    inputs.receive(state(0, 0, False), 1.03)
    assert inputs.snapshot(1.03) == (2.0, 2.0, True)
    inputs.receive(state(0, 0, True), 1.04)
    assert inputs.snapshot(1.04) == (0.0, 0.0, False)
    inputs.receive(state(0, 0, True), 1.05)
    assert inputs.snapshot(1.05) == (0.0, 0.0, False)


def test_timeout_and_reconnect_do_not_restore_enabled_mode():
    inputs = buffer()
    inputs.receive(state(0, 0, True), 1.01)
    assert inputs.snapshot(1.261) == (0.0, 0.0, False)
    inputs.receive(state(0, 0, True), 1.27)
    assert inputs.snapshot(1.27) == (0.0, 0.0, False)
    inputs.receive(state(0, 0, False), 1.28)
    inputs.receive(state(0, 0, True), 1.29)
    assert inputs.snapshot(1.29) == (2.0, 2.0, True)
    inputs.connection(False)
    assert inputs.snapshot(1.29) == (0.0, 0.0, False)
    inputs.connection(True)
    inputs.receive(state(0, 0, True), 1.30)
    assert inputs.snapshot(1.30) == (0.0, 0.0, False)


def test_input_gap_resets_the_latch_without_a_timer_tick():
    inputs = buffer()
    inputs.receive(state(0, 0, True), 1.01)
    inputs.receive(state(0, 0, False), 1.5)
    assert inputs.snapshot(1.5) == (0.0, 0.0, False)


def test_clock_or_graph_gate_requires_a_new_button_press():
    inputs = buffer()
    inputs.receive(state(0, 0, True), 1.01)
    inputs.gate(True)
    inputs.receive(state(0, 0, False), 1.02)
    inputs.receive(state(0, 0, True), 1.03)
    assert inputs.snapshot(1.03) == (0.0, 0.0, False)
    inputs.gate(False)
    inputs.receive(state(0, 0, True), 1.04)
    assert inputs.snapshot(1.04) == (0.0, 0.0, False)
    inputs.receive(state(0, 0, False), 1.05)
    inputs.receive(state(0, 0, True), 1.06)
    assert inputs.snapshot(1.06) == (2.0, 2.0, True)


def test_panel_and_l1_share_one_switch():
    inputs = buffer()
    inputs.receive(state(0, 255), 1.01)
    assert inputs.set_enabled(True, 1.02)[0]
    assert inputs.snapshot(1.02) == (2.0, -2.0, True)
    inputs.receive(state(0, 255, True), 1.03)
    assert inputs.snapshot(1.03) == (0.0, 0.0, False)
    assert inputs.set_enabled(True, 1.04)[0]
    assert inputs.set_enabled(False, 1.05)[0]
    inputs.receive(state(0, 255, True), 1.06)
    assert inputs.snapshot(1.06) == (0.0, 0.0, False)


def test_panel_cannot_enable_stale_or_blocked_input_and_can_always_disable():
    inputs = buffer()
    assert not inputs.set_enabled(True, 1.26)[0]
    inputs.receive(state(), 1.27)
    inputs.gate(True)
    assert not inputs.set_enabled(True, 1.28)[0]
    assert inputs.set_enabled(False, 1.28)[0]
    inputs.connection(False)
    assert not inputs.set_enabled(True, 1.29)[0]


def test_speed_change_immediately_rescales_cached_axes_and_zero_disables_one_axis():
    inputs = buffer()
    inputs.receive(state(0, 255, True), 1.01)
    inputs.set_limits(0.4, 0.7)
    assert inputs.snapshot(1.02) == (0.4, -0.7, True)
    inputs.set_limits(0.0, 2.0)
    assert inputs.snapshot(1.03) == (0.0, -2.0, True)
    inputs.disable()
    inputs.set_limits(1.0, 1.0)
    assert inputs.snapshot(1.04) == (0.0, 0.0, False)
