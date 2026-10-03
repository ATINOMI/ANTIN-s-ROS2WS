import os
import time
import threading
if os.name == "nt":
    os.add_dll_directory(os.path.dirname(os.path.abspath(__file__)) + "\\..")

from . import trigger_effects
from . import feature
from .device import DualSenseDevice
from .input_parser import parse
from .constants import (
    DPad, PowerState, MuteLight,
    LightBrightness, LightFadeAnimation,
    TriggerEffectType
)

__all__ = ['DualForge', 'trigger_effects', 'feature',
           'DPad', 'PowerState', 'MuteLight',
           'LightBrightness', 'LightFadeAnimation',
           'TriggerEffectType']

_ZERO_REPORT = b'\x00' * 47  # 模块级常量，清零用


class DualForge:

    def __init__(self):
        self._device         = DualSenseDevice()
        self._running        = False
        self._send_lock      = threading.Lock()
        self._led_reset_sent = False
        self._report_buf     = bytearray(47)  # 预分配缓冲区（P-03）
        self._device.add_input_listener(self._on_raw_input)
        self._device.add_state_listener(self._on_state)
        self._input_callbacks = []
        self._state_callbacks = []

    # ── 连接 / 断开 ──────────────────────────────────────────

    def connect(self):
        self._device.connect()
        self._running = True
        time.sleep(0.5)
        self._send_reset_lights()

    def disconnect(self):
        self._running = False
        self._device.disconnect()

    def is_connected(self) -> bool:
        return self._device.is_connected()

    # ── 事件监听 ─────────────────────────────────────────────

    def on_input(self, fn):
        self._input_callbacks.append(fn)
        return fn

    def on_state(self, fn):
        self._state_callbacks.append(fn)
        return fn

    # ── 内部发送 ─────────────────────────────────────────────

    def _send_buf(self):
        """在锁内发送缓冲区（调用前必须已持有锁或在锁内）。"""
        self._device.send_report(self._report_buf)

    def _send_reset_lights(self):
        with self._send_lock:
            self._report_buf[:] = _ZERO_REPORT
            self._report_buf[1] |= 0x04  # AllowLedColor
            self._report_buf[1] |= 0x08  # ResetLights
            self._led_reset_sent = True
            self._send_buf()

    # ── LED ──────────────────────────────────────────────────

    def set_led(self, r: int, g: int, b: int):
        with self._send_lock:
            self._report_buf[:] = _ZERO_REPORT
            self._report_buf[1] |= 0x04
            if not self._led_reset_sent:
                self._report_buf[1] |= 0x08
                self._led_reset_sent = True
            self._report_buf[44] = max(0, min(255, r))
            self._report_buf[45] = max(0, min(255, g))
            self._report_buf[46] = max(0, min(255, b))
            self._send_buf()

    def set_player_lights(self, mask: int):
        with self._send_lock:
            self._report_buf[:] = _ZERO_REPORT
            self._report_buf[1] |= 0x10
            self._report_buf[43] = mask & 0x1F
            self._send_buf()

    def set_light_brightness(self, brightness: int):
        with self._send_lock:
            self._report_buf[:] = _ZERO_REPORT
            self._report_buf[38] |= 0x01
            self._report_buf[42] = brightness
            self._send_buf()

    def set_light_fade_animation(self, animation: int):
        with self._send_lock:
            self._report_buf[:] = _ZERO_REPORT
            self._report_buf[38] |= 0x02
            self._report_buf[41] = animation
            self._send_buf()

    # ── 震动 ─────────────────────────────────────────────────

    def set_rumble(self, left: int, right: int):
        with self._send_lock:
            self._report_buf[:] = _ZERO_REPORT
            self._report_buf[0] |= 0x01
            self._report_buf[0] |= 0x02
            self._report_buf[2] = max(0, min(255, right))
            self._report_buf[3] = max(0, min(255, left))
            self._send_buf()

    def stop_rumble(self):
        self.set_rumble(0, 0)

    # ── 扳机 FFB ─────────────────────────────────────────────

    def set_trigger_effect(self, target: str, effect_bytes: bytes):
        with self._send_lock:
            self._report_buf[:] = _ZERO_REPORT
            if target == 'right':
                self._report_buf[0] |= 0x04
                self._report_buf[10:21] = effect_bytes[:11]
            elif target == 'left':
                self._report_buf[0] |= 0x08
                self._report_buf[21:32] = effect_bytes[:11]
            self._send_buf()

    # ── 音频 ─────────────────────────────────────────────────

    def set_mute_light(self, mode: int):
        with self._send_lock:
            self._report_buf[:] = _ZERO_REPORT
            self._report_buf[1] |= 0x01
            self._report_buf[8] = mode
            self._send_buf()

    def set_mic_mute(self, muted: bool):
        with self._send_lock:
            self._report_buf[:] = _ZERO_REPORT
            self._report_buf[1] |= 0x02
            if muted:
                self._report_buf[9] |= 0x10
            self._send_buf()

    def set_headphone_volume(self, volume: int):
        with self._send_lock:
            self._report_buf[:] = _ZERO_REPORT
            self._report_buf[0] |= 0x10
            self._report_buf[4] = max(0, min(127, volume))
            self._send_buf()

    def set_speaker_volume(self, volume: int):
        with self._send_lock:
            self._report_buf[:] = _ZERO_REPORT
            self._report_buf[0] |= 0x20
            self._report_buf[5] = max(0, min(100, volume))
            self._send_buf()

    # ── 设备信息 ─────────────────────────────────────────────

    def read_calibration(self) -> dict:
        return feature.read_calibration(self._device)

    def read_mac(self) -> dict:
        return feature.read_mac(self._device)

    def read_firmware(self) -> dict:
        return feature.read_firmware(self._device)

    # ── 内部回调 ─────────────────────────────────────────────

    def _on_raw_input(self, data: bytes):
        state = parse(data)  # Q-02 修复：删掉重复 import
        for fn in self._input_callbacks:
            try:
                fn(state)
            except Exception as e:
                print(f"[DualForge] 输入回调异常: {e}")

    def _on_state(self, connected: bool):
        if not connected:
            self._running = False
        for fn in self._state_callbacks:
            try:
                fn(connected)
            except Exception as e:
                print(f"[DualForge] 状态回调异常: {e}")