import math
from .constants import DPad, PowerState

# ── 模块级工具函数（P-01 修复：消除每帧闭包开销）─────────────

def _u8(data: bytes, offset: int) -> int:
    return data[offset]

def _i16(data: bytes, offset: int) -> int:
    val = data[offset] | (data[offset + 1] << 8)
    return val - 0x10000 if val >= 0x8000 else val  # B-01 修复：return val

def _u32(data: bytes, offset: int) -> int:
    return (data[offset]
          | data[offset + 1] << 8
          | data[offset + 2] << 16
          | data[offset + 3] << 24)

def _parse_finger(data: bytes, offset: int) -> dict:
    raw = _u32(data, offset)
    return {
        'touching': not bool((raw >> 7) & 1),
        'x': (raw >> 8)  & 0xFFF,
        'y': (raw >> 20) & 0xFFF,
    }


def parse(data: bytes) -> dict:
    """
    把 63 字节的输入报告解析成结构化字典。
    data: 不含 Report ID 的 63 字节原始数据。
    """

    # ── 字节 7：方向键 + 四个按键 ─────────────────────────────
    byte7    = _u8(data, 7)
    dpad_raw = byte7 & 0x0F

    # ── 字节 8：肩键 ──────────────────────────────────────────
    byte8 = _u8(data, 8)

    # ── 字节 9：系统键 ────────────────────────────────────────
    byte9 = _u8(data, 9)

    # ── 字节 52：电源 ─────────────────────────────────────────
    byte52 = _u8(data, 52)

    # ── 字节 53：插入状态 ─────────────────────────────────────
    byte53 = _u8(data, 53)

    return {
        # 摇杆（0~255，128=居中）
        'sticks': {
            'left':  {'x': _u8(data, 0), 'y': _u8(data, 1)},
            'right': {'x': _u8(data, 2), 'y': _u8(data, 3)},
        },

        # 扳机（0~255）
        'triggers': {
            'left':  _u8(data, 4),
            'right': _u8(data, 5),
        },

        # 按键
        'buttons': {
            'dpad':     dpad_raw,
            'square':   bool((byte7 >> 4) & 1),
            'cross':    bool((byte7 >> 5) & 1),
            'circle':   bool((byte7 >> 6) & 1),
            'triangle': bool((byte7 >> 7) & 1),
            'l1':       bool((byte8 >> 0) & 1),
            'r1':       bool((byte8 >> 1) & 1),
            'l2':       bool((byte8 >> 2) & 1),
            'r2':       bool((byte8 >> 3) & 1),
            'create':   bool((byte8 >> 4) & 1),
            'options':  bool((byte8 >> 5) & 1),
            'l3':       bool((byte8 >> 6) & 1),
            'r3':       bool((byte8 >> 7) & 1),
            'home':     bool((byte9 >> 0) & 1),
            'pad':      bool((byte9 >> 1) & 1),
            'mute':     bool((byte9 >> 2) & 1),
        },

        # IMU 传感器（注意顺序是 X/Z/Y，不是 X/Y/Z）
        'gyro': {
            'x': _i16(data, 15),
            'z': _i16(data, 17),
            'y': _i16(data, 19),
        },
        'accelerometer': {
            'x': _i16(data, 21),
            'y': _i16(data, 23),
            'z': _i16(data, 25),
        },

        # 触摸板
        'touch': {
            'finger': [
                _parse_finger(data, 32),
                _parse_finger(data, 36),
            ]
        },

        # 扳机 FFB 状态反馈
        'trigger_feedback': {
            'right': {
                'stop_location': _u8(data, 41) & 0x0F,
                'status':        (_u8(data, 41) >> 4) & 0x0F,
                'effect':        _u8(data, 47) & 0x0F,
            },
            'left': {
                'stop_location': _u8(data, 42) & 0x0F,
                'status':        (_u8(data, 42) >> 4) & 0x0F,
                'effect':        (_u8(data, 47) >> 4) & 0x0F,
            },
        },

        # 电源
        'battery': {
            'percent': (byte52 & 0x0F) * 10,
            'state':   (byte52 >> 4) & 0x0F,
        },

        # 插入状态
        'plugged': {
            'headphones': bool((byte53 >> 0) & 1),
            'mic':        bool((byte53 >> 1) & 1),
            'mic_muted':  bool((byte53 >> 2) & 1),
            'usb_data':   bool((byte53 >> 3) & 1),
            'usb_power':  bool((byte53 >> 4) & 1),
        },
    }