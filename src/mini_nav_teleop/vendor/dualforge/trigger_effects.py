from .constants import TriggerEffectType


def _make_report() -> bytearray:
    """创建一个空的 11 字节效果缓冲区。"""
    return bytearray(11)


def off() -> bytes:
    """关闭扳机效果。"""
    r = _make_report()
    r[0] = TriggerEffectType.OFF
    return bytes(r)


def feedback(position: int, strength: int) -> bytes:
    """
    阻力反馈：从 position 区域到底部施加均匀阻力。
    position: 0~9
    strength: 1~8
    """
    if not (0 <= position <= 9):
        raise ValueError("position 必须在 0~9 之间")
    if not (1 <= strength <= 8):
        raise ValueError("strength 必须在 1~8 之间")

    r = _make_report()
    force_value  = (strength - 1) & 0x07
    active_zones = 0
    force_zones  = 0

    for i in range(position, 10):
        active_zones |= (1 << i)
        force_zones  |= (force_value << (3 * i))

    r[0] = TriggerEffectType.FEEDBACK
    r[1] = (active_zones >>  0) & 0xFF
    r[2] = (active_zones >>  8) & 0xFF
    r[3] = (force_zones  >>  0) & 0xFF
    r[4] = (force_zones  >>  8) & 0xFF
    r[5] = (force_zones  >> 16) & 0xFF
    r[6] = (force_zones  >> 24) & 0xFF
    return bytes(r)


def weapon(start: int, end: int, strength: int) -> bytes:
    """
    武器触感：在起止区域之间施加阻力，通过后突然释放。
    start:    2~7
    end:      start+1~8
    strength: 1~8
    """
    if not (2 <= start <= 7):
        raise ValueError("start 必须在 2~7 之间")
    if not (start < end <= 8):
        raise ValueError("end 必须在 start+1~8 之间")
    if not (1 <= strength <= 8):
        raise ValueError("strength 必须在 1~8 之间")

    r = _make_report()
    zones = (1 << start) | (1 << end)

    r[0] = TriggerEffectType.WEAPON
    r[1] = (zones >> 0) & 0xFF
    r[2] = (zones >> 8) & 0xFF
    r[3] = (strength - 1) & 0x07
    return bytes(r)


def vibration(position: int, amplitude: int, frequency: int) -> bytes:
    """
    触发器振动：从 position 区域开始振动。
    position:  0~9
    amplitude: 1~8
    frequency: 1~255 Hz
    """
    if not (0 <= position <= 9):
        raise ValueError("position 必须在 0~9 之间")
    if not (1 <= amplitude <= 8):
        raise ValueError("amplitude 必须在 1~8 之间")
    if not (1 <= frequency <= 255):
        raise ValueError("frequency 必须在 1~255 之间")

    r = _make_report()
    strength_value = (amplitude - 1) & 0x07
    active_zones   = 0
    amp_zones      = 0

    for i in range(position, 10):
        active_zones |= (1 << i)
        amp_zones    |= (strength_value << (3 * i))

    r[0] = TriggerEffectType.VIBRATION
    r[1] = (active_zones >>  0) & 0xFF
    r[2] = (active_zones >>  8) & 0xFF
    r[3] = (amp_zones    >>  0) & 0xFF
    r[4] = (amp_zones    >>  8) & 0xFF
    r[5] = (amp_zones    >> 16) & 0xFF
    r[6] = (amp_zones    >> 24) & 0xFF
    r[9] = frequency
    return bytes(r)


def bow(start: int, end: int, strength: int, snap_force: int) -> bytes:
    """
    弓弦张力：类似 weapon，但松开后有回弹力。
    start:      0~8
    end:        start+1~8
    strength:   1~8
    snap_force: 1~8
    """
    if not (0 <= start <= 8):
        raise ValueError("start 必须在 0~8 之间")
    if not (start < end <= 8):
        raise ValueError("end 必须在 start+1~8 之间")
    if not (1 <= strength <= 8):
        raise ValueError("strength 必须在 1~8 之间")
    if not (1 <= snap_force <= 8):
        raise ValueError("snap_force 必须在 1~8 之间")

    r = _make_report()
    zones      = (1 << start) | (1 << end)
    force_pair = (((strength   - 1) & 0x07) << 0) \
               | (((snap_force - 1) & 0x07) << 3)

    r[0] = TriggerEffectType.BOW
    r[1] = (zones      >> 0) & 0xFF
    r[2] = (zones      >> 8) & 0xFF
    r[3] = (force_pair >> 0) & 0xFF
    r[4] = (force_pair >> 8) & 0xFF
    return bytes(r)


def machine(start: int, end: int,
            amp_a: int, amp_b: int,
            frequency: int, period: int) -> bytes:
    """
    机枪振动：双振幅交替振动。
    start:     0~8
    end:       start+1~9
    amp_a:     0~7  主振幅
    amp_b:     0~7  副振幅
    frequency: 1~255 Hz
    period:    0~255（单位：1/10秒）
    """
    if not (1 <= start <= 8):
        raise ValueError("start 必须在 0~8 之间")
    if not (start < end <= 9):
        raise ValueError("end 必须在 start+1~9 之间")
    if not (0 <= amp_a <= 7):
        raise ValueError("amp_a 必须在 0~7 之间")
    if not (0 <= amp_b <= 7):
        raise ValueError("amp_b 必须在 0~7 之间")
    if not (1 <= frequency <= 255):
        raise ValueError("frequency 必须在 1~255 之间")

    r = _make_report()
    zones         = (1 << start) | (1 << end)
    strength_pair = ((amp_a & 0x07) << 0) | ((amp_b & 0x07) << 3)

    r[0] = TriggerEffectType.MACHINE
    r[1] = (zones         >> 0) & 0xFF
    r[2] = (zones         >> 8) & 0xFF
    r[3] = (strength_pair >> 0) & 0xFF
    r[4] = frequency
    r[5] = period
    return bytes(r)

def galloping(start: int, end: int,
              first_foot: int, second_foot: int,
              frequency: int) -> bytes:
    """
    马蹄振动：模拟骑马时的节奏感。
    start:       0~8
    end:         start+1~9
    first_foot:  0~6  第一蹄落点
    second_foot: first_foot+1~7  第二蹄落点
    frequency:   1~255 Hz（建议不超过 40）
    """
    if not (0 <= start <= 8):
        raise ValueError("start 必须在 0~8 之间")
    if not (start < end <= 9):
        raise ValueError("end 必须在 start+1~9 之间")
    if not (0 <= first_foot <= 6):
        raise ValueError("first_foot 必须在 0~6 之间")
    if not (first_foot < second_foot <= 7):
        raise ValueError("second_foot 必须在 first_foot+1~7 之间")
    if not (1 <= frequency <= 255):
        raise ValueError("frequency 必须在 1~255 之间")

    r = _make_report()
    zones        = (1 << start) | (1 << end)
    time_and_ratio = ((second_foot & 0x07) << 0) \
                   | ((first_foot  & 0x07) << 3)

    r[0] = TriggerEffectType.GALLOPING
    r[1] = (zones         >> 0) & 0xFF
    r[2] = (zones         >> 8) & 0xFF
    r[3] = (time_and_ratio >> 0) & 0xFF
    r[4] = frequency
    return bytes(r)

# ══════════════════════════════════════════════════════════════
# 预设效果（基于上面的底层函数封装固定参数）
# ══════════════════════════════════════════════════════════════

# ── 阻力预设系列（基于 feedback）────────────────────────────

def normal() -> bytes:
    """无效果，扳机自由行程。"""
    return off()

def very_soft() -> bytes:
    """非常轻的阻力。"""
    return feedback(0, 1)

def soft() -> bytes:
    """轻阻力。"""
    return feedback(0, 2)

def medium() -> bytes:
    """中等阻力。"""
    return feedback(0, 4)

def hard() -> bytes:
    """较强阻力。"""
    return feedback(0, 6)

def very_hard() -> bytes:
    """很强阻力。"""
    return feedback(0, 7)

def hardest() -> bytes:
    """最强阻力。"""
    return feedback(0, 8)

def rigid() -> bytes:
    """
    完全锁死，扳机几乎无法按下。
    与 hardest 相同强度，但从区域0开始，几乎无法移动。
    """
    return feedback(0, 8)

# ── 武器预设（基于 weapon）───────────────────────────────────

def semi_automatic_gun(start: int = 2,
                       end: int = 7,
                       strength: int = 8) -> bytes:
    """
    半自动枪触感：weapon() 的别名，参数有默认值。
    start:    2~7
    end:      start+1~8
    strength: 1~8
    """
    return weapon(start, end, strength)

def game_cube() -> bytes:
    """
    GameCube 手柄扳机手感：两段式，前段自由，后段突然有阻力。
    模拟 GameCube L/R 扳机的经典手感。
    """
    return weapon(start=4, end=7, strength=8)

def choppy() -> bytes:
    """
    断断续续阻力：扳机按下时感受到间隔分布的阻力块。
    模拟棘轮、齿轮、上膛等机械感。
    """
    r = _make_report()
    r[0] = TriggerEffectType.FEEDBACK
    r[1] = 0x02  # 区域使能低字节
    r[2] = 0x27  # 区域使能高字节（区域1、5、6、9有效）
    r[3] = 0x18  # 力度低字节
    r[4] = 0x00
    r[5] = 0x00
    r[6] = 0x26  # 力度高字节
    return bytes(r)

# ── 振动预设（基于 vibration）────────────────────────────────

def automatic_gun(position: int = 0,
                  amplitude: int = 8,
                  frequency: int = 10) -> bytes:
    """
    自动枪触感：vibration() 的别名，参数有默认值。
    position:  0~9
    amplitude: 1~8
    frequency: 1~255 Hz
    """
    return vibration(position, amplitude, frequency)

def vibrate_trigger_pulse() -> bytes:
    """
    脉冲振动：短促的周期性冲击感，模拟枪械点射。
    """
    return vibration(position=0, amplitude=8, frequency=5)

def vibrate_trigger(intensity: int = 10) -> bytes:
    """
    持续振动：扳机按下后持续振动。
    intensity: 1~255，振动频率
    """
    if not (1 <= intensity <= 255):
        raise ValueError("intensity 必须在 1~255 之间")
    return vibration(position=0, amplitude=8, frequency=intensity)

def vibrate_trigger_custom(position: int,
                           amplitude: int,
                           frequency: int) -> bytes:
    """
    自定义振动：完全自定义所有参数。
    position:  0~9
    amplitude: 1~8
    frequency: 1~255 Hz
    """
    return vibration(position, amplitude, frequency)

# ── 完全自定义（直接操作原始字节）───────────────────────────

def custom(effect_type: int, params: list) -> bytes:
    """
    完全自定义效果：直接指定效果类型和7个参数字节。
    effect_type: TriggerEffectType 枚举值
    params:      长度为 10 的字节列表（字节1~10）

    用法：
        from dualforge.constants import TriggerEffectType
        trigger_effects.custom(
            TriggerEffectType.FEEDBACK,
            [0xFF, 0x03, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        )
    """
    if len(params) != 10:
        raise ValueError("params 必须是长度为 10 的列表")
    r = _make_report()
    r[0] = effect_type
    for i, v in enumerate(params):
        r[i + 1] = v & 0xFF
    return bytes(r)