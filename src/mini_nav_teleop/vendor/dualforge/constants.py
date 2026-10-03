from enum import IntEnum, IntFlag

# ── 设备标识 ──────────────────────────────────────────────────
VENDOR_ID  = 0x054C
PRODUCT_ID = 0x0CE6

# ── Report ID ─────────────────────────────────────────────────
REPORT_ID_INPUT    = 0x01
REPORT_ID_OUTPUT   = 0x02
REPORT_ID_CALIB    = 0x05
REPORT_ID_MAC      = 0x09
REPORT_ID_FIRMWARE = 0x20

# ── 报告大小 ──────────────────────────────────────────────────
OUTPUT_REPORT_SIZE = 47
INPUT_REPORT_SIZE  = 63

# ── 枚举（D-02：改用 IntEnum，兼容现有整数比较）──────────────

class DPad(IntEnum):
    NORTH     = 0
    NORTHEAST = 1
    EAST      = 2
    SOUTHEAST = 3
    SOUTH     = 4
    SOUTHWEST = 5
    WEST      = 6
    NORTHWEST = 7
    NONE      = 8

class PowerState(IntEnum):
    DISCHARGING          = 0x00
    CHARGING             = 0x01
    COMPLETE             = 0x02
    ABNORMAL_VOLTAGE     = 0x0A
    ABNORMAL_TEMPERATURE = 0x0B
    CHARGING_ERROR       = 0x0F

class MuteLight(IntEnum):
    OFF       = 0
    ON        = 1
    BREATHING = 2

class LightBrightness(IntEnum):
    BRIGHT = 0
    MID    = 1
    DIM    = 2

class LightFadeAnimation(IntEnum):
    NOTHING  = 0
    FADE_IN  = 1
    FADE_OUT = 2

class TriggerEffectType(IntEnum):
    OFF       = 0x05
    FEEDBACK  = 0x21
    WEAPON    = 0x25
    VIBRATION = 0x26
    BOW       = 0x22
    GALLOPING = 0x23
    MACHINE   = 0x27

# ── 标志位（IntFlag 支持位运算）──────────────────────────────

class OutputFlag0(IntFlag):
    ENABLE_RUMBLE_EMULATION = 0x01
    USE_RUMBLE_NOT_HAPTICS  = 0x02
    ALLOW_RIGHT_TRIGGER_FFB = 0x04
    ALLOW_LEFT_TRIGGER_FFB  = 0x08
    ALLOW_HEADPHONE_VOLUME  = 0x10
    ALLOW_SPEAKER_VOLUME    = 0x20
    ALLOW_MIC_VOLUME        = 0x40
    ALLOW_AUDIO_CONTROL     = 0x80

class OutputFlag1(IntFlag):
    ALLOW_MUTE_LIGHT        = 0x01
    ALLOW_AUDIO_MUTE        = 0x02
    ALLOW_LED_COLOR         = 0x04
    RESET_LIGHTS            = 0x08
    ALLOW_PLAYER_INDICATORS = 0x10

class OutputFlag38(IntFlag):
    ALLOW_LIGHT_BRIGHTNESS     = 0x01
    ALLOW_LIGHT_FADE_ANIMATION = 0x02