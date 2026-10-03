import math
from .constants import REPORT_ID_CALIB, REPORT_ID_MAC, REPORT_ID_FIRMWARE

# 模块级常量，避免重复计算
_DEG2RAD = math.pi / 180.0


def read_calibration(device) -> dict:
    """
    读取陀螺仪/加速度计校准数据（Feature Report 0x05）。
    返回原始校准值和计算好的换算系数。
    """
    data = device.get_feature_report(REPORT_ID_CALIB, 40)

    def i16(offset):
        val = data[offset] | (data[offset + 1] << 8)
        if val >= 0x8000:
            val -= 0x10000
        return val

    gyro_pitch_bias  = i16(1)
    gyro_yaw_bias    = i16(3)
    gyro_roll_bias   = i16(5)
    gyro_pitch_plus  = i16(7)
    gyro_pitch_minus = i16(9)
    gyro_yaw_plus    = i16(11)
    gyro_yaw_minus   = i16(13)
    gyro_roll_plus   = i16(15)
    gyro_roll_minus  = i16(17)
    gyro_speed_plus  = i16(19)
    gyro_speed_minus = i16(21)
    accel_x_plus     = i16(23)
    accel_x_minus    = i16(25)
    accel_y_plus     = i16(27)
    accel_y_minus    = i16(29)
    accel_z_plus     = i16(31)
    accel_z_minus    = i16(33)

    speed_2x = gyro_speed_plus + abs(gyro_speed_minus)

    def gyro_scale(plus, minus):
        denom = plus - minus
        if denom == 0:
            return 1.0
        return speed_2x * _DEG2RAD / denom

    def accel_scale(plus, minus):
        denom = plus - minus
        if denom == 0:
            return 1.0
        return 2.0 / denom

    def accel_bias(plus, minus):
        return plus - (plus - minus) / 2

    return {
        'raw': {
            'gyro_pitch_bias':  gyro_pitch_bias,
            'gyro_yaw_bias':    gyro_yaw_bias,
            'gyro_roll_bias':   gyro_roll_bias,
            'gyro_pitch_plus':  gyro_pitch_plus,
            'gyro_pitch_minus': gyro_pitch_minus,
            'gyro_yaw_plus':    gyro_yaw_plus,
            'gyro_yaw_minus':   gyro_yaw_minus,
            'gyro_roll_plus':   gyro_roll_plus,
            'gyro_roll_minus':  gyro_roll_minus,
            'gyro_speed_plus':  gyro_speed_plus,
            'gyro_speed_minus': gyro_speed_minus,
            'accel_x_plus':     accel_x_plus,
            'accel_x_minus':    accel_x_minus,
            'accel_y_plus':     accel_y_plus,
            'accel_y_minus':    accel_y_minus,
            'accel_z_plus':     accel_z_plus,
            'accel_z_minus':    accel_z_minus,
        },
        'computed': {
            'gyro_scale_x':  gyro_scale(gyro_pitch_plus, gyro_pitch_minus),
            'gyro_scale_y':  gyro_scale(gyro_roll_plus,  gyro_roll_minus),
            'gyro_scale_z':  gyro_scale(gyro_yaw_plus,   gyro_yaw_minus),
            'gyro_bias_x':   gyro_pitch_bias,
            'gyro_bias_y':   gyro_roll_bias,
            'gyro_bias_z':   gyro_yaw_bias,
            'accel_scale_x': accel_scale(accel_x_plus, accel_x_minus),
            'accel_scale_y': accel_scale(accel_y_plus, accel_y_minus),
            'accel_scale_z': accel_scale(accel_z_plus, accel_z_minus),
            'accel_bias_x':  accel_bias(accel_x_plus, accel_x_minus),
            'accel_bias_y':  accel_bias(accel_y_plus, accel_y_minus),
            'accel_bias_z':  accel_bias(accel_z_plus, accel_z_minus),
        }
    }


def read_mac(device) -> dict:
    """读取手柄和主机 MAC 地址（Feature Report 0x09）。"""
    data = device.get_feature_report(REPORT_ID_MAC, 19)

    def parse_mac(start):
        b = [data[start + i] for i in range(6)]
        b.reverse()
        return ':'.join(f'{x:02X}' for x in b)

    return {
        'controller_mac': parse_mac(1),
        'host_mac':       parse_mac(10),
    }


def read_firmware(device) -> dict:
    """读取固件版本和硬件信息（Feature Report 0x20）。"""
    data = device.get_feature_report(REPORT_ID_FIRMWARE, 63)

    def u16(offset):
        return data[offset] | (data[offset + 1] << 8)

    def u32(offset):
        return (data[offset]
              | data[offset + 1] << 8
              | data[offset + 2] << 16
              | data[offset + 3] << 24)

    def read_string(offset, length):
        raw = data[offset:offset + length]
        return bytes(raw).decode('ascii', errors='ignore').rstrip('\x00')

    hardware_info    = u32(24)
    firmware_version = u32(28)
    hw_generation    = (hardware_info >> 8) & 0xFF
    fw_major         = (firmware_version >> 24) & 0xFF
    fw_minor         = (firmware_version >> 16) & 0xFF
    fw_patch         =  firmware_version & 0xFFFF

    return {
        'build_date':         read_string(1,  11),
        'build_time':         read_string(12, 8),
        'fw_type':            u16(20),
        'sw_series':          u16(22),
        'hardware_info':      hardware_info,
        'hw_variation':       (hardware_info >> 16) & 0xFF,
        'hw_generation':      hw_generation,
        'hw_generation_name': f'Gen{hw_generation}',
        'firmware_version':   f'{fw_major}.{fw_minor}.{fw_patch}',
    }