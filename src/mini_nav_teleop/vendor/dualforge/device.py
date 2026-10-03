import os
import threading
from .constants import VENDOR_ID, PRODUCT_ID, INPUT_REPORT_SIZE

#确保 hidapi.all 能被找到
if os.name == "nt":
    os.add_dll_directory(os.path.dirname(os.path.abspath(__file__)) + "\\..")

import hid

class DualSenseDevice:
    """
    底层 HID 设备管理。
    负责连接、断开、原始数据读写。
    不包含任何业务逻辑。
    """

    def __init__(self):
        self._device  = None
        self._running = False
        self._thread  = None
        self._input_listeners = []
        self._state_listeners = []

    # ── 连接 / 断开 ──────────────────────────────────────────

    def connect(self):
        """连接 DualSense，找不到设备则抛出异常。"""
        devices = hid.enumerate(VENDOR_ID, PRODUCT_ID)
        if not devices:
            raise ConnectionError("Controller No Found,Check The USB Connection")
        
        # 找 usage_page=1, usage=5 的接口（主 HID 接口）
        target = None
        for d in devices:
            if d.get("bus_type", hid.BusType.UNKNOWN) not in (hid.BusType.USB, hid.BusType.UNKNOWN):
                continue
            if d['usage_page'] == 1 and d['usage'] == 5:
                target = d
                break

        if target is None:
            raise ConnectionError("Device found but can't locate main HID interface")
        
        self._device  = hid.Device(path=target['path'])
        self._running = True

        # 启动后台读取线程
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
    
        self._notify_state(True)

    def disconnect(self):
        """断开连接"""
        self._running = False
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join()
        if self._device:
            self._device.close()
            self._device = None
        self._notify_state(False)

    def is_connected(self):
        return self._device is not None

    # ── 读写 ─────────────────────────────────────────────────

    def _read_loop(self):
        while self._running:
            try:
                data = self._device.read(INPUT_REPORT_SIZE + 1, timeout=100)
                if data and len(data) == INPUT_REPORT_SIZE + 1 and data[0] == 0x01:
                    payload = bytes(data[1:])
                    self._notify_input(payload)
            except Exception as e:
                if self._running:  # ← 只有不是主动断开才报错
                    print(f"[DualForge] 读取线程异常: {e}")
                    self._running = False
                    self._device = None
                    self._notify_state(False)
                break

    def send_report(self, data: bytes):
        if not self._device:
            raise ConnectionError("Device not connected")
        payload = bytes([0x02]) + data
        self._device.write(payload)

    def get_feature_report(self, report_id: int, size: int) -> bytes:
        """读取 Feature Report，返回数据（含 Report ID 在字节0）。"""
        if not self._device:
            raise ConnectionError("Device not connected")
        return self._device.get_feature_report(report_id, size + 1)
    
    # ── 监听器 ───────────────────────────────────────────────

    def add_input_listener(self, fn):
        """注册输入监听器，fn(data: bytes) 每次收到数据时调用。"""
        self._input_listeners.append(fn)

    def add_state_listener(self, fn):
        """注册连接状态监听器，fn(connected: bool)。"""
        self._state_listeners.append(fn)

    def _notify_input(self, data: bytes):
        for fn in self._input_listeners:
            try:
                fn(data)
            except Exception as e:
                print(f"[DualForge] 输入监听器异常: {e}")

    def _notify_state(self, connected: bool):
        for fn in self._state_listeners:
            try:
                fn(connected)
            except Exception as e:
                print(f"[DualForge] 状态监听器异常: {e}")

        