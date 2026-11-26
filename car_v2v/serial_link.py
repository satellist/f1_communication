"""Jetson <-> ESP32-C3 的 USB 序列連線：自動找埠、斷線自動重連、背景執行緒收封包。

ROS 節點和不需要 ROS 的 link_test 都用這個模組。
"""

import glob
import os
import threading
import time

import serial

from . import protocol as P

ESP_BY_ID_GLOB = '/dev/serial/by-id/usb-Espressif*'


def find_esp32_ports():
    """回傳所有 Espressif USB 裝置的穩定路徑（by-id，不受 ttyACM 編號影響）。"""
    return sorted(glob.glob(ESP_BY_ID_GLOB))


def resolve_port(port: str):
    if port and port != 'auto':
        return port if os.path.exists(port) else None
    ports = find_esp32_ports()
    return ports[0] if ports else None


class SerialLink:
    """on_frame(type, payload) 和 on_state(connected: bool, port) 都在背景執行緒被呼叫。"""

    def __init__(self, port='auto', channel=1, group=7, tx_power_qdbm=60,
                 on_frame=None, on_state=None, log=print):
        self.port_req = port
        self.channel = channel
        self.group = group
        self.tx_power_qdbm = tx_power_qdbm
        self.on_frame = on_frame or (lambda t, p: None)
        self.on_state = on_state or (lambda c, port: None)
        self.log = log
        self.ser = None
        self.port = None
        self.connected = False
        self._wlock = threading.Lock()
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._run, daemon=True)
        self.parser = P.UsbFrameParser()

    def start(self):
        self._th.start()
        return self

    def stop(self):
        self._stop.set()
        try:
            self._th.join(timeout=0.5)
        except KeyboardInterrupt:   # launch 關閉時可能連送兩次 SIGINT
            pass
        self._close()

    # ---- 送 ----
    def send(self, ftype, payload=b''):
        if not self.connected:
            return False
        frame = P.encode_usb(ftype, payload)
        with self._wlock:
            try:
                self.ser.write(frame)
                return True
            except (serial.SerialException, OSError) as e:
                self.log(f'USB 寫入失敗：{e}')
                self._mark_lost()
                return False

    def send_radio(self, raw: bytes):
        if len(raw) > P.ESPNOW_MAX:
            raise ValueError(f'radio payload > {P.ESPNOW_MAX} bytes')
        return self.send(P.USB_TX, raw)

    def configure(self):
        self.send(P.USB_CFG, bytes([self.channel & 0xFF, self.group & 0xFF,
                                    self.tx_power_qdbm & 0xFF]))
        self.send(P.USB_GET_INFO)

    # ---- 內部 ----
    def _open(self, port):
        s = serial.Serial()
        s.port = port
        s.baudrate = 921600     # USB CDC 不看 baud，只是填個值
        s.timeout = 0.02
        # ESP32-C3 內建 USB-Serial/JTAG：DTR/RTS 保持預設（都拉高），不要動。
        # Linux 開埠時兩條線會先被拉高；如果程式再把它們拉低，中間會出現 RTS=1、DTR=0，
        # 這個組合會讓晶片重置。反覆重置會讓 USB 控制指令逾時、序列埠卡死。
        s.open()
        return s

    def _close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _mark_lost(self):
        if self.connected:
            self.connected = False
            self.on_state(False, self.port)
        self._close()

    def _run(self):
        warned = False
        while not self._stop.is_set():
            if not self.connected:
                port = resolve_port(self.port_req)
                if port is None:
                    if not warned:
                        self.log(f'找不到 ESP32（{self.port_req}），等待插入…')
                        warned = True
                    time.sleep(1.0)
                    continue
                try:
                    self.ser = self._open(port)
                except (serial.SerialException, OSError) as e:
                    if not warned:
                        self.log(f'開啟 {port} 失敗：{e}（是否被其他程式或燒錄佔用？）')
                        warned = True
                    time.sleep(1.0)
                    continue
                warned = False
                self.port = port
                self.parser = P.UsbFrameParser()
                self.connected = True
                self.on_state(True, port)
                self.configure()
            try:
                # 先等 1 byte（最多 timeout），有資料就把緩衝區剩下的一次讀完，不多等
                data = self.ser.read(1)
                if data and self.ser.in_waiting:
                    data += self.ser.read(self.ser.in_waiting)
            except (serial.SerialException, OSError, TypeError) as e:
                self.log(f'USB 讀取失敗：{e}，重新連線')
                self._mark_lost()
                time.sleep(0.5)
                continue
            if data:
                for ftype, payload in self.parser.feed(data):
                    try:
                        self.on_frame(ftype, payload)
                    except Exception as e:   # 不讓上層的錯誤弄掛讀取執行緒
                        self.log(f'處理封包時發生錯誤：{e!r}')
