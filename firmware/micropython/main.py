# ESP32-C3 雙車通訊韌體（MicroPython 版）—— 兩台車燒同一份，不用改設定
#
# 功能和 USB 封包格式與 C 版 firmware/v2v_esp32/v2v_esp32.ino 完全相同，
# Jetson 端（car_v2v）不需要知道 ESP32 跑的是哪一版。
#   - USB 收到 TX -> ESP-NOW 送出（認識對方前用廣播，之後 unicast，有硬體 ACK）
#   - ESP-NOW 收到同 group 的封包 -> 加上來源 MAC、RSSI，經 USB 交給 Jetson
#   - 每秒回報 STATS，每 2 秒回報 INFO
# 維護：Jetson 送 USB 封包 type 0x1F 會離開程式回到 REPL（flash_mpy.sh 更新程式時用）

import sys
import select
import struct
import time

import espnow
import micropython
import network

FW_VER = 1
PEER_STALE_MS = 1500
RADIO_MAX = 248

USB_CFG, USB_TX, USB_GET_INFO, USB_EXIT = 0x10, 0x11, 0x12, 0x1F
USB_INFO, USB_RX, USB_STATS, USB_LOG = 0x20, 0x21, 0x22, 0x23
BCAST = b'\xff' * 6

# CRC-16/CCITT-FALSE 查表
_T = []
for _i in range(256):
    _c = _i << 8
    for _ in range(8):
        _c = ((_c << 1) ^ 0x1021) if _c & 0x8000 else (_c << 1)
    _T.append(_c & 0xFFFF)
CRC_T = bytes(struct.pack('<256H', *_T))
del _T


@micropython.viper
def crc16(buf, n: int) -> int:
    t = ptr16(CRC_T)
    b = ptr8(buf)
    crc = 0xFFFF
    for i in range(n):
        crc = ((crc << 8) & 0xFFFF) ^ t[((crc >> 8) ^ b[i]) & 0xFF]
    return crc


class V2V:
    def __init__(self):
        self.out = sys.stdout.buffer
        self.inp = sys.stdin.buffer
        self.channel, self.group, self.txp = 1, 7, 60
        self.sta = network.WLAN(network.STA_IF)
        self.sta.active(True)
        try:
            self.sta.disconnect()
        except OSError:
            pass
        try:
            self.sta.config(pm=self.sta.PM_NONE)   # 關省電，降低延遲
        except Exception:
            pass
        self.mac = self.sta.config('mac')
        self.e = espnow.ESPNow()
        self.e.active(True)
        self.e.add_peer(BCAST)
        self.peer = None
        self.peer_last = 0
        self.apply_radio()
        self.rxbuf = bytearray(4 + 255 + 2)
        self.rxn = 0
        self.fbuf = bytearray(4 + 255 + 2)

    def apply_radio(self):
        self.sta.config(channel=self.channel)
        try:
            self.sta.config(txpower=self.txp / 4)
        except Exception:
            pass

    # ---- USB 送 ----
    def usb_send(self, t, payload):
        n = len(payload)
        f = self.fbuf
        f[0] = 0xAA
        f[1] = 0x55
        f[2] = t
        f[3] = n
        f[4:4 + n] = payload
        c = crc16(memoryview(f)[2:], 2 + n)
        f[4 + n] = c & 0xFF
        f[5 + n] = c >> 8
        try:
            self.out.write(memoryview(f)[:6 + n])
        except OSError:
            pass

    def send_info(self):
        pm = self.peer if self.peer else bytes(6)
        self.usb_send(USB_INFO, struct.pack('<6sBBBB6sB', self.mac, self.channel, self.group,
                                            FW_VER, 1 if self.peer else 0, pm, self.txp))

    def send_stats(self):
        # stats() = (tx_pkts, tx_responses, tx_failures, rx_packets, rx_dropped)
        s = self.e.stats()
        self.usb_send(USB_STATS, struct.pack('<IIII', s[1], s[2], s[3], s[4]))

    def log(self, msg):
        self.usb_send(USB_LOG, msg.encode())

    # ---- 對方 ----
    def set_peer(self, mac):
        if self.peer == mac:
            return
        if self.peer:
            try:
                self.e.del_peer(self.peer)
            except OSError:
                pass
        try:
            self.e.add_peer(mac)
        except OSError:
            pass
        self.peer = bytes(mac)
        self.log('peer ' + ':'.join('%02X' % b for b in mac))
        self.send_info()

    def radio_send(self, data):
        unicast = self.peer and time.ticks_diff(time.ticks_ms(), self.peer_last) < PEER_STALE_MS
        try:
            self.e.send(self.peer if unicast else BCAST, data, False)
        except OSError:
            pass

    # ---- USB 收 ----
    def handle(self, t, p):
        if t == USB_TX:
            if 3 <= len(p) <= RADIO_MAX:
                self.radio_send(p)
        elif t == USB_CFG:
            if len(p) >= 3:
                if 1 <= p[0] <= 13:
                    self.channel = p[0]
                self.group = p[1]
                if 8 <= p[2] <= 84:
                    self.txp = p[2]
                self.apply_radio()
                if self.peer:
                    try:
                        self.e.del_peer(self.peer)
                    except OSError:
                        pass
                    self.peer = None
            self.send_info()
        elif t == USB_GET_INFO:
            self.send_info()
        elif t == USB_EXIT:
            raise SystemExit

    def feed(self, b):
        buf = self.rxbuf
        n = self.rxn
        if n == 0:
            if b == 0xAA:
                buf[0] = b
                self.rxn = 1
            return
        if n == 1:
            if b == 0x55:
                buf[1] = b
                self.rxn = 2
            else:
                self.rxn = 1 if b == 0xAA else 0
            return
        buf[n] = b
        n += 1
        self.rxn = n
        if n >= 4 and n == 6 + buf[3]:
            ln = buf[3]
            c = crc16(memoryview(buf)[2:], 2 + ln)
            self.rxn = 0
            if (c & 0xFF) == buf[4 + ln] and (c >> 8) == buf[5 + ln]:
                self.handle(buf[2], bytes(buf[4:4 + ln]))

    def on_radio(self, mac, msg):
        if len(msg) < 3 or len(msg) > RADIO_MAX:
            return
        if msg[0] != 0x56 or msg[1] != 0x32 or msg[2] != self.group:   # 'V' '2' group
            return
        self.set_peer(mac)
        self.peer_last = time.ticks_ms()
        rssi = 0
        try:
            rssi = self.e.peers_table[mac][0]
        except (KeyError, AttributeError):
            pass
        self.usb_send(USB_RX, bytes(mac) + struct.pack('<b', rssi) + msg)

    def run(self):
        micropython.kbd_intr(-1)    # 二進位資料裡的 0x03 不能被當成 Ctrl-C
        poll = select.poll()
        poll.register(sys.stdin, select.POLLIN)
        poll.register(self.e, select.POLLIN)
        spoll = select.poll()
        spoll.register(sys.stdin, select.POLLIN)
        self.send_info()
        t_stats = t_info = time.ticks_ms()
        try:
            while True:
                for obj, ev in poll.ipoll(20):
                    if obj is self.e:
                        while self.e.any():
                            mac, msg = self.e.irecv(0)
                            if msg is None:
                                break
                            self.on_radio(mac, msg)
                    else:
                        # 有資料就一直讀到 stdin 沒有東西為止
                        while spoll.poll(0):
                            r = self.inp.read(1)
                            if not r:
                                break
                            self.feed(r[0])
                now = time.ticks_ms()
                if time.ticks_diff(now, t_stats) >= 1000:
                    t_stats = now
                    self.send_stats()
                if time.ticks_diff(now, t_info) >= 2000:
                    t_info = now
                    self.send_info()
        finally:
            micropython.kbd_intr(3)


V2V().run()
