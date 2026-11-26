"""雙車通訊協定（Jetson 端）。韌體 firmware/v2v_esp32/v2v_esp32.ino 用的是同一份定義。

兩層封包：

1. USB 封包（Jetson <-> 自己車上的 ESP32-C3）
       0xAA 0x55 | type(1) | len(1) | payload(len) | crc16(2, little-endian)
   crc16 = CRC-16/CCITT-FALSE，計算範圍是 type + len + payload。

2. 無線封包（ESP-NOW，車 <-> 車），放在 USB 封包 TX / RX 的 payload 裡
       header（12 bytes）：magic 'V2' | group | proto_ver | msg_type | car_id | seq(u16) | t_ms(u32)
       body：依 msg_type 而定
   ESP32 韌體只檢查前 3 bytes（magic + group），不同 group 的封包直接丟掉，
   其他欄位只有 Jetson 會解讀。
"""

import struct
from dataclasses import dataclass, field

# ---------------- USB 層 ----------------
SYNC = b'\xAA\x55'
MAX_USB_PAYLOAD = 255

# Jetson -> ESP32
USB_CFG = 0x10        # channel(u8) group(u8) tx_power_qdbm(u8)
USB_TX = 0x11         # 無線封包原始 bytes（<= 250）
USB_GET_INFO = 0x12   # 空
USB_EXIT = 0x1F       # 只有 MicroPython 版：離開程式回到 REPL（更新韌體用）
# ESP32 -> Jetson
USB_INFO = 0x20       # mac(6) channel group fw_ver peer_known peer_mac(6) tx_power_qdbm
USB_RX = 0x21         # src_mac(6) rssi(i8) 無線封包原始 bytes
USB_STATS = 0x22      # tx_ok tx_fail rx rx_drop（u32 x4，累計值）
USB_LOG = 0x23        # 文字訊息

ESPNOW_MAX = 248   # USB RX 封包要多放 MAC+RSSI 7 bytes，len 欄位上限 255


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode_usb(ftype: int, payload: bytes = b'') -> bytes:
    if len(payload) > MAX_USB_PAYLOAD:
        raise ValueError('payload too long')
    body = bytes([ftype, len(payload)]) + payload
    return SYNC + body + struct.pack('<H', crc16_ccitt(body))


class UsbFrameParser:
    """把序列埠讀到的 bytes 切成 (type, payload)。遇到雜訊或 CRC 錯誤會自動重新找 0xAA 0x55 同步。"""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes):
        self.buf += data
        frames = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                # 保留最後一個 byte（可能是 0xAA，下一包才接到 0x55）
                del self.buf[:max(0, len(self.buf) - 1)]
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < 4:
                break
            n = self.buf[3]
            total = 2 + 2 + n + 2
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:4 + n])
            (crc,) = struct.unpack_from('<H', self.buf, 4 + n)
            if crc == crc16_ccitt(body):
                frames.append((body[0], body[2:]))
                del self.buf[:total]
            else:
                self.crc_errors += 1
                del self.buf[:2]   # 跳過這組同步字，繼續往後找
        return frames


def mac_str(mac: bytes) -> str:
    return ':'.join(f'{b:02X}' for b in mac)


@dataclass
class EspInfo:
    mac: bytes
    channel: int
    group: int
    fw_ver: int
    peer_known: bool
    peer_mac: bytes
    tx_power_qdbm: int

    @classmethod
    def unpack(cls, p: bytes):
        mac, ch, grp, ver, pk, peer, txp = struct.unpack('<6sBBBB6sB', p[:17])
        return cls(mac, ch, grp, ver, bool(pk), peer, txp)


@dataclass
class EspStats:
    tx_ok: int
    tx_fail: int
    rx: int
    rx_drop: int

    @classmethod
    def unpack(cls, p: bytes):
        return cls(*struct.unpack('<IIII', p[:16]))


# ---------------- 無線層 ----------------
MAGIC = b'V2'
PROTO_VER = 1

MSG_STATE = 1
MSG_PING = 2
MSG_PONG = 3

_HDR = struct.Struct('<2sBBBBHI')          # 12 bytes
_STATE = struct.Struct('<ffffffHBB')        # 28 bytes
_PONG = struct.Struct('<II')                # nonce, 對方 ping 的 t_ms

# STATE.flags
FLAG_POSE_VALID = 0x01    # x, y, yaw 有效（TF map->base_link 拿得到）
FLAG_TRACK_VALID = 0x02   # s, d, lap 有效（有載入中心線）
FLAG_SPEED_VALID = 0x04   # v 有效（有收到 /odom）

# STATE.state：超車協商用的狀態碼，由上層邏輯透過 /v2v/ego_state 設定
STATE_NAMES = {
    0: 'NORMAL',
    1: 'REQUEST_OVERTAKE',
    2: 'YIELDING',
    3: 'OVERTAKING',
    4: 'DONE',
    255: 'STOP',
}


@dataclass
class Header:
    group: int
    msg_type: int
    car_id: int
    seq: int
    t_ms: int
    ver: int = PROTO_VER


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    s: float = 0.0
    d: float = 0.0
    lap: int = 0
    state: int = 0
    flags: int = 0


@dataclass
class Ping:
    nonce: int = 0


@dataclass
class Pong:
    nonce: int = 0
    echo_t_ms: int = 0


@dataclass
class Packet:
    header: Header
    body: object = field(default=None)


def encode_radio(h: Header, body) -> bytes:
    hdr = _HDR.pack(MAGIC, h.group & 0xFF, h.ver, h.msg_type, h.car_id & 0xFF,
                    h.seq & 0xFFFF, h.t_ms & 0xFFFFFFFF)
    if h.msg_type == MSG_STATE:
        b = body
        return hdr + _STATE.pack(b.x, b.y, b.yaw, b.v, b.s, b.d,
                                 b.lap & 0xFFFF, b.state & 0xFF, b.flags & 0xFF)
    if h.msg_type == MSG_PING:
        return hdr + struct.pack('<I', body.nonce & 0xFFFFFFFF)
    if h.msg_type == MSG_PONG:
        return hdr + _PONG.pack(body.nonce & 0xFFFFFFFF, body.echo_t_ms & 0xFFFFFFFF)
    raise ValueError(f'unknown msg_type {h.msg_type}')


def decode_radio(raw: bytes):
    """解不出來（格式不符、版本不同）回傳 None。"""
    if len(raw) < _HDR.size:
        return None
    magic, grp, ver, mtype, cid, seq, t_ms = _HDR.unpack_from(raw)
    if magic != MAGIC or ver != PROTO_VER:
        return None
    h = Header(grp, mtype, cid, seq, t_ms, ver)
    p = raw[_HDR.size:]
    try:
        if mtype == MSG_STATE:
            return Packet(h, State(*_STATE.unpack_from(p)))
        if mtype == MSG_PING:
            return Packet(h, Ping(*struct.unpack_from('<I', p)))
        if mtype == MSG_PONG:
            return Packet(h, Pong(*_PONG.unpack_from(p)))
    except struct.error:
        return None
    return Packet(h, None)


def seq_gap(prev: int, cur: int) -> int:
    """兩個 u16 序號之間漏掉幾包（處理 65535 -> 0 迴繞）。"""
    return ((cur - prev) & 0xFFFF) - 1
