import math

import numpy as np

from car_v2v import protocol as P
from car_v2v.track import Track


def test_crc_known_value():
    assert P.crc16_ccitt(b'123456789') == 0x29B1   # CRC-16/CCITT-FALSE 標準檢查值


def test_usb_roundtrip_with_noise_and_split():
    f1 = P.encode_usb(P.USB_TX, b'hello')
    f2 = P.encode_usb(P.USB_INFO, bytes(17))
    bad = bytearray(P.encode_usb(P.USB_TX, b'xx'))
    bad[-1] ^= 0xFF
    stream = b'\x00\xAA garbage' + f1 + bytes(bad) + f2
    p = P.UsbFrameParser()
    out = []
    for i in range(0, len(stream), 3):      # 故意切碎餵進去
        out += p.feed(stream[i:i + 3])
    assert out == [(P.USB_TX, b'hello'), (P.USB_INFO, bytes(17))]
    assert p.crc_errors == 1


def test_radio_state_roundtrip():
    h = P.Header(7, P.MSG_STATE, 42, 65535, 123456)
    st = P.State(1.5, -2.25, 0.5, 3.0, 10.0, -0.2, 3, 1, P.FLAG_POSE_VALID | P.FLAG_TRACK_VALID)
    raw = P.encode_radio(h, st)
    assert raw[:3] == b'V2\x07'           # 韌體只看這 3 bytes
    assert len(raw) == 40 and len(raw) <= P.ESPNOW_MAX
    pkt = P.decode_radio(raw)
    assert pkt.header.car_id == 42 and pkt.header.seq == 65535
    assert math.isclose(pkt.body.x, 1.5) and pkt.body.lap == 3 and pkt.body.state == 1


def test_ping_pong_and_bad_input():
    raw = P.encode_radio(P.Header(7, P.MSG_PONG, 1, 0, 5), P.Pong(9, 77))
    assert P.decode_radio(raw).body == P.Pong(9, 77)
    assert P.decode_radio(b'xx') is None
    assert P.decode_radio(b'XX' + raw[2:]) is None


def test_seq_gap_wraparound():
    assert P.seq_gap(10, 11) == 0
    assert P.seq_gap(10, 13) == 2
    assert P.seq_gap(65535, 0) == 0


def test_track_projection_square():
    # 4m x 2m 逆時針矩形
    xy = np.array([[0, 0], [4, 0], [4, 2], [0, 2]], dtype=float)
    tr = Track(xy)
    assert math.isclose(tr.length, 12.0)
    s, d, _ = tr.project(1.0, 0.3)          # 下邊、偏左（賽道內側）
    assert math.isclose(s, 1.0) and math.isclose(d, 0.3)
    s, d, _ = tr.project(4.2, 1.0)          # 右邊、偏外側 = 右邊 = 負
    assert math.isclose(s, 5.0) and math.isclose(d, -0.2)
    assert math.isclose(tr.gap(11.0, 1.0), 2.0)    # 跨過起點，對方在前 2m
    assert math.isclose(tr.gap(1.0, 11.0), -2.0)
