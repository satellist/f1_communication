"""不需要 ROS 的連線測試。兩台車（或同一台 Jetson 插兩顆 ESP32）同時執行，
每秒印出：收包率、掉包率、RSSI、來回延遲、ESP32 發送成功/失敗次數。

    ros2 run car_v2v link_test                 # 自動找 ESP32
    ros2 run car_v2v link_test --port /dev/serial/by-id/usb-Espressif_...  # 指定
    ros2 run car_v2v link_test --list          # 列出所有 ESP32
"""

import argparse
import sys
import time

from . import protocol as P
from .serial_link import SerialLink, find_esp32_ports


def now_ms():
    return int(time.monotonic() * 1000) & 0xFFFFFFFF


def main(argv=None):
    ap = argparse.ArgumentParser(description='ESP32-C3 雙車通訊連線測試')
    ap.add_argument('--port', default='auto')
    ap.add_argument('--channel', type=int, default=1)
    ap.add_argument('--group', type=int, default=7)
    ap.add_argument('--tx-power', type=int, default=60, help='單位 0.25 dBm')
    ap.add_argument('--rate', type=float, default=30.0, help='STATE 發送頻率 Hz')
    ap.add_argument('--car-id', type=int, default=0, help='0 = 依 MAC 自動')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args(argv)

    if a.list:
        ports = find_esp32_ports()
        print('\n'.join(ports) if ports else '沒有找到 ESP32')
        return 0

    st = {'info': None, 'stats': None, 'my_id': a.car_id, 'rx': 0, 'lost': 0, 'last_seq': None,
          'rssi': None, 'peer': None, 'rtt': [], 'last_rx': None}

    def on_frame(ftype, p):
        if ftype == P.USB_INFO:
            info = P.EspInfo.unpack(p)
            if st['info'] is None:
                if st['my_id'] <= 0:
                    st['my_id'] = (P.crc16_ccitt(info.mac) % 254) + 1
                print(f'ESP32 MAC {P.mac_str(info.mac)}  韌體 v{info.fw_ver}  頻道 {info.channel}  '
                      f'group {info.group}  功率 {info.tx_power_qdbm / 4:.2f} dBm  car_id={st["my_id"]}')
            st['info'] = info
        elif ftype == P.USB_STATS:
            st['stats'] = P.EspStats.unpack(p)
        elif ftype == P.USB_LOG:
            print('[ESP32]', p.decode(errors='replace'))
        elif ftype == P.USB_RX:
            mac, rssi, raw = p[:6], int.from_bytes(p[6:7], 'little', signed=True), p[7:]
            pkt = P.decode_radio(raw)
            if pkt is None:
                return
            h = pkt.header
            if h.msg_type == P.MSG_PING:
                rh = P.Header(a.group, P.MSG_PONG, st['my_id'], 0, now_ms())
                link.send_radio(P.encode_radio(rh, P.Pong(pkt.body.nonce, h.t_ms)))
            elif h.msg_type == P.MSG_PONG:
                st['rtt'].append((now_ms() - pkt.body.echo_t_ms) & 0xFFFFFFFF)
            elif h.msg_type == P.MSG_STATE:
                if st['last_seq'] is not None:
                    g = P.seq_gap(st['last_seq'], h.seq)
                    if 0 < g < 1000:
                        st['lost'] += g
                st['last_seq'] = h.seq
                st['rx'] += 1
                st['rssi'] = rssi
                st['peer'] = (h.car_id, P.mac_str(mac))
                st['last_rx'] = time.monotonic()

    link = SerialLink(a.port, a.channel, a.group, a.tx_power, on_frame=on_frame,
                      on_state=lambda c, port: print(('已連上 ' if c else '斷線 ') + str(port)))
    link.start()

    seq = 0
    nonce = 0
    period = 1.0 / a.rate
    t_next = time.monotonic()
    t_report = t_next + 1.0
    t_ping = t_next
    prev = {'rx': 0, 'lost': 0, 'ok': 0, 'fail': 0}
    print('Ctrl+C 結束。對方也要同時執行這個程式（或 v2v_bridge）。')
    try:
        while True:
            t = time.monotonic()
            if t >= t_next:
                t_next += period
                if link.connected and st['my_id'] > 0:
                    seq = (seq + 1) & 0xFFFF
                    h = P.Header(a.group, P.MSG_STATE, st['my_id'], seq, now_ms())
                    link.send_radio(P.encode_radio(h, P.State(x=1.0, y=2.0, v=0.5)))
            if t >= t_ping:
                t_ping += 0.25
                if link.connected and st['my_id'] > 0:
                    nonce += 1
                    h = P.Header(a.group, P.MSG_PING, st['my_id'], 0, now_ms())
                    link.send_radio(P.encode_radio(h, P.Ping(nonce)))
            if t >= t_report:
                t_report += 1.0
                rx = st['rx'] - prev['rx']
                lost = st['lost'] - prev['lost']
                prev['rx'], prev['lost'] = st['rx'], st['lost']
                s = st['stats']
                ok = fail = '-'
                if s:
                    ok, fail = s.tx_ok - prev['ok'], s.tx_fail - prev['fail']
                    prev['ok'], prev['fail'] = s.tx_ok, s.tx_fail
                rtt = st['rtt']
                st['rtt'] = []
                rtt_s = (f'{sum(rtt) / len(rtt):5.1f}ms(max {max(rtt)})' if rtt else '   -   ')
                loss = 100.0 * lost / (rx + lost) if rx + lost else 0.0
                peer = f'對方 id={st["peer"][0]} {st["peer"][1]}' if st['peer'] else '尚未收到對方'
                alive = st['last_rx'] is not None and t - st['last_rx'] < 0.3
                print(f'{"●" if alive else "○"} 收 {rx:3d}/s 掉 {loss:4.1f}%  RSSI {st["rssi"]} dBm  '
                      f'RTT {rtt_s}  ESP送出 成功{ok} 失敗{fail}  | {peer}', flush=True)
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        link.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
