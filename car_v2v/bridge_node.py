"""v2v_bridge：雙車通訊 ROS 2 節點。兩台車跑完全一樣的程式。

自己 -> 對方（send_rate Hz）：
    位置 x,y,yaw（TF map->base_link）、速度 v（/odom）、賽道進度 s / 橫向偏移 d / 圈數（中心線），
    以及超車協商狀態碼（上層邏輯發布到 /v2v/ego_state）

對方 -> 自己，發布成 topic：
    /v2v/opponent         std_msgs/String  JSON，對方的完整狀態 + 資料年齡 + RSSI + 前後距離 gap
    /v2v/opponent_pose    geometry_msgs/PoseStamped（map 座標，已用速度補償延遲）
    /v2v/opponent_gap     std_msgs/Float32  對方在前面幾公尺（沿中心線，負值 = 對方在後面）
    /v2v/opponent_marker  visualization_msgs/Marker，在 RViz 顯示對手
    /v2v/peer_alive       std_msgs/Bool     peer_timeout 內有收到對方 = True
    /v2v/link             std_msgs/String  JSON，連線品質（每秒一次）
"""

import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String, UInt8
from visualization_msgs.msg import Marker
import tf2_ros

from . import protocol as P
from .serial_link import SerialLink
from .track import Track


def now_ms():
    return int(time.monotonic() * 1000) & 0xFFFFFFFF


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class V2VBridge(Node):
    def __init__(self):
        super().__init__('v2v_bridge')
        dp = self.declare_parameter
        dp('port', 'auto')               # auto = 自動找 /dev/serial/by-id/usb-Espressif*
        dp('channel', 1)                 # ESP-NOW 頻道 1~13，兩台車必須相同
        dp('group_id', 7)                # 隊伍代號 0~255，兩台車必須相同；別隊用不同值就互不干擾
        dp('tx_power_qdbm', 60)          # 發射功率，單位 0.25 dBm（60 = 15 dBm，最大 78）
        dp('car_id', 0)                  # 0 = 依 ESP32 MAC 自動產生；兩台車要不同
        dp('send_rate', 30.0)            # Hz
        dp('ping_rate', 2.0)             # Hz，量來回延遲
        dp('peer_timeout', 0.3)          # 秒，超過沒收到對方就視為斷線
        dp('map_frame', 'map')
        dp('base_frame', 'base_link')
        dp('odom_topic', '/odom')
        dp('centerline_csv', '')         # 空字串 = 不算 s/d

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.map_frame = gp('map_frame')
        self.base_frame = gp('base_frame')
        self.peer_timeout = float(gp('peer_timeout'))
        self.car_id_param = int(gp('car_id'))
        self.group = int(gp('group_id'))

        # 中心線
        self.track = None
        csv_path = gp('centerline_csv')
        if csv_path:
            try:
                self.track = Track.from_csv(csv_path)
                self.get_logger().info(
                    f'中心線：{csv_path}（{len(self.track.p0)} 點，全長 {self.track.length:.2f} m）')
            except Exception as e:
                self.get_logger().warn(f'中心線載入失敗（{csv_path}）：{e}；s/d 不會送出')
        else:
            self.get_logger().warn('沒有設定 centerline_csv；s/d/gap 不會送出')
        self.track_hint = None
        self.last_s = None
        self.lap = 0

        # 自己的狀態
        self.my_mac = None
        self.my_id = self.car_id_param if self.car_id_param > 0 else 0
        self.seq = 0
        self.ego_state = 0
        self.odom_v = 0.0
        self.odom_t = 0.0
        self.ego = None          # 最後一次送出的 State（算 gap 用）
        self.esp_info = None
        self.esp_stats = None

        # 對方的狀態
        self.peer = None             # 最後一次收到的 State
        self.peer_hdr = None
        self.peer_mac = None
        self.peer_rssi = None
        self.peer_rx_t = None        # time.monotonic()
        self.peer_last_seq = None
        self.rx_count = 0
        self.lost_count = 0
        self.rtt_ms = None
        self.ping_nonce = 0
        self.was_alive = False
        self.id_conflict_warned = False

        # ROS 介面
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, gp('odom_topic'), self.on_odom, 10)
        self.create_subscription(UInt8, '/v2v/ego_state', self.on_ego_state, 10)
        self.pub_opp = self.create_publisher(String, '/v2v/opponent', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/v2v/opponent_pose', 10)
        self.pub_gap = self.create_publisher(Float32, '/v2v/opponent_gap', 10)
        self.pub_marker = self.create_publisher(Marker, '/v2v/opponent_marker', 10)
        self.pub_alive = self.create_publisher(Bool, '/v2v/peer_alive', 10)
        self.pub_link = self.create_publisher(String, '/v2v/link', 10)

        self.link = SerialLink(port=gp('port'), channel=int(gp('channel')), group=self.group,
                               tx_power_qdbm=int(gp('tx_power_qdbm')),
                               on_frame=self.on_frame, on_state=self.on_serial_state,
                               log=lambda m: self.get_logger().warn(m)).start()

        self.create_timer(1.0 / float(gp('send_rate')), self.send_state)
        self.create_timer(1.0 / float(gp('ping_rate')), self.send_ping)
        self.create_timer(0.1, self.check_alive)
        self.create_timer(1.0, self.publish_link)
        self.stat_t0 = time.monotonic()
        self.stat_rx0 = 0
        self.stat_lost0 = 0
        self.rx_hz = 0.0
        self.loss_pct = 0.0

    # ---------------- 自己 ----------------
    def on_odom(self, msg):
        self.odom_v = msg.twist.twist.linear.x
        self.odom_t = time.monotonic()

    def on_ego_state(self, msg):
        if msg.data != self.ego_state:
            self.get_logger().info(
                f'ego_state -> {P.STATE_NAMES.get(msg.data, msg.data)}')
        self.ego_state = msg.data

    def build_state(self):
        st = P.State(state=self.ego_state)
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
            stamp = Time.from_msg(tf.header.stamp)
            # 時間戳 0 = 固定 TF（測試用），視為有效；動態 TF 超過 1 秒沒更新視為定位失效
            age = 0.0 if stamp.nanoseconds == 0 else \
                (self.get_clock().now() - stamp).nanoseconds * 1e-9
            if age < 1.0:
                t = tf.transform
                st.x, st.y = t.translation.x, t.translation.y
                st.yaw = yaw_from_quat(t.rotation)
                st.flags |= P.FLAG_POSE_VALID
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass
        if time.monotonic() - self.odom_t < 0.5:
            st.v = self.odom_v
            st.flags |= P.FLAG_SPEED_VALID
        if self.track is not None and st.flags & P.FLAG_POSE_VALID:
            s, d, self.track_hint = self.track.project(st.x, st.y, self.track_hint)
            L = self.track.length
            if self.last_s is not None:
                if self.last_s > 0.75 * L and s < 0.25 * L:
                    self.lap += 1
                elif self.last_s < 0.25 * L and s > 0.75 * L:
                    self.lap -= 1
            self.last_s = s
            st.s, st.d, st.lap = s, d, max(0, self.lap)
            st.flags |= P.FLAG_TRACK_VALID
        return st

    def send_state(self):
        if not self.link.connected or self.my_id == 0:
            return
        st = self.build_state()
        self.ego = st
        self.seq = (self.seq + 1) & 0xFFFF
        h = P.Header(self.group, P.MSG_STATE, self.my_id, self.seq, now_ms())
        self.link.send_radio(P.encode_radio(h, st))

    def send_ping(self):
        if not self.link.connected or self.my_id == 0:
            return
        self.ping_nonce = (self.ping_nonce + 1) & 0xFFFFFFFF
        h = P.Header(self.group, P.MSG_PING, self.my_id, 0, now_ms())
        self.link.send_radio(P.encode_radio(h, P.Ping(self.ping_nonce)))

    # ---------------- USB / ESP32 ----------------
    def on_serial_state(self, connected, port):
        if connected:
            self.get_logger().info(f'已連上 ESP32：{port}')
        else:
            self.get_logger().error(f'ESP32 斷線：{port}')
            self.esp_info = None

    def on_frame(self, ftype, payload):
        if ftype == P.USB_RX:
            self.on_radio(payload[:6], int.from_bytes(payload[6:7], 'little', signed=True),
                          payload[7:])
        elif ftype == P.USB_INFO:
            info = P.EspInfo.unpack(payload)
            first = self.esp_info is None
            self.esp_info = info
            self.my_mac = info.mac
            if self.car_id_param <= 0:
                self.my_id = (P.crc16_ccitt(info.mac) % 254) + 1
            if first:
                self.get_logger().info(
                    f'ESP32 MAC {P.mac_str(info.mac)}，韌體 v{info.fw_ver}，頻道 {info.channel}，'
                    f'group {info.group}，本車 car_id={self.my_id}')
                if info.channel != self.link.channel or info.group != self.group:
                    self.get_logger().warn('ESP32 設定和參數不一致，重送設定')
                    self.link.configure()
        elif ftype == P.USB_STATS:
            self.esp_stats = P.EspStats.unpack(payload)
        elif ftype == P.USB_LOG:
            self.get_logger().info(f'[ESP32] {payload.decode(errors="replace")}')

    def on_radio(self, src_mac, rssi, raw):
        pkt = P.decode_radio(raw)
        if pkt is None or pkt.header.group != self.group:
            return
        h = pkt.header
        if h.car_id == self.my_id:
            if not self.id_conflict_warned:
                self.get_logger().error(
                    f'收到相同 car_id={h.car_id} 的封包！請在其中一台車手動設定 car_id 參數')
                self.id_conflict_warned = True
            return
        if h.msg_type == P.MSG_PING:
            rh = P.Header(self.group, P.MSG_PONG, self.my_id, 0, now_ms())
            self.link.send_radio(P.encode_radio(rh, P.Pong(pkt.body.nonce, h.t_ms)))
            return
        if h.msg_type == P.MSG_PONG:
            rtt = (now_ms() - pkt.body.echo_t_ms) & 0xFFFFFFFF
            if rtt < 5000:
                self.rtt_ms = rtt if self.rtt_ms is None else 0.8 * self.rtt_ms + 0.2 * rtt
            return
        if h.msg_type != P.MSG_STATE:
            return

        t = time.monotonic()
        if self.peer_last_seq is not None and self.peer_hdr is not None \
                and self.peer_hdr.car_id == h.car_id:
            gap = P.seq_gap(self.peer_last_seq, h.seq)
            if 0 < gap < 1000:
                self.lost_count += gap
            elif gap > 30000:      # 序號倒退（對方重開機）或重複封包
                pass
        self.peer_last_seq = h.seq
        self.peer, self.peer_hdr, self.peer_mac, self.peer_rssi = pkt.body, h, src_mac, rssi
        self.peer_rx_t = t
        self.rx_count += 1
        self.publish_opponent()

    # ---------------- 發布 ----------------
    def peer_alive(self):
        return self.peer_rx_t is not None and time.monotonic() - self.peer_rx_t < self.peer_timeout

    def publish_opponent(self):
        o = self.peer
        # 資料年齡 = 收到後經過多久 + 單程延遲（用量到的 RTT/2 估計），不需要兩台車對時
        one_way = (self.rtt_ms / 2000.0) if self.rtt_ms is not None else 0.010
        age = time.monotonic() - self.peer_rx_t + one_way
        gap = None
        if (self.track is not None and self.ego is not None
                and o.flags & P.FLAG_TRACK_VALID and self.ego.flags & P.FLAG_TRACK_VALID):
            # 取賽道上較近的方向（±半圈內），不看圈數；套圈判斷請另外比 lap
            gap = self.track.gap(self.ego.s, o.s)
            self.pub_gap.publish(Float32(data=float(gap)))

        d = {
            'car_id': self.peer_hdr.car_id, 'mac': P.mac_str(self.peer_mac),
            'seq': self.peer_hdr.seq, 'age_ms': round(age * 1000, 1), 'rssi': self.peer_rssi,
            'x': round(o.x, 3), 'y': round(o.y, 3), 'yaw': round(o.yaw, 4), 'v': round(o.v, 3),
            's': round(o.s, 3), 'd': round(o.d, 3), 'lap': o.lap,
            'state': o.state, 'state_name': P.STATE_NAMES.get(o.state, str(o.state)),
            'pose_valid': bool(o.flags & P.FLAG_POSE_VALID),
            'track_valid': bool(o.flags & P.FLAG_TRACK_VALID),
            'speed_valid': bool(o.flags & P.FLAG_SPEED_VALID),
            'gap': None if gap is None else round(gap, 3),
        }
        self.pub_opp.publish(String(data=json.dumps(d)))

        if o.flags & P.FLAG_POSE_VALID:
            # 用速度往前推，補償資料年齡
            x = o.x + o.v * age * math.cos(o.yaw)
            y = o.y + o.v * age * math.sin(o.yaw)
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.z = math.sin(o.yaw / 2)
            ps.pose.orientation.w = math.cos(o.yaw / 2)
            self.pub_pose.publish(ps)

            m = Marker()
            m.header = ps.header
            m.ns, m.id, m.type, m.action = 'v2v_opponent', 0, Marker.CUBE, Marker.ADD
            m.pose = ps.pose
            m.pose.position.z = 0.08
            m.scale.x, m.scale.y, m.scale.z = 0.50, 0.28, 0.16
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 0.8
            m.lifetime.nanosec = int(self.peer_timeout * 1e9)
            self.pub_marker.publish(m)

    def check_alive(self):
        alive = self.peer_alive()
        self.pub_alive.publish(Bool(data=alive))
        if alive != self.was_alive:
            if alive:
                self.get_logger().info(
                    f'對方上線：car_id={self.peer_hdr.car_id} MAC {P.mac_str(self.peer_mac)}')
            else:
                self.get_logger().warn('對方斷線（超過 peer_timeout 沒收到資料）')
            self.was_alive = alive

    def publish_link(self):
        t = time.monotonic()
        dt = t - self.stat_t0
        rx = self.rx_count - self.stat_rx0
        lost = self.lost_count - self.stat_lost0
        self.rx_hz = rx / dt if dt > 0 else 0.0
        self.loss_pct = 100.0 * lost / (rx + lost) if (rx + lost) > 0 else 0.0
        self.stat_t0, self.stat_rx0, self.stat_lost0 = t, self.rx_count, self.lost_count
        s = self.esp_stats
        d = {
            'usb_connected': self.link.connected, 'port': self.link.port,
            'my_mac': P.mac_str(self.my_mac) if self.my_mac else None, 'my_id': self.my_id,
            'channel': self.link.channel, 'group': self.group,
            'peer_alive': self.peer_alive(),
            'peer_id': self.peer_hdr.car_id if self.peer_hdr else None,
            'peer_mac': P.mac_str(self.peer_mac) if self.peer_mac else None,
            'rssi': self.peer_rssi, 'rx_hz': round(self.rx_hz, 1),
            'loss_pct': round(self.loss_pct, 1),
            'rtt_ms': None if self.rtt_ms is None else round(self.rtt_ms, 1),
            'esp_tx_ok': s.tx_ok if s else None, 'esp_tx_fail': s.tx_fail if s else None,
            'esp_rx_drop': s.rx_drop if s else None,
            'usb_crc_errors': self.link.parser.crc_errors,
            'ego_state': P.STATE_NAMES.get(self.ego_state, self.ego_state),
        }
        self.pub_link.publish(String(data=json.dumps(d)))

    def destroy_node(self):
        self.link.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = V2VBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
