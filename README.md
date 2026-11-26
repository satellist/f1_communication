# car_v2v：ESP32-C3 雙車通訊

兩台 F1TENTH 車各接一顆 ESP32-C3，用 ESP-NOW 互傳位置、速度、賽道進度和超車協商狀態。
**兩台車的程式、韌體、設定檔完全一樣**，不用分車修改。

```
車A Jetson ⇄ USB ⇄ ESP32-C3 ⇄ ESP-NOW 無線 ⇄ ESP32-C3 ⇄ USB ⇄ Jetson 車B
             └── v2v_bridge ──┘                           └── v2v_bridge ──┘
```

## 資料夾內容

| 路徑 | 用途 |
|---|---|
| `setup_car.sh` | **每台車的一鍵安裝 + 檢查**（建置、確認 ESP32 韌體、提醒 ROS_DOMAIN_ID） |
| `firmware/micropython/main.py` | **ESP32-C3 韌體（目前使用中）**：MicroPython 版，透明轉送，自動配對對方 |
| `firmware/micropython/ESP32_GENERIC_C3-v1.24.1.bin` | MicroPython 本體 |
| `firmware/flash_mpy.sh` | 燒錄 MicroPython 版（只需要 esptool + mpremote，約 2MB） |
| `firmware/v2v_esp32/v2v_esp32.ino` | C 版韌體（延遲較低；需要 1GB 的 Arduino 工具，尚未實測） |
| `firmware/install_tools.sh`、`firmware/flash.sh` | C 版的工具安裝與燒錄 |
| `car_v2v/protocol.py` | 封包格式（USB 層 + 無線層），韌體用同一份定義 |
| `car_v2v/serial_link.py` | USB 連線：自動找埠、斷線重連 |
| `car_v2v/track.py` | 把位置投影到中心線，算 s（賽道進度）、d（橫向偏移） |
| `car_v2v/bridge_node.py` | ROS 2 節點 `v2v_bridge` |
| `car_v2v/link_test.py` | 不需要 ROS 的連線測試工具 |
| `config/v2v.yaml` | 參數（頻道、隊伍代號、發送頻率…） |
| `launch/v2v.launch.py` | 啟動檔 |

## 安裝步驟

兩顆 ESP32 **已經在第一台 Jetson 上燒好韌體並實測互傳成功**（2026-11-26）：

| ESP32 MAC | 自動 car_id |
|---|---|
| `20:D5:C2:0E:FE:98` | 208 |
| `20:D5:C2:0F:04:2C` | 101 |

### 第二台車（ESP32 已燒好，不需要燒錄工具）

1. 下載程式到第二台的工作區：
   ```bash
   cd f1tenth-main/src
   git clone https://github.com/satellist/f1_communication.git car_v2v
   ```
2. 把其中一顆 ESP32 插到第二台 Jetson
3. 執行：
   ```bash
   cd f1tenth-main/src/car_v2v
   ./setup_car.sh
   ```
   它會建置程式、確認 ESP32 韌體有回應，並提醒你設定 ROS_DOMAIN_ID。全部 ✔ 就完成了。

### 需要重燒或換新的 ESP32 時

```bash
cd f1tenth-main/src/car_v2v/firmware
./flash_mpy.sh              # 燒錄所有插著的 ESP32（第一次會自動 pip 安裝 esptool、mpremote）
./flash_mpy.sh --update     # 只改了 main.py 時用，比較快
```

燒錄前要先關掉 `v2v_bridge` 和 `link_test`。

## 測試步驟

### 1. 連線測試（不需要 ROS、不需要定位）

兩台車**同時**執行：

```bash
ros2 run car_v2v link_test
```

正常的輸出（每秒一行）：

```
ESP32 MAC 20:D5:C2:0E:FE:98  韌體 v1  頻道 1  group 7  功率 15.00 dBm  car_id=123
● 收  30/s 掉  0.0%  RSSI -42 dBm  RTT   6.2ms(max 9)  ESP送出 成功34 失敗0  | 對方 id=57 34:85:18:...
```

- `●` = 有收到對方；`○` = 沒收到
- 收包率應該接近 30/s，掉包率應該低於 5%（實測：0%）
- RTT（來回延遲）實測約 20–25 ms（MicroPython 版）；節點會用 RTT/2 補償對手位置
- RSSI 大於 -80 dBm 算穩定；低於 -85 開始會掉包
- 同一台 Jetson 插兩顆 ESP32 也能測，開兩個終端，分別用 `--port` 指定 by-id 路徑（`link_test --list` 可以列出）

### 2. 跑 ROS 節點

```bash
ros2 launch car_v2v v2v.launch.py
# 指定中心線：ros2 launch car_v2v v2v.launch.py centerline_csv:=/path/to/centerline.csv
```

要送出位置，先開 VESC（`/odom`）和定位（TF `map -> base_link`）。沒開也能跑，
只是對應欄位會標成無效（`pose_valid` / `speed_valid` / `track_valid` = false）。

```bash
ros2 topic echo /v2v/link          # 連線品質
ros2 topic echo /v2v/opponent      # 對方狀態
```

RViz 加上 Marker `/v2v/opponent_marker`，就會看到代表對手的紅色方塊。

## ROS 介面

**發布**

| Topic | 型別 | 內容 |
|---|---|---|
| `/v2v/opponent` | `std_msgs/String` | JSON：對方 x, y, yaw, v, s, d, lap, state, age_ms, rssi, gap… |
| `/v2v/opponent_pose` | `geometry_msgs/PoseStamped` | 對方位置（map 座標，已用速度補償延遲） |
| `/v2v/opponent_gap` | `std_msgs/Float32` | 對方在前面幾公尺（沿中心線；負值 = 對方在後面） |
| `/v2v/opponent_marker` | `visualization_msgs/Marker` | RViz 顯示 |
| `/v2v/peer_alive` | `std_msgs/Bool` | `peer_timeout`（0.3 秒）內有收到對方 = true，10 Hz |
| `/v2v/link` | `std_msgs/String` | JSON：USB 連線狀態、RSSI、收包率、掉包率、RTT，1 Hz |

**訂閱**

| Topic | 型別 | 內容 |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | 自己的速度 |
| TF `map -> base_link` | | 自己的位置 |
| `/v2v/ego_state` | `std_msgs/UInt8` | 自己的超車協商狀態，會跟著每一包送出 |

`ego_state` 的狀態碼：0 NORMAL、1 REQUEST_OVERTAKE、2 YIELDING、3 OVERTAKING、4 DONE、255 STOP。
測試時可以手動送：`ros2 topic pub --once /v2v/ego_state std_msgs/UInt8 "{data: 1}"`

## 運作原理重點

- **自動配對**：韌體一開始用廣播；收到同 group 的封包後記住對方 MAC，改用 unicast（有硬體 ACK 和重送）。
  對方 1.5 秒沒聲音就退回廣播。
- **自動 car_id**：預設依 ESP32 MAC 算出 1~254。萬一兩台撞號，log 會報錯，
  這時在其中一台加上 `car_id:=<其他數字>`。
- **隊伍隔離**：只收 `group_id` 相同的封包。比賽現場如果有別隊也用 ESP-NOW，改成不同的 group_id。
- **不需要對時**：ESP-NOW 單程延遲只有幾 ms，資料年齡直接用「收到後經過多久」計算。
- **s / d 要能比較**：兩台車必須載入**同一份地圖**（同一個 .pbstream）並使用**同一條中心線 csv**。

## 注意事項

1. **ROS_DOMAIN_ID**：兩台 Jetson 的 `~/.bashrc` 目前都設成 `ROS_DOMAIN_ID=99`（如果是同一份映像檔複製的）。
   兩台連到同一個 WiFi 時，ROS topic 會互相串到（`/drive`、`/scan` 會打架）。
   **兩台車要設成不同的 ROS_DOMAIN_ID**，例如 98 和 99。雙車的資料交換一律走 ESP-NOW，不要走 ROS 網路。
2. **頻道**：ESP32-C3 只支援 2.4 GHz。兩台的 `channel` 必須相同；如果 Jetson 的 WiFi 也在 2.4 GHz，選不同頻道（1 / 6 / 11 擇一）。
3. **燒錄前**要先關掉 `v2v_bridge` 和 `link_test`，否則序列埠被佔用。
   **不要用會把 DTR/RTS 拉低的序列埠工具開 ESP32**（例如 pyserial 設 `dtr=False, rts=False`）：
   ESP32-C3 會被重置，反覆重置會讓序列埠卡死，只能拔插 USB 恢復。
4. **ESP32-C3 Super Mini 這類小板**天線設計比較差，功率開太大反而會不穩。
   如果收包不穩，試著把 `tx_power_qdbm` 降到 34（8.5 dBm）。
5. **通訊只是輔助**：超車邏輯要在 `/v2v/peer_alive` = false 時退回保守模式，
   而且光達的防撞永遠優先於通訊資料。
