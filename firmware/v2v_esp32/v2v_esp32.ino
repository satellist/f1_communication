// ESP32-C3 雙車通訊韌體（兩台車燒同一份，不用改任何設定）
//
// 角色：透明轉送的「無線數據機」
//   Jetson --USB--> ESP32 --ESP-NOW--> 對方 ESP32 --USB--> 對方 Jetson
//   - USB 收到 TX 封包 -> 用 ESP-NOW 送出
//   - ESP-NOW 收到同 group 的封包 -> 加上來源 MAC、RSSI，經 USB 交給 Jetson
//   - 一開始用廣播；收到對方第一包後自動記住對方 MAC，改用 unicast（有硬體 ACK 與重送）
//   - 對方超過 PEER_STALE_MS 沒聲音，退回廣播
//
// USB 封包格式與欄位定義見 car_v2v/protocol.py（兩邊必須一致）。
// 編譯：Arduino-ESP32 core 3.x，板子 ESP32C3 Dev Module，USB CDC On Boot = Enabled
//       （firmware/flash.sh 已經設好）

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>

#define FW_VER 1
#define PEER_STALE_MS 1500
#define STATS_PERIOD_MS 1000
#define INFO_PERIOD_MS 2000
// USB 封包 len 只有 1 byte（<=255），RX 要多放 MAC(6)+RSSI(1)，所以無線封包上限 248
#define RADIO_MAX 248

// USB 封包類型（和 protocol.py 相同）
#define USB_CFG      0x10
#define USB_TX       0x11
#define USB_GET_INFO 0x12
#define USB_INFO     0x20
#define USB_RX       0x21
#define USB_STATS    0x22
#define USB_LOG      0x23

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint8_t g_channel = 1;
static uint8_t g_group = 7;
static uint8_t g_txpower = 60;   // 0.25 dBm 單位
static uint8_t my_mac[6];

static uint8_t peer_mac[6];
static bool peer_known = false;
static uint32_t peer_last_ms = 0;

static volatile uint32_t tx_ok = 0, tx_fail = 0, rx_cnt = 0, rx_drop = 0;

struct RxItem {
  uint8_t mac[6];
  int8_t rssi;
  uint8_t len;
  uint8_t data[ESP_NOW_MAX_DATA_LEN];
};
static QueueHandle_t rxq;

// ---------------- CRC / USB ----------------
static uint16_t crc16(const uint8_t *d, size_t n, uint16_t crc = 0xFFFF) {
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

// 一次寫完整封包；USB 緩衝不夠就整包丟掉，避免送出半包
static void usb_send(uint8_t type, const uint8_t *p, uint8_t len) {
  static uint8_t buf[2 + 2 + 255 + 2];
  buf[0] = 0xAA; buf[1] = 0x55; buf[2] = type; buf[3] = len;
  if (len) memcpy(buf + 4, p, len);
  uint16_t c = crc16(buf + 2, 2 + len);
  buf[4 + len] = c & 0xFF; buf[5 + len] = c >> 8;
  size_t n = 6 + len;
  if (Serial.availableForWrite() < (int)n) return;
  Serial.write(buf, n);
}

static void usb_log(const char *s) { usb_send(USB_LOG, (const uint8_t *)s, strlen(s)); }

static void send_info() {
  uint8_t p[17];
  memcpy(p, my_mac, 6);
  p[6] = g_channel; p[7] = g_group; p[8] = FW_VER; p[9] = peer_known ? 1 : 0;
  if (peer_known) memcpy(p + 10, peer_mac, 6); else memset(p + 10, 0, 6);
  p[16] = g_txpower;
  usb_send(USB_INFO, p, sizeof(p));
}

static void send_stats() {
  uint32_t v[4] = {tx_ok, tx_fail, rx_cnt, rx_drop};
  usb_send(USB_STATS, (const uint8_t *)v, sizeof(v));
}

// ---------------- ESP-NOW ----------------
static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  // 只收自己隊伍的封包：magic 'V2' + group
  if (len < 3 || len > RADIO_MAX) return;
  if (data[0] != 'V' || data[1] != '2' || data[2] != g_group) return;
  RxItem it;
  memcpy(it.mac, info->src_addr, 6);
  it.rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
  it.len = len;
  memcpy(it.data, data, len);
  if (xQueueSend(rxq, &it, 0) == pdTRUE) rx_cnt++; else rx_drop++;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void on_sent(const wifi_tx_info_t *, esp_now_send_status_t st) {
#else
static void on_sent(const uint8_t *, esp_now_send_status_t st) {
#endif
  if (st == ESP_NOW_SEND_SUCCESS) tx_ok++; else tx_fail++;
}

static void add_peer(const uint8_t *mac) {
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;            // 0 = 跟著目前頻道
  p.ifidx = WIFI_IF_STA;
  p.encrypt = false;
  if (!esp_now_is_peer_exist(mac)) esp_now_add_peer(&p);
}

static void set_peer(const uint8_t *mac) {
  if (peer_known && memcmp(peer_mac, mac, 6) == 0) return;
  if (peer_known) esp_now_del_peer(peer_mac);
  memcpy(peer_mac, mac, 6);
  add_peer(peer_mac);
  peer_known = true;
  char s[48];
  snprintf(s, sizeof(s), "peer %02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  usb_log(s);
  send_info();
}

static void apply_radio() {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(g_channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_max_tx_power((int8_t)g_txpower);
}

static void radio_send(const uint8_t *p, uint8_t len) {
  bool unicast = peer_known && (millis() - peer_last_ms) < PEER_STALE_MS;
  if (esp_now_send(unicast ? peer_mac : BCAST, p, len) != ESP_OK) tx_fail++;
}

// ---------------- USB 收封包 ----------------
static void handle_usb_frame(uint8_t type, const uint8_t *p, uint8_t len) {
  switch (type) {
    case USB_TX:
      if (len >= 3 && len <= RADIO_MAX) radio_send(p, len);
      break;
    case USB_CFG:
      if (len >= 3) {
        uint8_t ch = p[0];
        if (ch >= 1 && ch <= 13) g_channel = ch;
        g_group = p[1];
        if (p[2] >= 8 && p[2] <= 84) g_txpower = p[2];
        apply_radio();
        if (peer_known) { esp_now_del_peer(peer_mac); peer_known = false; }
      }
      send_info();
      break;
    case USB_GET_INFO:
      send_info();
      break;
  }
}

static void poll_usb() {
  static uint8_t buf[2 + 2 + 255 + 2];
  static size_t n = 0;
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (n == 0 && b != 0xAA) continue;
    if (n == 1 && b != 0x55) { n = (b == 0xAA) ? 1 : 0; continue; }
    buf[n++] = b;
    if (n >= 4) {
      size_t total = 6 + buf[3];
      if (n == total) {
        uint16_t c = crc16(buf + 2, 2 + buf[3]);
        if ((c & 0xFF) == buf[total - 2] && (c >> 8) == buf[total - 1])
          handle_usb_frame(buf[2], buf + 4, buf[3]);
        n = 0;
      }
    }
  }
}

// ---------------- main ----------------
void setup() {
  Serial.setRxBufferSize(2048);
  Serial.setTxBufferSize(2048);
  Serial.begin(921600);
  Serial.setTxTimeoutMs(0);   // Jetson 沒在讀時不要卡住

  rxq = xQueueCreate(32, sizeof(RxItem));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);   // 關省電，降低延遲
  esp_wifi_get_mac(WIFI_IF_STA, my_mac);
  apply_radio();

  if (esp_now_init() != ESP_OK) {
    while (true) { usb_log("esp_now_init failed"); delay(1000); }
  }
  esp_now_register_recv_cb(on_recv);
  esp_now_register_send_cb(on_sent);
  add_peer(BCAST);
  send_info();
}

void loop() {
  poll_usb();

  RxItem it;
  while (xQueueReceive(rxq, &it, 0) == pdTRUE) {
    set_peer(it.mac);
    peer_last_ms = millis();
    static uint8_t p[7 + ESP_NOW_MAX_DATA_LEN];
    memcpy(p, it.mac, 6);
    p[6] = (uint8_t)it.rssi;
    memcpy(p + 7, it.data, it.len);
    usb_send(USB_RX, p, 7 + it.len);
  }

  uint32_t now = millis();
  static uint32_t t_stats = 0, t_info = 0;
  if (now - t_stats >= STATS_PERIOD_MS) { t_stats = now; send_stats(); }
  if (now - t_info >= INFO_PERIOD_MS) { t_info = now; send_info(); }
  delay(1);
}
