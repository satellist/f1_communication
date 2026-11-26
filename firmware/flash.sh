#!/bin/bash
# 編譯並燒錄 v2v_esp32 韌體。兩台車燒同一份，不用改任何設定。
#   ./flash.sh              自動找 ESP32（只插一顆時）
#   ./flash.sh /dev/ttyACM1 指定埠
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PATH="$HOME/.local/bin:$PATH"
FQBN="esp32:esp32:esp32c3:CDCOnBoot=cdc"
command -v arduino-cli >/dev/null || { echo "找不到 arduino-cli，先執行 ./install_tools.sh"; exit 1; }

PORT="$1"
if [ -z "$PORT" ]; then
    mapfile -t PORTS < <(ls /dev/serial/by-id/usb-Espressif* 2>/dev/null)
    if [ ${#PORTS[@]} -eq 0 ]; then echo "沒有找到 ESP32，確認 USB 有插好"; exit 1; fi
    if [ ${#PORTS[@]} -gt 1 ]; then printf '找到多顆 ESP32，請指定：\n'; printf '  %s\n' "${PORTS[@]}"; exit 1; fi
    PORT="${PORTS[0]}"
fi
PORT="$(readlink -f "$PORT")"
if command -v fuser >/dev/null && fuser "$PORT" >/dev/null 2>&1; then
    echo "$PORT 正被其他程式使用（v2v_bridge / link_test？），先關掉再燒錄"; exit 1
fi
BUILD="$HOME/.cache/v2v_esp32_build"
echo "== 編譯 =="
arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD" "$DIR/v2v_esp32"
echo "== 燒錄到 $PORT =="
arduino-cli upload --fqbn "$FQBN" --input-dir "$BUILD" -p "$PORT"
echo "完成。用 'ros2 run car_v2v link_test' 確認。"
