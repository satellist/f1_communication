#!/bin/bash
# 燒錄 MicroPython 版 v2v 韌體（推薦：只需要 esptool + mpremote，約 2MB，不用裝 1GB 的 Arduino 工具）
#   ./flash_mpy.sh              燒錄所有插著的 ESP32
#   ./flash_mpy.sh <埠>          只燒指定的那一顆
#   ./flash_mpy.sh --update     只更新 main.py（不重燒 MicroPython，比較快）
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MP="$DIR/micropython"
BIN="$(ls "$MP"/ESP32_GENERIC_C3-*.bin | head -1)"
export PATH="$HOME/.local/bin:$PATH"

if ! command -v esptool.py >/dev/null || ! command -v mpremote >/dev/null; then
    echo "安裝 esptool、mpremote…"
    pip3 install --user esptool mpremote
fi

UPDATE=0
if [ "$1" = "--update" ]; then UPDATE=1; shift; fi
if [ -n "$1" ]; then PORTS=("$1")
else mapfile -t PORTS < <(ls /dev/serial/by-id/usb-Espressif* 2>/dev/null); fi
[ ${#PORTS[@]} -eq 0 ] && { echo "沒有找到 ESP32，確認 USB 有插好"; exit 1; }

# 送 USB 封包 0x1F 讓正在跑的 v2v 程式離開，回到 REPL（mpremote 才能操作）
exit_v2v() {
    python3 - "$1" <<'EOF'
import sys, serial, time
s = serial.Serial(sys.argv[1])   # DTR/RTS 保持預設，避免重置晶片
s.write(bytes([0xAA, 0x55, 0x1F, 0x00, 0x42, 0x0E]))  # crc16(1F 00) = 0x0E42
time.sleep(0.3); s.close()
EOF
}

for P in "${PORTS[@]}"; do
    DEV="$(readlink -f "$P")"
    echo "================ $P"
    if command -v fuser >/dev/null && fuser "$DEV" >/dev/null 2>&1; then
        echo "$DEV 正被其他程式使用（v2v_bridge / link_test？），先關掉"; exit 1
    fi
    if [ $UPDATE -eq 0 ]; then
        esptool.py --chip esp32c3 -p "$DEV" erase-flash
        esptool.py --chip esp32c3 -p "$DEV" --baud 460800 write-flash 0x0 "$BIN"
        sleep 3   # 等 MicroPython 第一次開機（會建立檔案系統）
    else
        exit_v2v "$DEV"
    fi
    mpremote connect "$DEV" fs cp "$MP/main.py" :main.py + reset
    echo "完成：$P"
done
sleep 2
echo
echo "檢查：python3 -m car_v2v.link_test --list，或執行 ../setup_car.sh"
