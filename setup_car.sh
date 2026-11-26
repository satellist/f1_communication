#!/bin/bash
# 每台車的一鍵安裝 + 檢查（ESP32 已經燒好韌體時用；不需要燒錄工具）
#   cd f1tenth-main/src/car_v2v && ./setup_car.sh
# 不用 set -u：ROS 的 setup.bash 會用到未定義變數
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$PKG/../.." && pwd)"
ok()   { echo -e "  \033[32m✔\033[0m $*"; }
warn() { echo -e "  \033[33m!\033[0m $*"; }
fail() { echo -e "  \033[31m✘\033[0m $*"; FAILED=1; }
FAILED=0

echo "== 1. 環境 =="
if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; ok "ROS 2 Humble"; else fail "找不到 /opt/ros/humble"; exit 1; fi
python3 -c "import serial" 2>/dev/null && ok "pyserial" || { warn "安裝 pyserial"; pip3 install --user pyserial || fail "pyserial 安裝失敗"; }
python3 -c "import numpy" 2>/dev/null && ok "numpy" || fail "缺少 numpy（sudo apt install python3-numpy）"
if id -nG | grep -qw dialout; then ok "使用者在 dialout 群組（可以開序列埠）"
else fail "使用者不在 dialout 群組：執行 sudo usermod -aG dialout $USER 後重新登入"; fi

echo "== 2. 建置 car_v2v =="
(cd "$WS" && colcon build --symlink-install --packages-select car_v2v >/tmp/car_v2v_build.log 2>&1) \
    && ok "建置完成" || { fail "建置失敗，看 /tmp/car_v2v_build.log"; exit 1; }
source "$WS/install/setup.bash" 2>/dev/null

echo "== 3. ESP32 =="
mapfile -t PORTS < <(ls /dev/serial/by-id/usb-Espressif* 2>/dev/null)
if [ ${#PORTS[@]} -eq 0 ]; then
    fail "沒有偵測到 ESP32，確認 USB 有插好"
else
    for p in "${PORTS[@]}"; do
        R=$(cd "$PKG" && timeout 5 python3 - "$p" <<'EOF'
import sys, time
from car_v2v.serial_link import SerialLink
from car_v2v import protocol as P
info = []
l = SerialLink(sys.argv[1], on_frame=lambda t, p: info.append(P.EspInfo.unpack(p)) if t == P.USB_INFO else None,
               log=lambda m: None).start()
t = time.time()
while not info and time.time() - t < 3:
    time.sleep(0.05)
l.stop()
print(f'OK {P.mac_str(info[0].mac)} v{info[0].fw_ver}' if info else 'NOFW')
EOF
)
        case "$R" in
            OK*) ok "$p -> v2v 韌體正常（MAC ${R#OK })" ;;
            *)   fail "$p 沒有回應 v2v 韌體：還沒燒錄，或埠被其他程式佔用" ;;
        esac
    done
fi

echo "== 4. ROS 網路 =="
DID="${ROS_DOMAIN_ID:-$(grep -E '^[[:space:]]*export[[:space:]]+ROS_DOMAIN_ID=' ~/.bashrc 2>/dev/null | tail -1 | sed -E 's/.*=//')}"
warn "ROS_DOMAIN_ID = ${DID:-0}：兩台車必須不同（例如 98 / 99），否則同一個 WiFi 下 /drive /scan 會互相干擾"
CL="$WS/src/car_control/car_control/waypoints/waypoints.csv"
if [ -f "$CL" ]; then ok "中心線：$CL（md5 $(md5sum "$CL" | cut -c1-8)，兩台要一樣）"
else warn "找不到預設中心線 $CL；s/d/gap 不會送出（可用 centerline_csv:= 指定）"; fi

echo
if [ $FAILED -eq 0 ]; then
    echo "全部完成。測試：兩台同時執行  ros2 run car_v2v link_test"
    echo "正式使用：                    ros2 launch car_v2v v2v.launch.py"
    echo "（新開的終端要先 source $WS/install/setup.bash）"
else
    echo "有項目失敗，請依上面的訊息處理後重跑。"; exit 1
fi
