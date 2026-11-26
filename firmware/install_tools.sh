#!/bin/bash
# 安裝 ESP32-C3 燒錄工具（arduino-cli + ESP32 core），每台 Jetson 只要做一次。不需要 sudo。
set -e
CORE_VER="3.0.7"
BIN="$HOME/.local/bin"
mkdir -p "$BIN"
if ! command -v arduino-cli >/dev/null && [ ! -x "$BIN/arduino-cli" ]; then
    echo "下載 arduino-cli…"
    TMP=$(mktemp -d)
    case "$(uname -m)" in
        aarch64) ARCH=ARM64 ;;
        x86_64)  ARCH=64bit ;;
        *) echo "不支援的架構 $(uname -m)"; exit 1 ;;
    esac
    curl -sL -o "$TMP/acli.tgz" "https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_${ARCH}.tar.gz"
    tar xzf "$TMP/acli.tgz" -C "$TMP" arduino-cli
    mv "$TMP/arduino-cli" "$BIN/"
    rm -rf "$TMP"
fi
export PATH="$BIN:$PATH"
arduino-cli version
[ -f "$HOME/.arduino15/arduino-cli.yaml" ] || arduino-cli config init >/dev/null
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json 2>/dev/null || true
arduino-cli core update-index
if ! arduino-cli core list | grep -q "esp32:esp32 *$CORE_VER"; then
    echo "安裝 ESP32 core $CORE_VER（約 1GB，需要幾分鐘）…"
    arduino-cli core install "esp32:esp32@$CORE_VER"
fi
echo "完成。接著執行 ./flash.sh 燒錄韌體。"
