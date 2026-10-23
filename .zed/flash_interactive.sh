#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BOARD="${1:-}"
PROBE="${2:-}"
BUILD_TYPE="Debug"

if [[ -z "$BOARD" || -z "$PROBE" ]]; then
  echo "[ERROR] Usage: $0 <board> <probe>"
  exit 1
fi

case "$BOARD" in
  damiao_h7)
    TARGET="stm32h7x"
    JLINK_DEV="STM32H743BI"
    ;;
  dji_c)
    TARGET="stm32f4x"
    JLINK_DEV="STM32F407IG"
    ;;
  *)
    echo "[ERROR] Unknown board: $BOARD"
    exit 1
    ;;
esac

ELF_PATH="${WORKSPACE_ROOT}/build/${BOARD}/${BUILD_TYPE}/base.elf"
if [[ ! -f "$ELF_PATH" ]]; then
  echo "[ERROR] ELF file not found: $ELF_PATH"
  exit 1
fi

echo "Flashing: [${PROBE}] ${BOARD}"

case "$PROBE" in
  jlink)
    # jlink路径
    JLINK_DIR="${JLINK_DIR:-/opt/SEGGER/JLink}"
    JLINK_EXE="${JLINK_DIR}/JLinkExe"
    if [[ ! -x "$JLINK_EXE" ]]; then
      echo "[ERROR] JLink executable not found: $JLINK_EXE"
      exit 1
    fi
    JLINK_SCRIPT="$(mktemp /tmp/vscode_flash_${BOARD}.jlink.XXXXXX)"
    cat > "$JLINK_SCRIPT" <<JLINK_EOF
loadfile "$ELF_PATH"
r
g
exit
JLINK_EOF
    "$JLINK_EXE" -device "$JLINK_DEV" -if SWD -speed 4000 -autoconnect 1 -CommanderScript "$JLINK_SCRIPT"
    RET=$?
    rm -f "$JLINK_SCRIPT"
    if [[ $RET -ne 0 ]]; then
      echo "[ERROR] J-Link flash failed"
      exit $RET
    fi
    ;;
  stlink|daplink)
    if [[ "$PROBE" == "stlink" ]]; then
      IFACE="interface/stlink.cfg"
    else
      IFACE="interface/cmsis-dap.cfg"
    fi
    if ! command -v openocd >/dev/null 2>&1; then
      echo "[ERROR] openocd not found in PATH"
      exit 1
    fi
    openocd -f "$IFACE" -f "target/${TARGET}.cfg" -c "program build/${BOARD}/${BUILD_TYPE}/base.elf verify reset exit"
    ;;
  *)
    echo "[ERROR] Unknown probe: $PROBE"
    exit 1
    ;;
esac

echo "Flash complete!"
exit 0
