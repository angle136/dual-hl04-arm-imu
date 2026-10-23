#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo ""
echo "Select board:"
echo "  1. damiao_h7"
echo "  2. dji_c"
echo ""

read -r -p "Enter choice (1-2): " BOARD_CHOICE

case "$BOARD_CHOICE" in
  1) BOARD="damiao_h7" ;;
  2) BOARD="dji_c" ;;
  *)
    echo "[ERROR] Invalid choice"
    exit 1
    ;;
esac

echo ""
echo "Select probe:"
echo "  1. stlink"
echo "  2. daplink"
echo "  3. jlink"
echo ""

read -r -p "Enter choice (1-3): " PROBE_CHOICE

case "$PROBE_CHOICE" in
  1) PROBE="stlink" ;;
  2) PROBE="daplink" ;;
  3) PROBE="jlink" ;;
  *)
    echo "[ERROR] Invalid choice"
    exit 1
    ;;
esac

echo ""
echo "Flashing: [${PROBE}] ${BOARD}"
echo ""

# Call flash_interactive.sh
exec "${SCRIPT_DIR}/flash_interactive.sh" "${BOARD}" "${PROBE}"
