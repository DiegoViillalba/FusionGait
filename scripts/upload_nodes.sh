#!/usr/bin/env bash
# upload_nodes.sh — Compila y sube firmware a los 3 Arduino Nano 33 BLE Sense.
#
# Uso:
#   bash scripts/upload_nodes.sh wired   # firmware Serial USB (data_logger_node)
#   bash scripts/upload_nodes.sh ble     # firmware BLE (data_logger_ble)
#
# Antes de correr: edita los puertos con los valores de: arduino-cli board list

set -euo pipefail

# ─── EDITAR SEGÚN TU MÁQUINA ──────────────────────────────────────────────────
PORT_1="/dev/cu.usbmodem11401"  # nodo 1 — pelvis
PORT_2="/dev/cu.usbmodem1201"   # nodo 2 — thigh
PORT_3="/dev/cu.usbmodem1301"   # nodo 3 — ankle
# ──────────────────────────────────────────────────────────────────────────────

FQBN="arduino:mbed_nano:nano33ble"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MODE="${1:-wired}"

if [ "$MODE" = "ble" ]; then
  SKETCH="$ROOT/firmware/data_logger_ble"
else
  SKETCH="$ROOT/firmware/data_logger_node"
fi

echo "══════════════════════════════════════════"
echo "  FusionGait — upload_nodes.sh  [$MODE]"
echo "══════════════════════════════════════════"
echo "  Sketch: $SKETCH"
echo ""

upload_node() {
  local sid="$1"
  local placement="$2"
  local port="$3"
  local cfg="$SKETCH/config.h"

  echo "──────────────────────────────────────────"
  echo "  Nodo $sid  placement=$placement  puerto=$port"
  echo "──────────────────────────────────────────"

  # Parchear config.h (macOS sed requiere -i '')
  sed -i '' "s/^#define SENSOR_ID.*/#define SENSOR_ID  $sid/" "$cfg"
  sed -i '' "s/^#define PLACEMENT.*/#define PLACEMENT  \"$placement\"/" "$cfg"

  # Si es BLE, también actualizar BLE_DEVICE_NAME
  if [ "$MODE" = "ble" ]; then
    sed -i '' "s/^#define BLE_DEVICE_NAME.*/#define BLE_DEVICE_NAME  \"GaitNode_$sid\"/" "$cfg"
  fi

  echo "  Compilando…"
  arduino-cli compile --fqbn "$FQBN" "$SKETCH" --log-level warn

  echo "  Subiendo a $port…"
  arduino-cli upload --fqbn "$FQBN" --port "$port" "$SKETCH" --log-level warn

  echo "  ✓  Nodo $sid listo"
  echo ""

  # Pausa entre uploads para que el Arduino re-enumere el USB
  sleep 4
}

upload_node 1 "pelvis" "$PORT_1"
upload_node 2 "thigh"  "$PORT_2"
upload_node 3 "ankle"  "$PORT_3"

# Dejar config.h con los valores del nodo 3 (último subido)
# para que la próxima compilación manual sea coherente
echo "══════════════════════════════════════════"
echo "  Todos los nodos actualizados."
echo "  config.h queda configurado para nodo 3."
echo "══════════════════════════════════════════"
