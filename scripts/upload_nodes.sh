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
PORT_1="/dev/cu.usbmodem11201"  # nodo 1 — pelvis
PORT_2="/dev/cu.usbmodem11301"   # nodo 2 — thigh
PORT_3="/dev/cu.usbmodem11401"   # nodo 3 — ankle
# ──────────────────────────────────────────────────────────────────────────────

FQBN="arduino:mbed_nano:nano33ble"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MODE="${1:-wired}"

if [ "$MODE" = "ble" ]; then
  SKETCH="$ROOT/firmware/data_logger_ble"
elif [ "$MODE" = "hub" ]; then
  SKETCH_HUB="$ROOT/firmware/data_logger_ble_host"
  SKETCH="$ROOT/firmware/data_logger_ble"   # esclavos 2 y 3
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

  # Actualizar BLE_DEVICE_NAME en modo BLE o hub (esclavos)
  if [ "$MODE" = "ble" ] || [ "$MODE" = "hub" ]; then
    sed -i '' "s/^#define BLE_DEVICE_NAME.*/#define BLE_DEVICE_NAME  \"GaitNode_$sid\"/" "$cfg"
  fi

  echo "  Compilando..."
  arduino-cli compile --fqbn "$FQBN" "$SKETCH" --log-level warn

  echo "  Subiendo a ${port}..."
  arduino-cli upload --fqbn "$FQBN" --port "${port}" "$SKETCH" --log-level warn

  echo "  ✓  Nodo $sid listo"
  echo ""

  # Pausa entre uploads para que el Arduino re-enumere el USB
  sleep 4
}

if [ "$MODE" = "hub" ]; then
  # ── Modo hub: nodo 1 = hub, nodos 2/3 = esclavos BLE ─────────────────────
  echo "  Modo HUB: nodo 1 → data_logger_ble_host"
  echo "            nodos 2/3 → data_logger_ble (esclavos)"
  echo ""

  # Subir firmware hub al nodo 1
  echo "──────────────────────────────────────────"
  echo "  HUB (nodo 1 / pelvis / $PORT_1)"
  echo "──────────────────────────────────────────"
  echo "  Compilando hub..."
  arduino-cli compile --fqbn "$FQBN" "$SKETCH_HUB" --log-level warn
  echo "  Subiendo a ${PORT_1}..."
  arduino-cli upload --fqbn "$FQBN" --port "${PORT_1}" "$SKETCH_HUB" --log-level warn
  echo "  ✓  Hub listo"
  echo ""
  sleep 4

  # Subir firmware esclavo a nodos 2 y 3
  upload_node 2 "thigh" "$PORT_2"
  upload_node 3 "ankle" "$PORT_3"

  echo "══════════════════════════════════════════"
  echo "  Hub mode listo."
  echo "  Nodo 1 = GaitHub (hub BLE)"
  echo "  Nodo 2 = GaitNode_2 (thigh, esclavo)"
  echo "  Nodo 3 = GaitNode_3 (ankle, esclavo)"
  echo "══════════════════════════════════════════"
else
  upload_node 1 "pelvis" "$PORT_1"
  upload_node 2 "thigh"  "$PORT_2"
  upload_node 3 "ankle"  "$PORT_3"

  echo "══════════════════════════════════════════"
  echo "  Todos los nodos actualizados."
  echo "  config.h queda configurado para nodo 3."
  echo "══════════════════════════════════════════"
fi
