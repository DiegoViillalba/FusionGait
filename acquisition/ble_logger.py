#!/usr/bin/env python3
"""
ble_logger.py — Captura IMU de 1 a 3 Arduino Nano 33 BLE Sense Rev2 via BLE.

Requiere: pip install bleak pyserial pandas numpy
Requiere: Python 3.12+  (3.14 tiene bug en pyexpat que rompe pip/bleak)

Uso:
    # Un sensor (descubre automáticamente el primero que encuentre)
    python ble_logger.py --subject_id s001 --trial_id t001 \
        --output_dir ../data/raw/

    # Tres sensores (espera a que los 3 estén en BLE range)
    python ble_logger.py --nodes GaitNode_1 GaitNode_2 GaitNode_3 \
        --subject_id s001 --trial_id t001 --output_dir ../data/raw/

Notas sobre BLE vs USB:
    - BLE send rate: ~20 Hz (5 muestras por paquete → 100 Hz efectivos)
    - Puede haber pérdida de paquetes si el sujeto se aleja > ~8 m
    - packet_seq permite detectar paquetes perdidos
    - Ventaja: sin cables, movimiento libre
"""

import argparse
import asyncio
import csv
import struct
import sys
import time
from pathlib import Path

# bleak requiere Python 3.12+
try:
    from bleak import BleakScanner, BleakClient
except ImportError:
    sys.exit(
        "ERROR: bleak no está instalado.\n"
        "Necesitas Python 3.12:\n"
        "  brew install python@3.12\n"
        "  python3.12 -m venv .venv && source .venv/bin/activate\n"
        "  pip install bleak pyserial pandas numpy\n"
    )

# ─── UUIDs (deben coincidir con config.h del firmware) ────────────────────────
SERVICE_UUID  = "12340000-cafe-4b0b-a5b8-c2e8c9e9d1a0"
CHAR_IMU_UUID = "12340001-cafe-4b0b-a5b8-c2e8c9e9d1a0"
CHAR_CMD_UUID = "12340002-cafe-4b0b-a5b8-c2e8c9e9d1a0"
CHAR_STS_UUID = "12340003-cafe-4b0b-a5b8-c2e8c9e9d1a0"

BATCH_SIZE = 5   # muestras por paquete (debe coincidir con firmware)
PKT_SIZE   = 3 + BATCH_SIZE * 16   # 83 bytes

COLUMNS = [
    "timestamp_pc_ms", "timestamp_arduino_ms",
    "sensor_id", "placement",
    "ax", "ay", "az",
    "gx", "gy", "gz",
    "label", "trial_id", "subject_id",
]


# ─── Decodificación de paquete ────────────────────────────────────────────────

def decode_packet(data: bytes, placement: str,
                  trial_id: str, subject_id: str) -> list[dict]:
    """
    Decodifica un paquete BLE y devuelve una lista de filas CSV.

    Formato del paquete:
      Byte 0:     sensor_id (uint8)
      Bytes 1-2:  packet_seq (uint16 LE)
      Por cada muestra (16 bytes):
        uint32 timestamp_ms
        int16  ax×10000, ay×10000, az×10000
        int16  gx×10,    gy×10,    gz×10
    """
    if len(data) < PKT_SIZE:
        return []

    sensor_id  = data[0]
    packet_seq = struct.unpack_from("<H", data, 1)[0]
    ts_pc      = int(time.monotonic_ns() // 1_000_000)
    rows = []

    for i in range(BATCH_SIZE):
        off = 3 + i * 16
        ts_ard, ax16, ay16, az16, gx16, gy16, gz16 = \
            struct.unpack_from("<Ihhhhhh", data, off)

        rows.append({
            "timestamp_pc_ms":      ts_pc,
            "timestamp_arduino_ms": ts_ard,
            "sensor_id":            sensor_id,
            "placement":            placement,
            "ax":  round(ax16 / 10000.0, 4),
            "ay":  round(ay16 / 10000.0, 4),
            "az":  round(az16 / 10000.0, 4),
            "gx":  round(gx16 / 10.0,   2),
            "gy":  round(gy16 / 10.0,   2),
            "gz":  round(gz16 / 10.0,   2),
            "label":      "",
            "trial_id":   trial_id,
            "subject_id": subject_id,
        })

    return rows


# ─── Cliente BLE para un nodo ─────────────────────────────────────────────────

class BLENode:
    def __init__(self, device_name: str, trial_id: str, subject_id: str,
                 output_dir: Path):
        self.device_name = device_name
        self.trial_id    = trial_id
        self.subject_id  = subject_id
        self.output_dir  = output_dir
        self.placement   = "unknown"
        self.client      = None
        self._csv_file   = None
        self._writer     = None
        self.samples     = 0
        self.packets     = 0
        self.lost_pkts   = 0
        self._last_seq   = None

    async def connect(self):
        print(f"  [{self.device_name}] Buscando dispositivo BLE…")
        device = await BleakScanner.find_device_by_name(
            self.device_name, timeout=15.0
        )
        if device is None:
            raise RuntimeError(
                f"  [{self.device_name}] No encontrado. "
                f"¿Está encendido y en range BLE?"
            )
        self.client = BleakClient(device)
        await self.client.connect()
        print(f"  [{self.device_name}] Conectado ({device.address})")

        # Leer status: "sensor_id,placement,trial,subject"
        sts_raw = await self.client.read_gatt_char(CHAR_STS_UUID)
        sts = sts_raw.decode("utf-8", errors="replace").strip()
        parts = sts.split(",")
        if len(parts) >= 2:
            self.placement = parts[1]
        print(f"  [{self.device_name}] Status: {sts}")

    def _open_csv(self):
        fname = (self.output_dir /
                 f"{self.subject_id}_{self.trial_id}"
                 f"_sensor{self.device_name.split('_')[-1]}"
                 f"_{self.placement}.csv")
        self._csv_file = open(fname, "w", newline="", encoding="utf-8")
        self._writer   = csv.DictWriter(self._csv_file, fieldnames=COLUMNS)
        self._writer.writeheader()
        print(f"  [{self.device_name}] CSV → {fname}")
        return fname

    async def start_capture(self, trial_id: str, subject_id: str):
        self.trial_id   = trial_id
        self.subject_id = subject_id
        self._open_csv()

        # Configurar y arrancar
        for cmd in (f"SET_TRIAL:{trial_id}", f"SET_SUBJECT:{subject_id}", "START"):
            await self.client.write_gatt_char(
                CHAR_CMD_UUID, cmd.encode(), response=False
            )
            await asyncio.sleep(0.05)

        await self.client.start_notify(CHAR_IMU_UUID, self._on_notify)
        print(f"  [{self.device_name}] Capturando…")

    def _on_notify(self, _sender, data: bytearray):
        rows = decode_packet(
            bytes(data), self.placement,
            self.trial_id, self.subject_id
        )
        if not rows:
            return

        # Detectar paquetes perdidos por salto en packet_seq
        seq = struct.unpack_from("<H", data, 1)[0]
        if self._last_seq is not None:
            expected = (self._last_seq + 1) % 65536
            if seq != expected:
                gap = (seq - expected) % 65536
                self.lost_pkts += gap
        self._last_seq = seq

        for row in rows:
            self._writer.writerow(row)
        self.samples += len(rows)
        self.packets += 1

    async def stop_capture(self):
        await self.client.stop_notify(CHAR_IMU_UUID)
        await self.client.write_gatt_char(
            CHAR_CMD_UUID, b"STOP", response=False
        )
        if self._csv_file:
            self._csv_file.close()
        await self.client.disconnect()

    def print_stats(self):
        eff = 100.0 * self.packets / max(1, self.packets + self.lost_pkts)
        print(f"  [{self.device_name}] {self.samples:,} muestras | "
              f"{self.packets} paquetes | "
              f"perdidos: {self.lost_pkts} | "
              f"eficiencia: {eff:.1f}%")


# ─── Sesión multi-nodo ────────────────────────────────────────────────────────

async def run_session(node_names: list[str], subject_id: str,
                      trial_id: str, output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)

    nodes = [BLENode(name, trial_id, subject_id, output_dir)
             for name in node_names]

    # Conectar a todos en paralelo
    print(f"\nConectando a {len(nodes)} nodo(s) BLE…")
    await asyncio.gather(*[n.connect() for n in nodes])

    # Arrancar captura en todos
    print("\nIniciando captura…")
    await asyncio.gather(*[n.start_capture(trial_id, subject_id) for n in nodes])

    print(f"\nCapturando — {len(nodes)} sensor(es) activos.")
    print("Ctrl+C para detener.\n")

    t0 = time.monotonic()
    try:
        while True:
            await asyncio.sleep(2.0)
            elapsed = time.monotonic() - t0
            totals  = [n.samples for n in nodes]
            print(f"  t={elapsed:5.1f}s  muestras: "
                  + "  ".join(f"{n.device_name}={n.samples:,}"
                               for n in nodes))
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass

    # Detener
    print("\nDeteniendo…")
    await asyncio.gather(*[n.stop_capture() for n in nodes])

    print("\n─── Resumen ──────────────────────────────")
    for n in nodes:
        n.print_stats()
    print("──────────────────────────────────────────")


# ─── CLI ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="Logger BLE para Arduino Nano 33 BLE Sense")
    p.add_argument("--nodes", nargs="+",
                   default=["GaitNode_1", "GaitNode_2", "GaitNode_3"],
                   help="Nombres BLE de los nodos (default: los 3)")
    p.add_argument("--subject_id",  default="s001")
    p.add_argument("--trial_id",    default="t001")
    p.add_argument("--output_dir",  default="../data/raw/")
    args = p.parse_args()

    asyncio.run(run_session(
        node_names  = args.nodes,
        subject_id  = args.subject_id,
        trial_id    = args.trial_id,
        output_dir  = Path(args.output_dir),
    ))


if __name__ == "__main__":
    main()
