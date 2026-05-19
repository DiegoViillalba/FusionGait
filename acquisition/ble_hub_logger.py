#!/usr/bin/env python3
"""
ble_hub_logger.py — Captura de 3 sensores IMU vía nodo hub GaitHub.

El hub (nodo 1/pelvis) actúa como central BLE hacia los nodos 2 y 3,
y expone los 3 streams al PC en un único servicio GATT.
El PC solo necesita conectarse a "GaitHub".

Produce los mismos CSV que ble_logger.py / serial_logger.py.

Requiere: pip install bleak

Uso:
    python ble_hub_logger.py --subject_id s001 --trial_id t001 \\
        --output_dir ../data/raw/
    python ble_hub_logger.py --hub GaitHub --subject_id s001 --trial_id t001
"""

import argparse
import asyncio
import csv
import struct
import sys
import time
from pathlib import Path

try:
    from bleak import BleakScanner, BleakClient
except ImportError:
    sys.exit(
        "ERROR: bleak no instalado.\n"
        "  pip install bleak\n"
        "  (requiere Python 3.12+ recomendado)"
    )

# ─── UUIDs del hub (deben coincidir con data_logger_ble_host/config.h) ────────
HUB_SERVICE_UUID = "56780000-cafe-4b0b-a5b8-c2e8c9e9d1a0"
HUB_IMU1_UUID    = "56780001-cafe-4b0b-a5b8-c2e8c9e9d1a0"   # pelvis
HUB_IMU2_UUID    = "56780002-cafe-4b0b-a5b8-c2e8c9e9d1a0"   # thigh
HUB_IMU3_UUID    = "56780003-cafe-4b0b-a5b8-c2e8c9e9d1a0"   # ankle
HUB_CMD_UUID     = "56780004-cafe-4b0b-a5b8-c2e8c9e9d1a0"
HUB_STS_UUID     = "56780005-cafe-4b0b-a5b8-c2e8c9e9d1a0"

BATCH_SIZE  = 5
PKT_SIZE    = 3 + BATCH_SIZE * 16   # 83 bytes

SENSORS = {
    1: {"placement": "pelvis", "imu_uuid": HUB_IMU1_UUID},
    2: {"placement": "thigh",  "imu_uuid": HUB_IMU2_UUID},
    3: {"placement": "ankle",  "imu_uuid": HUB_IMU3_UUID},
}

COLUMNS = [
    "timestamp_pc_ms", "timestamp_arduino_ms",
    "sensor_id", "placement",
    "ax", "ay", "az",
    "gx", "gy", "gz",
    "label", "trial_id", "subject_id",
]


# ─── Decodificación de paquete IMU (mismo formato que ble_logger.py) ──────────

def decode_packet(data: bytes, placement: str,
                  trial_id: str, subject_id: str) -> list[dict]:
    if len(data) < PKT_SIZE:
        return []
    sensor_id = data[0]
    ts_pc     = int(time.monotonic_ns() // 1_000_000)
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


# ─── Sesión ───────────────────────────────────────────────────────────────────

async def run_session(hub_name: str, subject_id: str,
                      trial_id: str, output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"\nBuscando hub '{hub_name}'…")
    device = None
    for timeout in (10.0, 15.0):
        devices = await BleakScanner.discover(timeout=timeout)
        for d in devices:
            if d.name == hub_name:
                device = d
                break
        if device is not None:
            break
    if device is None:
        sys.exit(
            f"ERROR: '{hub_name}' no encontrado.\n"
            "  Asegúrate de que:\n"
            "  1. El hub está encendido (powerbank conectada)\n"
            "  2. Los 2 esclavos (GaitNode_2, GaitNode_3) están encendidos\n"
            "  3. El hub ha terminado de conectarse (LED azul fijo)"
        )

    # Abrir CSVs — uno por sensor
    csv_files: dict[int, object] = {}
    writers:   dict[int, csv.DictWriter] = {}
    for sid, info in SENSORS.items():
        fname = (output_dir /
                 f"{subject_id}_{trial_id}_sensor{sid}_{info['placement']}.csv")
        f = open(fname, "w", newline="", encoding="utf-8")
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        csv_files[sid] = f
        writers[sid]   = w
        print(f"  CSV sensor {sid} → {fname}")

    samples     = {sid: 0 for sid in SENSORS}
    lost_pkts   = {sid: 0 for sid in SENSORS}
    last_seq    = {sid: None for sid in SENSORS}

    def make_callback(sid: int):
        placement = SENSORS[sid]["placement"]
        def _cb(_sender, data: bytearray):
            rows = decode_packet(bytes(data), placement, trial_id, subject_id)
            if not rows:
                return
            # Detectar paquetes perdidos
            seq = struct.unpack_from("<H", data, 1)[0]
            if last_seq[sid] is not None:
                expected = (last_seq[sid] + 1) % 65536
                if seq != expected:
                    lost_pkts[sid] += (seq - expected) % 65536
            last_seq[sid] = seq
            for row in rows:
                writers[sid].writerow(row)
            samples[sid] += len(rows)
        return _cb

    async with BleakClient(device) as client:
        print(f"\nConectado a {hub_name} ({device.address})")

        # Leer status del hub
        try:
            sts = (await client.read_gatt_char(HUB_STS_UUID))\
                      .decode("utf-8", errors="replace").strip()
            print(f"  Status: {sts}")
            if "s2=0" in sts or "s3=0" in sts:
                print("  ADVERTENCIA: el hub no tiene todos los esclavos conectados.")
                print("  Espera a que el LED del hub sea azul fijo antes de continuar.")
        except Exception:
            pass

        # Suscribir a los 3 streams IMU
        for sid, info in SENSORS.items():
            await client.start_notify(info["imu_uuid"], make_callback(sid))
        print("  Suscrito a los 3 streams IMU")

        # Enviar metadatos + START
        for cmd in (f"SET_TRIAL:{trial_id}", f"SET_SUBJECT:{subject_id}", "START"):
            await client.write_gatt_char(HUB_CMD_UUID, cmd.encode(), response=False)
            await asyncio.sleep(0.05)

        print(f"\nCapturando — Ctrl+C para detener.\n")
        t0 = time.monotonic()
        try:
            while True:
                await asyncio.sleep(2.0)
                elapsed = time.monotonic() - t0
                print(f"  t={elapsed:5.1f}s  "
                      + "  ".join(
                          f"S{sid}={samples[sid]:,}" for sid in SENSORS
                      ))
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass

        # Detener
        print("\nDeteniendo…")
        await client.write_gatt_char(HUB_CMD_UUID, b"STOP", response=False)
        for info in SENSORS.values():
            try:
                await client.stop_notify(info["imu_uuid"])
            except Exception:
                pass

    for f in csv_files.values():
        f.close()

    print("\n─── Resumen ──────────────────────────────")
    for sid, info in SENSORS.items():
        eff = 100.0 * samples[sid] / max(1, samples[sid] + lost_pkts[sid] * BATCH_SIZE)
        print(f"  S{sid}/{info['placement']:6s}: "
              f"{samples[sid]:,} muestras | "
              f"pkts perdidos: {lost_pkts[sid]} | "
              f"eficiencia: {eff:.1f}%")
    print("──────────────────────────────────────────")


# ─── CLI ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description="Logger BLE via hub GaitHub para FusionGait"
    )
    p.add_argument("--hub",        default="GaitHub",
                   help="Nombre BLE del hub (default: GaitHub)")
    p.add_argument("--subject_id", default="s001")
    p.add_argument("--trial_id",   default="t001")
    p.add_argument("--output_dir", default="../data/raw/")
    args = p.parse_args()

    asyncio.run(run_session(
        hub_name   = args.hub,
        subject_id = args.subject_id,
        trial_id   = args.trial_id,
        output_dir = Path(args.output_dir),
    ))


if __name__ == "__main__":
    main()
