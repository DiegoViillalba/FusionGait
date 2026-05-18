#!/usr/bin/env python3
"""
serial_logger.py — Captura datos de uno o tres Arduino Nano 33 BLE Sense por USB Serial.

Modo un sensor:
    python serial_logger.py --port /dev/cu.usbmodem1101 \
        --sensor_id 3 --placement ankle \
        --subject_id s001 --trial_id t001 \
        --output ../data/raw/s001_t001_sensor3_ankle.csv

Modo tres sensores (--multi):
    python serial_logger.py --multi \
        --config ../config/acquisition_config.yaml \
        --subject_id s001 --trial_id t001 \
        --output_dir ../data/raw/
"""

import argparse
import csv
import sys
import threading
import time
from pathlib import Path

import serial
import serial.tools.list_ports

COLUMNS = [
    "timestamp_pc_ms",
    "timestamp_arduino_ms",
    "sensor_id",
    "placement",
    "ax", "ay", "az",
    "gx", "gy", "gz",
    "label",
    "trial_id",
    "subject_id",
]


# ─── Utilidades de puerto ──────────────────────────────────────────────────────

def list_arduino_ports():
    """Devuelve una lista de puertos con Arduino Nano 33 BLE Sense conectados."""
    ports = []
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "arduino" in desc or "2341" in hwid or "usbmodem" in p.device:
            ports.append(p.device)
    return ports


def open_serial(port: str, baud: int = 115200, timeout: float = 3.0) -> serial.Serial:
    ser = serial.Serial(port, baud, timeout=timeout)
    time.sleep(2.0)   # esperar reset del Arduino tras abrir el puerto
    return ser


def send_cmd(ser: serial.Serial, cmd: str):
    ser.write((cmd + "\n").encode())
    time.sleep(0.05)


def wait_ready(ser: serial.Serial, timeout_s: float = 8.0) -> bool:
    """Espera el mensaje READY del Arduino."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                print(f"  [Arduino] {line}")
            if line == "READY":
                return True
    return False


# ─── Hilo de lectura para un sensor ───────────────────────────────────────────

class SensorReader(threading.Thread):
    def __init__(self, ser: serial.Serial, writer: csv.DictWriter,
                 trial_id: str, subject_id: str, label: str,
                 stats: dict, lock: threading.Lock):
        super().__init__(daemon=True)
        self.ser       = ser
        self.writer    = writer
        self.trial_id  = trial_id
        self.subject_id = subject_id
        self.label     = label
        self.stats     = stats
        self.lock      = lock
        self.running   = True

    def run(self):
        while self.running:
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                break

            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()

            # Ignorar líneas de control
            if not line or not line[0].isdigit():
                if line:
                    print(f"  [Arduino] {line}")
                continue

            parts = line.split(",")
            if len(parts) != 9:
                self.stats["corrupt"] += 1
                continue

            ts_pc = int(time.monotonic_ns() // 1_000_000)
            row = {
                "timestamp_pc_ms":      ts_pc,
                "timestamp_arduino_ms": parts[0],
                "sensor_id":            parts[1],
                "placement":            parts[2],
                "ax":  parts[3],
                "ay":  parts[4],
                "az":  parts[5],
                "gx":  parts[6],
                "gy":  parts[7],
                "gz":  parts[8],
                "label":      self.label,
                "trial_id":   self.trial_id,
                "subject_id": self.subject_id,
            }
            with self.lock:
                self.writer.writerow(row)
            self.stats["samples"] += 1

    def stop(self):
        self.running = False


# ─── Modo un sensor ───────────────────────────────────────────────────────────

def run_single(args):
    port = args.port
    if not port:
        found = list_arduino_ports()
        if not found:
            sys.exit("No se encontró ningún Arduino. Conecta el cable y reintenta.")
        port = found[0]
        print(f"Puerto auto-detectado: {port}")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    print(f"\nConectando a {port} @ 115200 baud...")
    ser = open_serial(port)

    if not wait_ready(ser):
        sys.exit("Timeout esperando READY del Arduino. Verifica el firmware.")

    send_cmd(ser, f"SET_TRIAL:{args.trial_id}")
    send_cmd(ser, f"SET_SUBJECT:{args.subject_id}")
    send_cmd(ser, "SYNC")
    sync_line = ser.readline().decode("utf-8", errors="replace").strip()
    print(f"  [Sync] {sync_line}")

    stats = {"samples": 0, "corrupt": 0}
    lock  = threading.Lock()

    with open(output, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        writer.writeheader()

        reader = SensorReader(ser, writer, args.trial_id, args.subject_id,
                              args.label, stats, lock)
        reader.start()

        send_cmd(ser, "START")
        print(f"\nCapturando → {output}")
        print("Presiona ENTER para detener...\n")

        try:
            input()
        except KeyboardInterrupt:
            pass

        reader.stop()
        send_cmd(ser, "STOP")
        reader.join(timeout=2.0)

    ser.close()
    _print_qc(output, stats)


# ─── Modo multi-sensor (3 Arduinos) ───────────────────────────────────────────

def run_multi(args):
    try:
        import yaml
    except ImportError:
        sys.exit("Instala PyYAML: pip install pyyaml")

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    sensor_cfgs = cfg["sensors"]
    baud        = cfg.get("serial_baud", 115200)
    output_dir  = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    serials, writers, files, readers, stats = {}, {}, {}, {}, {}
    lock = threading.Lock()

    print("\nConectando a los 3 sensores...")
    for s in sensor_cfgs:
        sid   = s["sensor_id"]
        port  = s["port"]
        place = s["placement"]

        fname = output_dir / (
            f"{args.subject_id}_{args.trial_id}_sensor{sid}_{place}.csv"
        )
        fobj = open(fname, "w", newline="", encoding="utf-8")
        w    = csv.DictWriter(fobj, fieldnames=COLUMNS)
        w.writeheader()

        ser = open_serial(port, baud)
        print(f"  Sensor {sid} ({place}) → {port}")

        files[sid]  = fobj
        writers[sid] = w
        serials[sid] = ser
        stats[sid]   = {"samples": 0, "corrupt": 0}

    # Esperar READY de todos
    print("Esperando READY de los 3 Arduinos...")
    for sid, ser in serials.items():
        if not wait_ready(ser):
            sys.exit(f"Timeout en sensor {sid}. Verifica el firmware.")

    # Configurar todos
    for sid, ser in serials.items():
        s_cfg = next(s for s in sensor_cfgs if s["sensor_id"] == sid)
        send_cmd(ser, f"SET_TRIAL:{args.trial_id}")
        send_cmd(ser, f"SET_SUBJECT:{args.subject_id}")

    # SYNC simultáneo
    t_sync_pc = int(time.monotonic_ns() // 1_000_000)
    for ser in serials.values():
        send_cmd(ser, "SYNC")
    print(f"  SYNC enviado @ PC ms={t_sync_pc}")

    # Iniciar lectores
    for sid, ser in serials.items():
        r = SensorReader(ser, writers[sid], args.trial_id, args.subject_id,
                         "", stats[sid], lock)
        readers[sid] = r
        r.start()

    # START simultáneo
    for ser in serials.values():
        ser.write(b"START\n")
    print(f"\nCapturando en {output_dir}")
    print("Presiona ENTER para detener...\n")

    try:
        input()
    except KeyboardInterrupt:
        pass

    for ser in serials.values():
        ser.write(b"STOP\n")
    for r in readers.values():
        r.stop()
        r.join(timeout=2.0)
    for f in files.values():
        f.close()
    for ser in serials.values():
        ser.close()

    print("\n─── Resumen ─────────────────────────")
    for sid, st in stats.items():
        rate = st["samples"] / max(1, st["samples"] + st["corrupt"]) * 100
        print(f"  Sensor {sid}: {st['samples']} muestras | "
              f"corrupt: {st['corrupt']} | ok: {rate:.1f}%")


# ─── QC rápido al terminar ────────────────────────────────────────────────────

def _print_qc(csv_path: Path, stats: dict):
    print("\n─── Resumen ─────────────────────────")
    print(f"  Archivo:  {csv_path}")
    print(f"  Muestras: {stats['samples']}")
    print(f"  Corrupt:  {stats['corrupt']}")

    try:
        import pandas as pd
        import numpy as np
        df = pd.read_csv(csv_path)
        dt = df["timestamp_pc_ms"].diff().dropna()
        odr = 1000.0 / dt.mean()
        gaps = (dt > 25).sum()
        print(f"  ODR est.: {odr:.1f} Hz  (target 100 Hz)")
        print(f"  Gaps>25ms: {gaps}")
        df["acc_norm"] = np.sqrt(df.ax**2 + df.ay**2 + df.az**2)
        print(f"  acc_norm μ: {df.acc_norm.mean():.3f} g  (reposo ≈ 1.0 g)")
        if gaps > stats["samples"] * 0.02:
            print("  ⚠  Muchos gaps — revisar conexión USB o velocidad del PC")
        if abs(odr - 100) > 10:
            print(f"  ⚠  ODR fuera de rango — verificar firmware SAMPLE_HZ")
    except ImportError:
        pass   # pandas no disponible, saltar QC

    print("─────────────────────────────────────")


# ─── CLI ──────────────────────────────────────────────────────────────────────

def build_parser():
    p = argparse.ArgumentParser(
        description="Logger Serial para Arduino Nano 33 BLE Sense"
    )
    p.add_argument("--multi",       action="store_true",
                   help="Modo multi-sensor (lee config YAML)")
    p.add_argument("--config",      default="../config/acquisition_config.yaml",
                   help="YAML con puertos de los 3 sensores (solo --multi)")

    # Modo single
    p.add_argument("--port",       default="",
                   help="Puerto Serial, e.g. /dev/cu.usbmodem1101. "
                        "Auto-detecta si se omite.")
    p.add_argument("--sensor_id",  type=int, default=3)
    p.add_argument("--placement",  default="ankle")
    p.add_argument("--output",     default="../data/raw/capture.csv",
                   help="Ruta del CSV de salida (modo single)")

    # Modo multi
    p.add_argument("--output_dir", default="../data/raw/",
                   help="Directorio de CSVs de salida (modo --multi)")

    # Comunes
    p.add_argument("--subject_id", default="s001")
    p.add_argument("--trial_id",   default="t001")
    p.add_argument("--label",      default="",
                   help="Label fijo (vacío = etiquetar después)")
    return p


def main():
    args = build_parser().parse_args()
    if args.multi:
        run_multi(args)
    else:
        run_single(args)


if __name__ == "__main__":
    main()
