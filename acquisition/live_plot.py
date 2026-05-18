#!/usr/bin/env python3
"""
live_plot.py — Visualización en tiempo real de la IMU.

Uso:
    python live_plot.py --port /dev/cu.usbmodem1101
    python live_plot.py          # auto-detecta el puerto
"""

import argparse
import collections
import threading
import time

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


WINDOW_S   = 5       # segundos visibles en la gráfica
SAMPLE_HZ  = 100
MAXLEN     = WINDOW_S * SAMPLE_HZ   # muestras en el buffer

ACC_COLS   = ("ax", "ay", "az")
GYRO_COLS  = ("gx", "gy", "gz")
ACC_COLORS = ("#e74c3c", "#2ecc71", "#3498db")
GYRO_COLORS= ("#e67e22", "#1abc9c", "#9b59b6")


def auto_port():
    for p in serial.tools.list_ports.comports():
        if "usbmodem" in p.device or "Arduino" in (p.description or ""):
            return p.device
    return None


class IMUStream:
    def __init__(self, port: str, baud: int = 115200):
        self.ser  = serial.Serial(port, baud, timeout=1)
        time.sleep(2.0)
        self.buffers = {c: collections.deque([0.0] * MAXLEN, maxlen=MAXLEN)
                        for c in ACC_COLS + GYRO_COLS}
        self.ts = collections.deque([0.0] * MAXLEN, maxlen=MAXLEN)
        self._lock    = threading.Lock()
        self._running = True
        self._thread  = threading.Thread(target=self._read_loop, daemon=True)

    def start(self):
        self.ser.write(b"START\n")
        time.sleep(0.1)
        self._thread.start()

    def stop(self):
        self._running = False
        self.ser.write(b"STOP\n")
        self._thread.join(timeout=2)
        self.ser.close()

    def _read_loop(self):
        t0 = time.monotonic()
        while self._running:
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line or not line[0].isdigit():
                continue
            parts = line.split(",")
            if len(parts) != 9:
                continue
            try:
                vals = [float(x) for x in parts[3:9]]
            except ValueError:
                continue
            t_rel = time.monotonic() - t0
            with self._lock:
                self.ts.append(t_rel)
                for col, val in zip(ACC_COLS + GYRO_COLS, vals):
                    self.buffers[col].append(val)

    def snapshot(self):
        with self._lock:
            ts   = np.array(self.ts)
            data = {c: np.array(self.buffers[c]) for c in ACC_COLS + GYRO_COLS}
        return ts, data


def run(port: str):
    stream = IMUStream(port)
    stream.start()
    print(f"Conectado a {port}. Cierra la ventana para detener.")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    fig.suptitle(f"IMU en tiempo real — {port}", fontsize=11)

    lines_acc  = [ax1.plot([], [], color=c, lw=1.0, label=lbl)[0]
                  for lbl, c in zip(ACC_COLS, ACC_COLORS)]
    lines_gyro = [ax2.plot([], [], color=c, lw=1.0, label=lbl)[0]
                  for lbl, c in zip(GYRO_COLS, GYRO_COLORS)]

    ax1.set_ylabel("Aceleración (g)")
    ax1.set_ylim(-6, 6)
    ax1.axhline(0, color="gray", lw=0.4, ls="--")
    ax1.legend(loc="upper left", fontsize=8, ncol=3)
    ax1.grid(True, lw=0.3, alpha=0.5)

    ax2.set_ylabel("Vel. angular (°/s)")
    ax2.set_xlabel("Tiempo (s)")
    ax2.set_ylim(-500, 500)
    ax2.axhline(0, color="gray", lw=0.4, ls="--")
    ax2.legend(loc="upper left", fontsize=8, ncol=3)
    ax2.grid(True, lw=0.3, alpha=0.5)

    # Texto de ODR en tiempo real
    odr_text = ax1.text(0.02, 0.92, "", transform=ax1.transAxes,
                        fontsize=8, color="gray")

    prev_ts_last = [None]
    prev_count   = [0]

    def update(_frame):
        ts, data = stream.snapshot()
        if len(ts) == 0 or ts[-1] == 0:
            return []

        t_now  = ts[-1]
        t_min  = max(0.0, t_now - WINDOW_S)
        mask   = ts >= t_min
        t_plot = ts[mask] - t_now   # x en [-WINDOW_S, 0]

        for line, col in zip(lines_acc, ACC_COLS):
            line.set_data(t_plot, data[col][mask])
        for line, col in zip(lines_gyro, GYRO_COLS):
            line.set_data(t_plot, data[col][mask])

        ax1.set_xlim(-WINDOW_S, 0)
        ax2.set_xlim(-WINDOW_S, 0)

        # Estimar ODR cada ~1 s
        n_now = int(mask.sum())
        if prev_ts_last[0] is not None:
            dt_s = t_now - prev_ts_last[0]
            if dt_s >= 1.0:
                odr_est = (n_now - prev_count[0]) / dt_s
                odr_text.set_text(f"ODR ≈ {odr_est:.0f} Hz")
                prev_count[0] = n_now
                prev_ts_last[0] = t_now
        else:
            prev_ts_last[0] = t_now
            prev_count[0]   = n_now

        return lines_acc + lines_gyro + [odr_text]

    ani = animation.FuncAnimation(
        fig, update, interval=80, blit=True, cache_frame_data=False
    )

    try:
        plt.tight_layout()
        plt.show()
    finally:
        stream.stop()


def main():
    p = argparse.ArgumentParser(description="Visualización IMU en tiempo real")
    p.add_argument("--port", default="",
                   help="Puerto Serial. Auto-detecta si se omite.")
    args = p.parse_args()

    port = args.port or auto_port()
    if not port:
        raise SystemExit("No se encontró ningún Arduino. Usa --port /dev/cu.usbmodemXXXX")

    run(port)


if __name__ == "__main__":
    main()
