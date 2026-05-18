#!/usr/bin/env python3
"""
live_plot.py — Visualización en tiempo real de la IMU (pyqtgraph).

Dependencias:
    pip install pyqtgraph PyQt6 pyserial numpy

Uso:
    python live_plot.py                            # auto-detecta puerto
    python live_plot.py --port /dev/cu.usbmodem1201
    python live_plot.py --port /dev/cu.usbmodem1201 --window 8
"""

import argparse
import collections
import sys
import threading
import time

import numpy as np
import serial
import serial.tools.list_ports
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

SAMPLE_HZ = 100

ACC_COLS    = ("ax", "ay", "az")
GYRO_COLS   = ("gx", "gy", "gz")
ACC_COLORS  = ("#e74c3c", "#2ecc71", "#3498db")
GYRO_COLORS = ("#e67e22", "#1abc9c", "#9b59b6")


# ─── Puerto ───────────────────────────────────────────────────────────────────

def auto_port() -> str:
    for p in serial.tools.list_ports.comports():
        if "usbmodem" in p.device or "Arduino" in (p.description or ""):
            return p.device
    return ""


# ─── Handshake STATUS ─────────────────────────────────────────────────────────

def handshake(ser: serial.Serial, timeout_s: float = 10.0) -> bool:
    ser.reset_input_buffer()
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        ser.write(b"STATUS\n")
        t1 = time.monotonic()
        while time.monotonic() - t1 < 2.0:
            raw = ser.readline()
            if not raw:
                break
            line = raw.decode("utf-8", errors="replace").strip()
            if line.startswith("STATUS:") or line == "READY":
                print(f"  Arduino listo: {line}")
                return True
        remaining = timeout_s - (time.monotonic() - t0)
        if remaining > 1:
            print(f"  Sin respuesta, reintentando… ({remaining:.0f} s)")
    return False


# ─── Hilo de lectura IMU ───────────────────────────────────────────────────────

class IMUStream:
    def __init__(self, port: str, window_s: int):
        maxlen = window_s * SAMPLE_HZ * 3
        self.ser     = serial.Serial(port, 115200, timeout=2)
        self.times   = collections.deque(maxlen=maxlen)
        self.buffers = {c: collections.deque(maxlen=maxlen)
                        for c in ACC_COLS + GYRO_COLS}
        self._lock    = threading.Lock()
        self._running = False
        self._thread  = threading.Thread(target=self._read_loop, daemon=True)
        self._t0      = None
        self.total    = 0
        # ODR tracking
        self._odr_count = 0
        self._odr_ts    = time.monotonic()
        self.odr_est    = 0.0

    def start(self):
        time.sleep(0.3)
        if not handshake(self.ser):
            raise RuntimeError("Arduino no respondió. Verifica firmware y puerto.")
        self.ser.reset_input_buffer()
        self.ser.write(b"START\n")
        ack = self.ser.readline().decode("utf-8", errors="replace").strip()
        print(f"  {ack}")
        self._t0      = time.monotonic()
        self._running = True
        self._thread.start()

    def stop(self):
        self._running = False
        try:
            self.ser.write(b"STOP\n")
        except Exception:
            pass
        self._thread.join(timeout=2)
        try:
            self.ser.close()
        except Exception:
            pass

    def _read_loop(self):
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

            t_rel = time.monotonic() - self._t0
            with self._lock:
                self.times.append(t_rel)
                for col, val in zip(ACC_COLS + GYRO_COLS, vals):
                    self.buffers[col].append(val)
                self.total += 1
                self._odr_count += 1
                now = time.monotonic()
                dt  = now - self._odr_ts
                if dt >= 1.0:
                    self.odr_est    = self._odr_count / dt
                    self._odr_count = 0
                    self._odr_ts    = now

    def snapshot(self):
        with self._lock:
            return (
                np.array(self.times),
                {c: np.array(self.buffers[c]) for c in ACC_COLS + GYRO_COLS},
                self.total,
                self.odr_est,
            )


# ─── Ventana pyqtgraph ────────────────────────────────────────────────────────

def run(port: str, window_s: int):
    print(f"\nConectando a {port}…")
    stream = IMUStream(port, window_s)
    stream.start()
    print("Streaming activo. Cierra la ventana para detener.\n")

    app = pg.mkQApp("FusionGait — Live IMU")
    pg.setConfigOptions(antialias=True)

    win = pg.GraphicsLayoutWidget(title=f"FusionGait — {port}")
    win.resize(1200, 650)
    win.setBackground("#1e1e1e")

    # ── Título / stats ─────────────────────────────────────────────────────
    label = win.addLabel("Iniciando…", row=0, col=0,
                         color="#aaaaaa", size="9pt")

    # ── Plot acelerómetro ──────────────────────────────────────────────────
    p_acc = win.addPlot(row=1, col=0)
    p_acc.setLabel("left",   "Aceleración", units="g",
                   **{"color": "#cccccc", "font-size": "9pt"})
    p_acc.setYRange(-6, 6, padding=0)
    p_acc.setXRange(-window_s, 0, padding=0)
    p_acc.showGrid(x=True, y=True, alpha=0.25)
    p_acc.addLegend(offset=(10, 5), labelTextColor="#cccccc",
                    brush=pg.mkBrush("#2a2a2a"))
    p_acc.getAxis("bottom").setStyle(showValues=False)

    curves_acc = [
        p_acc.plot(pen=pg.mkPen(c, width=1.5), name=lbl)
        for lbl, c in zip(ACC_COLS, ACC_COLORS)
    ]

    # ── Plot giroscopio ────────────────────────────────────────────────────
    p_gyro = win.addPlot(row=2, col=0)
    p_gyro.setLabel("left",   "Vel. angular", units="°/s",
                    **{"color": "#cccccc", "font-size": "9pt"})
    p_gyro.setLabel("bottom", "Tiempo",       units="s",
                    **{"color": "#cccccc", "font-size": "9pt"})
    p_gyro.setYRange(-600, 600, padding=0)
    p_gyro.setXRange(-window_s, 0, padding=0)
    p_gyro.showGrid(x=True, y=True, alpha=0.25)
    p_gyro.addLegend(offset=(10, 5), labelTextColor="#cccccc",
                     brush=pg.mkBrush("#2a2a2a"))
    p_gyro.setXLink(p_acc)

    curves_gyro = [
        p_gyro.plot(pen=pg.mkPen(c, width=1.5), name=lbl)
        for lbl, c in zip(GYRO_COLS, GYRO_COLORS)
    ]

    win.ci.layout.setRowStretchFactor(0, 1)   # label: poco espacio
    win.ci.layout.setRowStretchFactor(1, 10)  # acc:   más espacio
    win.ci.layout.setRowStretchFactor(2, 10)  # gyro:  más espacio

    # ── Líneas de referencia en y=0 ────────────────────────────────────────
    for plot in (p_acc, p_gyro):
        plot.addItem(pg.InfiniteLine(
            pos=0, angle=0,
            pen=pg.mkPen("#555555", width=0.8, style=QtCore.Qt.PenStyle.DashLine)
        ))

    # ── Timer de actualización ─────────────────────────────────────────────
    def update():
        ts, data, total, odr = stream.snapshot()
        if len(ts) < 2:
            return

        t_now = ts[-1]
        mask  = ts >= (t_now - window_s)
        t_x   = ts[mask] - t_now

        for curve, col in zip(curves_acc, ACC_COLS):
            curve.setData(t_x, data[col][mask])
        for curve, col in zip(curves_gyro, GYRO_COLS):
            curve.setData(t_x, data[col][mask])

        acc_n = np.sqrt(
            data["ax"][mask]**2 + data["ay"][mask]**2 + data["az"][mask]**2
        )
        mean_g = acc_n.mean() if len(acc_n) else 0.0
        label.setText(
            f"ODR: <b>{odr:.0f} Hz</b>  │  "
            f"acc_norm: <b>{mean_g:.2f} g</b>  │  "
            f"muestras: <b>{total:,}</b>  │  "
            f"puerto: {port}"
        )

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(80)    # ~12 fps de refresco

    win.show()

    try:
        app.exec()
    finally:
        stream.stop()
        print(f"\nDetenido. Total muestras recibidas: {stream.total:,}")


# ─── CLI ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="Visualización IMU en tiempo real")
    p.add_argument("--port",   default="",  help="Puerto Serial (auto-detecta si se omite)")
    p.add_argument("--window", type=int, default=6, help="Segundos visibles (default 6)")
    args = p.parse_args()

    port = args.port or auto_port()
    if not port:
        sys.exit("No se encontró ningún Arduino. Usa --port /dev/cu.usbmodemXXXX")

    run(port, args.window)


if __name__ == "__main__":
    main()
