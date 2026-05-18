#!/usr/bin/env python3
"""
live_plot.py — Visualización en tiempo real: 1 sensor o 3 sensores en paralelo.

Dependencias:
    pip install pyqtgraph PyQt6 pyserial numpy pyyaml

Uso (un sensor):
    python live_plot.py --port /dev/cu.usbmodem1201

Uso (multi-sensor, lee config/acquisition_config.yaml):
    python live_plot.py --multi
    python live_plot.py --multi --config config/acquisition_config.yaml
    python live_plot.py --multi --window 8
"""

import argparse
import collections
import sys
import threading
import time
from pathlib import Path

import numpy as np
import serial
import serial.tools.list_ports
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

SAMPLE_HZ   = 100
ACC_COLS    = ("ax", "ay", "az")
GYRO_COLS   = ("gx", "gy", "gz")
ACC_COLORS  = ("#e74c3c", "#2ecc71", "#3498db")
GYRO_COLORS = ("#e67e22", "#1abc9c", "#9b59b6")


# ─── Config ───────────────────────────────────────────────────────────────────

def load_config(path: str) -> list[dict]:
    """Returns list of {sensor_id, placement, port}."""
    import yaml
    with open(path) as f:
        cfg = yaml.safe_load(f)
    return cfg["sensors"]


def auto_port() -> str:
    for p in serial.tools.list_ports.comports():
        if "usbmodem" in p.device or "Arduino" in (p.description or ""):
            return p.device
    return ""


# ─── Handshake ────────────────────────────────────────────────────────────────

def handshake(ser: serial.Serial, label: str = "", timeout_s: float = 10.0) -> bool:
    tag = f"[{label}] " if label else ""
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
                print(f"  {tag}Listo: {line}")
                return True
        remaining = timeout_s - (time.monotonic() - t0)
        if remaining > 1:
            print(f"  {tag}Sin respuesta, reintentando… ({remaining:.0f} s)")
    return False


# ─── Hilo de lectura por sensor ───────────────────────────────────────────────

class IMUStream:
    def __init__(self, port: str, placement: str, sensor_id: int, window_s: int):
        maxlen = window_s * SAMPLE_HZ * 3
        self.port      = port
        self.placement = placement
        self.sensor_id = sensor_id
        self.ser       = serial.Serial(port, 115200, timeout=2)
        self.times     = collections.deque(maxlen=maxlen)
        self.buffers   = {c: collections.deque(maxlen=maxlen)
                          for c in ACC_COLS + GYRO_COLS}
        self._lock     = threading.Lock()
        self._running  = False
        self._thread   = threading.Thread(target=self._read_loop, daemon=True)
        self._t0       = None
        self.total     = 0
        self._odr_cnt  = 0
        self._odr_ts   = time.monotonic()
        self.odr_est   = 0.0

    def start(self):
        time.sleep(0.3)
        tag = f"sensor{self.sensor_id}/{self.placement}"
        if not handshake(self.ser, label=tag):
            raise RuntimeError(f"{tag}: Arduino no respondió ({self.port})")
        self.ser.reset_input_buffer()
        self.ser.write(b"START\n")
        ack = self.ser.readline().decode("utf-8", errors="replace").strip()
        print(f"  [{tag}] {ack}")
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
                self.total   += 1
                self._odr_cnt += 1
                now = time.monotonic()
                dt  = now - self._odr_ts
                if dt >= 1.0:
                    self.odr_est  = self._odr_cnt / dt
                    self._odr_cnt = 0
                    self._odr_ts  = now

    def snapshot(self):
        with self._lock:
            return (
                np.array(self.times),
                {c: np.array(self.buffers[c]) for c in ACC_COLS + GYRO_COLS},
                self.total,
                self.odr_est,
            )


# ─── UI helpers ───────────────────────────────────────────────────────────────

def _make_plot(win, row, col, ylabel, yunits, yrange, show_x: bool,
               link_x=None, title: str = "") -> pg.PlotItem:
    p = win.addPlot(row=row, col=col, title=title)
    p.setLabel("left", ylabel, units=yunits,
               **{"color": "#cccccc", "font-size": "8pt"})
    if show_x:
        p.setLabel("bottom", "Tiempo", units="s",
                   **{"color": "#cccccc", "font-size": "8pt"})
    else:
        p.getAxis("bottom").setStyle(showValues=False)
    p.setYRange(*yrange, padding=0)
    p.showGrid(x=True, y=True, alpha=0.25)
    p.addLegend(offset=(5, 5), labelTextColor="#cccccc",
                brush=pg.mkBrush("#2a2a2a"), labelTextSize="7pt")
    p.addItem(pg.InfiniteLine(
        pos=0, angle=0,
        pen=pg.mkPen("#555555", width=0.7,
                     style=QtCore.Qt.PenStyle.DashLine)
    ))
    if link_x is not None:
        p.setXLink(link_x)
    return p


# ─── Ventana multi-sensor ─────────────────────────────────────────────────────

def run_multi(sensors: list[dict], window_s: int):
    n = len(sensors)
    print(f"\nConectando a {n} sensores…")

    streams = []
    for s in sensors:
        st = IMUStream(s["port"], s["placement"], s["sensor_id"], window_s)
        streams.append(st)

    # Arrancar todos (secuencial para no saturar USB)
    for st in streams:
        st.start()
    print("Streaming activo en todos los nodos. Cierra la ventana para detener.\n")

    app = pg.mkQApp("FusionGait — Live Multi-IMU")
    pg.setConfigOptions(antialias=True)

    win = pg.GraphicsLayoutWidget(title="FusionGait — Multi-sensor Live")
    win.resize(420 * n, 700)
    win.setBackground("#1e1e1e")

    # Fila 0: label de estadísticas (span all columns)
    stats_label = win.addLabel("Iniciando…", row=0, col=0, colspan=n,
                               color="#aaaaaa", size="8pt")

    # Filas 1-2: acc y gyro por columna/sensor
    plots_acc  = []
    plots_gyro = []
    curves_acc  = []   # list of lists
    curves_gyro = []

    for col, st in enumerate(streams):
        title_str = f"<span style='color:#dddddd'>Sensor {st.sensor_id} · {st.placement}</span>"

        # Acc plot — row 1
        link_acc = plots_acc[0] if col > 0 else None
        p_acc = _make_plot(win, row=1, col=col,
                           ylabel="Aceleración", yunits="g", yrange=(-6, 6),
                           show_x=False, link_x=link_acc, title=title_str)
        p_acc.setXRange(-window_s, 0, padding=0)
        ca = [p_acc.plot(pen=pg.mkPen(c, width=1.5), name=lbl)
              for lbl, c in zip(ACC_COLS, ACC_COLORS)]
        plots_acc.append(p_acc)
        curves_acc.append(ca)

        # Gyro plot — row 2
        link_gyro = plots_gyro[0] if col > 0 else None
        p_gyro = _make_plot(win, row=2, col=col,
                            ylabel="Vel. angular", yunits="°/s", yrange=(-600, 600),
                            show_x=True, link_x=link_gyro)
        p_gyro.setXRange(-window_s, 0, padding=0)
        p_gyro.setXLink(p_acc)
        cg = [p_gyro.plot(pen=pg.mkPen(c, width=1.5), name=lbl)
              for lbl, c in zip(GYRO_COLS, GYRO_COLORS)]
        plots_gyro.append(p_gyro)
        curves_gyro.append(cg)

    win.ci.layout.setRowStretchFactor(0, 1)
    win.ci.layout.setRowStretchFactor(1, 10)
    win.ci.layout.setRowStretchFactor(2, 10)

    def update():
        parts = []
        for i, st in enumerate(streams):
            ts, data, total, odr = st.snapshot()
            if len(ts) < 2:
                continue
            t_now = ts[-1]
            mask  = ts >= (t_now - window_s)
            t_x   = ts[mask] - t_now

            for curve, col in zip(curves_acc[i], ACC_COLS):
                curve.setData(t_x, data[col][mask])
            for curve, col in zip(curves_gyro[i], GYRO_COLS):
                curve.setData(t_x, data[col][mask])

            acc_n  = np.sqrt(sum(data[c][mask]**2 for c in ACC_COLS))
            mean_g = acc_n.mean() if len(acc_n) else 0.0
            parts.append(
                f"<b>S{st.sensor_id}/{st.placement}</b>: "
                f"{odr:.0f} Hz · {mean_g:.2f} g · {total:,} muestras"
            )

        stats_label.setText("    │    ".join(parts))

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(80)

    win.show()
    try:
        app.exec()
    finally:
        for st in streams:
            st.stop()
        for st in streams:
            print(f"  S{st.sensor_id}/{st.placement}: {st.total:,} muestras")


# ─── Ventana single-sensor (backward compat) ──────────────────────────────────

def run_single(port: str, window_s: int):
    print(f"\nConectando a {port}…")
    stream = IMUStream(port, placement="?", sensor_id=0, window_s=window_s)
    stream.start()
    print("Streaming activo. Cierra la ventana para detener.\n")

    app = pg.mkQApp("FusionGait — Live IMU")
    pg.setConfigOptions(antialias=True)

    win = pg.GraphicsLayoutWidget(title=f"FusionGait — {port}")
    win.resize(1200, 650)
    win.setBackground("#1e1e1e")

    label = win.addLabel("Iniciando…", row=0, col=0,
                         color="#aaaaaa", size="9pt")
    p_acc  = _make_plot(win, row=1, col=0, ylabel="Aceleración", yunits="g",
                        yrange=(-6, 6), show_x=False)
    p_acc.setXRange(-window_s, 0, padding=0)
    p_gyro = _make_plot(win, row=2, col=0, ylabel="Vel. angular", yunits="°/s",
                        yrange=(-600, 600), show_x=True, link_x=p_acc)
    p_gyro.setXRange(-window_s, 0, padding=0)

    curves_acc  = [p_acc.plot(pen=pg.mkPen(c, width=1.5), name=lbl)
                   for lbl, c in zip(ACC_COLS, ACC_COLORS)]
    curves_gyro = [p_gyro.plot(pen=pg.mkPen(c, width=1.5), name=lbl)
                   for lbl, c in zip(GYRO_COLS, GYRO_COLORS)]

    win.ci.layout.setRowStretchFactor(0, 1)
    win.ci.layout.setRowStretchFactor(1, 10)
    win.ci.layout.setRowStretchFactor(2, 10)

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
        acc_n  = np.sqrt(sum(data[c][mask]**2 for c in ACC_COLS))
        mean_g = acc_n.mean() if len(acc_n) else 0.0
        label.setText(
            f"ODR: <b>{odr:.0f} Hz</b>  │  "
            f"acc_norm: <b>{mean_g:.2f} g</b>  │  "
            f"muestras: <b>{total:,}</b>  │  "
            f"puerto: {port}"
        )

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(80)
    win.show()
    try:
        app.exec()
    finally:
        stream.stop()
        print(f"\nDetenido. Total muestras: {stream.total:,}")


# ─── CLI ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="Visualización IMU en tiempo real")
    p.add_argument("--multi",  action="store_true",
                   help="Modo multi-sensor (lee config/acquisition_config.yaml)")
    p.add_argument("--config", default="config/acquisition_config.yaml",
                   help="YAML de configuración (default: config/acquisition_config.yaml)")
    p.add_argument("--port",   default="",
                   help="Puerto único (modo single-sensor)")
    p.add_argument("--window", type=int, default=6,
                   help="Segundos visibles (default 6)")
    args = p.parse_args()

    if args.multi:
        cfg_path = Path(args.config)
        if not cfg_path.exists():
            sys.exit(f"No se encontró {cfg_path}")
        sensors = load_config(str(cfg_path))
        run_multi(sensors, args.window)
    else:
        port = args.port or auto_port()
        if not port:
            sys.exit("No se encontró ningún Arduino. Usa --port /dev/cu.usbmodemXXXX")
        run_single(port, args.window)


if __name__ == "__main__":
    main()
