#!/usr/bin/env python3
"""
live_plot.py — Visualización IMU en tiempo real. Tres modos:

  Single sensor USB:
      python live_plot.py --port /dev/cu.usbmodem1201

  Multi-sensor USB (3 columnas, lee config/acquisition_config.yaml):
      python live_plot.py --multi

  Hub BLE inalámbrico (3 columnas, conecta a GaitHub):
      python live_plot.py --ble-hub
      python live_plot.py --ble-hub --hub-name GaitHub

Dependencias:
    pip install pyqtgraph PyQt6 pyserial numpy pyyaml bleak
"""

import argparse
import asyncio
import collections
import struct
import sys
import threading
import time

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

# ─── UUIDs hub BLE (deben coincidir con data_logger_ble_host/config.h) ────────
HUB_IMU_UUIDS = [
    "56780001-cafe-4b0b-a5b8-c2e8c9e9d1a0",   # sensor 1 / pelvis
    "56780002-cafe-4b0b-a5b8-c2e8c9e9d1a0",   # sensor 2 / thigh
    "56780003-cafe-4b0b-a5b8-c2e8c9e9d1a0",   # sensor 3 / ankle
]
HUB_CMD_UUID  = "56780004-cafe-4b0b-a5b8-c2e8c9e9d1a0"
HUB_STS_UUID  = "56780005-cafe-4b0b-a5b8-c2e8c9e9d1a0"
HUB_BATCH     = 5
HUB_PKT_SIZE  = 3 + HUB_BATCH * 16


# ─── Config YAML ──────────────────────────────────────────────────────────────

def load_config(path: str) -> list[dict]:
    import yaml
    with open(path) as f:
        return yaml.safe_load(f)["sensors"]


def auto_port() -> str:
    for p in serial.tools.list_ports.comports():
        if "usbmodem" in p.device or "Arduino" in (p.description or ""):
            return p.device
    return ""


# ─── Handshake STATUS (USB) ───────────────────────────────────────────────────

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
        print(f"  {tag}Sin respuesta, reintentando…")
    return False


# ─── Buffer genérico (hilo seguro) ────────────────────────────────────────────

class NodeBuffer:
    """Buffer de datos IMU de un sensor. Thread-safe."""
    def __init__(self, window_s: int):
        maxlen = window_s * SAMPLE_HZ * 3
        self.times   = collections.deque(maxlen=maxlen)
        self.buffers = {c: collections.deque(maxlen=maxlen)
                        for c in ACC_COLS + GYRO_COLS}
        self.total    = 0
        self._odr_cnt = 0
        self._odr_ts  = time.monotonic()
        self.odr_est  = 0.0
        self._t0      = None

    def push_row(self, row: dict):
        if self._t0 is None:
            self._t0 = time.monotonic()
        t_rel = time.monotonic() - self._t0
        self.times.append(t_rel)
        for c in ACC_COLS + GYRO_COLS:
            self.buffers[c].append(row[c])
        self.total    += 1
        self._odr_cnt += 1
        now = time.monotonic()
        dt  = now - self._odr_ts
        if dt >= 1.0:
            self.odr_est  = self._odr_cnt / dt
            self._odr_cnt = 0
            self._odr_ts  = now

    def snapshot(self):
        return (
            np.array(self.times),
            {c: np.array(self.buffers[c]) for c in ACC_COLS + GYRO_COLS},
            self.total,
            self.odr_est,
        )


# ─── Stream USB (un sensor) ───────────────────────────────────────────────────

class IMUStream:
    def __init__(self, port: str, placement: str, sensor_id: int, window_s: int):
        self.port      = port
        self.placement = placement
        self.sensor_id = sensor_id
        self._buf      = NodeBuffer(window_s)
        self.ser       = serial.Serial(port, 115200, timeout=2)
        self._lock     = threading.Lock()
        self._running  = False
        self._thread   = threading.Thread(target=self._read_loop, daemon=True)

    def start(self):
        time.sleep(0.3)
        tag = f"S{self.sensor_id}/{self.placement}"
        if not handshake(self.ser, label=tag):
            raise RuntimeError(f"{tag}: Arduino no respondió ({self.port})")
        self.ser.reset_input_buffer()
        self.ser.write(b"START\n")
        ack = self.ser.readline().decode("utf-8", errors="replace").strip()
        print(f"  [{tag}] {ack}")
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
            row = {c: v for c, v in zip(ACC_COLS + GYRO_COLS, vals)}
            with self._lock:
                self._buf.push_row(row)

    def snapshot(self):
        with self._lock:
            return self._buf.snapshot()

    @property
    def total(self):
        return self._buf.total


# ─── Stream BLE hub ───────────────────────────────────────────────────────────

class BLEHubStream:
    """Conecta al hub GaitHub y expone snapshot(i) para cada uno de los 3 sensores."""

    PLACEMENTS = ["pelvis", "thigh", "ankle"]

    def __init__(self, hub_name: str, window_s: int):
        self.hub_name  = hub_name
        self._lock     = threading.Lock()
        self.nodes     = [NodeBuffer(window_s) for _ in range(3)]
        self.connected = False
        self.status    = "Buscando hub…"
        self._running  = False
        self._loop     = None
        self._thread   = None

    def start(self):
        self._running = True
        self._loop    = asyncio.new_event_loop()
        self._thread  = threading.Thread(
            target=lambda: self._loop.run_until_complete(self._ble_run()),
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        self._running = False
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)

    async def _ble_run(self):
        try:
            from bleak import BleakScanner, BleakClient
        except ImportError:
            self.status = "ERROR: bleak no instalado (pip install bleak)"
            return

        self.status = f"Buscando '{self.hub_name}'…"
        print(f"\n{self.status}")
        device = None
        try:
            devices = await BleakScanner.discover(timeout=10.0)
            for d in devices:
                if d.name == self.hub_name:
                    device = d
                    break
            if device is None:
                # second attempt with longer timeout
                devices = await BleakScanner.discover(timeout=15.0)
                for d in devices:
                    if d.name == self.hub_name:
                        device = d
                        break
        except Exception as e:
            self.status = f"ERROR escáner: {e}"
            return

        if device is None:
            self.status = f"'{self.hub_name}' no encontrado"
            print(f"ERROR: {self.status}")
            print("  Verifica que el hub esté encendido y con LED azul.")
            return

        def decode(data: bytes) -> list[dict]:
            if len(data) < HUB_PKT_SIZE:
                return []
            rows = []
            for i in range(HUB_BATCH):
                off = 3 + i * 16
                _, ax16, ay16, az16, gx16, gy16, gz16 = \
                    struct.unpack_from("<Ihhhhhh", data, off)
                rows.append({
                    "ax": ax16 / 10000.0, "ay": ay16 / 10000.0, "az": az16 / 10000.0,
                    "gx": gx16 / 10.0,   "gy": gy16 / 10.0,   "gz": gz16 / 10.0,
                })
            return rows

        def make_cb(idx: int):
            def _cb(_sender, data: bytearray):
                rows = decode(bytes(data))
                with self._lock:
                    for row in rows:
                        self.nodes[idx].push_row(row)
            return _cb

        try:
            async with BleakClient(device) as client:
                self.connected = True
                self.status    = f"Conectado a {self.hub_name} ({device.address})"
                print(f"\n{self.status}")

                for i, uuid in enumerate(HUB_IMU_UUIDS):
                    await client.start_notify(uuid, make_cb(i))

                await client.write_gatt_char(
                    HUB_CMD_UUID, b"START", response=False
                )
                print("Streaming activo. Cierra la ventana para detener.\n")

                while self._running:
                    await asyncio.sleep(0.1)

                await client.write_gatt_char(
                    HUB_CMD_UUID, b"STOP", response=False
                )
                for uuid in HUB_IMU_UUIDS:
                    try:
                        await client.stop_notify(uuid)
                    except Exception:
                        pass
        except Exception as e:
            self.status = f"ERROR BLE: {e}"
            print(f"ERROR BLE: {e}")

    def snapshot(self, idx: int):
        with self._lock:
            return self.nodes[idx].snapshot()


# ─── UI helpers ───────────────────────────────────────────────────────────────

def _make_plot(win, row, col, ylabel, yunits, yrange,
               show_x: bool, link_x=None, title: str = "") -> pg.PlotItem:
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


def _build_multi_window(win, n_sensors: int, sensor_labels: list[str],
                        window_s: int):
    """Construye la ventana de N columnas y devuelve (plots_acc, plots_gyro, curves_acc, curves_gyro)."""
    plots_acc, plots_gyro = [], []
    curves_acc, curves_gyro = [], []

    for col in range(n_sensors):
        title_str = (f"<span style='color:#dddddd'>"
                     f"{sensor_labels[col]}</span>")

        link_acc  = plots_acc[0]  if col > 0 else None
        p_acc = _make_plot(win, row=1, col=col,
                           ylabel="Aceleración", yunits="g", yrange=(-6, 6),
                           show_x=False, link_x=link_acc, title=title_str)
        p_acc.setXRange(-window_s, 0, padding=0)
        ca = [p_acc.plot(pen=pg.mkPen(c, width=1.5), name=lbl)
              for lbl, c in zip(ACC_COLS, ACC_COLORS)]
        plots_acc.append(p_acc)
        curves_acc.append(ca)

        link_gyro = plots_gyro[0] if col > 0 else None
        p_gyro = _make_plot(win, row=2, col=col,
                            ylabel="Vel. angular", yunits="°/s", yrange=(-600, 600),
                            show_x=True, link_x=link_gyro)
        p_gyro.setXLink(p_acc)
        p_gyro.setXRange(-window_s, 0, padding=0)
        cg = [p_gyro.plot(pen=pg.mkPen(c, width=1.5), name=lbl)
              for lbl, c in zip(GYRO_COLS, GYRO_COLORS)]
        plots_gyro.append(p_gyro)
        curves_gyro.append(cg)

    win.ci.layout.setRowStretchFactor(0, 1)
    win.ci.layout.setRowStretchFactor(1, 10)
    win.ci.layout.setRowStretchFactor(2, 10)

    return plots_acc, plots_gyro, curves_acc, curves_gyro


# ─── Modo single USB ──────────────────────────────────────────────────────────

def run_single(port: str, window_s: int):
    print(f"\nConectando a {port}…")
    stream = IMUStream(port, placement="?", sensor_id=0, window_s=window_s)
    stream.start()
    print("Streaming activo. Cierra la ventana para detener.\n")

    app = pg.mkQApp("FusionGait — Live IMU")
    pg.setConfigOptions(antialias=True)
    win = pg.GraphicsLayoutWidget(title=f"FusionGait — {port}")
    win.resize(1100, 650)
    win.setBackground("#1e1e1e")

    label = win.addLabel("Iniciando…", row=0, col=0,
                         color="#aaaaaa", size="9pt")
    p_acc  = _make_plot(win, row=1, col=0, ylabel="Aceleración", yunits="g",
                        yrange=(-6, 6), show_x=False)
    p_acc.setXRange(-window_s, 0, padding=0)
    p_gyro = _make_plot(win, row=2, col=0, ylabel="Vel. angular", yunits="°/s",
                        yrange=(-600, 600), show_x=True, link_x=p_acc)
    p_gyro.setXRange(-window_s, 0, padding=0)
    win.ci.layout.setRowStretchFactor(0, 1)
    win.ci.layout.setRowStretchFactor(1, 10)
    win.ci.layout.setRowStretchFactor(2, 10)

    curves_acc  = [p_acc.plot(pen=pg.mkPen(c, width=1.5), name=lbl)
                   for lbl, c in zip(ACC_COLS, ACC_COLORS)]
    curves_gyro = [p_gyro.plot(pen=pg.mkPen(c, width=1.5), name=lbl)
                   for lbl, c in zip(GYRO_COLS, GYRO_COLORS)]

    def update():
        ts, data, total, odr = stream.snapshot()
        if len(ts) < 2:
            return
        t_now = ts[-1]; mask = ts >= (t_now - window_s); t_x = ts[mask] - t_now
        for curve, col in zip(curves_acc, ACC_COLS):
            curve.setData(t_x, data[col][mask])
        for curve, col in zip(curves_gyro, GYRO_COLS):
            curve.setData(t_x, data[col][mask])
        acc_n  = np.sqrt(sum(data[c][mask]**2 for c in ACC_COLS))
        mean_g = acc_n.mean() if len(acc_n) else 0.0
        label.setText(f"ODR: <b>{odr:.0f} Hz</b>  │  "
                      f"acc_norm: <b>{mean_g:.2f} g</b>  │  "
                      f"muestras: <b>{total:,}</b>  │  puerto: {port}")

    timer = QtCore.QTimer(); timer.timeout.connect(update); timer.start(80)
    win.show()
    try:
        app.exec()
    finally:
        stream.stop()
        print(f"\nDetenido. Total muestras: {stream.total:,}")


# ─── Modo multi USB ───────────────────────────────────────────────────────────

def run_multi(sensors: list[dict], window_s: int):
    n = len(sensors)
    print(f"\nConectando a {n} sensores USB…")
    streams = [IMUStream(s["port"], s["placement"], s["sensor_id"], window_s)
               for s in sensors]
    for st in streams:
        st.start()
    print("Streaming activo. Cierra la ventana para detener.\n")

    app = pg.mkQApp("FusionGait — Live Multi-IMU")
    pg.setConfigOptions(antialias=True)
    win = pg.GraphicsLayoutWidget(title="FusionGait — Multi-sensor USB")
    win.resize(420 * n, 700); win.setBackground("#1e1e1e")

    stats_label = win.addLabel("Iniciando…", row=0, col=0, colspan=n,
                               color="#aaaaaa", size="8pt")

    labels = [f"Sensor {st.sensor_id} · {st.placement}" for st in streams]
    _, _, curves_acc, curves_gyro = _build_multi_window(win, n, labels, window_s)

    def update():
        parts = []
        for i, st in enumerate(streams):
            ts, data, total, odr = st.snapshot()
            if len(ts) < 2:
                continue
            t_now = ts[-1]; mask = ts >= (t_now - window_s); t_x = ts[mask] - t_now
            for curve, col in zip(curves_acc[i], ACC_COLS):
                curve.setData(t_x, data[col][mask])
            for curve, col in zip(curves_gyro[i], GYRO_COLS):
                curve.setData(t_x, data[col][mask])
            acc_n  = np.sqrt(sum(data[c][mask]**2 for c in ACC_COLS))
            mean_g = acc_n.mean() if len(acc_n) else 0.0
            parts.append(f"<b>S{st.sensor_id}/{st.placement}</b>: "
                         f"{odr:.0f} Hz · {mean_g:.2f} g · {total:,} muestras")
        stats_label.setText("    │    ".join(parts))

    timer = QtCore.QTimer(); timer.timeout.connect(update); timer.start(80)
    win.show()
    try:
        app.exec()
    finally:
        for st in streams:
            st.stop()
        for st in streams:
            print(f"  S{st.sensor_id}/{st.placement}: {st.total:,} muestras")


# ─── Modo BLE hub ─────────────────────────────────────────────────────────────

def run_ble_hub(hub_name: str, window_s: int):
    stream = BLEHubStream(hub_name, window_s)
    stream.start()

    app = pg.mkQApp("FusionGait — Live BLE Hub")
    pg.setConfigOptions(antialias=True)
    win = pg.GraphicsLayoutWidget(title=f"FusionGait — {hub_name} (BLE)")
    win.resize(1260, 700); win.setBackground("#1e1e1e")

    stats_label = win.addLabel("Conectando al hub…", row=0, col=0, colspan=3,
                               color="#aaaaaa", size="8pt")

    labels = [f"Sensor {i+1} · {p}"
              for i, p in enumerate(BLEHubStream.PLACEMENTS)]
    _, _, curves_acc, curves_gyro = _build_multi_window(win, 3, labels, window_s)

    def update():
        if not stream.connected:
            stats_label.setText(f"<b>{stream.status}</b>")
            return
        parts = []
        for i, placement in enumerate(BLEHubStream.PLACEMENTS):
            ts, data, total, odr = stream.snapshot(i)
            if len(ts) < 2:
                continue
            t_now = ts[-1]; mask = ts >= (t_now - window_s); t_x = ts[mask] - t_now
            for curve, col in zip(curves_acc[i], ACC_COLS):
                curve.setData(t_x, data[col][mask])
            for curve, col in zip(curves_gyro[i], GYRO_COLS):
                curve.setData(t_x, data[col][mask])
            acc_n  = np.sqrt(sum(data[c][mask]**2 for c in ACC_COLS))
            mean_g = acc_n.mean() if len(acc_n) else 0.0
            parts.append(f"<b>S{i+1}/{placement}</b>: "
                         f"{odr:.0f} Hz · {mean_g:.2f} g · {total:,} muestras")
        if parts:
            stats_label.setText(f"[BLE Hub]    │    ".join(parts))

    timer = QtCore.QTimer(); timer.timeout.connect(update); timer.start(80)
    win.show()
    try:
        app.exec()
    finally:
        stream.stop()
        for i, p in enumerate(BLEHubStream.PLACEMENTS):
            print(f"  S{i+1}/{p}: {stream.nodes[i].total:,} muestras")


# ─── CLI ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="Visualización IMU en tiempo real")
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--multi",    action="store_true",
                      help="3 sensores USB (lee config/acquisition_config.yaml)")
    mode.add_argument("--ble-hub",  action="store_true",
                      help="3 sensores BLE vía hub GaitHub (inalámbrico)")
    p.add_argument("--hub-name", default="GaitHub",
                   help="Nombre BLE del hub (default: GaitHub)")
    p.add_argument("--config", default="config/acquisition_config.yaml",
                   help="YAML de configuración para modo --multi")
    p.add_argument("--port",   default="",
                   help="Puerto USB para modo single-sensor")
    p.add_argument("--window", type=int, default=6,
                   help="Segundos visibles (default: 6)")
    args = p.parse_args()

    if args.ble_hub:
        run_ble_hub(args.hub_name, args.window)
    elif args.multi:
        from pathlib import Path
        cfg_path = Path(args.config)
        if not cfg_path.exists():
            sys.exit(f"No se encontró {cfg_path}")
        run_multi(load_config(str(cfg_path)), args.window)
    else:
        port = args.port or auto_port()
        if not port:
            sys.exit("No se encontró ningún Arduino. Usa --port /dev/cu.usbmodemXXXX")
        run_single(port, args.window)


if __name__ == "__main__":
    main()
