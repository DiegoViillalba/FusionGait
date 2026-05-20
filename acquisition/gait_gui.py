#!/usr/bin/env python3
"""
gait_gui.py — Interfaz de adquisición de marcha por fases.

Modos de operación
──────────────────
• USB directo   : conecta N Arduinos por USB independientes.
• Hub BLE       : 1 Arduino hub (data_logger_hub) por USB retransmite
                  datos de hasta 2 sensores BLE inalámbricos.
• Mac BLE       : la Mac actúa directamente como hub BLE (bleak);
                  no se necesita ningún Arduino conectado por USB.

Uso:
    python3.11 acquisition/gait_gui.py

Teclas:
    1  →  Loading (Heel Strike)
    2  →  Mid Stance
    3  →  Terminal Stance
    4  →  Swing
    Space → Toggle grabar / detener

Requiere:
    pip install pyqtgraph PyQt6 pyserial numpy bleak
"""

import asyncio
import collections
import csv
import struct
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
import serial.tools.list_ports
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui

# ── Constantes ────────────────────────────────────────────────────────────────
SAMPLE_HZ = 100
BAUD_RATE  = 115200
WINDOW_S   = 6
DATA_DIR   = Path(__file__).parent.parent / "data" / "raw"

PHASES = [
    ("loading",   "1 · Loading HS",       "#e74c3c"),
    ("midstance", "2 · Mid Stance",        "#f39c12"),
    ("terminal",  "3 · Terminal Stance",   "#2ecc71"),
    ("swing",     "4 · Swing",             "#3498db"),
]

PLACEMENT_OPTIONS  = ["cadera", "pierna", "peroné", "tobillo"]
PLACEMENT_DEFAULTS = ["cadera", "pierna", "peroné", "tobillo"]  # default por sensor (hasta 4)
MAX_SENSORS        = 4   # máximo de Arduinos soportados por la GUI

COLUMNS = [
    "timestamp_pc_ms", "timestamp_arduino_ms",
    "sensor_id", "placement", "phase",
    "ax", "ay", "az", "gx", "gy", "gz",
    "trial_id", "subject_id",
]

ACC_COLS    = ("ax", "ay", "az")
GYRO_COLS   = ("gx", "gy", "gz")
ACC_COLORS  = ("#e74c3c", "#2ecc71", "#3498db")
GYRO_COLORS = ("#e67e22", "#1abc9c", "#9b59b6")


# ── Un sensor USB ─────────────────────────────────────────────────────────────

class SerialSensor:
    """Gestiona un sensor USB: lectura en hilo, buffer para live plot y CSV."""

    def __init__(self, port: str, sensor_id: int, placement: str):
        self.port      = port
        self.sensor_id = sensor_id
        self.placement = placement

        maxlen = WINDOW_S * SAMPLE_HZ * 2
        self._times = collections.deque(maxlen=maxlen)
        self._bufs  = {c: collections.deque(maxlen=maxlen)
                       for c in ACC_COLS + GYRO_COLS}
        self.total    = 0
        self.odr_est  = 0.0
        self._t0      = None
        self._odr_ts  = time.monotonic()
        self._odr_cnt = 0

        self._lock       = threading.Lock()
        self._ser        = None
        self._thread     = None
        self._running    = False
        self.thread_alive = False

        self._writer          = None
        self._csv_file        = None
        self.recording        = False
        self.samples_recorded = 0
        self._phase           = "none"
        self._trial_id        = ""
        self._subject_id      = ""

    # ── Conexión ──────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        try:
            self._ser = serial.Serial(self.port, BAUD_RATE, timeout=2.0)
            time.sleep(0.3)
            self._ser.reset_input_buffer()
            return True
        except Exception as e:
            print(f"  ERR {self.port}: {e}")
            return False

    def handshake(self, timeout_s: float = 10.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            self._ser.write(b"STATUS\n")
            t1 = time.monotonic()
            while time.monotonic() - t1 < 2.0:
                raw = self._ser.readline()
                if not raw:
                    break
                line = raw.decode("utf-8", errors="replace").strip()
                if line.startswith("STATUS:") or line == "READY":
                    return True
        return False

    def start_stream(self):
        self._ser.reset_input_buffer()   # limpiar basura acumulada durante handshake
        time.sleep(0.05)
        self._ser.write(b"START\n")
        print(f"  [S{self.sensor_id}/{self.placement}] START enviado a {self.port}")
        self._running = True
        self.thread_alive = True
        self._diag_lines = 0            # para imprimir las primeras líneas recibidas
        self._thread  = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop_stream(self):
        self._running = False
        try:
            self._ser.write(b"STOP\n")
        except Exception:
            pass

    # ── CSV ───────────────────────────────────────────────────────────────────

    def open_csv(self, subject_id: str, trial_id: str, out_dir: Path) -> Path:
        self._subject_id = subject_id
        self._trial_id   = trial_id
        ts    = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = out_dir / f"{subject_id}_{trial_id}_s{self.sensor_id}_{self.placement}_{ts}.csv"
        self._csv_file = open(fname, "w", newline="", encoding="utf-8")
        self._writer   = csv.DictWriter(self._csv_file, fieldnames=COLUMNS)
        self._writer.writeheader()
        self.samples_recorded = 0
        self.recording = True
        return fname

    def close_csv(self):
        self.recording = False
        if self._csv_file:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file = None
            self._writer   = None

    def set_phase(self, phase: str):
        self._phase = phase

    # ── Lectura ───────────────────────────────────────────────────────────────

    def _read_loop(self):
        try:
            self._read_loop_inner()
        except Exception as e:
            print(f"  [S{self.sensor_id}/{self.placement}] ERROR en hilo: {e}")
        finally:
            self.thread_alive = False
            print(f"  [S{self.sensor_id}/{self.placement}] Hilo de lectura terminado")

    def _read_loop_inner(self):
        while self._running:
            try:
                raw = self._ser.readline()
            except serial.SerialException as e:
                print(f"  [S{self.sensor_id}] SerialException: {e}")
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line or not line[0].isdigit():
                if line:
                    print(f"  [S{self.sensor_id}/{self.placement}] ctrl: {line}")
                continue

            # Imprimir las primeras 3 líneas de datos para confirmar formato
            if self._diag_lines < 3:
                print(f"  [S{self.sensor_id}/{self.placement}] datos: {line[:80]}")
                self._diag_lines += 1
            parts = line.split(",")
            if len(parts) != 9:
                print(f"  [S{self.sensor_id}] línea malformada ({len(parts)} campos): {line[:60]}")
                continue

            ts_pc = int(time.monotonic_ns() // 1_000_000)
            try:
                ts_ard = int(parts[0])
                ax = float(parts[3]); ay = float(parts[4]); az = float(parts[5])
                gx = float(parts[6]); gy = float(parts[7]); gz = float(parts[8])
            except ValueError as e:
                print(f"  [S{self.sensor_id}] ValueError: {e} en '{line[:60]}'")
                continue

            if self._t0 is None:
                self._t0 = ts_ard
            t_rel = (ts_ard - self._t0) / 1000.0

            with self._lock:
                self._times.append(t_rel)
                self._bufs["ax"].append(ax); self._bufs["ay"].append(ay)
                self._bufs["az"].append(az); self._bufs["gx"].append(gx)
                self._bufs["gy"].append(gy); self._bufs["gz"].append(gz)
                self.total    += 1
                self._odr_cnt += 1
                now = time.monotonic()
                if now - self._odr_ts >= 2.0:
                    self.odr_est  = self._odr_cnt / (now - self._odr_ts)
                    self._odr_cnt = 0
                    self._odr_ts  = now

                if self.recording and self._writer:
                    self._writer.writerow({
                        "timestamp_pc_ms":      ts_pc,
                        "timestamp_arduino_ms": parts[0],
                        "sensor_id":            parts[1],
                        "placement":            parts[2],
                        "phase":                self._phase,
                        "ax": parts[3], "ay": parts[4], "az": parts[5],
                        "gx": parts[6], "gy": parts[7], "gz": parts[8],
                        "trial_id":   self._trial_id,
                        "subject_id": self._subject_id,
                    })
                    self.samples_recorded += 1

    def snapshot(self):
        with self._lock:
            t = np.array(self._times)
            d = {c: np.array(self._bufs[c]) for c in ACC_COLS + GYRO_COLS}
            return t, d, self.total, self.odr_est

    def close(self):
        self._running = False
        self.close_csv()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass


# ── Hub BLE: un sensor virtual por cada sensor_id en el stream del hub ────────

class VirtualSensor:
    """
    Un sensor lógico dentro de un HubConnection.

    Expone la misma interfaz que SerialSensor (snapshot, set_phase,
    open_csv, close_csv, thread_alive, odr_est, total) para que la
    ventana principal no necesite distinguir entre modos.

    Los datos son inyectados por HubConnection._dispatch().
    """

    def __init__(self, sensor_id: int, placement: str):
        self.sensor_id = sensor_id
        self.placement = placement
        # El hilo de lectura pertenece al HubConnection; aquí siempre True
        # mientras el HubConnection esté vivo.
        self.thread_alive     = False
        self.total            = 0
        self.odr_est          = 0.0
        self.samples_recorded = 0
        self.recording        = False

        maxlen = WINDOW_S * SAMPLE_HZ * 2
        self._times = collections.deque(maxlen=maxlen)
        self._bufs  = {c: collections.deque(maxlen=maxlen)
                       for c in ACC_COLS + GYRO_COLS}
        self._lock   = threading.Lock()

        self._phase      = "none"
        self._trial_id   = ""
        self._subject_id = ""
        self._writer     = None
        self._csv_file   = None

        self._t0      = None
        self._odr_ts  = time.monotonic()
        self._odr_cnt = 0

    # ── API pública (igual que SerialSensor) ──────────────────────────────

    def set_phase(self, phase: str):
        self._phase = phase

    def open_csv(self, subject_id: str, trial_id: str, out_dir: Path) -> Path:
        self._subject_id = subject_id
        self._trial_id   = trial_id
        ts    = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = (out_dir /
                 f"{subject_id}_{trial_id}_s{self.sensor_id}_{self.placement}_{ts}.csv")
        self._csv_file = open(fname, "w", newline="", encoding="utf-8")
        self._writer   = csv.DictWriter(self._csv_file, fieldnames=COLUMNS)
        self._writer.writeheader()
        self.samples_recorded = 0
        self.recording = True
        return fname

    def close_csv(self):
        self.recording = False
        if self._csv_file:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file = None
            self._writer   = None

    def snapshot(self):
        with self._lock:
            t = np.array(self._times)
            d = {c: np.array(self._bufs[c]) for c in ACC_COLS + GYRO_COLS}
            return t, d, self.total, self.odr_est

    def close(self):
        self.close_csv()

    # ── Interno: llamado por HubConnection ───────────────────────────────

    def _dispatch(self, ts_pc: int, ts_ard: int,
                  ax: float, ay: float, az: float,
                  gx: float, gy: float, gz: float):
        if self._t0 is None:
            self._t0 = ts_ard
        t_rel = (ts_ard - self._t0) / 1000.0

        with self._lock:
            self._times.append(t_rel)
            self._bufs["ax"].append(ax); self._bufs["ay"].append(ay)
            self._bufs["az"].append(az); self._bufs["gx"].append(gx)
            self._bufs["gy"].append(gy); self._bufs["gz"].append(gz)
            self.total    += 1
            self._odr_cnt += 1
            now = time.monotonic()
            if now - self._odr_ts >= 2.0:
                self.odr_est  = self._odr_cnt / (now - self._odr_ts)
                self._odr_cnt = 0
                self._odr_ts  = now

            if self.recording and self._writer:
                self._writer.writerow({
                    "timestamp_pc_ms":      ts_pc,
                    "timestamp_arduino_ms": ts_ard,
                    "sensor_id":            self.sensor_id,
                    "placement":            self.placement,
                    "phase":                self._phase,
                    "ax": ax, "ay": ay, "az": az,
                    "gx": gx, "gy": gy, "gz": gz,
                    "trial_id":   self._trial_id,
                    "subject_id": self._subject_id,
                })
                self.samples_recorded += 1


class HubConnection:
    """
    Gestiona la conexión USB con un Arduino data_logger_hub.

    Lee un único puerto Serial que multiplexea los datos de N sensores
    (hub propio + N esclavos BLE) en el formato CSV estándar:
        timestamp_ms,sensor_id,placement,ax,ay,az,gx,gy,gz

    Cada línea se enruta al VirtualSensor correspondiente según sensor_id.
    """

    def __init__(self, port: str, sensors: list[VirtualSensor]):
        self.port     = port
        self._sensors: dict[int, VirtualSensor] = {s.sensor_id: s for s in sensors}
        self._ser     = None
        self._thread  = None
        self._running = False
        self.thread_alive = False

    # ── Conexión ──────────────────────────────────────────────────────────

    def connect(self) -> bool:
        try:
            self._ser = serial.Serial(self.port, BAUD_RATE, timeout=2.0)
            time.sleep(0.3)
            self._ser.reset_input_buffer()
            return True
        except Exception as e:
            print(f"  ERR HubConnection {self.port}: {e}")
            return False

    def handshake(self, timeout_s: float = 20.0) -> bool:
        """
        Espera a que el hub diga READY (después de conectar a sus esclavos BLE).
        También acepta STATUS: como señal de que el hub está vivo.
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            self._ser.write(b"STATUS\n")
            t1 = time.monotonic()
            while time.monotonic() - t1 < 2.0:
                raw = self._ser.readline()
                if not raw:
                    break
                line = raw.decode("utf-8", errors="replace").strip()
                if line == "READY" or line.startswith("STATUS:"):
                    print(f"  [Hub] {line}")
                    return True
                if line:
                    print(f"  [Hub] {line}")
        return False

    def start_stream(self):
        self._ser.reset_input_buffer()
        time.sleep(0.05)
        self._ser.write(b"START\n")
        print(f"  [Hub] START enviado a {self.port}")
        self._running     = True
        self.thread_alive = True
        for s in self._sensors.values():
            s.thread_alive = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop_stream(self):
        self._running = False
        try:
            self._ser.write(b"STOP\n")
        except Exception:
            pass

    def set_phase(self, phase: str):
        for s in self._sensors.values():
            s.set_phase(phase)

    def close(self):
        self._running = False
        for s in self._sensors.values():
            s.close_csv()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        self.thread_alive = False
        for s in self._sensors.values():
            s.thread_alive = False

    # ── Hilo de lectura ───────────────────────────────────────────────────

    def _read_loop(self):
        try:
            self._read_loop_inner()
        except Exception as e:
            print(f"  [Hub] ERROR en hilo: {e}")
        finally:
            self.thread_alive = False
            for s in self._sensors.values():
                s.thread_alive = False
            print("  [Hub] Hilo de lectura terminado")

    def _read_loop_inner(self):
        diag_lines = 0
        while self._running:
            try:
                raw = self._ser.readline()
            except serial.SerialException as e:
                print(f"  [Hub] SerialException: {e}")
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            # Líneas de control (no CSV)
            if not line[0].isdigit():
                print(f"  [Hub] ctrl: {line}")
                continue

            if diag_lines < 3:
                print(f"  [Hub] datos: {line[:80]}")
                diag_lines += 1

            parts = line.split(",")
            if len(parts) != 9:
                print(f"  [Hub] línea malformada ({len(parts)} campos): {line[:60]}")
                continue

            ts_pc = int(time.monotonic_ns() // 1_000_000)
            try:
                ts_ard    = int(parts[0])
                sensor_id = int(parts[1])
                # parts[2] = placement (usamos el del VirtualSensor)
                ax = float(parts[3]); ay = float(parts[4]); az = float(parts[5])
                gx = float(parts[6]); gy = float(parts[7]); gz = float(parts[8])
            except ValueError as e:
                print(f"  [Hub] ValueError: {e} en '{line[:60]}'")
                continue

            vsensor = self._sensors.get(sensor_id)
            if vsensor is None:
                # Sensor_id no esperado — imprimir una sola vez
                if not hasattr(self, '_unknown_ids'):
                    self._unknown_ids = set()
                if sensor_id not in self._unknown_ids:
                    print(f"  [Hub] sensor_id={sensor_id} inesperado, ignorando")
                    self._unknown_ids.add(sensor_id)
                continue

            vsensor._dispatch(ts_pc, ts_ard, ax, ay, az, gx, gy, gz)


# ── Mac BLE: la Mac actúa como hub usando bleak ───────────────────────────────

# Sensores BLE de adquisición (data_logger_ble_sensor)
BLE_ACQ_SERVICE = "19b20000-e8f2-537e-4f6c-d104768a1214"
BLE_ACQ_IMU_CHR = "19b20001-e8f2-537e-4f6c-d104768a1214"

# Configuración de cada sensor BLE: nombre anunciado, sensor_id, placement default.
# Ajusta los placements en la GUI antes de conectar.
BLE_SENSOR_CONFIGS = [
    {"name": "GaitNode_2", "sensor_id": 2, "placement": "pierna"},
    {"name": "GaitNode_3", "sensor_id": 3, "placement": "cadera"},
]


class BleakHubConnection:
    """
    Usa el Bluetooth interno de la Mac como hub BLE (sin Arduino USB).

    Conecta a los sensores GaitNode_* via bleak en un hilo con su propio
    event loop asyncio. Los datos llegan por notificación BLE y se despachan
    a los VirtualSensor correspondientes.

    Protocolo de uso (igual que HubConnection):
        conn = BleakHubConnection(sensors, names)
        conn.connect()         → True siempre
        conn.handshake(30)     → bloquea hasta que ≥1 sensor conecta (o timeout)
        conn.start_stream()    → no-op (el loop corre desde handshake)
        conn.set_phase(phase)
        conn.close()
    """

    def __init__(self, sensors: list[VirtualSensor], names: list[str],
                 scan_timeout: float = 15.0):
        self._vsensors    = sensors          # orden = orden de names
        self._names       = names            # ["GaitNode_2", "GaitNode_3", ...]
        self._scan_timeout = scan_timeout

        self._loop    = None
        self._thread  = None
        self._running = False
        self.thread_alive = False
        self._clients: list = []

        # Se dispara en cuanto ≥1 sensor conecta exitosamente
        self._ready_event = threading.Event()

    # ── API pública ───────────────────────────────────────────────────────────

    def connect(self) -> bool:
        return True  # no hay puerto serial que abrir

    def handshake(self, timeout_s: float = 30.0) -> bool:
        """Arranca el loop BLE y espera a que al menos 1 sensor se conecte."""
        self._running     = True
        self.thread_alive = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        ok = self._ready_event.wait(timeout=timeout_s)
        if not ok:
            print("  [MacBLE] Timeout: ningún sensor conectó en el tiempo límite")
        return ok

    def start_stream(self):
        # El loop ya corre desde handshake(); aquí solo marcamos los vsensors
        for s in self._vsensors:
            s.thread_alive = True

    def stop_stream(self):
        pass  # la GUI controla grabación; el stream BLE sigue corriendo

    def set_phase(self, phase: str):
        for s in self._vsensors:
            s.set_phase(phase)

    def close(self):
        self._running = False
        for s in self._vsensors:
            s.close_csv()
        # Pedir al loop que se detenga desde su propio hilo
        if self._loop and not self._loop.is_closed():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join(timeout=5.0)
        self.thread_alive = False
        for s in self._vsensors:
            s.thread_alive = False

    # ── Loop asyncio en hilo separado ─────────────────────────────────────────

    def _run_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._ble_main())
        except Exception as e:
            print(f"  [MacBLE] Error en loop asyncio: {e}")
        finally:
            self.thread_alive = False
            for s in self._vsensors:
                s.thread_alive = False
            print("  [MacBLE] Loop BLE terminado")

    async def _ble_main(self):
        from bleak import BleakScanner, BleakClient

        # Conectar a cada sensor en paralelo
        tasks = [
            self._connect_sensor(BleakScanner, BleakClient, name, vsensor)
            for name, vsensor in zip(self._names, self._vsensors)
        ]
        await asyncio.gather(*tasks, return_exceptions=True)

        # Mantener el loop vivo mientras haya clientes conectados
        while self._running:
            await asyncio.sleep(0.5)
            # Detectar desconexiones
            for client, vsensor in zip(self._clients, self._vsensors):
                if not client.is_connected:
                    vsensor.thread_alive = False

        # Desconectar limpiamente
        for client in self._clients:
            try:
                if client.is_connected:
                    await client.disconnect()
            except Exception:
                pass

    async def _connect_sensor(self, BleakScanner, BleakClient,
                               name: str, vsensor: VirtualSensor):
        print(f"  [MacBLE] Buscando {name} (timeout {self._scan_timeout}s)…")
        device = await BleakScanner.find_device_by_name(
            name, timeout=self._scan_timeout)

        if device is None:
            print(f"  [MacBLE] {name} no encontrado")
            return

        print(f"  [MacBLE] Encontrado {name} @ {device.address}")

        def on_disconnect(client):
            print(f"  [MacBLE] {name} desconectado")
            vsensor.thread_alive = False

        client = BleakClient(device, disconnected_callback=on_disconnect)

        try:
            await client.connect()
        except Exception as e:
            print(f"  [MacBLE] Error al conectar {name}: {e}")
            return

        def make_notify_cb(vs):
            def cb(sender, data: bytearray):
                if len(data) != 24:
                    return
                ax, ay, az, gx, gy, gz = struct.unpack('<6f', bytes(data))
                ts_pc = int(time.monotonic_ns() // 1_000_000)
                vs._dispatch(ts_pc, ts_pc, ax, ay, az, gx, gy, gz)
            return cb

        try:
            await client.start_notify(BLE_ACQ_IMU_CHR, make_notify_cb(vsensor))
        except Exception as e:
            print(f"  [MacBLE] Error al suscribir {name}: {e}")
            await client.disconnect()
            return

        self._clients.append(client)
        vsensor.thread_alive = True
        print(f"  [MacBLE] {name} conectado y suscrito — datos fluyendo")
        self._ready_event.set()   # al menos 1 sensor listo


# ── Ventana principal ─────────────────────────────────────────────────────────

class GaitWindow(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()
        self.sensors: list[SerialSensor | VirtualSensor] = []
        self._hub_conn: HubConnection | None = None
        self.recording       = False
        self.current_phase   = PHASES[0][0]
        self._setup_ui()

        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self._update_plots)
        self._timer.start(50)   # 20 Hz refresh

    # ── UI ────────────────────────────────────────────────────────────────────

    def _setup_ui(self):
        self.setWindowTitle("FusionGait — Adquisición por Fases")
        self.resize(1400, 820)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setSpacing(6)
        root.setContentsMargins(10, 10, 10, 10)

        # ── Top bar ───────────────────────────────────────────────────────
        top = QtWidgets.QHBoxLayout()

        top.addWidget(self._label("Sujeto:"))
        self.subject_edit = QtWidgets.QLineEdit("s001")
        self.subject_edit.setMaximumWidth(70)
        top.addWidget(self.subject_edit)

        top.addSpacing(10)
        top.addWidget(self._label("Trial:"))
        self.trial_edit = QtWidgets.QLineEdit("t001")
        self.trial_edit.setMaximumWidth(70)
        top.addWidget(self.trial_edit)

        top.addSpacing(10)
        top.addWidget(self._label("Sensores:"))
        self.n_spin = QtWidgets.QSpinBox()
        self.n_spin.setRange(1, MAX_SENSORS)
        self.n_spin.setValue(2)
        self.n_spin.setMaximumWidth(50)
        top.addWidget(self.n_spin)

        top.addSpacing(10)
        top.addWidget(self._label("Modo:"))
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(["USB directo", "Hub BLE (Arduino)", "Mac BLE (Bluetooth)"])
        self.mode_combo.setMinimumWidth(170)
        self.mode_combo.setStyleSheet(
            "QComboBox { background:#16213e; color:#eee; border:1px solid #555;"
            " border-radius:3px; padding:3px 8px; font-size:12px; }"
            "QComboBox QAbstractItemView { background:#16213e; color:#eee;"
            " selection-background-color:#3498db; }"
        )
        self.mode_combo.currentIndexChanged.connect(self._on_mode_change)
        top.addWidget(self.mode_combo)

        top.addSpacing(10)
        self.connect_btn = QtWidgets.QPushButton("Conectar")
        self.connect_btn.setStyleSheet(
            "background:#27ae60; color:white; font-weight:bold; padding:5px 16px; border-radius:4px;")
        self.connect_btn.clicked.connect(self._on_connect)
        top.addWidget(self.connect_btn)

        self.conn_status = QtWidgets.QLabel("● Desconectado")
        self.conn_status.setStyleSheet("color:#e74c3c; font-weight:bold; margin-left:8px;")
        top.addWidget(self.conn_status)

        top.addStretch()

        self.out_label = QtWidgets.QLabel(f"→ {DATA_DIR}")
        self.out_label.setStyleSheet("color:#666; font-size:11px;")
        top.addWidget(self.out_label)

        root.addLayout(top)

        # ── Fila de ubicación de sensores ─────────────────────────────────
        placement_bar = QtWidgets.QHBoxLayout()
        placement_bar.setContentsMargins(0, 0, 0, 0)
        placement_bar.setSpacing(8)

        placement_bar.addWidget(self._label("Ubicación:"))

        self.placement_combos: list[QtWidgets.QComboBox] = []
        self._placement_labels: list[QtWidgets.QLabel] = []
        for i in range(MAX_SENSORS):
            lbl = self._label(f"S{i+1}:")
            lbl.setStyleSheet("color:#aaa; font-size:12px;")
            self._placement_labels.append(lbl)
            placement_bar.addWidget(lbl)

            combo = QtWidgets.QComboBox()
            combo.addItems(PLACEMENT_OPTIONS)
            combo.setCurrentText(PLACEMENT_DEFAULTS[i])
            combo.setMinimumWidth(110)
            combo.setStyleSheet(
                "QComboBox { background:#16213e; color:#eee; border:1px solid #555;"
                " border-radius:3px; padding:3px 8px; font-size:12px; }"
                "QComboBox QAbstractItemView { background:#16213e; color:#eee;"
                " selection-background-color:#3498db; }"
            )
            self.placement_combos.append(combo)
            placement_bar.addWidget(combo)

        placement_bar.addStretch()
        self.n_spin.valueChanged.connect(self._update_placement_visibility)
        self._update_placement_visibility(self.n_spin.value())

        root.addLayout(placement_bar)

        # ── Plots ─────────────────────────────────────────────────────────
        pg.setConfigOptions(antialias=True, background="#12121f", foreground="#ddd")
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setMinimumHeight(420)
        root.addWidget(self.plot_widget, stretch=1)
        self._plots = []
        self._rebuild_plots(self.n_spin.value())

        # ── Selector de fases ─────────────────────────────────────────────
        phase_bar = QtWidgets.QFrame()
        phase_bar.setStyleSheet(
            "QFrame { background:#0d0d1a; border-radius:6px; padding:2px; }")
        phase_layout = QtWidgets.QHBoxLayout(phase_bar)
        phase_layout.setContentsMargins(8, 4, 8, 4)
        phase_layout.setSpacing(8)
        phase_layout.addWidget(self._label("FASE ACTUAL:", bold=True))

        self.phase_btns: list[QtWidgets.QPushButton] = []
        for pid, label, color in PHASES:
            btn = QtWidgets.QPushButton(label)
            btn.setCheckable(True)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background:{color}33; color:#ddd;
                    border:2px solid {color}88;
                    border-radius:4px; padding:6px 18px;
                    font-size:13px; font-weight:bold;
                }}
                QPushButton:checked {{
                    background:{color}; color:#fff;
                    border:2px solid {color};
                }}
                QPushButton:hover {{ background:{color}66; }}
            """)
            btn.clicked.connect(lambda _, p=pid: self._select_phase(p))
            phase_layout.addWidget(btn)
            self.phase_btns.append(btn)

        self.phase_btns[0].setChecked(True)
        phase_layout.addStretch()
        root.addWidget(phase_bar)

        # ── Barra de grabación ────────────────────────────────────────────
        rec_bar = QtWidgets.QHBoxLayout()
        rec_bar.setContentsMargins(0, 4, 0, 0)

        self.record_btn = QtWidgets.QPushButton("● GRABAR  [Space]")
        self.record_btn.setMinimumHeight(42)
        self.record_btn.setMinimumWidth(200)
        self.record_btn.setStyleSheet(self._rec_style(False))
        self.record_btn.setEnabled(False)
        self.record_btn.clicked.connect(self._toggle_record)
        rec_bar.addWidget(self.record_btn)

        self.phase_ind = QtWidgets.QLabel("Fase: —")
        self.phase_ind.setStyleSheet("color:#aaa; font-size:13px; margin-left:14px;")
        rec_bar.addWidget(self.phase_ind)

        rec_bar.addStretch()

        self.sample_lbl = QtWidgets.QLabel("Muestras: 0")
        self.sample_lbl.setStyleSheet("color:#aaa; font-size:12px;")
        rec_bar.addWidget(self.sample_lbl)

        self.odr_lbl = QtWidgets.QLabel("ODR: — Hz")
        self.odr_lbl.setStyleSheet("color:#aaa; font-size:12px; margin-left:14px;")
        rec_bar.addWidget(self.odr_lbl)

        root.addLayout(rec_bar)

        # ── Status por sensor ─────────────────────────────────────────────
        self.sensor_status_bar = QtWidgets.QHBoxLayout()
        self.sensor_status_bar.setContentsMargins(0, 2, 0, 0)
        self.sensor_lbls: list[QtWidgets.QLabel] = []
        root.addLayout(self.sensor_status_bar)

    def _update_placement_visibility(self, n: int):
        for i, (lbl, combo) in enumerate(
                zip(self._placement_labels, self.placement_combos)):
            visible = i < n
            lbl.setVisible(visible)
            combo.setVisible(visible)

    # ── Helpers UI ────────────────────────────────────────────────────────────

    @staticmethod
    def _label(text: str, bold: bool = False) -> QtWidgets.QLabel:
        lbl = QtWidgets.QLabel(text)
        if bold:
            lbl.setStyleSheet("font-weight:bold; font-size:13px;")
        return lbl

    @staticmethod
    def _rec_style(recording: bool) -> str:
        if recording:
            return ("background:#27ae60; color:white; font-weight:bold; "
                    "font-size:14px; border-radius:4px; padding:8px 24px;")
        return ("background:#c0392b; color:white; font-weight:bold; "
                "font-size:14px; border-radius:4px; padding:8px 24px;")

    def _rebuild_plots(self, n: int, placements: list[str] | None = None):
        self.plot_widget.clear()
        self._plots = []
        for col in range(n):
            place = (placements[col] if placements and col < len(placements)
                     else PLACEMENT_DEFAULTS[col] if col < len(PLACEMENT_DEFAULTS) else f"S{col+1}")
            sp = {}

            p_acc = self.plot_widget.addPlot(row=0, col=col,
                                             title=f"{place} — Aceleración (g)")
            p_acc.addLegend(offset=(5, 5))
            p_acc.setYRange(-3, 3)
            p_acc.showGrid(x=True, y=True, alpha=0.25)
            ca = {c: p_acc.plot([], [], name=c,
                                pen=pg.mkPen(ACC_COLORS[i], width=1.5))
                  for i, c in enumerate(ACC_COLS)}
            sp["acc"] = (p_acc, ca)

            p_gyr = self.plot_widget.addPlot(row=1, col=col,
                                             title=f"{place} — Giroscopio (°/s)")
            p_gyr.addLegend(offset=(5, 5))
            p_gyr.setYRange(-500, 500)
            p_gyr.showGrid(x=True, y=True, alpha=0.25)
            cg = {c: p_gyr.plot([], [], name=c,
                                pen=pg.mkPen(GYRO_COLORS[i], width=1.5))
                  for i, c in enumerate(GYRO_COLS)}
            sp["gyr"] = (p_gyr, cg)

            self._plots.append(sp)

    # ── Cambio de modo ────────────────────────────────────────────────────────

    def _on_mode_change(self, idx: int):
        mode = self.mode_combo.currentText()
        if mode == "USB directo":
            self.n_spin.setEnabled(True)
            self.n_spin.setRange(1, MAX_SENSORS)
        elif mode == "Hub BLE (Arduino)":
            self.n_spin.setEnabled(True)
            self.n_spin.setRange(1, MAX_SENSORS)
            self.n_spin.setValue(min(3, MAX_SENSORS))
        else:  # Mac BLE
            # Máximo = número de sensores BLE configurados
            max_ble = len(BLE_SENSOR_CONFIGS)
            self.n_spin.setEnabled(True)
            self.n_spin.setRange(1, max_ble)
            self.n_spin.setValue(max_ble)

        self.conn_status.setText("● Desconectado")
        self.conn_status.setStyleSheet("color:#e74c3c; font-weight:bold;")

    # ── Conexión ──────────────────────────────────────────────────────────────

    def _on_connect(self):
        mode = self.mode_combo.currentText()
        if mode == "USB directo":
            self._connect_direct()
        elif mode == "Hub BLE (Arduino)":
            self._connect_hub()
        else:
            self._connect_mac_ble()

    # ── Modo directo: N Arduinos por USB ──────────────────────────────────────

    def _connect_direct(self):
        n = self.n_spin.value()

        ports = sorted(
            p.device for p in serial.tools.list_ports.comports()
            if "usbmodem" in p.device or "Arduino" in (p.description or "")
        )

        if len(ports) < n:
            QtWidgets.QMessageBox.warning(
                self, "Sin puertos",
                f"Se necesitan {n} Arduino(s). Detectados: {len(ports)}\n"
                + (f"Puertos: {ports}" if ports else "Ninguno encontrado.")
            )
            return

        self.connect_btn.setEnabled(False)
        self.conn_status.setText("● Conectando…")
        self.conn_status.setStyleSheet("color:#f39c12; font-weight:bold;")
        QtWidgets.QApplication.processEvents()

        self._close_all()

        for i in range(n):
            placement = self.placement_combos[i].currentText()
            s = SerialSensor(ports[i], i + 1, placement)
            if not s.connect():
                self._conn_error(f"No se pudo abrir {ports[i]}")
                return
            self.conn_status.setText(f"● Handshake S{i+1}…")
            QtWidgets.QApplication.processEvents()
            if not s.handshake():
                self._conn_error(f"Sensor {i+1} ({ports[i]}) no respondió")
                return
            self.sensors.append(s)

        self._finish_connect(n)

    # ── Modo Hub BLE: 1 puerto USB → N sensores virtuales ─────────────────────

    def _connect_hub(self):
        n = self.n_spin.value()  # número total de sensores esperados (hub + esclavos)

        ports = sorted(
            p.device for p in serial.tools.list_ports.comports()
            if "usbmodem" in p.device or "Arduino" in (p.description or "")
        )

        if not ports:
            QtWidgets.QMessageBox.warning(
                self, "Sin puertos",
                "No se detectó ningún Arduino/hub por USB.\n"
                "Conecta el hub (data_logger_hub) y vuelve a intentarlo."
            )
            return

        self.connect_btn.setEnabled(False)
        self.conn_status.setText("● Conectando hub…")
        self.conn_status.setStyleSheet("color:#f39c12; font-weight:bold;")
        QtWidgets.QApplication.processEvents()

        self._close_all()

        # Crear sensores virtuales con las ubicaciones configuradas en los combos
        virtual_sensors = []
        for i in range(n):
            placement = self.placement_combos[i].currentText()
            virtual_sensors.append(VirtualSensor(i + 1, placement))

        hub_port = ports[0]
        hub = HubConnection(hub_port, virtual_sensors)

        if not hub.connect():
            self._conn_error(f"No se pudo abrir {hub_port}")
            return

        self.conn_status.setText("● Handshake hub (esperar esclavos BLE)…")
        QtWidgets.QApplication.processEvents()

        # El hub tarda hasta 15 s × N_SLAVES buscando esclavos antes de decir READY
        if not hub.handshake(timeout_s=40.0):
            self._conn_error(
                f"Hub ({hub_port}) no respondió.\n"
                "Verifica que esté ejecutando data_logger_hub y que los\n"
                "sensores BLE estén encendidos (LED azul)."
            )
            hub.close()
            return

        self._hub_conn = hub
        self.sensors   = virtual_sensors

        self._finish_connect(n)

    # ── Modo Mac BLE: Mac como hub Bluetooth directo ──────────────────────────

    def _connect_mac_ble(self):
        n = min(self.n_spin.value(), len(BLE_SENSOR_CONFIGS))

        self.connect_btn.setEnabled(False)
        self.conn_status.setText("● Buscando sensores BLE…")
        self.conn_status.setStyleSheet("color:#f39c12; font-weight:bold;")
        QtWidgets.QApplication.processEvents()

        self._close_all()

        # Crear VirtualSensors con los IDs y placements del BLE_SENSOR_CONFIGS,
        # pero usando la ubicación que el usuario tiene en los combos.
        virtual_sensors = []
        for i in range(n):
            cfg       = BLE_SENSOR_CONFIGS[i]
            placement = self.placement_combos[i].currentText()
            virtual_sensors.append(VirtualSensor(cfg["sensor_id"], placement))

        names = [BLE_SENSOR_CONFIGS[i]["name"] for i in range(n)]

        hub = BleakHubConnection(virtual_sensors, names, scan_timeout=15.0)
        hub.connect()  # siempre True

        # handshake() arranca el loop BLE y bloquea hasta que ≥1 sensor conecta
        # (máx 15 s × n sensores en paralelo, no secuencial)
        self.conn_status.setText(
            f"● Buscando {', '.join(names)}…  (hasta 15 s)")
        QtWidgets.QApplication.processEvents()

        if not hub.handshake(timeout_s=20.0):
            self._conn_error(
                "Ningún sensor BLE respondió en 20 s.\n\n"
                "Verifica que los sensores estén encendidos (LED azul)\n"
                "y ejecutando data_logger_ble_sensor.\n\n"
                f"Nombres esperados: {', '.join(names)}"
            )
            hub.close()
            return

        self._hub_conn = hub
        self.sensors   = virtual_sensors

        self._finish_connect(n)

    # ── Finalizar conexión (común a los dos modos) ────────────────────────────

    def _finish_connect(self, n: int):
        selected_placements = [self.placement_combos[i].currentText() for i in range(n)]
        self._rebuild_plots(n, selected_placements)

        # Labels de estado por sensor
        for lbl in self.sensor_lbls:
            self.sensor_status_bar.removeWidget(lbl)
            lbl.deleteLater()
        self.sensor_lbls = []
        for s in self.sensors:
            lbl = QtWidgets.QLabel(f"S{s.sensor_id}/{s.placement}: iniciando…")
            lbl.setStyleSheet("color:#f39c12; font-size:11px; padding:0 10px;")
            self.sensor_status_bar.addWidget(lbl)
            self.sensor_lbls.append(lbl)
        self.sensor_status_bar.addStretch()

        if self._hub_conn is not None:
            self._hub_conn.start_stream()
        else:
            for s in self.sensors:
                s.start_stream()

        mode = self.mode_combo.currentText()
        mode_tag = "" if mode == "USB directo" else f" ({mode})"
        self.conn_status.setText(f"● {n} sensor(es) conectados{mode_tag}")
        self.conn_status.setStyleSheet("color:#2ecc71; font-weight:bold;")
        self.record_btn.setEnabled(True)
        self.connect_btn.setEnabled(True)
        self._select_phase(PHASES[0][0])

    # ── Cerrar todas las conexiones ───────────────────────────────────────────

    def _close_all(self):
        for s in self.sensors:
            s.close()
        self.sensors = []
        if self._hub_conn is not None:
            self._hub_conn.close()
            self._hub_conn = None

    def _conn_error(self, msg: str):
        QtWidgets.QMessageBox.critical(self, "Error de conexión", msg)
        self.conn_status.setText("● Error")
        self.conn_status.setStyleSheet("color:#e74c3c; font-weight:bold;")
        self.connect_btn.setEnabled(True)

    # ── Fases ─────────────────────────────────────────────────────────────────

    def _select_phase(self, phase_id: str):
        self.current_phase = phase_id
        if self._hub_conn is not None:
            self._hub_conn.set_phase(phase_id)
        for s in self.sensors:
            s.set_phase(phase_id)
        for i, (pid, label, color) in enumerate(PHASES):
            self.phase_btns[i].setChecked(pid == phase_id)
        label = next(lbl for pid, lbl, _ in PHASES if pid == phase_id)
        self.phase_ind.setText(f"Fase: {label}")

    # ── Grabación ─────────────────────────────────────────────────────────────

    def _toggle_record(self):
        if not self.recording:
            self._start_recording()
        else:
            self._stop_recording()

    def _start_recording(self):
        subject = self.subject_edit.text().strip() or "s001"
        trial   = self.trial_edit.text().strip() or "t001"
        DATA_DIR.mkdir(parents=True, exist_ok=True)

        for s in self.sensors:
            fname = s.open_csv(subject, trial, DATA_DIR)
            s.set_phase(self.current_phase)
            print(f"  Grabando → {fname}")

        self.recording = True
        self.record_btn.setText("■ DETENER  [Space]")
        self.record_btn.setStyleSheet(self._rec_style(True))

    def _stop_recording(self):
        for s in self.sensors:
            s.close_csv()
        self.recording = False
        total = sum(s.samples_recorded for s in self.sensors)
        secs  = total / (SAMPLE_HZ * max(1, len(self.sensors)))
        self.record_btn.setText("● GRABAR  [Space]")
        self.record_btn.setStyleSheet(self._rec_style(False))
        self.sample_lbl.setText("Muestras: 0")
        QtWidgets.QMessageBox.information(
            self, "Trial guardado",
            f"Archivo guardado en {DATA_DIR}\n"
            f"Muestras totales: {total}  ({secs:.1f} s)"
        )

    # ── Actualización de plots ────────────────────────────────────────────────

    def _update_plots(self):
        total_samples = 0
        odr_vals      = []

        for i, s in enumerate(self.sensors):
            if i >= len(self._plots):
                break
            t, d, total, odr = s.snapshot()
            total_samples += total

            # Actualizar label de estado por sensor
            if i < len(self.sensor_lbls):
                lbl = self.sensor_lbls[i]
                if not s.thread_alive and total == 0:
                    lbl.setText(f"S{s.sensor_id}/{s.placement}: sin datos")
                    lbl.setStyleSheet("color:#e74c3c; font-size:11px; padding:0 10px;")
                elif not s.thread_alive:
                    lbl.setText(f"S{s.sensor_id}/{s.placement}: hilo caído ⚠")
                    lbl.setStyleSheet("color:#e74c3c; font-size:11px; padding:0 10px;")
                elif odr > 0:
                    lbl.setText(f"S{s.sensor_id}/{s.placement}: {odr:.0f} Hz  ✓")
                    lbl.setStyleSheet("color:#2ecc71; font-size:11px; padding:0 10px;")
                    odr_vals.append(odr)
                else:
                    lbl.setText(f"S{s.sensor_id}/{s.placement}: esperando datos…")
                    lbl.setStyleSheet("color:#f39c12; font-size:11px; padding:0 10px;")

            if len(t) < 2:
                continue

            mask = t >= (t[-1] - WINDOW_S)
            _, ca = self._plots[i]["acc"]
            _, cg = self._plots[i]["gyr"]
            for c in ACC_COLS:
                ca[c].setData(t[mask], d[c][mask])
            for c in GYRO_COLS:
                cg[c].setData(t[mask], d[c][mask])

        if self.recording:
            rec = sum(s.samples_recorded for s in self.sensors)
            secs = rec / (SAMPLE_HZ * max(1, len(self.sensors)))
            self.sample_lbl.setText(f"Grabando: {rec} muestras  ({secs:.1f} s)")
        else:
            self.sample_lbl.setText(f"Muestras en buffer: {total_samples}")

        if odr_vals:
            self.odr_lbl.setText(f"ODR: {np.mean(odr_vals):.0f} Hz")

    # ── Teclado ───────────────────────────────────────────────────────────────

    def keyPressEvent(self, event):
        key = event.text()
        if key == " " and self.record_btn.isEnabled():
            self._toggle_record()
        elif key in "1234":
            idx = int(key) - 1
            if idx < len(PHASES):
                self._select_phase(PHASES[idx][0])
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event):
        if self.recording:
            self._stop_recording()
        self._close_all()
        event.accept()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")

    pal = QtGui.QPalette()
    dark = {
        QtGui.QPalette.ColorRole.Window:          "#1a1a2e",
        QtGui.QPalette.ColorRole.WindowText:      "#eeeeee",
        QtGui.QPalette.ColorRole.Base:            "#16213e",
        QtGui.QPalette.ColorRole.AlternateBase:   "#0f3460",
        QtGui.QPalette.ColorRole.Button:          "#16213e",
        QtGui.QPalette.ColorRole.ButtonText:      "#eeeeee",
        QtGui.QPalette.ColorRole.Text:            "#eeeeee",
        QtGui.QPalette.ColorRole.Highlight:       "#3498db",
        QtGui.QPalette.ColorRole.HighlightedText: "#ffffff",
    }
    for role, color in dark.items():
        pal.setColor(role, QtGui.QColor(color))
    app.setPalette(pal)

    win = GaitWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
