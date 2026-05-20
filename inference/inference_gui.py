#!/usr/bin/env python3
"""
inference_gui.py — FusionGait Visualización de Inferencia en Tiempo Real

Lee líneas INFER del Arduino Master por USB Serial y muestra:
  • Fase predicha actual (etiqueta grande con color)
  • Barras de probabilidad por clase
  • Timeline scrolling (últimos 12 s)
  • Tasa de inferencia

Uso:
    python3 inference/inference_gui.py
    python3 inference/inference_gui.py --port /dev/cu.usbmodem11201
"""

import argparse
import sys
import time
import threading
from collections import deque

import serial
import serial.tools.list_ports
import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox,
    QProgressBar, QFrame, QSizePolicy,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtGui import QFont

import pyqtgraph as pg

# ── Constantes ────────────────────────────────────────────────────────────
PHASES = ["loading", "midstance", "terminal", "swing"]
PHASE_COLORS = {
    "loading":   "#e74c3c",
    "midstance": "#f39c12",
    "terminal":  "#2ecc71",
    "swing":     "#3498db",
}
PHASE_LABELS = {
    "loading":   "1 · Loading",
    "midstance": "2 · Mid Stance",
    "terminal":  "3 · Terminal Stance",
    "swing":     "4 · Swing",
}

BAUD_RATE  = 115200
HISTORY_S  = 12       # segundos de timeline
INFER_HZ   = 4        # inferencias/s estimadas (100 Hz / STEP=25)


# ── Señales del hilo Serial → hilo Qt ────────────────────────────────────
class SerialSignals(QObject):
    infer    = pyqtSignal(str, list)   # (phase, [p0,p1,p2,p3])
    status   = pyqtSignal(str)
    disconn  = pyqtSignal()


# ── Hilo lector de Serial ────────────────────────────────────────────────
class SerialReader(threading.Thread):
    def __init__(self, port: str, signals: SerialSignals):
        super().__init__(daemon=True)
        self.port    = port
        self.signals = signals
        self.running = True
        self._ser    = None

    def run(self):
        try:
            self._ser = serial.Serial(self.port, BAUD_RATE, timeout=1.0)
            time.sleep(0.3)
            self._ser.reset_input_buffer()
            self.signals.status.emit(f"Conectado → {self.port}")
        except serial.SerialException as e:
            self.signals.status.emit(f"ERROR: {e}")
            return

        while self.running:
            try:
                raw = self._ser.readline()
            except serial.SerialException:
                break

            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            self._parse(line)

        if self._ser and self._ser.is_open:
            self._ser.close()
        self.signals.disconn.emit()

    def _parse(self, line: str):
        if line.startswith("INFER,"):
            # Formato: INFER,swing,0.01,0.03,0.04,0.92
            parts = line.split(",")
            if len(parts) == 2 + len(PHASES):
                phase = parts[1]
                try:
                    probs = [float(p) for p in parts[2:]]
                    self.signals.infer.emit(phase, probs)
                except ValueError:
                    pass
        elif line.startswith("STATUS:") or line.startswith("ARENA:") \
                or line.startswith("ERROR:"):
            self.signals.status.emit(line)

    def stop(self):
        self.running = False
        if self._ser and self._ser.is_open:
            self._ser.close()


# ── Ventana principal ─────────────────────────────────────────────────────
class InferenceWindow(QMainWindow):
    def __init__(self, initial_port: str = ""):
        super().__init__()
        self.setWindowTitle("FusionGait — Inferencia en Tiempo Real")
        self.setMinimumSize(960, 640)

        self._reader       = None
        self._signals      = SerialSignals()
        self._timeline     = deque(maxlen=HISTORY_S * INFER_HZ * 4)
        self._infer_ts     = deque(maxlen=30)
        self._phase_counts = {p: 0 for p in PHASES}

        self._build_ui(initial_port)
        self._wire_signals()

        # Timer de refresco de la timeline (100 ms)
        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh_plots)
        self._timer.start(100)

    # ── Construcción de la UI ─────────────────────────────────────────────
    def _build_ui(self, initial_port: str):
        root_w = QWidget()
        self.setCentralWidget(root_w)
        root = QVBoxLayout(root_w)
        root.setContentsMargins(12, 12, 12, 8)
        root.setSpacing(8)

        # ── Barra de conexión ─────────────────────────────────────────────
        conn_bar = QHBoxLayout()

        lbl_port = QLabel("Puerto:")
        lbl_port.setFixedWidth(52)
        conn_bar.addWidget(lbl_port)

        self._port_cb = QComboBox()
        self._port_cb.setMinimumWidth(200)
        self._fill_ports(initial_port)
        conn_bar.addWidget(self._port_cb)

        btn_refresh = QPushButton("↺")
        btn_refresh.setFixedWidth(30)
        btn_refresh.setToolTip("Refrescar puertos")
        btn_refresh.clicked.connect(lambda: self._fill_ports(""))
        conn_bar.addWidget(btn_refresh)

        self._btn_conn = QPushButton("Conectar")
        self._btn_conn.setCheckable(True)
        self._btn_conn.setFixedWidth(110)
        self._btn_conn.clicked.connect(self._toggle_conn)
        conn_bar.addWidget(self._btn_conn)

        conn_bar.addStretch()

        self._lbl_rate = QLabel("— Hz")
        self._lbl_rate.setFont(QFont("Monospace", 10))
        self._lbl_rate.setToolTip("Tasa de inferencia")
        conn_bar.addWidget(self._lbl_rate)

        root.addLayout(conn_bar)

        # ── Fila central: fase grande + barras de prob ────────────────────
        mid = QHBoxLayout()
        mid.setSpacing(10)

        # Recuadro de fase
        self._phase_frame = QFrame()
        self._phase_frame.setFrameShape(QFrame.Shape.Box)
        self._phase_frame.setMinimumHeight(160)
        ph_layout = QVBoxLayout(self._phase_frame)
        ph_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self._lbl_phase = QLabel("—")
        self._lbl_phase.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_phase.setFont(QFont("Sans Serif", 34, QFont.Weight.Bold))
        ph_layout.addWidget(self._lbl_phase)

        self._lbl_conf = QLabel("")
        self._lbl_conf.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_conf.setFont(QFont("Sans Serif", 13))
        ph_layout.addWidget(self._lbl_conf)

        self._set_phase_style("#555555")
        mid.addWidget(self._phase_frame, stretch=5)

        # Panel de probabilidades
        prob_frame = QFrame()
        prob_frame.setFrameShape(QFrame.Shape.Box)
        prob_layout = QVBoxLayout(prob_frame)
        prob_layout.setSpacing(6)
        prob_layout.addWidget(QLabel("<b>Probabilidades</b>"))

        self._bars = {}
        for ph in PHASES:
            row = QHBoxLayout()
            lbl = QLabel(PHASE_LABELS[ph])
            lbl.setFixedWidth(155)
            lbl.setFont(QFont("Sans Serif", 10))
            row.addWidget(lbl)

            bar = QProgressBar()
            bar.setRange(0, 100)
            bar.setValue(0)
            bar.setFormat("%v %")
            c = PHASE_COLORS[ph]
            bar.setStyleSheet(
                f"QProgressBar {{ border:1px solid #aaa; border-radius:3px; "
                f"background:#f0f0f0; text-align:center; }}"
                f"QProgressBar::chunk {{ background:{c}; border-radius:3px; }}"
            )
            row.addWidget(bar)
            prob_layout.addLayout(row)
            self._bars[ph] = bar

        prob_layout.addStretch()

        # Stats breves
        self._lbl_stats = QLabel("")
        self._lbl_stats.setFont(QFont("Monospace", 9))
        self._lbl_stats.setAlignment(Qt.AlignmentFlag.AlignLeft)
        prob_layout.addWidget(self._lbl_stats)

        mid.addWidget(prob_frame, stretch=3)
        root.addLayout(mid)

        # ── Timeline ──────────────────────────────────────────────────────
        pg.setConfigOption("background", "w")
        pg.setConfigOption("foreground", "k")

        self._timeline_plot = pg.PlotWidget()
        self._timeline_plot.setFixedHeight(165)
        self._timeline_plot.setLabel("left",   "Fase")
        self._timeline_plot.setLabel("bottom", "Tiempo (s)")
        self._timeline_plot.setYRange(-0.6, len(PHASES) - 0.4)
        self._timeline_plot.getAxis("left").setTicks(
            [[(i, PHASES[i]) for i in range(len(PHASES))]]
        )
        self._timeline_plot.showGrid(x=True, y=False, alpha=0.25)
        self._timeline_plot.setTitle("Timeline de fases — últimos 12 s")

        self._scatters = {}
        for i, ph in enumerate(PHASES):
            sc = pg.ScatterPlotItem(
                size=9,
                pen=pg.mkPen(None),
                brush=pg.mkBrush(PHASE_COLORS[ph]),
            )
            self._timeline_plot.addItem(sc)
            self._scatters[ph] = sc

        root.addWidget(self._timeline_plot)

        # ── Status bar ────────────────────────────────────────────────────
        self.statusBar().showMessage("Sin conexión")

    # ── Señales ───────────────────────────────────────────────────────────
    def _wire_signals(self):
        self._signals.infer.connect(self._on_infer)
        self._signals.status.connect(self._on_status)
        self._signals.disconn.connect(self._on_disconn)

    # ── Puertos ───────────────────────────────────────────────────────────
    def _fill_ports(self, prefer: str):
        prev = self._port_cb.currentText() if hasattr(self, "_port_cb") else prefer
        ports = sorted(
            p.device for p in serial.tools.list_ports.comports()
            if "usbmodem" in p.device.lower()
            or "arduino" in (p.description or "").lower()
            or "2341" in (p.hwid or "")
        )
        if not ports:
            ports = [p.device for p in serial.tools.list_ports.comports()]

        self._port_cb.clear()
        self._port_cb.addItems(ports)
        target = prefer or prev
        if target and target in ports:
            self._port_cb.setCurrentText(target)

    # ── Conexión ──────────────────────────────────────────────────────────
    def _toggle_conn(self):
        if self._btn_conn.isChecked():
            port = self._port_cb.currentText()
            if not port:
                self._btn_conn.setChecked(False)
                return
            self._reader = SerialReader(port, self._signals)
            self._reader.start()
            self._btn_conn.setText("Desconectar")
            self._port_cb.setEnabled(False)
        else:
            self._disconnect()

    def _disconnect(self):
        if self._reader:
            self._reader.stop()
            self._reader = None
        self._btn_conn.setChecked(False)
        self._btn_conn.setText("Conectar")
        self._port_cb.setEnabled(True)
        self._set_phase_style("#555555")
        self._lbl_phase.setText("—")
        self._lbl_conf.setText("")

    # ── Handlers de datos ────────────────────────────────────────────────
    def _on_infer(self, phase: str, probs: list):
        now = time.monotonic()
        self._infer_ts.append(now)
        self._timeline.append((now, phase))
        self._phase_counts[phase] = self._phase_counts.get(phase, 0) + 1

        # Recuadro de fase
        color = PHASE_COLORS.get(phase, "#888888")
        self._set_phase_style(color)
        self._lbl_phase.setText(PHASE_LABELS.get(phase, phase))

        if probs:
            conf = max(probs) * 100
            self._lbl_conf.setText(f"{conf:.0f} % confianza")

        # Barras
        for i, ph in enumerate(PHASES):
            val = int(probs[i] * 100) if i < len(probs) else 0
            self._bars[ph].setValue(val)

        # Tasa
        if len(self._infer_ts) >= 2:
            dt = self._infer_ts[-1] - self._infer_ts[-2]
            self._lbl_rate.setText(f"{1/dt:.1f} Hz" if dt > 0 else "— Hz")

        # Stats
        total = max(sum(self._phase_counts.values()), 1)
        lines = [f"{p}: {v/total*100:.0f}%" for p, v in self._phase_counts.items()]
        self._lbl_stats.setText("  ".join(lines))

    def _on_status(self, msg: str):
        self.statusBar().showMessage(msg)

    def _on_disconn(self):
        self.statusBar().showMessage("Desconectado")
        self._btn_conn.setChecked(False)
        self._btn_conn.setText("Conectar")
        self._port_cb.setEnabled(True)

    # ── Refresco del timeline ─────────────────────────────────────────────
    def _refresh_plots(self):
        if not self._timeline:
            return

        now    = time.monotonic()
        cutoff = now - HISTORY_S

        pts = {ph: ([], []) for ph in PHASES}
        for ts, ph in self._timeline:
            if ts < cutoff or ph not in pts:
                continue
            pts[ph][0].append(ts - now)           # relativo, 0 = ahora
            pts[ph][1].append(PHASES.index(ph))

        for ph, sc in self._scatters.items():
            xs, ys = pts[ph]
            sc.setData(x=xs, y=ys) if xs else sc.setData(x=[], y=[])

        self._timeline_plot.setXRange(-HISTORY_S, 0.2)

    # ── Helpers de estilo ─────────────────────────────────────────────────
    def _set_phase_style(self, hex_color: str):
        self._phase_frame.setStyleSheet(
            f"QFrame {{ background-color:{hex_color}22; "
            f"border:3px solid {hex_color}; border-radius:10px; }}"
        )
        self._lbl_phase.setStyleSheet(f"color:{hex_color};")

    def closeEvent(self, event):
        if self._reader:
            self._reader.stop()
        event.accept()


# ── Entry point ───────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="FusionGait Inference GUI")
    ap.add_argument("--port", default="",
                    help="Puerto Serial del Arduino Master, ej. /dev/cu.usbmodem11201")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    win = InferenceWindow(initial_port=args.port)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
