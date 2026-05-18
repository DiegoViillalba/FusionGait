#!/usr/bin/env python3
"""
live_print.py — Monitor de IMU en terminal (sin dependencias de GUI).

Funciona con solo pyserial + stdlib. Útil como fallback o para verificar
la señal cuando no hay entorno gráfico disponible.

Uso:
    python live_print.py --port /dev/cu.usbmodem1201
    python live_print.py          # auto-detecta
"""

import argparse
import collections
import math
import os
import sys
import threading
import time
import serial
import serial.tools.list_ports

SAMPLE_HZ = 100
HISTORY   = 80    # muestras para calcular stats (~0.8 s)

# ─── ANSI ─────────────────────────────────────────────────────────────────────
RESET  = "\033[0m"
BOLD   = "\033[1m"
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
BLUE   = "\033[94m"
CYAN   = "\033[96m"
GRAY   = "\033[90m"
CLEAR  = "\033[2J\033[H"   # limpiar pantalla + mover al inicio


def bar(value: float, vmin: float, vmax: float, width: int = 30,
        color: str = GREEN) -> str:
    """Barra horizontal proporcional al valor dentro de [vmin, vmax]."""
    span = vmax - vmin
    if span == 0:
        frac = 0.5
    else:
        frac = max(0.0, min(1.0, (value - vmin) / span))
    filled = round(frac * width)
    center = round(width / 2)
    # Dibujar con distinción izquierda/derecha del cero
    chars = [" "] * width
    chars[center] = GRAY + "│" + RESET
    for i in range(min(filled, center), center):
        chars[i] = color + "█" + RESET
    for i in range(center, min(center + filled, width)):
        chars[i] = color + "█" + RESET
    if filled >= center:
        for i in range(center, min(center + filled, width)):
            chars[i] = color + "█" + RESET
    else:
        for i in range(filled, center):
            chars[i] = color + "█" + RESET
    return "".join(chars)


def row(label: str, val: float, unit: str,
        vmin: float, vmax: float, color: str) -> str:
    b   = bar(val, vmin, vmax, width=34, color=color)
    num = f"{val:+8.3f} {unit}"
    return f"  {color}{BOLD}{label:3s}{RESET}  [{b}]  {num}"


# ─── Puerto ───────────────────────────────────────────────────────────────────

def auto_port() -> str:
    for p in serial.tools.list_ports.comports():
        if "usbmodem" in p.device or "Arduino" in (p.description or ""):
            return p.device
    return ""


# ─── Handshake ────────────────────────────────────────────────────────────────

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
                print(f"  Arduino: {line}")
                return True
        remaining = timeout_s - (time.monotonic() - t0)
        if remaining > 1:
            print(f"  Sin respuesta, reintentando… ({remaining:.0f} s)")
    return False


# ─── Hilo de lectura ──────────────────────────────────────────────────────────

class IMUStream:
    COLS = ("ax", "ay", "az", "gx", "gy", "gz")

    def __init__(self, port: str):
        self.ser  = serial.Serial(port, 115200, timeout=2)
        self.bufs = {c: collections.deque(maxlen=HISTORY) for c in self.COLS}
        self._lock    = threading.Lock()
        self._running = False
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self.total    = 0
        self._n_sec   = 0
        self._t_sec   = time.monotonic()
        self.odr      = 0.0

    def start(self):
        time.sleep(0.3)
        if not handshake(self.ser):
            raise RuntimeError("Arduino no respondió.")
        self.ser.reset_input_buffer()
        self.ser.write(b"START\n")
        ack = self.ser.readline().decode("utf-8", errors="replace").strip()
        print(f"  {ack}\n")
        self._running = True
        self._thread.start()

    def stop(self):
        self._running = False
        try:
            self.ser.write(b"STOP\n")
        except Exception:
            pass
        self._thread.join(timeout=2)
        self.ser.close()

    def _loop(self):
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
            with self._lock:
                for col, val in zip(self.COLS, vals):
                    self.bufs[col].append(val)
                self.total  += 1
                self._n_sec += 1
                now = time.monotonic()
                if now - self._t_sec >= 1.0:
                    self.odr     = self._n_sec / (now - self._t_sec)
                    self._n_sec  = 0
                    self._t_sec  = now

    def latest(self) -> dict:
        with self._lock:
            return {c: list(self.bufs[c]) for c in self.COLS}

    @property
    def stats(self):
        with self._lock:
            out = {}
            for c in self.COLS:
                d = list(self.bufs[c])
                if d:
                    out[c] = (d[-1], min(d), max(d),
                              sum(d)/len(d),
                              math.sqrt(sum((x - sum(d)/len(d))**2
                                           for x in d) / len(d)))
                else:
                    out[c] = (0, 0, 0, 0, 0)
            return out


# ─── Render ───────────────────────────────────────────────────────────────────

ACC_COLORS  = [RED, GREEN, BLUE]
GYRO_COLORS = [YELLOW, CYAN, "\033[95m"]   # rojo, verde, azul / amarillo, cyan, magenta

def render(stream: IMUStream, port: str):
    s = stream.stats
    odr   = stream.odr
    total = stream.total

    acc_vals  = [s[c][0] for c in ("ax", "ay", "az")]
    gyro_vals = [s[c][0] for c in ("gx", "gy", "gz")]
    acc_norm  = math.sqrt(sum(v**2 for v in acc_vals))

    lines = []
    lines.append(
        f"{BOLD}FusionGait — Live IMU{RESET}   "
        f"{GRAY}puerto: {port}   "
        f"ODR: {odr:5.1f} Hz   "
        f"muestras: {total:,}   "
        f"acc_norm: {acc_norm:.3f} g{RESET}"
    )
    lines.append(GRAY + "─" * 70 + RESET)

    lines.append(f"  {BOLD}Acelerómetro{RESET}  (g)       "
                 f"{GRAY}─ {HISTORY/SAMPLE_HZ:.1f} s de historial ─{RESET}")
    for lbl, col, color in zip(("ax", "ay", "az"),
                                ("ax", "ay", "az"), ACC_COLORS):
        cur, mn, mx, mean, std = s[col]
        lines.append(row(lbl, cur, "g", -5, 5, color) +
                     f"  {GRAY}[{mn:+.2f} … {mx:+.2f}]  μ={mean:+.2f}  σ={std:.2f}{RESET}")

    lines.append("")
    lines.append(f"  {BOLD}Giroscopio{RESET}    (°/s)")
    for lbl, col, color in zip(("gx", "gy", "gz"),
                                ("gx", "gy", "gz"), GYRO_COLORS):
        cur, mn, mx, mean, std = s[col]
        lines.append(row(lbl, cur, "°/s", -400, 400, color) +
                     f"  {GRAY}[{mn:+6.1f} … {mx:+6.1f}]  μ={mean:+5.1f}  σ={std:.1f}{RESET}")

    lines.append("")
    # Mini-barra de acc_norm
    norm_bar = bar(acc_norm, 0, 3, width=34, color=GREEN)
    lines.append(f"  {BOLD}acc_norm{RESET}  [{norm_bar}]  {acc_norm:.4f} g"
                 f"  {GRAY}(reposo ≈ 1.00 g){RESET}")

    # Indicador de calidad
    if odr > 0:
        if 90 <= odr <= 115:
            qc = f"{GREEN}● ODR OK{RESET}"
        else:
            qc = f"{RED}⚠ ODR fuera de rango{RESET}"
        if 0.8 <= acc_norm <= 1.2:
            qc2 = f"{GREEN}● acc_norm OK{RESET}"
        else:
            qc2 = f"{YELLOW}● acc_norm inusual{RESET}"
        lines.append(f"\n  {qc}   {qc2}")

    lines.append(f"\n  {GRAY}Ctrl+C para detener{RESET}")
    return "\n".join(lines)


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="Monitor IMU en terminal")
    p.add_argument("--port",     default="")
    p.add_argument("--fps",      type=float, default=10.0,
                   help="Refrescos por segundo (default: 10)")
    args = p.parse_args()

    port = args.port or auto_port()
    if not port:
        sys.exit("No se encontró ningún Arduino. Usa --port /dev/cu.usbmodemXXXX")

    print(f"\nConectando a {port}…")
    stream = IMUStream(port)
    stream.start()

    interval = 1.0 / args.fps
    try:
        while True:
            frame = render(stream, port)
            # Mover cursor al inicio sin limpiar (evita parpadeo)
            sys.stdout.write("\033[H" + frame + "\033[J")
            sys.stdout.flush()
            time.sleep(interval)
    except KeyboardInterrupt:
        pass
    finally:
        stream.stop()
        print(f"\n\nDetenido. Total muestras: {stream.total:,}")


if __name__ == "__main__":
    # Limpiar pantalla al arrancar
    print(CLEAR, end="")
    main()
