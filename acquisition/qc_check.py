#!/usr/bin/env python3
"""
qc_check.py — Verificación de calidad de un CSV capturado.

Uso:
    python qc_check.py ../data/raw/s001_t001_sensor3_ankle.csv
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd


def check(csv_path: str):
    path = Path(csv_path)
    if not path.exists():
        sys.exit(f"Archivo no encontrado: {csv_path}")

    df = pd.read_csv(path)
    n  = len(df)
    print(f"\n{'─'*50}")
    print(f"  Archivo : {path.name}")
    print(f"  Filas   : {n:,}")
    print(f"{'─'*50}")

    # ── 1. Columnas ────────────────────────────────────────
    required = ["timestamp_pc_ms", "sensor_id", "placement",
                 "ax", "ay", "az", "gx", "gy", "gz"]
    missing  = [c for c in required if c not in df.columns]
    _check("Columnas OK", len(missing) == 0,
           f"Faltan: {missing}")

    if missing:
        return

    # ── 2. NaNs ────────────────────────────────────────────
    imu_cols = ["ax", "ay", "az", "gx", "gy", "gz"]
    nan_counts = df[imu_cols].isna().sum()
    total_nans  = nan_counts.sum()
    _check("Sin NaN en IMU", total_nans == 0,
           f"{total_nans} NaN — {nan_counts.to_dict()}")

    # ── 3. ODR ─────────────────────────────────────────────
    dt = df["timestamp_pc_ms"].diff().dropna()
    odr_est = 1000.0 / dt.mean()
    ok_odr  = 90 <= odr_est <= 115
    _check(f"ODR ≈ {odr_est:.1f} Hz  (target 100 Hz)", ok_odr,
           "Fuera de rango 90–115 Hz")

    # ── 4. Gaps ────────────────────────────────────────────
    gaps       = dt[dt > 25]
    gap_frac   = len(gaps) / max(n, 1)
    ok_gaps    = gap_frac < 0.02
    _check(f"Gaps >25 ms : {len(gaps)}  ({100*gap_frac:.2f}%)", ok_gaps,
           "Más del 2% — revisar USB")

    if len(gaps) > 0:
        worst = gaps.nlargest(3)
        print(f"     Peores gaps (ms): {worst.values.tolist()}")

    # ── 5. Rangos físicos ───────────────────────────────────
    acc_max = df[["ax","ay","az"]].abs().max().max()
    ok_acc  = acc_max < 16.0
    _check(f"Accel. máx = {acc_max:.2f} g  (límite 16 g)", ok_acc,
           "Valores fuera de rango físico")

    gyro_max = df[["gx","gy","gz"]].abs().max().max()
    ok_gyro  = gyro_max < 2000.0
    _check(f"Gyro. máx  = {gyro_max:.1f} °/s  (límite 2000)", ok_gyro,
           "Valores fuera de rango físico")

    # ── 6. acc_norm en reposo ───────────────────────────────
    df["acc_norm"] = np.sqrt(df.ax**2 + df.ay**2 + df.az**2)
    acn_mean = df["acc_norm"].mean()
    acn_std  = df["acc_norm"].std()
    print(f"\n  acc_norm  μ={acn_mean:.3f} g   σ={acn_std:.3f} g")
    if acn_std < 0.05:
        print("  ⚠  Señal casi estática — ¿el sujeto estuvo quieto todo el trial?")

    # ── 7. Varianza por canal ───────────────────────────────
    print("\n  Varianza por canal (señal plana = posible sensor muerto):")
    for col in imu_cols:
        var = df[col].var()
        flag = " ⚠ MUY BAJA" if var < 1e-4 else ""
        print(f"    {col:3s}: {var:.4f}{flag}")

    # ── 8. Duración y metadatos ─────────────────────────────
    dur_s = (df["timestamp_pc_ms"].iloc[-1] - df["timestamp_pc_ms"].iloc[0]) / 1000
    print(f"\n  Duración  : {dur_s:.1f} s")
    if "sensor_id" in df.columns:
        print(f"  sensor_id : {df['sensor_id'].iloc[0]}")
    if "placement" in df.columns:
        print(f"  placement : {df['placement'].iloc[0]}")
    if "trial_id" in df.columns and "trial_id" in df.columns:
        print(f"  trial     : {df['trial_id'].iloc[0]}  "
              f"subject: {df['subject_id'].iloc[0]}")

    print(f"{'─'*50}\n")


def _check(label: str, ok: bool, detail: str = ""):
    icon = "✓" if ok else "✗"
    msg  = f"  {icon}  {label}"
    if not ok and detail:
        msg += f"\n     → {detail}"
    print(msg)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit("Uso: python qc_check.py <ruta_csv>")
    check(sys.argv[1])
