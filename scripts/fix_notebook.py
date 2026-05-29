#!/usr/bin/env python3
"""
fix_notebook.py  — Aplica todas las correcciones al notebook gait_tinyml.ipynb
Uso: python3 scripts/fix_notebook.py

Reescribe celdas completas (no string-replace frágil).
Es idempotente: busca marcadores antes de reescribir.
"""
import json, pathlib, sys

NB = pathlib.Path(__file__).parent.parent / 'training' / 'notebooks' / 'gait_tinyml.ipynb'

def load():
    return json.loads(NB.read_text(encoding='utf-8'))

def save(nb):
    NB.write_text(json.dumps(nb, ensure_ascii=False, indent=1), encoding='utf-8')

def src(nb, i):
    return ''.join(nb['cells'][i]['source'])

def set_cell(nb, i, code, clear_outputs=True):
    nb['cells'][i]['source'] = code.splitlines(keepends=True)
    if clear_outputs:
        nb['cells'][i]['outputs'] = []
        nb['cells'][i]['execution_count'] = None

# ─────────────────────────────────────────────────────────────────────────────
# Patch 1 — Cell 2: Config
# NOTA CRÍTICA sobre sensor_ids:
#   Los datos brutos tienen sensor_id=3 para el tobillo (error hardware).
#   Las celdas EDA [12,17,20,23,26,29,32] remapean 3→1 por sujeto.
#   Por eso el filtro inicial DEBE incluir 3 para que el EDA funcione.
#   Después de EDA: sensor_id=1=tobillo, sensor_id=2=pierna.
# ─────────────────────────────────────────────────────────────────────────────
PATCH1_MARKER = "## CONFIG_FIXED_V3"
PATCH1_CODE = """\
## CONFIG_FIXED_V3
import os
import glob
import warnings
import random
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from collections import Counter
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import confusion_matrix, classification_report
from sklearn.utils.class_weight import compute_class_weight
import tensorflow as tf
from tensorflow.keras.callbacks import EarlyStopping, ReduceLROnPlateau

warnings.filterwarnings('ignore')

# ── Constants ──────────────────────────────────────────────────────────────
WINDOW_SIZE = 120         # samples per window (1.2 s @ 100 Hz) — mismo que modelo original
STEP        = 10          # hop → inferencia cada 100 ms
SAMPLE_MS   = 16          # ms entre muestras (medido: 16 ms = 62.5 Hz real)
SAMPLE_HZ   = 1000 / SAMPLE_MS   # 62.5 Hz

# SENSOR_IDS para el filtro inicial: incluye 3 porque el EDA remapaea 3→1
# (los datos brutos tienen sensor_id=3 para tobillo por error de captura)
# Después del EDA: sensor_id=1=tobillo, sensor_id=2=pierna
SENSOR_IDS  = [1, 2, 3]               # filtro inicial; EDA normaliza a [1,2]
N_FEATURES  = 6 * 2                   # 12: ax,ay,az,gx,gy,gz × 2 sensores

DATA_DIR   = os.path.abspath('../../data/raw')
MODELS_DIR = os.path.abspath('../models')
os.makedirs(MODELS_DIR, exist_ok=True)

PHASES   = ['loading', 'midstance', 'terminal', 'swing']
PHASE2ID = {p: i for i, p in enumerate(PHASES)}
N_CLASSES = len(PHASES)

# ── Reproducibility ────────────────────────────────────────────────────────
SEED = 42
random.seed(SEED)
np.random.seed(SEED)
tf.random.set_seed(SEED)

print(f'TensorFlow : {tf.__version__}')
print(f'Data dir   : {DATA_DIR}')
print(f'Models dir : {MODELS_DIR}')
print(f'SENSOR_IDS : {SENSOR_IDS}  →  {N_FEATURES} features/ventana')
print(f'WINDOW_SIZE: {WINDOW_SIZE} samples  STEP: {STEP}')
print(f'Phases     : {PHASE2ID}')
"""

# ─────────────────────────────────────────────────────────────────────────────
# Patch 2 — Cell 37: align_sensors call
# Hardcodear [1,2] (post-EDA) en lugar de usar SENSOR_IDS=[1,2,3]
# ─────────────────────────────────────────────────────────────────────────────
PATCH2_MARKER = "sensor_ids=[1, 2]"  # post-EDA: tobillo=1, pierna=2

# ─────────────────────────────────────────────────────────────────────────────
# Patch 3 — Cell 40: FEAT_COLS + make_windows con purity filter
# FEAT_COLS usa [1,2] hardcoded: tobillo(s1) primero, pierna(s2) segundo.
# Esto debe coincidir con el orden en que el firmware construye el vector:
#   features[0..5]  = slave tobillo (sensor_id=1)
#   features[6..11] = master pierna (sensor_id=2)
# ─────────────────────────────────────────────────────────────────────────────
PATCH3_MARKER = "## FEAT_COLS_FIXED_V3"
PATCH3_CODE = """\
## FEAT_COLS_FIXED_V3
# Columnas alineadas: tobillo(s1) primero, pierna(s2) segundo
# DEBE coincidir con el orden del firmware inference_master.ino
IMU_COLS  = ['ax', 'ay', 'az', 'gx', 'gy', 'gz']
FEAT_COLS = [f'{c}_s{sid}' for sid in [1, 2] for c in IMU_COLS]
# tobillo (s1): FEAT_COLS[:6]  → features[0..5]
# pierna  (s2): FEAT_COLS[6:]  → features[6..11]
print(f'Feature columns ({len(FEAT_COLS)}): {FEAT_COLS}')


def make_windows(df_aligned, window=WINDOW_SIZE, step=STEP,
                 feat_cols=None, phase2id=None):
    \"\"\"Ventana deslizante sobre cada trial.

    Etiqueta = fase mayoritaria (voto simple). Sin filtro de pureza,
    mismo comportamiento que el pipeline original.
    \"\"\"
    from collections import Counter
    if feat_cols is None: feat_cols = FEAT_COLS
    if phase2id  is None: phase2id  = PHASE2ID

    X_list, y_list, trial_list = [], [], []

    for trial_key, grp in df_aligned.groupby('trial_key'):
        grp = grp.reset_index(drop=True)
        n   = len(grp)
        if n < window:
            continue

        vals   = grp[feat_cols].values.astype(np.float32)
        labels = grp['phase'].values

        for start in range(0, n - window + 1, step):
            win_labels = labels[start:start + window]
            majority   = Counter(win_labels).most_common(1)[0][0]
            if majority not in phase2id:
                continue
            X_list.append(vals[start:start + window])
            y_list.append(phase2id[majority])
            trial_list.append(trial_key)

    X = np.stack(X_list, axis=0)
    y = np.array(y_list, dtype=np.int32)
    return X, y, trial_list


X_all, y_all, trial_keys_all = make_windows(
    df_aligned_clean, window=WINDOW_SIZE, step=STEP,
    feat_cols=FEAT_COLS, phase2id=PHASE2ID
)

from collections import Counter
cnt = Counter(y_all)
print(f'X shape: {X_all.shape}  (ventanas × muestras × features)')
print(f'y shape: {y_all.shape}')
print('\\nVentanas por fase:')
cnt = Counter(y_all)
for pid, pname in enumerate(PHASES):
    print(f'  {pname:12s}: {cnt[pid]:4d}')
"""

# ─────────────────────────────────────────────────────────────────────────────
# Patch 4 — Cell 50: CNN2D residual
# ─────────────────────────────────────────────────────────────────────────────
PATCH4_MARKER = "GaitCNN2D_Residual"
PATCH4_CODE = """\
from tensorflow.keras.layers import (
    Input, Conv2D, BatchNormalization, Activation, Add,
    GlobalAveragePooling2D, Dense, Dropout
)


def _res_block(x, filters, kernel):
    \"\"\"Bloque residual 2D: Conv→BN→ReLU→Conv→BN + skip.\"\"\"
    skip = x
    if x.shape[-1] != filters:
        skip = Conv2D(filters, (1, 1), padding='same')(x)
        skip = BatchNormalization()(skip)
    x = Conv2D(filters, kernel, padding='same')(x)
    x = BatchNormalization()(x)
    x = Activation('relu')(x)
    x = Conv2D(filters, kernel, padding='same')(x)
    x = BatchNormalization()(x)
    x = Add()([x, skip])
    x = Activation('relu')(x)
    return x


def build_cnn2d(window=WINDOW_SIZE, n_features=N_FEATURES, n_classes=N_CLASSES):
    \"\"\"
    GaitCNN2D_Residual — Input (1, T=50, F=12, 1)
    Arquitectura:
      Stem Conv2D(32, 5×3) → ResBlock(32, 3×3) → ResBlock(64, 3×3)
      → ResBlock(64, 5×1) → GAP → Dense(64) → Drop(0.5)
      → Dense(32) → Drop(0.3) → Softmax(4)
    \"\"\"
    inp = Input((window, n_features, 1), name='imu_image')
    x = Conv2D(32, (5, 3), padding='same', name='stem')(inp)
    x = BatchNormalization(name='stem_bn')(x)
    x = Activation('relu', name='stem_relu')(x)
    x = _res_block(x, 32, (3, 3))
    x = _res_block(x, 64, (3, 3))
    x = _res_block(x, 64, (5, 1))
    x = GlobalAveragePooling2D(name='gap')(x)
    x = Dense(64, activation='relu', name='fc1')(x)
    x = Dropout(0.5, name='drop1')(x)
    x = Dense(32, activation='relu', name='fc2')(x)
    x = Dropout(0.3, name='drop2')(x)
    out = Dense(n_classes, activation='softmax', name='predictions')(x)
    return tf.keras.Model(inp, out, name='GaitCNN2D_Residual')


# ── Selección de arquitectura ─────────────────────────────────────────────
ARCH = 'cnn2d'   # 'cnn2d' | 'multibranch'

if ARCH == 'multibranch':
    from tensorflow.keras.layers import Conv1D, GlobalAveragePooling1D, Concatenate
    def build_multibranch(window=WINDOW_SIZE, n_axes=6, n_classes=N_CLASSES):
        def branch(inp, name):
            x = Conv1D(32, 5, padding='same', activation='relu', name=f'c1_{name}')(inp)
            x = BatchNormalization(name=f'bn1_{name}')(x)
            x = Conv1D(64, 3, padding='same', activation='relu', name=f'c2_{name}')(x)
            x = BatchNormalization(name=f'bn2_{name}')(x)
            return GlobalAveragePooling1D(name=f'gap_{name}')(x)
        s1 = Input((window, n_axes), name='sensor_a')
        s2 = Input((window, n_axes), name='sensor_b')
        x  = Concatenate(name='fusion')([branch(s1, 'sa'), branch(s2, 'sb')])
        x  = Dense(64, activation='relu', name='fc1')(x)
        x  = Dropout(0.5, name='drop')(x)
        out = Dense(n_classes, activation='softmax', name='out')(x)
        return tf.keras.Model([s1, s2], out, name='GaitMultiBranch')
    model = build_multibranch()
    s1_idx = list(range(0, 6))
    s2_idx = list(range(6, 12))
    X_train_in = [X_train_bal[:, :, s1_idx], X_train_bal[:, :, s2_idx]]
    X_val_in   = [X_val_scaled[:, :, s1_idx], X_val_scaled[:, :, s2_idx]]
    X_test_in  = [X_test_scaled[:, :, s1_idx], X_test_scaled[:, :, s2_idx]]
else:
    model = build_cnn2d()

model.summary()
p = model.count_params()
print(f'\\nParámetros totales: {p:,}')
print(f'Float32           : {p*4/1024:.1f} KB')
print(f'Int8 estimado     : {p/1024:.1f} KB')
"""

# ─────────────────────────────────────────────────────────────────────────────
# Patch 5 — Cell 52: AdamW + epochs=150
# ─────────────────────────────────────────────────────────────────────────────
PATCH5_MARKER = "## COMPILE_ADAMW_V1"
PATCH5_CODE = """\
## COMPILE_ADAMW_V1
model.compile(
    optimizer=tf.keras.optimizers.AdamW(learning_rate=1e-3, weight_decay=1e-4),
    loss=FOCAL_LOSS,
    metrics=['accuracy']
)

callbacks = [
    tf.keras.callbacks.EarlyStopping(
        monitor='val_loss', patience=20,
        restore_best_weights=True, verbose=1
    ),
    tf.keras.callbacks.ReduceLROnPlateau(
        monitor='val_loss', factor=0.5,
        patience=8, min_lr=1e-6, verbose=1
    ),
]

val_data = (X_val_in, y_val_oh) if len(X_val) > 0 else None

history = model.fit(
    X_train_in, y_train_bal_oh,
    validation_data=val_data,
    epochs=150,
    batch_size=32,
    callbacks=callbacks,
    verbose=1
)

print(f'\\nEntrenamiento completo: {len(history.history["loss"])} epocas')
"""

# ─────────────────────────────────────────────────────────────────────────────
# Patch 6 — Cell 69: export CNN2D int8 con nombre correcto
# ─────────────────────────────────────────────────────────────────────────────
PATCH6_MARKER = "## CNN2D_EXPORT_V2"
PATCH6_CODE = """\
## CNN2D_EXPORT_V2
import os
int8_tflite_path = os.path.join(MODELS_DIR, 'gait_cnn2d_int8.tflite')

N_CALIB   = min(200, len(X_train_bal))
calib_idx = np.random.choice(len(X_train_bal), N_CALIB, replace=False)

if isinstance(X_train_in, list):
    calib_s1 = X_train_in[0][calib_idx].astype(np.float32)
    calib_s2 = X_train_in[1][calib_idx].astype(np.float32)
    def representative_dataset():
        for i in range(N_CALIB):
            yield [calib_s1[i:i+1], calib_s2[i:i+1]]
else:
    calib_data = X_train_in[calib_idx].astype(np.float32)
    def representative_dataset():
        for i in range(N_CALIB):
            yield [calib_data[i:i+1]]

converter_int8 = tf.lite.TFLiteConverter.from_keras_model(model)
converter_int8.optimizations            = [tf.lite.Optimize.DEFAULT]
converter_int8.representative_dataset  = representative_dataset
converter_int8.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter_int8.inference_input_type    = tf.int8
converter_int8.inference_output_type   = tf.int8

tflite_int8 = converter_int8.convert()
with open(int8_tflite_path, 'wb') as f:
    f.write(tflite_int8)

# Verificar
import tensorflow as tf as _tf
interp = _tf.lite.Interpreter(model_content=tflite_int8)
interp.allocate_tensors()
inp_d = interp.get_input_details()[0]
out_d = interp.get_output_details()[0]
print(f'gait_cnn2d_int8.tflite  → {len(tflite_int8)/1024:.1f} KB')
print(f'  input : {tuple(inp_d["shape"])}  dtype={inp_d["dtype"].__name__}')
print(f'  output: {tuple(out_d["shape"])}  quant={out_d["quantization"]}')
"""

# Fix the import error in PATCH6
PATCH6_CODE = """\
## CNN2D_EXPORT_V2
import os
int8_tflite_path = os.path.join(MODELS_DIR, 'gait_cnn2d_int8.tflite')

N_CALIB   = min(200, len(X_train_bal))
calib_idx = np.random.choice(len(X_train_bal), N_CALIB, replace=False)

if isinstance(X_train_in, list):
    calib_s1 = X_train_in[0][calib_idx].astype(np.float32)
    calib_s2 = X_train_in[1][calib_idx].astype(np.float32)
    def representative_dataset():
        for i in range(N_CALIB):
            yield [calib_s1[i:i+1], calib_s2[i:i+1]]
else:
    calib_data = X_train_in[calib_idx].astype(np.float32)
    def representative_dataset():
        for i in range(N_CALIB):
            yield [calib_data[i:i+1]]

converter_int8 = tf.lite.TFLiteConverter.from_keras_model(model)
converter_int8.optimizations            = [tf.lite.Optimize.DEFAULT]
converter_int8.representative_dataset  = representative_dataset
converter_int8.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter_int8.inference_input_type    = tf.int8
converter_int8.inference_output_type   = tf.int8

tflite_int8 = converter_int8.convert()
with open(int8_tflite_path, 'wb') as f:
    f.write(tflite_int8)

interp = tf.lite.Interpreter(model_content=tflite_int8)
interp.allocate_tensors()
inp_d = interp.get_input_details()[0]
out_d = interp.get_output_details()[0]
print(f'gait_cnn2d_int8.tflite  → {len(tflite_int8)/1024:.1f} KB')
print(f'  input : {tuple(inp_d["shape"])}   dtype={inp_d["dtype"].__name__}')
print(f'  output: {tuple(out_d["shape"])}   quant={out_d["quantization"]}')
"""

# ─────────────────────────────────────────────────────────────────────────────
# Patch 7 — Cell 72: gait_model.h (nombre correcto + alignas)
# ─────────────────────────────────────────────────────────────────────────────
PATCH7_MARKER = "## GAIT_MODEL_H_V2"
PATCH7_CODE = """\
## GAIT_MODEL_H_V2
def tflite_to_c_header(tflite_path, var_name, out_path):
    \"\"\"Convierte .tflite → array uint8 en C con alignas(8).\"\"\"
    with open(tflite_path, 'rb') as f:
        data = f.read()
    lines = [
        f'// Auto-generated by FusionGait training pipeline',
        f'// Model  : {os.path.basename(tflite_path)}',
        f'// Size   : {len(data):,} bytes ({len(data)/1024:.1f} KB)',
        f'// WINDOW_SIZE={WINDOW_SIZE}  N_FEATURES={N_FEATURES}  N_CLASSES={N_CLASSES}',
        '#pragma once',
        '#include <stdint.h>',
        '',
        f'alignas(8) const uint8_t {var_name}[] = {{',
    ]
    for i in range(0, len(data), 12):
        chunk = data[i:i+12]
        lines.append('  ' + ', '.join(f'0x{b:02x}' for b in chunk) + ',')
    lines.append('};')
    lines.append(f'const unsigned int {var_name}_len = {len(data)};')
    with open(out_path, 'w') as f:
        f.write('\\n'.join(lines) + '\\n')
    print(f'  {out_path}  ({len(data)/1024:.1f} KB)')
    return len(data)


c_header_path = os.path.join(MODELS_DIR, 'gait_model.h')
tflite_to_c_header(int8_tflite_path, 'gait_model', c_header_path)
print(f'Header C: {c_header_path}')
"""

# ─────────────────────────────────────────────────────────────────────────────
# Patch 8 — Cell 74: scaler export con nombre correcto
# ─────────────────────────────────────────────────────────────────────────────
PATCH8_MARKER = "## SCALER_EXPORT_V2"
PATCH8_CODE = """\
## SCALER_EXPORT_V2
scaler_mean = scaler.mean_.astype(np.float32)
scaler_std  = scaler.scale_.astype(np.float32)

print(f'SCALER_MEAN shape: {scaler_mean.shape}')
print(f'SCALER_STD  shape: {scaler_std.shape}')
print(f'Features   : {FEAT_COLS}')

def arr_to_c(arr, name):
    vals = ', '.join(f'{v:.8f}f' for v in arr)
    return f'const float {name}[{len(arr)}] = {{{vals}}};'

print('\\n// Para scaler.h del Arduino:')
print(f'// Feature order: {FEAT_COLS}')
print(arr_to_c(scaler_mean, 'SCALER_MEAN'))
print(arr_to_c(scaler_std,  'SCALER_STD'))

# Guardar como .npz (lo usa export_to_arduino.py)
npz_path = os.path.join(MODELS_DIR, 'scaler_params_2s.npz')
np.savez(npz_path,
         mean=scaler_mean,
         std=scaler_std,
         feature_cols=np.array(FEAT_COLS))
print(f'\\nScaler guardado: {npz_path}')

# Generar scaler.h directamente
scaler_h_path = os.path.join(MODELS_DIR, 'scaler.h')
lines = [
    '// Auto-generated by FusionGait training pipeline — DO NOT EDIT',
    f'// Source: {os.path.basename(npz_path)}',
    f'// Features ({len(FEAT_COLS)}): {", ".join(FEAT_COLS)}',
    '#pragma once',
    '',
    '// Z-score: norm[i] = (x[i] - SCALER_MEAN[i]) / (SCALER_STD[i] + 1e-8f)',
    arr_to_c(scaler_mean, 'SCALER_MEAN'),
    arr_to_c(scaler_std,  'SCALER_STD'),
    '',
]
with open(scaler_h_path, 'w') as f:
    f.write('\\n'.join(lines))
print(f'scaler.h guardado  : {scaler_h_path}')
"""

# ─────────────────────────────────────────────────────────────────────────────
# Apply patches
# ─────────────────────────────────────────────────────────────────────────────
def apply(nb):
    applied = []

    # Patch 1 – cell 2
    if PATCH1_MARKER not in src(nb, 2):
        set_cell(nb, 2, PATCH1_CODE)
        applied.append('P1: cell[2] config SENSOR_IDS=[1,2,3], STEP=10')
    else:
        print('  skip P1 (ya aplicado)')

    # Patch 2 – cell 37: fix align_sensors call to hardcoded [1,2] (post-EDA)
    s37 = src(nb, 37)
    if PATCH2_MARKER not in s37:
        # Reemplaza cualquier variante de la llamada con la versión correcta
        new37 = s37
        for old in [
            'df_aligned = align_sensors(df, sensor_ids=SENSOR_IDS)',
            'df_aligned = align_sensors(df, sensor_ids=[1,2])',
            'df_aligned = align_sensors(df, sensor_ids=[2, 3])',
            'df_aligned = align_sensors(df, sensor_ids=[1, 2, 3])',
        ]:
            if old in new37:
                new37 = new37.replace(old,
                    'df_aligned = align_sensors(df, sensor_ids=[1, 2])  # tobillo=1, pierna=2 (post-EDA)')
                break
        set_cell(nb, 37, new37)
        applied.append('P2: cell[37] sensor_ids=[1, 2]  (post-EDA hardcoded)')
    else:
        print('  skip P2 (ya aplicado)')

    # Patch 3 – cell 40
    if PATCH3_MARKER not in src(nb, 40):
        set_cell(nb, 40, PATCH3_CODE)
        applied.append('P3: cell[40] FEAT_COLS=[s1,s2] tobillo-first + purity filter')
    else:
        print('  skip P3 (ya aplicado)')

    # Patch 4 – cell 50
    if PATCH4_MARKER not in src(nb, 50):
        set_cell(nb, 50, PATCH4_CODE)
        applied.append('P4: cell[50] GaitCNN2D_Residual')
    else:
        print('  skip P4 (ya aplicado)')

    # Patch 5 – cell 52
    if PATCH5_MARKER not in src(nb, 52):
        set_cell(nb, 52, PATCH5_CODE)
        applied.append('P5: cell[52] AdamW + epochs=150')
    else:
        print('  skip P5 (ya aplicado)')

    # Patch 6 – cell 69
    if PATCH6_MARKER not in src(nb, 69):
        set_cell(nb, 69, PATCH6_CODE)
        applied.append('P6: cell[69] export gait_cnn2d_int8.tflite')
    else:
        print('  skip P6 (ya aplicado)')

    # Patch 7 – cell 72
    if PATCH7_MARKER not in src(nb, 72):
        set_cell(nb, 72, PATCH7_CODE)
        applied.append('P7: cell[72] tflite_to_c_header → gait_model.h')
    else:
        print('  skip P7 (ya aplicado)')

    # Patch 8 – cell 74
    if PATCH8_MARKER not in src(nb, 74):
        set_cell(nb, 74, PATCH8_CODE)
        applied.append('P8: cell[74] scaler export + scaler.h')
    else:
        print('  skip P8 (ya aplicado)')

    return applied


if __name__ == '__main__':
    nb = load()
    print(f'Notebook: {NB}  ({len(nb["cells"])} celdas)\n')
    applied = apply(nb)
    save(nb)
    if applied:
        print('\nPatches aplicados:')
        for msg in applied: print(f'  ✅  {msg}')
    else:
        print('Todos los patches ya estaban aplicados.')
    print(f'\nTotal aplicados: {len(applied)}/8')
    print('\nSiguiente paso:')
    print('  Abre gait_tinyml.ipynb → Kernel → Restart & Run All')
    print('  Luego: python3 scripts/export_to_arduino.py')
