#!/usr/bin/env python3
"""
add_spectral_pipeline.py
========================
Inyecta en gait_tinyml.ipynb un pipeline de extracción espectral
al estilo Edge Impulse (FFT + estadísticos) + verificación de
exportación para 2 Arduinos (master pierna, slave tobillo).

Uso:
    cd /Users/diegovillaba/Code/FusionGait/training/notebooks
    python3 add_spectral_pipeline.py

El script es idempotente: si ya existen las celdas con el marcador
## SPECTRAL_PIPELINE_V1 no las duplica.
"""
import json, textwrap, sys, pathlib

NB_PATH = pathlib.Path(__file__).parent / "gait_tinyml.ipynb"
MARKER  = "## SPECTRAL_PIPELINE_V1"

# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────
def md_cell(text: str) -> dict:
    return {"cell_type": "markdown", "metadata": {},
            "source": text.splitlines(keepends=True)}

def code_cell(text: str) -> dict:
    return {"cell_type": "code", "metadata": {}, "execution_count": None,
            "outputs": [], "source": text.splitlines(keepends=True)}

def already_injected(nb: dict) -> bool:
    for cell in nb["cells"]:
        if MARKER in "".join(cell["source"]):
            return True
    return False

def find_insert_idx(nb: dict) -> int:
    """Inserta después de la última celda que entrena o compila el modelo."""
    target_keywords = ["model.fit", "history =", "val_loss", "FOCAL_LOSS"]
    last_idx = 0
    for i, cell in enumerate(nb["cells"]):
        src = "".join(cell["source"])
        if any(kw in src for kw in target_keywords):
            last_idx = i
    return last_idx + 1   # insertar justo después del bloque de entrenamiento

# ─────────────────────────────────────────────────────────────────────────────
# Celdas nuevas
# ─────────────────────────────────────────────────────────────────────────────
NEW_CELLS = [

# ── Separador markdown ──────────────────────────────────────────────────────
md_cell("""\
---
## 6 · Extractor espectral (Edge Impulse–style) + modelo MLP compacto
## SPECTRAL_PIPELINE_V1

Edge Impulse genera automáticamente features espectrales (FFT + estadísticos)
antes del clasificador.  Aquí reproducimos ese pipeline para ver si mejora
la separación de fases y reducir el tamaño del modelo embebido.

### Por qué puede ayudar
| Dominio | Captura | Límite |
|---|---|---|
| Tiempo crudo (CNN) | patrones de forma de onda | pesado (~100+ KB int8) |
| Frecuencia (FFT→MLP) | energía por banda, armónicos | compacto (~10–30 KB int8) |

Estrategia: extraer `n_fft_bins` energías espectrales + 4 estadísticos
(media, std, RMS, rango) por canal → vector 1-D → MLP de 3 capas.
"""),

# ── Celda 1: extracción espectral ────────────────────────────────────────────
code_cell("""\
## SPECTRAL_PIPELINE_V1  –  Extracción de features espectrales
import numpy as np

# ── Parámetros del extractor ──────────────────────────────────────────────
N_FFT_BINS  = 10      # bins FFT por canal (Edge Impulse usa 10–16)
FREQ_AXIS   = np.fft.rfftfreq(WINDOW_SIZE, d=1.0/SAMPLE_HZ)  # Hz
BIN_EDGES   = np.linspace(0, SAMPLE_HZ / 2, N_FFT_BINS + 1)  # bandas uniformes

def spectral_features(window: np.ndarray, n_bins: int = N_FFT_BINS) -> np.ndarray:
    \"\"\"
    Extrae features espectrales + estadísticos de una ventana IMU.

    Parámetros
    ----------
    window : ndarray (T, F)  –  T timesteps, F canales
    n_bins : int             –  número de bandas espectrales por canal

    Retorna
    -------
    feats : ndarray (F * (n_bins + 4),)
        Por cada canal: energía en n_bins bandas + [mean, std, rms, p2p]
    \"\"\"
    T, F = window.shape
    feats = []
    freqs = np.fft.rfftfreq(T, d=1.0 / SAMPLE_HZ)   # eje frecuencial

    for ch in range(F):
        sig = window[:, ch]

        # 1) Estadísticos temporales
        mu    = sig.mean()
        sigma = sig.std()
        rms   = np.sqrt((sig ** 2).mean())
        p2p   = sig.max() - sig.min()

        # 2) FFT → energía por banda
        fft_mag = np.abs(np.fft.rfft(sig - mu))   # DC-free
        bin_idxs = np.digitize(freqs, BIN_EDGES, right=False).clip(1, n_bins)
        band_energy = np.array([
            fft_mag[bin_idxs == b].sum() / T
            for b in range(1, n_bins + 1)
        ])

        feats.extend([mu, sigma, rms, p2p])
        feats.extend(band_energy.tolist())

    return np.array(feats, dtype=np.float32)


# ── Aplicar sobre todo el dataset ────────────────────────────────────────────
def extract_spectral_dataset(X: np.ndarray, n_bins: int = N_FFT_BINS) -> np.ndarray:
    \"\"\"X: (N, T, F)  →  Xf: (N, F*(n_bins+4))\"\"\"
    return np.stack([spectral_features(x, n_bins) for x in X], axis=0)

SPECTRAL_DIM = N_FEATURES * (N_FFT_BINS + 4)   # = 12 * 14 = 168 con defaults
print(f"Dimensión vector espectral por ventana : {SPECTRAL_DIM}")
print(f"  {N_FEATURES} canales × ({N_FFT_BINS} bins FFT + 4 estadísticos)")
"""),

# ── Celda 2: split + extracción ──────────────────────────────────────────────
code_cell("""\
## SPECTRAL_PIPELINE_V1  –  Split y extracción de features

# Reutilizamos los mismos índices trial que el modelo CNN2D (ya partidos)
# Si el split anterior no está en memoria, volvemos a hacerlo aquí.
import numpy as np
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import GroupShuffleSplit

# ── Recuperar split por trial ─────────────────────────────────────────────
try:
    _ = X_train.shape   # ya existe de la celda anterior
    print("Usando splits CNN2D existentes")
    X_tr_raw, X_val_raw, X_te_raw = X_train, X_val, X_test
    y_tr, y_va, y_te = y_train, y_val, y_test
except NameError:
    print("Rehaciendo split por trial (GroupShuffleSplit 70/15/15)…")
    trial_arr = np.array(trial_keys_all)
    gss1 = GroupShuffleSplit(n_splits=1, test_size=0.30, random_state=SEED)
    tr_idx, tmp_idx = next(gss1.split(X_all, y_all, groups=trial_arr))
    gss2 = GroupShuffleSplit(n_splits=1, test_size=0.50, random_state=SEED)
    va_idx, te_idx = next(gss2.split(
        X_all[tmp_idx], y_all[tmp_idx], groups=trial_arr[tmp_idx]))
    va_idx = tmp_idx[va_idx]; te_idx = tmp_idx[te_idx]

    X_tr_raw = X_all[tr_idx];   y_tr = y_all[tr_idx]
    X_val_raw = X_all[va_idx];  y_va = y_all[va_idx]
    X_te_raw  = X_all[te_idx];  y_te = y_all[te_idx]

# ── Extraer features espectrales ──────────────────────────────────────────
print("Extrayendo features espectrales…")
Xf_train = extract_spectral_dataset(X_tr_raw)
Xf_val   = extract_spectral_dataset(X_val_raw)
Xf_test  = extract_spectral_dataset(X_te_raw)

# ── Normalizar el vector espectral ────────────────────────────────────────
scaler_sp = StandardScaler()
Xf_train_sc = scaler_sp.fit_transform(Xf_train)
Xf_val_sc   = scaler_sp.transform(Xf_val)
Xf_test_sc  = scaler_sp.transform(Xf_test)

print(f"Xf_train : {Xf_train_sc.shape}")
print(f"Xf_val   : {Xf_val_sc.shape}")
print(f"Xf_test  : {Xf_test_sc.shape}")
"""),

# ── Celda 3: visualización espectral ─────────────────────────────────────────
code_cell("""\
## SPECTRAL_PIPELINE_V1  –  Visualización: centroide espectral por fase

import matplotlib.pyplot as plt
import numpy as np

PHASE_COLORS_LOCAL = {
    'loading':    '#2196F3',
    'midstance':  '#4CAF50',
    'terminal':   '#FF9800',
    'swing':      '#9C27B0',
}

# Calcular espectro promedio por fase (primeros N_FFT_BINS bins del canal ax_s2)
spectral_by_phase = {}
for pid, pname in enumerate(PHASES):
    mask = y_tr == pid
    if mask.sum() == 0: continue
    windows_fase = X_tr_raw[mask]          # (N_fase, 50, 12)
    ch0 = windows_fase[:, :, 0]            # ax del sensor 1 (s2)
    spectra = np.abs(np.fft.rfft(ch0 - ch0.mean(axis=1, keepdims=True), axis=1))
    spectral_by_phase[pname] = spectra.mean(axis=0)

freqs_plot = np.fft.rfftfreq(WINDOW_SIZE, d=1.0/SAMPLE_HZ)

fig, axes = plt.subplots(1, 2, figsize=(14, 4))

# ── Panel 1: espectro promedio por fase ───────────────────────────────────
ax = axes[0]
for pname, spec in spectral_by_phase.items():
    ax.plot(freqs_plot, spec, color=PHASE_COLORS_LOCAL[pname], label=pname, lw=1.8)
ax.set_xlabel('Frecuencia (Hz)')
ax.set_ylabel('Magnitud media |FFT|')
ax.set_title('Espectro promedio por fase — canal ax_s2 (pierna)')
ax.legend(); ax.grid(alpha=0.3)

# ── Panel 2: heatmap de energía espectral por fase y banda ────────────────
ax = axes[1]
band_freqs = [(BIN_EDGES[i]+BIN_EDGES[i+1])/2 for i in range(N_FFT_BINS)]
heatmap = np.zeros((len(PHASES), N_FFT_BINS))
for pid, pname in enumerate(PHASES):
    if pname not in spectral_by_phase: continue
    spec = spectral_by_phase[pname]
    for b in range(N_FFT_BINS):
        lo = np.searchsorted(freqs_plot, BIN_EDGES[b])
        hi = np.searchsorted(freqs_plot, BIN_EDGES[b+1])
        heatmap[pid, b] = spec[lo:hi].sum()

im = ax.imshow(heatmap, aspect='auto', cmap='viridis')
ax.set_xticks(range(N_FFT_BINS))
ax.set_xticklabels([f'{f:.1f}' for f in band_freqs], rotation=45, ha='right')
ax.set_yticks(range(len(PHASES)))
ax.set_yticklabels(PHASES)
ax.set_title('Energía por banda (ax_s2)')
ax.set_xlabel('Freq. central de banda (Hz)')
plt.colorbar(im, ax=ax, label='Energía acumulada')

plt.suptitle('Análisis espectral de fases de marcha', fontsize=13, y=1.01)
plt.tight_layout()
plt.show()
"""),

# ── Celda 4: MLP espectral ───────────────────────────────────────────────────
code_cell("""\
## SPECTRAL_PIPELINE_V1  –  Modelo MLP sobre features espectrales

import tensorflow as tf
from tensorflow.keras.layers import Input, Dense, BatchNormalization, Dropout, Activation
from tensorflow.keras.callbacks import EarlyStopping, ReduceLROnPlateau

# ── Arquitectura MLP (3 capas ocultas) ───────────────────────────────────
def build_spectral_mlp(input_dim=SPECTRAL_DIM, n_classes=N_CLASSES,
                       label_smoothing=0.10):
    \"\"\"
    MLP compacto tipo Edge Impulse para features espectrales.
    Input  : (batch, SPECTRAL_DIM)  →  vector 1-D ya normalizado
    Output : (batch, N_CLASSES)

    Diseño deliberadamente pequeño para caber en Arduino Nano 33 BLE Sense
    (<256 KB Flash, ~32 KB SRAM).
    \"\"\"
    inp = Input((input_dim,), name='spectral_features')

    x = Dense(128, name='fc1')(inp)
    x = BatchNormalization(name='bn1')(x)
    x = Activation('relu', name='act1')(x)
    x = Dropout(0.40, name='drop1')(x)

    x = Dense(64, name='fc2')(x)
    x = BatchNormalization(name='bn2')(x)
    x = Activation('relu', name='act2')(x)
    x = Dropout(0.30, name='drop2')(x)

    x = Dense(32, name='fc3')(x)
    x = BatchNormalization(name='bn3')(x)
    x = Activation('relu', name='act3')(x)

    out = Dense(n_classes, activation='softmax', name='predictions')(x)

    model = tf.keras.Model(inp, out, name='GaitSpectralMLP')

    model.compile(
        optimizer=tf.keras.optimizers.AdamW(learning_rate=5e-4, weight_decay=1e-4),
        loss=tf.keras.losses.CategoricalCrossentropy(label_smoothing=label_smoothing),
        metrics=['accuracy']
    )
    return model


mlp_sp = build_spectral_mlp()
mlp_sp.summary()
p = mlp_sp.count_params()
print(f'\\nParámetros totales : {p:,}')
print(f'Float32            : {p*4/1024:.1f} KB')
print(f'Int8 (estimado)    : {p/1024:.1f} KB')
"""),

# ── Celda 5: entrenamiento MLP ───────────────────────────────────────────────
code_cell("""\
## SPECTRAL_PIPELINE_V1  –  Entrenamiento MLP espectral

from tensorflow.keras.utils import to_categorical
from sklearn.utils.class_weight import compute_class_weight
import numpy as np

# ── One-hot ───────────────────────────────────────────────────────────────
Y_tr_oh  = to_categorical(y_tr,  N_CLASSES)
Y_val_oh = to_categorical(y_va,  N_CLASSES)
Y_te_oh  = to_categorical(y_te,  N_CLASSES)

# ── Pesos de clase (misma lógica que CNN2D) ───────────────────────────────
cw = compute_class_weight('balanced', classes=np.unique(y_tr), y=y_tr)
class_weight_dict = dict(enumerate(cw))
print("Class weights:", {PHASES[k]: f'{v:.2f}' for k,v in class_weight_dict.items()})

# ── Callbacks ─────────────────────────────────────────────────────────────
cb_es  = EarlyStopping(monitor='val_loss', patience=20, restore_best_weights=True)
cb_rlr = ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=8, min_lr=1e-6)

# ── Entrenar ──────────────────────────────────────────────────────────────
history_sp = mlp_sp.fit(
    Xf_train_sc, Y_tr_oh,
    validation_data=(Xf_val_sc, Y_val_oh),
    epochs=200,
    batch_size=64,
    class_weight=class_weight_dict,
    callbacks=[cb_es, cb_rlr],
    verbose=1
)
print("Entrenamiento completado.")
"""),

# ── Celda 6: evaluación y comparación ───────────────────────────────────────
code_cell("""\
## SPECTRAL_PIPELINE_V1  –  Evaluación y comparación CNN2D vs MLP espectral

import matplotlib.pyplot as plt
import numpy as np
from sklearn.metrics import classification_report, confusion_matrix
import seaborn as sns

PHASE_COLORS_LOCAL = {
    'loading':'#2196F3','midstance':'#4CAF50','terminal':'#FF9800','swing':'#9C27B0'
}

# ── Predecir con MLP espectral ────────────────────────────────────────────
y_pred_sp = mlp_sp.predict(Xf_test_sc, verbose=0).argmax(axis=1)
report_sp  = classification_report(y_te, y_pred_sp, target_names=PHASES, output_dict=True)

# ── Predecir con CNN2D (si existe) ───────────────────────────────────────
try:
    y_pred_cnn = model.predict(
        X_test_scaled[..., np.newaxis] if X_test_scaled.ndim == 3 else X_test_scaled,
        verbose=0
    ).argmax(axis=1)
    report_cnn = classification_report(y_te if hasattr(y_te,'shape') else y_test,
                                        y_pred_cnn, target_names=PHASES, output_dict=True)
    have_cnn = True
except Exception as e:
    print(f"CNN2D no disponible en memoria ({e}), mostrando solo MLP.")
    have_cnn = False

# ── Tabla comparativa ─────────────────────────────────────────────────────
print("\\n{'─'*60}")
print(f"{'Fase':<14} {'MLP-F1':>8}", end="")
if have_cnn: print(f" {'CNN2D-F1':>10}", end="")
print()
print('─'*60)
for ph in PHASES:
    sp_f1  = report_sp[ph]['f1-score']
    line   = f"{ph:<14} {sp_f1:>8.3f}"
    if have_cnn:
        cnn_f1 = report_cnn[ph]['f1-score']
        diff   = sp_f1 - cnn_f1
        arrow  = '▲' if diff > 0.01 else ('▼' if diff < -0.01 else '≈')
        line  += f" {cnn_f1:>10.3f}  {arrow}{abs(diff):.3f}"
    print(line)
print('─'*60)
print(f"{'Overall acc':<14} {report_sp['accuracy']:>8.3f}", end="")
if have_cnn: print(f" {report_cnn['accuracy']:>10.3f}", end="")
print()

# ── Curvas de entrenamiento MLP ───────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(13, 4))

ax = axes[0]
ax.plot(history_sp.history['loss'],     label='Train', color='#2196F3')
ax.plot(history_sp.history['val_loss'], label='Val',   color='#FF9800', ls='--')
ax.set_title('MLP Espectral — Pérdida'); ax.set_xlabel('Epoch')
ax.legend(); ax.grid(alpha=0.3)

ax = axes[1]
ax.plot(history_sp.history['accuracy'],     label='Train', color='#2196F3')
ax.plot(history_sp.history['val_accuracy'], label='Val',   color='#FF9800', ls='--')
ax.set_title('MLP Espectral — Accuracy'); ax.set_xlabel('Epoch')
ax.legend(); ax.grid(alpha=0.3)

plt.suptitle('Entrenamiento GaitSpectralMLP', fontsize=13)
plt.tight_layout(); plt.show()

# ── Matriz de confusión MLP ───────────────────────────────────────────────
cm_sp = confusion_matrix(y_te, y_pred_sp)
cm_sp_norm = cm_sp.astype(float) / cm_sp.sum(axis=1, keepdims=True)

fig, ax = plt.subplots(figsize=(7, 5))
sns.heatmap(cm_sp_norm, annot=True, fmt='.2f', cmap='Blues',
            xticklabels=PHASES, yticklabels=PHASES, ax=ax,
            annot_kws={'size': 11})
ax.set_title('MLP Espectral — Matriz de Confusión (normalizada)')
ax.set_ylabel('Real'); ax.set_xlabel('Predicho')
plt.tight_layout(); plt.show()
"""),

# ── Separador exportación ────────────────────────────────────────────────────
md_cell("""\
---
## 7 · Exportación a TFLite int8 y verificación para Arduino

### Arquitectura de despliegue
```
┌─────────────────────────────────────┐
│  Arduino Nano 33 BLE Sense (MASTER) │  ← pierna (sensor_id=2)
│  • Lee su propio IMU @ 100 Hz       │
│  • Recibe BLE packet del slave      │
│  • Ejecuta el modelo TFLite int8    │
│  • Emite: fase_predicha (0-3)       │
└────────────────┬────────────────────┘
                 │ BLE (6×float32 comprimido)
┌────────────────▼────────────────────┐
│  Arduino Nano 33 BLE Sense (SLAVE)  │  ← tobillo (sensor_id=3)
│  • Lee su propio IMU @ 100 Hz       │
│  • Envía ventana cada STEP samples  │
└─────────────────────────────────────┘
```

**El modelo espectral tiene ventajas claras para embebido:**
- El SLAVE solo necesita enviar el **vector de features extraído** (168 floats × 4 bytes = 672 bytes)
  en vez de la ventana cruda (50 × 6 × 4 = 1200 bytes).
- El MASTER concatena sus propias features + las del SLAVE → 168+168=336 features → MLP.
- Reducción de ~44 % en payload BLE por paquete.
"""),

# ── Celda exportación ────────────────────────────────────────────────────────
code_cell("""\
## SPECTRAL_PIPELINE_V1  –  Exportación TFLite int8 y check de despliegue

import tensorflow as tf
import numpy as np
import os, struct

MODELS_DIR_PATH = os.path.abspath('../../training/models')
os.makedirs(MODELS_DIR_PATH, exist_ok=True)

def export_tflite_int8(keras_model, X_ref: np.ndarray,
                       name: str, models_dir: str) -> dict:
    \"\"\"
    Convierte un modelo Keras a TFLite int8 con cuantización completa.

    Parámetros
    ----------
    keras_model : tf.keras.Model
    X_ref       : datos representativos (subset de train) para calibrar int8
    name        : nombre base del archivo (sin extensión)
    models_dir  : carpeta destino

    Retorna
    -------
    dict con info: size_bytes, size_kb, input_shape, output_shape,
                   input_dtype, output_dtype
    \"\"\"
    def representative_dataset():
        idx = np.random.choice(len(X_ref), size=min(200, len(X_ref)), replace=False)
        for i in idx:
            sample = X_ref[i:i+1].astype(np.float32)
            yield [sample]

    converter = tf.lite.TFLiteConverter.from_keras_model(keras_model)
    converter.optimizations         = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = representative_dataset
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type   = tf.int8
    converter.inference_output_type  = tf.int8

    tflite_model = converter.convert()

    out_path = os.path.join(models_dir, f'{name}.tflite')
    with open(out_path, 'wb') as f:
        f.write(tflite_model)

    # Verificar con intérprete
    interp = tf.lite.Interpreter(model_content=tflite_model)
    interp.allocate_tensors()
    inp_det = interp.get_input_details()[0]
    out_det = interp.get_output_details()[0]

    return {
        'path':          out_path,
        'size_bytes':    len(tflite_model),
        'size_kb':       len(tflite_model) / 1024,
        'input_shape':   tuple(inp_det['shape']),
        'output_shape':  tuple(out_det['shape']),
        'input_dtype':   str(inp_det['dtype']),
        'output_dtype':  str(out_det['dtype']),
        'input_scale':   inp_det['quantization'],
        'output_scale':  out_det['quantization'],
    }

# ── Límites Arduino Nano 33 BLE Sense ────────────────────────────────────
ARDUINO_FLASH_KB  = 256   # Flash total disponible para modelo
ARDUINO_SRAM_KB   = 32    # SRAM disponible para arena de inferencia
ARDUINO_LATENCY_MS = 20   # Budget de latencia por inferencia (< 20 ms)

print("Exportando modelos a TFLite int8…\\n")
results = {}

# ── Modelo A: MLP Espectral ───────────────────────────────────────────────
print("▶ MLP Espectral…")
results['MLP_Spectral'] = export_tflite_int8(
    mlp_sp, Xf_train_sc, 'gait_spectral_mlp_int8', MODELS_DIR_PATH
)

# ── Modelo B: CNN2D Residual (si está en memoria) ─────────────────────────
try:
    print("▶ CNN2D Residual…")
    X_ref_cnn = X_train_scaled[..., np.newaxis] if X_train_scaled.ndim == 3 else X_train_scaled
    results['CNN2D_Residual'] = export_tflite_int8(
        model, X_ref_cnn, 'gait_cnn2d_residual_int8', MODELS_DIR_PATH
    )
except Exception as e:
    print(f"  CNN2D no disponible: {e}")

# ── Reporte de despliegue ─────────────────────────────────────────────────
print()
print("=" * 65)
print(f"{'Modelo':<20} {'Tamaño':>9} {'Flash':>8} {'¿OK?':>6}")
print("=" * 65)
for mname, info in results.items():
    kb    = info['size_kb']
    ok    = '✅' if kb < ARDUINO_FLASH_KB else '❌ OVERFLOW'
    print(f"{mname:<20} {kb:>8.1f}KB {ARDUINO_FLASH_KB:>7}KB {ok:>6}")
    print(f"  Input : {info['input_shape']}  dtype={info['input_dtype']}  "
          f"quant={info['input_scale']}")
    print(f"  Output: {info['output_shape']}  dtype={info['output_dtype']}  "
          f"quant={info['output_scale']}")
    print()

print("=" * 65)
print(f"Budget Flash Arduino : {ARDUINO_FLASH_KB} KB")
print(f"Budget SRAM arena    : {ARDUINO_SRAM_KB} KB  (arena ≈ 3× input_size)")
print()
print("Archivos generados:")
for mname, info in results.items():
    print(f"  {info['path']}")
"""),

# ── Celda generación de cabeceras C ─────────────────────────────────────────
code_cell("""\
## SPECTRAL_PIPELINE_V1  –  Generar cabeceras C para firmware Arduino

import numpy as np
import os

FIRMWARE_DIR = os.path.abspath('../../firmware')

def tflite_to_c_header(tflite_path: str, var_name: str, out_path: str):
    \"\"\"Convierte .tflite → array uint8 en C (mismo formato que xxd -i).\"\"\"
    with open(tflite_path, 'rb') as f:
        data = f.read()
    lines = [f'// Auto-generated from {os.path.basename(tflite_path)}']
    lines.append(f'// DO NOT EDIT — regenerar con add_spectral_pipeline.py')
    lines.append(f'#pragma once')
    lines.append(f'#include <stdint.h>')
    lines.append(f'')
    lines.append(f'alignas(8) const uint8_t {var_name}[] = {{')
    for i in range(0, len(data), 12):
        chunk = data[i:i+12]
        hex_str = ', '.join(f'0x{b:02x}' for b in chunk)
        lines.append(f'  {hex_str},')
    lines.append('};')
    lines.append(f'const unsigned int {var_name}_len = {len(data)};')
    with open(out_path, 'w') as f:
        f.write('\\n'.join(lines) + '\\n')
    print(f"  ✅  {out_path}  ({len(data)/1024:.1f} KB)")

def scaler_to_c_header(scaler, n_features: int, var_name: str, out_path: str):
    \"\"\"Serializa los parámetros del StandardScaler como arrays float C.\"\"\"
    mean = scaler.mean_.astype(np.float32)
    scale= scaler.scale_.astype(np.float32)

    def arr_str(arr, name):
        vals = ', '.join(f'{v:.6f}f' for v in arr)
        return f'const float {name}[{len(arr)}] = {{ {vals} }};'

    lines = [
        f'// Parámetros StandardScaler para features espectrales',
        f'// N_FEATURES={n_features}, SPECTRAL_DIM={len(mean)}',
        f'#pragma once',
        f'#include <stdint.h>',
        f'',
        f'#define SPECTRAL_DIM {len(mean)}',
        f'',
        arr_str(mean,  f'{var_name}_mean'),
        arr_str(scale, f'{var_name}_scale'),
        f'',
        f'inline void normalize_spectral(float* x) {{',
        f'  for (int i = 0; i < SPECTRAL_DIM; i++)',
        f'    x[i] = (x[i] - {var_name}_mean[i]) / {var_name}_scale[i];',
        f'}}',
    ]
    with open(out_path, 'w') as f:
        f.write('\\n'.join(lines) + '\\n')
    print(f"  ✅  {out_path}  (StandardScaler params)")

print("Generando cabeceras C…\\n")

# ── Carpetas destino ──────────────────────────────────────────────────────
master_fw = os.path.join(FIRMWARE_DIR, 'inference_master', 'src')
slave_fw  = os.path.join(FIRMWARE_DIR, 'sensor_slave',     'src')
os.makedirs(master_fw, exist_ok=True)
os.makedirs(slave_fw,  exist_ok=True)

# ── 1. Modelo MLP int8 → cabecera C (va al MASTER) ───────────────────────
mlp_tflite_path = os.path.join(MODELS_DIR_PATH, 'gait_spectral_mlp_int8.tflite')
if os.path.exists(mlp_tflite_path):
    tflite_to_c_header(
        mlp_tflite_path,
        var_name  = 'gait_model_data',
        out_path  = os.path.join(master_fw, 'gait_model.h')
    )
else:
    print("  ⚠️  MLP tflite no encontrado — ejecuta celda de exportación primero")

# ── 2. Parámetros del scaler → cabecera C (necesaria en AMBOS dispositivos) ──
scaler_to_c_header(
    scaler_sp,
    n_features = N_FEATURES,
    var_name   = 'sp_scaler',
    out_path   = os.path.join(master_fw, 'spectral_scaler.h')
)

# ── 3. Copia del scaler al SLAVE (el slave también necesita normalizar) ───
import shutil
src_scaler = os.path.join(master_fw, 'spectral_scaler.h')
dst_scaler = os.path.join(slave_fw,  'spectral_scaler.h')
shutil.copy(src_scaler, dst_scaler)
print(f"  ✅  {dst_scaler}  (copia para slave)")

# ── 4. Constantes de despliegue ───────────────────────────────────────────
deploy_h = f\"\"\"// deploy_config.h — generado automáticamente
#pragma once

// ── Arquitectura distribuida ──────────────────────────────────────────
// MASTER (pierna)  : sensor_id=2, ejecuta el modelo completo
// SLAVE  (tobillo) : sensor_id=3, extrae features y los envía por BLE

#define WINDOW_SIZE       {WINDOW_SIZE}   // muestras por ventana
#define STEP_SIZE         {STEP}          // hop entre ventanas
#define SAMPLE_HZ         {SAMPLE_HZ}     // Hz del IMU
#define N_SENSORS         2               // master + 1 slave
#define N_AXES            6               // ax,ay,az,gx,gy,gz
#define N_FFT_BINS        {N_FFT_BINS}    // bins FFT por canal
#define SPECTRAL_DIM      {SPECTRAL_DIM}  // features totales del extractor
#define N_CLASSES         {N_CLASSES}     // fases de marcha
#define MASTER_SENSOR_ID  2
#define SLAVE_SENSOR_ID   3

// Clases
// 0 = loading | 1 = midstance | 2 = terminal | 3 = swing
\"\"\"

deploy_h_master = os.path.join(master_fw, 'deploy_config.h')
deploy_h_slave  = os.path.join(slave_fw,  'deploy_config.h')
for p in [deploy_h_master, deploy_h_slave]:
    with open(p, 'w') as f: f.write(deploy_h)
    print(f"  ✅  {p}")

print("\\n✅ Todos los archivos de cabecera generados.")
print("\\n── Resumen de archivos para firmware ──────────────────────")
print(f"MASTER  → {master_fw}/")
print(f"         gait_model.h       (modelo TFLite int8)")
print(f"         spectral_scaler.h  (parámetros normalización)")
print(f"         deploy_config.h    (constantes)")
print(f"SLAVE   → {slave_fw}/")
print(f"         spectral_scaler.h  (mismos parámetros)")
print(f"         deploy_config.h    (constantes)")
"""),

# ── Celda checklist final ────────────────────────────────────────────────────
code_cell("""\
## SPECTRAL_PIPELINE_V1  –  Checklist de exportación y despliegue

import os, json
import numpy as np

MASTER_SRC = os.path.abspath('../../firmware/inference_master/src')
SLAVE_SRC  = os.path.abspath('../../firmware/sensor_slave/src')

checks = {
    'Modelo MLP int8 exportado':
        os.path.exists(os.path.join(MODELS_DIR_PATH, 'gait_spectral_mlp_int8.tflite')),

    'Cabecera modelo para MASTER':
        os.path.exists(os.path.join(MASTER_SRC, 'gait_model.h')),

    'Scaler MASTER':
        os.path.exists(os.path.join(MASTER_SRC, 'spectral_scaler.h')),

    'Scaler SLAVE':
        os.path.exists(os.path.join(SLAVE_SRC, 'spectral_scaler.h')),

    'deploy_config.h MASTER':
        os.path.exists(os.path.join(MASTER_SRC, 'deploy_config.h')),

    'deploy_config.h SLAVE':
        os.path.exists(os.path.join(SLAVE_SRC, 'deploy_config.h')),

    f'Tamaño modelo < {ARDUINO_FLASH_KB}KB':
        (lambda p: os.path.getsize(p)/1024 < ARDUINO_FLASH_KB
         if os.path.exists(p) else False)(
             os.path.join(MODELS_DIR_PATH, 'gait_spectral_mlp_int8.tflite')),

    f'SENSOR_IDS = [2,3] (correcto)':
        SENSOR_IDS == [2, 3],

    f'STEP = {STEP} (reducido)':
        STEP == 10,
}

print("\\n┌─────────────────────────────────────────────────────────────┐")
print("│          CHECKLIST DE EXPORTACIÓN PARA ARDUINO              │")
print("├─────────────────────────────────────────────────────────────┤")
all_pass = True
for name, ok in checks.items():
    icon = '✅' if ok else '❌'
    if not ok: all_pass = False
    print(f"│  {icon}  {name:<50} │")
print("├─────────────────────────────────────────────────────────────┤")
if all_pass:
    print("│  🚀 TODO LISTO PARA FLASHEAR LOS DOS ARDUINOS              │")
else:
    print("│  ⚠️  Hay items pendientes — revisar antes de flashear       │")
print("└─────────────────────────────────────────────────────────────┘")

# ── Estimación de latencia ─────────────────────────────────────────────────
# Arduino M4 @ 64 MHz: ~1 op/ciclo, int8 MLP de ~50K MACs
# MLP 168→128→64→32→4: MACs = 168*128 + 128*64 + 64*32 + 32*4 ≈ 30K
mlp_sp_params = mlp_sp.count_params()
macs_estimate  = SPECTRAL_DIM * 128 + 128 * 64 + 64 * 32 + 32 * N_CLASSES
latency_ms     = macs_estimate / (64e6 / 1e3)  # rough: 1 MAC/ciclo @ 64 MHz
print(f"\\n── Estimación de latencia de inferencia ──")
print(f"   MACs estimados (MLP)  : {macs_estimate:,}")
print(f"   Latencia estimada     : {latency_ms:.1f} ms @ 64 MHz M4")
print(f"   Budget step ({STEP} samp) : {STEP/SAMPLE_HZ*1000:.0f} ms")
latency_ok = latency_ms < STEP/SAMPLE_HZ*1000
print(f"   Cabe en el budget?    : {'✅ SÍ' if latency_ok else '❌ NO — reducir modelo'}")

# ── Extracción de features en C: dimensiones para firmware ──────────────
print(f"\\n── Tamaños de buffer para firmware ──")
print(f"   Ventana cruda (por sensor) : {WINDOW_SIZE} × {N_FEATURES//len(SENSOR_IDS)} = "
      f"{WINDOW_SIZE * N_FEATURES // len(SENSOR_IDS)} floats = "
      f"{WINDOW_SIZE * N_FEATURES // len(SENSOR_IDS) * 4} bytes")
print(f"   Vector spectral (por sensor): {SPECTRAL_DIM//2} floats = "
      f"{SPECTRAL_DIM//2*4} bytes")
print(f"   Payload BLE (slave→master) : {SPECTRAL_DIM//2*4} bytes "
      f"(vs {WINDOW_SIZE * N_FEATURES // len(SENSOR_IDS) * 4} crudo) "
      f"→ {100*(1 - SPECTRAL_DIM//2 / (WINDOW_SIZE*N_FEATURES//len(SENSOR_IDS))):.0f}% menos")
"""),

]  # fin NEW_CELLS


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    nb = json.loads(NB_PATH.read_text(encoding='utf-8'))

    if already_injected(nb):
        print("ℹ️  El pipeline espectral ya está inyectado (SPECTRAL_PIPELINE_V1).")
        print("   Borra el marcador del notebook si quieres reinsertar.")
        return

    insert_at = find_insert_idx(nb)
    print(f"Insertando {len(NEW_CELLS)} celdas en posición {insert_at} "
          f"(de {len(nb['cells'])} celdas totales)…")

    for offset, cell in enumerate(NEW_CELLS):
        nb['cells'].insert(insert_at + offset, cell)

    NB_PATH.write_text(json.dumps(nb, ensure_ascii=False, indent=1),
                       encoding='utf-8')
    print(f"✅  Notebook actualizado: {NB_PATH}")
    print(f"   Total celdas: {len(nb['cells'])}")
    print()
    print("Próximos pasos:")
    print("  1. Abre el notebook en Jupyter/VS Code")
    print("  2. Kernel → Restart & Run All  (o desde celda 1 hasta el final)")
    print("  3. El pipeline espectral aparece después del bloque de entrenamiento CNN2D")
    print("  4. Al finalizar revisa el CHECKLIST en la última celda")


if __name__ == '__main__':
    main()
