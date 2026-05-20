#!/usr/bin/env python3
"""Patch gait_tinyml.ipynb: add augmentation, focal loss, 2D CNN."""
import json, pathlib, uuid

nb_path = pathlib.Path('training/notebooks/gait_tinyml.ipynb')
nb = json.loads(nb_path.read_text())

def md(text):
    return {'id': uuid.uuid4().hex[:8], 'cell_type': 'markdown',
            'source': [text], 'metadata': {}}

def code(text):
    return {'id': uuid.uuid4().hex[:8], 'cell_type': 'code',
            'source': [text], 'metadata': {},
            'outputs': [], 'execution_count': None}

# ── §6b: Augmentación ────────────────────────────────────────────────────
AUG_MD = md("""\
## 6b. Augmentación y balanceo de clases

### Por qué es necesario
Las fases tienen duraciones muy distintas en la marcha real:
- **Swing** (~60 % del ciclo) → clase dominante
- **Loading / Terminal** (~10 % cada una) → clases minoritarias

Con ventanas deslizantes eso se amplifica directamente en el conteo de ventanas.

### Estrategia
1. **Augmentación temporal** sobre clases minoritarias:
   - *Jitter*: ruido gaussiano σ=0.03 (simula ruido del sensor)
   - *Scaling*: multiplicar por factor ∈ [0.9, 1.1] (simula variación de amplitud de paso)
   - *Magnitude warping*: curva suave de escala por canal
2. **Oversampling dirigido**: generar ventanas sintéticas hasta igualar la clase mayoritaria
3. **Focal Loss** en lugar de cross-entropy (siguiente celda)
""")

AUG_CODE = code("""\
import numpy as np
from scipy.interpolate import CubicSpline
from collections import Counter

# ── Funciones de augmentación ─────────────────────────────────────────────

def aug_jitter(x, sigma=0.03):
    \"\"\"Add Gaussian noise — simulates sensor noise variability.\"\"\"
    return x + np.random.normal(0, sigma, x.shape).astype(np.float32)

def aug_scaling(x, lo=0.9, hi=1.1):
    \"\"\"Per-channel random scaling — simulates different step amplitudes.\"\"\"
    scale = np.random.uniform(lo, hi, (1, x.shape[1])).astype(np.float32)
    return x * scale

def aug_magnitude_warp(x, n_knots=4, sigma=0.1):
    \"\"\"Smooth per-channel magnitude warp via cubic spline.\"\"\"
    T, F = x.shape
    knot_x = np.linspace(0, T - 1, n_knots)
    knot_y = np.random.normal(1.0, sigma, (n_knots, F))
    warps  = np.stack(
        [CubicSpline(knot_x, knot_y[:, f])(np.arange(T)) for f in range(F)],
        axis=1
    ).astype(np.float32)
    return x * warps

AUGMENTORS = [aug_jitter, aug_scaling, aug_magnitude_warp]

def augment_one(x):
    \"\"\"Apply 1-2 random augmentors to a single window.\"\"\"
    ops = np.random.choice(AUGMENTORS, size=np.random.randint(1, 3), replace=False)
    for op in ops:
        x = op(x.copy())
    return x


# ── Oversampling con augmentación ─────────────────────────────────────────

def oversample_minority(X, y, target_per_class=None, seed=42):
    \"\"\"Oversample minority classes via augmented synthetic windows.

    Parameters
    ----------
    X                : (N, W, F) float32 — normalised windows
    y                : (N,) int — class indices
    target_per_class : target count per class (default = majority class count)
    seed             : random seed

    Returns
    -------
    X_out, y_out — shuffled balanced arrays
    \"\"\"
    np.random.seed(seed)
    counts = Counter(y)
    if target_per_class is None:
        target_per_class = max(counts.values())

    X_parts = [X]
    y_parts = [y]

    for cls, cnt in sorted(counts.items()):
        needed = target_per_class - cnt
        if needed <= 0:
            print(f"  {PHASES[cls]:12s}: {cnt:4d} (sin cambio)")
            continue
        idxs  = np.where(y == cls)[0]
        chosen = np.random.choice(idxs, size=needed, replace=True)
        X_new  = np.stack([augment_one(X[i]) for i in chosen])
        y_new  = np.full(needed, cls, dtype=np.int32)
        X_parts.append(X_new)
        y_parts.append(y_new)
        print(f"  {PHASES[cls]:12s}: {cnt:4d} -> {cnt + needed:4d}  (+{needed} aug)")

    X_out = np.concatenate(X_parts, axis=0)
    y_out = np.concatenate(y_parts, axis=0)
    perm  = np.random.permutation(len(X_out))
    return X_out[perm], y_out[perm]


# ── Aplicar solo al train set ─────────────────────────────────────────────
print("Distribucion ANTES del balanceo (train):")
for i, p in enumerate(PHASES):
    print(f"  {p:12s}: {(y_train == i).sum():4d}")

print("\\nGenerando ventanas augmentadas...")
X_train_bal, y_train_bal = oversample_minority(X_train, y_train)

print(f"\\nTrain set: {X_train.shape[0]} -> {X_train_bal.shape[0]} ventanas")
y_train_bal_oh = tf.keras.utils.to_categorical(y_train_bal, N_CLASSES)
""")

# ── §6c: Focal Loss ──────────────────────────────────────────────────────
FOCAL_MD = md("""\
### Focal Loss

Cross-entropy estándar asigna el mismo peso a todos los ejemplos.
Focal Loss (Lin et al. 2017) baja el gradiente de los ejemplos fáciles
(clases frecuentes bien clasificadas) y sube el de los difíciles.

$$FL(p_t) = -(1-p_t)^{\\gamma} \\log(p_t), \\quad \\gamma=2$$

Combinado con pesos `alpha` por clase es especialmente efectivo para
desbalances moderados como el nuestro (~3x entre swing y terminal).
""")

FOCAL_CODE = code("""\
def focal_loss(gamma=2.0, alpha=None):
    \"\"\"Categorical focal loss for one-hot labels.

    Parameters
    ----------
    gamma : float  focusing exponent (0 = standard CE)
    alpha : array  per-class weights, shape (N_CLASSES,)
    \"\"\"
    if alpha is not None:
        alpha_t = tf.constant(alpha, dtype=tf.float32)

    def loss_fn(y_true, y_pred):
        y_pred = tf.clip_by_value(y_pred, 1e-7, 1.0 - 1e-7)
        # Probability of the true class
        p_t = tf.reduce_sum(y_true * y_pred, axis=-1, keepdims=True)
        fl  = -tf.pow(1.0 - p_t, gamma) * tf.math.log(p_t)
        if alpha is not None:
            at = tf.reduce_sum(y_true * alpha_t, axis=-1, keepdims=True)
            fl = at * fl
        return tf.reduce_mean(fl)

    loss_fn.__name__ = f"focal_loss_g{gamma}"
    return loss_fn


# Alpha inversamente proporcional a la frecuencia de cada clase
counts_bal = np.bincount(y_train_bal, minlength=N_CLASSES).astype(np.float32)
alpha_w    = (1.0 / counts_bal) / (1.0 / counts_bal).sum()
print("Alpha weights por clase (focal loss):")
for i, p in enumerate(PHASES):
    print(f"  {p:12s}: {alpha_w[i]:.4f}")

FOCAL_LOSS = focal_loss(gamma=2.0, alpha=alpha_w)
""")

# ── §7: 2D CNN ────────────────────────────────────────────────────────────
CNN2D_MD = md("""\
## 7. Modelo 2D CNN

### Dos arquitecturas disponibles

**Multi-branch** — una rama Conv1D por sensor, fusión en el bottleneck.
Más interpretable: las features de tobillo y pierna se extraen por separado.

**Conv2D** — trata la ventana `(50 timesteps × 12 features)` como imagen.
El kernel `(tiempo, feature)` aprende correlaciones cruzadas entre ejes IMU
y entre sensores al mismo tiempo. Más expresivo con el mismo número de parámetros.

Selecciona con la variable `ARCH = 'cnn2d'` o `'multibranch'`.
""")

CNN2D_CODE = code("""\
from tensorflow.keras.layers import (
    Input, Conv1D, Conv2D, BatchNormalization,
    GlobalAveragePooling1D, GlobalAveragePooling2D,
    Dense, Dropout, Concatenate
)

# ── Opcion A: Multi-branch Conv1D ─────────────────────────────────────────
def build_multibranch(window=WINDOW_SIZE, n_axes=6, n_classes=N_CLASSES):
    \"\"\"Separate Conv1D extractor per sensor, fused at bottleneck.\"\"\"
    def branch(inp, name):
        x = Conv1D(16, 5, padding='same', activation='relu', name=f'c1_{name}')(inp)
        x = BatchNormalization(name=f'bn1_{name}')(x)
        x = Conv1D(32, 3, padding='same', activation='relu', name=f'c2_{name}')(x)
        x = BatchNormalization(name=f'bn2_{name}')(x)
        return GlobalAveragePooling1D(name=f'gap_{name}')(x)

    s1 = Input((window, n_axes), name='tobillo')
    s2 = Input((window, n_axes), name='pierna')
    x  = Concatenate(name='fusion')([branch(s1, 's1'), branch(s2, 's2')])
    x  = Dense(32, activation='relu', name='fc1')(x)
    x  = Dropout(0.4, name='drop')(x)
    out = Dense(n_classes, activation='softmax', name='out')(x)
    return tf.keras.Model([s1, s2], out, name='GaitMultiBranch')


# ── Opcion B: Conv2D — ventana como imagen (50 x 12 x 1) ─────────────────
def build_cnn2d(window=WINDOW_SIZE, n_features=N_FEATURES, n_classes=N_CLASSES):
    \"\"\"
    Input  : (batch, T=50, F=12, 1)  — treated as grayscale image
    Kernel : (time_span, feat_span)
      conv1: (5,3) — 5 timesteps x 3 adjacent features
      conv2: (3,3) — refines spatial + temporal patterns
      conv3: (3,1) — pure temporal refinement
    \"\"\"
    inp = Input((window, n_features, 1), name='imu_image')
    x = Conv2D(16, (5, 3), padding='same', activation='relu', name='conv1')(inp)
    x = BatchNormalization(name='bn1')(x)
    x = Conv2D(32, (3, 3), padding='same', activation='relu', name='conv2')(x)
    x = BatchNormalization(name='bn2')(x)
    x = Conv2D(32, (3, 1), padding='same', activation='relu', name='conv3')(x)
    x = BatchNormalization(name='bn3')(x)
    x = GlobalAveragePooling2D(name='gap')(x)
    x = Dense(32, activation='relu', name='fc1')(x)
    x = Dropout(0.4, name='drop')(x)
    out = Dense(n_classes, activation='softmax', name='predictions')(x)
    return tf.keras.Model(inp, out, name='GaitCNN2D')


# ── Seleccion y preparacion de inputs ────────────────────────────────────
ARCH = 'cnn2d'   # 'cnn2d' | 'multibranch'

if ARCH == 'multibranch':
    model = build_multibranch()
    s1_idx = [FEAT_COLS.index(f) for f in FEAT_COLS if f.endswith('_s1')]
    s2_idx = [FEAT_COLS.index(f) for f in FEAT_COLS if f.endswith('_s2')]
    def split_sensors(X):
        return [X[:, :, s1_idx], X[:, :, s2_idx]]
    X_train_in = split_sensors(X_train_bal)
    X_val_in   = split_sensors(X_val)
    X_test_in  = split_sensors(X_test) if len(X_test) > 0 else X_test
else:
    model = build_cnn2d()
    # (N, 50, 12) -> (N, 50, 12, 1)
    X_train_in = X_train_bal[..., np.newaxis]
    X_val_in   = X_val[..., np.newaxis]
    X_test_in  = X_test[..., np.newaxis] if len(X_test) > 0 else X_test

model.summary()
total_params = model.count_params()
print(f"\\nParametros  : {total_params:,}")
print(f"Float32     : {total_params*4/1024:.1f} KB")
print(f"Int8 est.   : {total_params/1024:.1f} KB")
""")

# ── §8: Entrenamiento ─────────────────────────────────────────────────────
TRAIN_MD = md("## 8. Entrenamiento")

TRAIN_CODE = code("""\
model.compile(
    optimizer=tf.keras.optimizers.Adam(learning_rate=1e-3),
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
    epochs=120,
    batch_size=32,
    callbacks=callbacks,
    verbose=1
)

print(f'\\nEntrenamiento completo: {len(history.history[\"loss\"])} epocas')
""")

# ── §9: Evaluacion ────────────────────────────────────────────────────────
EVAL_MD = md("""\
## 9. Evaluacion

Matriz de confusion + recall por clase + reporte completo sobre el test set.
""")

EVAL_CODE = code("""\
if len(X_test) == 0:
    print('Test set vacio, evaluando sobre val set')
    X_eval_in, y_eval_oh, y_eval = X_val_in, y_val_oh, y_val
else:
    X_eval_in, y_eval_oh, y_eval = X_test_in, y_test_oh, y_test

loss, acc = model.evaluate(X_eval_in, y_eval_oh, verbose=0)
print(f'Test Loss    : {loss:.4f}')
print(f'Test Accuracy: {acc:.4f} ({acc*100:.1f}%)')

y_pred_proba = model.predict(X_eval_in, verbose=0)
y_pred       = np.argmax(y_pred_proba, axis=1)

cm = confusion_matrix(y_eval, y_pred, labels=list(range(N_CLASSES)))

fig, axes = plt.subplots(1, 2, figsize=(14, 5))

sns.heatmap(cm, annot=True, fmt='d', cmap='Blues',
            xticklabels=PHASES, yticklabels=PHASES, ax=axes[0])
axes[0].set_title(f'Matriz de confusion  Acc={acc*100:.1f}%', fontsize=13)
axes[0].set_xlabel('Predicho'); axes[0].set_ylabel('Real')

per_class_recall = cm.diagonal() / cm.sum(axis=1).clip(min=1)
bar_colors = ['#e74c3c' if r < 0.70 else '#f39c12' if r < 0.85 else '#2ecc71'
              for r in per_class_recall]
axes[1].bar(PHASES, per_class_recall, color=bar_colors)
axes[1].axhline(0.70,  color='red',    ls='--', lw=1, label='70%')
axes[1].axhline(0.85,  color='orange', ls='--', lw=1, label='85%')
axes[1].set_ylim(0, 1.05)
axes[1].set_title('Recall por clase')
axes[1].set_ylabel('Recall')
axes[1].legend()
plt.tight_layout()
plt.show()

print('\\nReporte de clasificacion:')
print(classification_report(
    y_eval, y_pred,
    labels=list(range(N_CLASSES)),
    target_names=PHASES, digits=3
))

# Training curves
fig, axes = plt.subplots(1, 2, figsize=(12, 4))
axes[0].plot(history.history['accuracy'], label='Train')
if 'val_accuracy' in history.history:
    axes[0].plot(history.history['val_accuracy'], label='Val')
axes[0].set_title('Accuracy'); axes[0].set_xlabel('Epoca')
axes[0].legend(); axes[0].grid(alpha=0.3)

axes[1].plot(history.history['loss'], label='Train')
if 'val_loss' in history.history:
    axes[1].plot(history.history['val_loss'], label='Val')
axes[1].set_title('Focal Loss'); axes[1].set_xlabel('Epoca')
axes[1].legend(); axes[1].grid(alpha=0.3)
plt.tight_layout()
plt.show()
""")

# ── Reconstruir notebook ──────────────────────────────────────────────────
cells = nb['cells']
# Keep §1–§6 (indices 0-11), replace §7 onward
new_cells = (
    cells[:12]
    + [AUG_MD, AUG_CODE, FOCAL_MD, FOCAL_CODE]   # §6b, §6c
    + [CNN2D_MD, CNN2D_CODE]                       # §7 (nuevo)
    + [TRAIN_MD, TRAIN_CODE]                       # §8
    + [EVAL_MD, EVAL_CODE]                         # §9
    + cells[21:]                                   # §10–§14 sin cambios
)

nb['cells'] = new_cells
nb_path.write_text(json.dumps(nb, ensure_ascii=False, indent=1))
print(f"Notebook guardado: {len(new_cells)} celdas (era {len(cells)})")
