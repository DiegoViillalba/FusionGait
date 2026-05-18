# training/src — Pipeline offline de entrenamiento

## Módulos y responsabilidades

| Módulo | Entrada | Salida | Descripción |
|--------|---------|--------|-------------|
| `load_data.py` | `data/raw/*.csv` + `data/labels/*.csv` | DataFrame validado | Carga, valida y limpia datos crudos |
| `preprocess.py` | DataFrame crudo | DataFrame limpio | Filtrado, interpolación, normalización |
| `windowing.py` | DataFrame limpio | `data/processed/*.npz` | Segmentación en ventanas y asignación de labels |
| `features.py` | Ventana `[T, C]` | Vector de features | Features manuales para modelos clásicos |
| `train_baselines.py` | `.npz` + features | Modelos `.pkl` | Reglas, LogReg, RF, SVM, MLP |
| `train_cnn1d.py` | `.npz` | Modelo `.h5` | CNN 1D con Keras |
| `evaluate.py` | Modelos + test data | Reports + figuras | Métricas completas y comparativa |
| `export_tflite.py` | Modelo `.h5` | `.tflite` + `model_data.h` | Cuantización INT8 y header C |

---

## Pseudocódigo: load_data.py

```python
# training/src/load_data.py

import pandas as pd
import numpy as np
from pathlib import Path

REQUIRED_COLUMNS = [
    "timestamp_pc_ms", "sensor_id", "placement",
    "ax", "ay", "az", "gx", "gy", "gz",
    "trial_id", "subject_id"
]
FLOAT_COLS = ["ax", "ay", "az", "gx", "gy", "gz"]


def load_raw_csv(path: str) -> pd.DataFrame:
    """Carga un CSV crudo con validación básica."""
    df = pd.read_csv(path)

    # Verificar columnas
    missing = set(REQUIRED_COLUMNS) - set(df.columns)
    if missing:
        raise ValueError(f"Columnas faltantes en {path}: {missing}")

    # Convertir timestamps a numérico
    df["timestamp_pc_ms"] = pd.to_numeric(df["timestamp_pc_ms"], errors="coerce")

    # Convertir IMU a float32
    for col in FLOAT_COLS:
        df[col] = pd.to_numeric(df[col], errors="coerce").astype("float32")

    # Eliminar filas completamente corruptas
    before = len(df)
    df.dropna(subset=FLOAT_COLS + ["timestamp_pc_ms"], inplace=True)
    after = len(df)
    if before - after > 0:
        print(f"  Eliminadas {before - after} filas corruptas en {Path(path).name}")

    # Ordenar por timestamp
    df.sort_values("timestamp_pc_ms", inplace=True)
    df.reset_index(drop=True, inplace=True)

    return df


def load_labels(path: str) -> pd.DataFrame:
    """Carga archivo de labels por intervalos."""
    ldf = pd.read_csv(path)
    required = ["start_ms", "end_ms", "label"]
    missing = set(required) - set(ldf.columns)
    if missing:
        raise ValueError(f"Columnas faltantes en labels {path}: {missing}")

    valid_labels = {"stance", "swing", "default"}
    invalid = set(ldf["label"].unique()) - valid_labels
    if invalid:
        raise ValueError(f"Labels inválidos en {path}: {invalid}")

    return ldf.sort_values("start_ms").reset_index(drop=True)


def merge_labels_into_df(df: pd.DataFrame,
                         labels_df: pd.DataFrame) -> pd.DataFrame:
    """
    Asigna un label a cada muestra del DataFrame basándose en los intervalos.
    """
    df = df.copy()
    # Normalizar timestamps de labels al inicio del trial
    t0 = df["timestamp_pc_ms"].iloc[0]
    df["t_rel"] = df["timestamp_pc_ms"] - t0

    labels_arr = np.full(len(df), "default", dtype=object)

    for _, row in labels_df.iterrows():
        mask = (df["t_rel"] >= row["start_ms"]) & (df["t_rel"] < row["end_ms"])
        labels_arr[mask.values] = row["label"]

    df["label"] = labels_arr
    df.drop(columns=["t_rel"], inplace=True)
    return df


def load_all_trials(raw_dir: str, labels_dir: str,
                    sensors: list = [1, 2, 3]) -> pd.DataFrame:
    """
    Carga todos los trials disponibles para los sensores indicados.
    Busca pares (raw CSV, labels CSV) automáticamente.
    Retorna un único DataFrame concatenado.
    """
    raw_dir    = Path(raw_dir)
    labels_dir = Path(labels_dir)
    all_dfs = []

    for labels_file in sorted(labels_dir.glob("*.csv")):
        # Extraer subject_id y trial_id del nombre del archivo
        stem = labels_file.stem  # e.g. "s001_t001_labels"
        parts = stem.split("_")
        subject_id = parts[0]  # s001
        trial_id   = parts[1]  # t001

        labels_df = load_labels(str(labels_file))

        for sensor_id in sensors:
            pattern = f"{subject_id}_{trial_id}_sensor{sensor_id}_*.csv"
            matches = list(raw_dir.glob(pattern))
            if not matches:
                print(f"  WARN: no se encontró {pattern}")
                continue

            df = load_raw_csv(str(matches[0]))
            df = merge_labels_into_df(df, labels_df)
            all_dfs.append(df)

    if not all_dfs:
        raise RuntimeError("No se encontraron datos")

    combined = pd.concat(all_dfs, ignore_index=True)
    print(f"Total filas cargadas: {len(combined):,}")
    print(f"Distribución de labels:\n{combined['label'].value_counts(normalize=True)}")
    return combined
```

---

## Pseudocódigo: windowing.py

```python
# training/src/windowing.py

import numpy as np
import pandas as pd
import json
from pathlib import Path


CHANNELS = ["ax", "ay", "az", "gx", "gy", "gz"]
CLASS_MAP = {"stance": 0, "swing": 1, "default": 2}

def assign_label_to_window(samples_labels: np.ndarray,
                           min_coverage: float = 0.70) -> int:
    """Asigna etiqueta a una ventana por mayoría. Default si no hay mayoría."""
    if len(samples_labels) == 0:
        return CLASS_MAP["default"]

    classes, counts = np.unique(samples_labels, return_counts=True)
    best_idx = np.argmax(counts)
    if counts[best_idx] / len(samples_labels) >= min_coverage:
        label_str = classes[best_idx]
        return CLASS_MAP.get(label_str, CLASS_MAP["default"])
    return CLASS_MAP["default"]


def compute_stats(X_train: np.ndarray):
    """Calcula media y std por canal sobre el set de entrenamiento."""
    # X_train: shape (N, T, C)
    X_flat = X_train.reshape(-1, X_train.shape[-1])  # (N*T, C)
    mean = X_flat.mean(axis=0)  # (C,)
    std  = X_flat.std(axis=0)   # (C,)
    std[std < 1e-8] = 1.0       # evitar división por cero
    return mean, std


def normalize(X: np.ndarray, mean: np.ndarray, std: np.ndarray) -> np.ndarray:
    return (X - mean[np.newaxis, np.newaxis, :]) / std[np.newaxis, np.newaxis, :]


def segment_dataframe(df: pd.DataFrame,
                      window_size: int = 50,
                      window_step: int = 25,
                      min_coverage: float = 0.70,
                      group_by: str = "trial_id") -> tuple:
    """
    Segmenta un DataFrame en ventanas.

    Retorna:
      X:        np.ndarray shape (N, window_size, 6)
      y:        np.ndarray shape (N,)
      metadata: list de dicts {subject_id, trial_id, sensor_id, window_start_ms}
    """
    X_list, y_list, meta_list = [], [], []

    for group_key, group_df in df.groupby(group_by):
        group_df = group_df.sort_values("timestamp_pc_ms").reset_index(drop=True)
        n = len(group_df)

        for start in range(0, n - window_size + 1, window_step):
            end = start + window_size
            window = group_df.iloc[start:end]

            # Verificar que no hay gaps grandes dentro de la ventana
            ts_diff = window["timestamp_pc_ms"].diff().dropna()
            if (ts_diff > 50).any():  # gap > 50 ms dentro de ventana
                continue

            x = window[CHANNELS].values.astype(np.float32)  # (T, C)
            labels_in_window = window["label"].values

            label = assign_label_to_window(labels_in_window, min_coverage)

            X_list.append(x)
            y_list.append(label)
            meta_list.append({
                "subject_id": group_df["subject_id"].iloc[0],
                "trial_id":   group_df["trial_id"].iloc[0],
                "sensor_id":  group_df["sensor_id"].iloc[0],
                "window_start_ms": window["timestamp_pc_ms"].iloc[0]
            })

    return (np.array(X_list, dtype=np.float32),
            np.array(y_list, dtype=np.int8),
            meta_list)


def build_dataset(df: pd.DataFrame,
                  train_subjects: list,
                  val_subjects: list,
                  test_subjects: list,
                  output_dir: str,
                  window_size: int = 50,
                  window_step: int = 25):
    """
    Pipeline completo: DataFrame → ventanas normalizadas → archivos .npz
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Split por sujeto (sin fuga de datos)
    df_train = df[df["subject_id"].isin(train_subjects)]
    df_val   = df[df["subject_id"].isin(val_subjects)]
    df_test  = df[df["subject_id"].isin(test_subjects)]

    print(f"Train: {len(df_train):,} muestras ({len(train_subjects)} sujetos)")
    print(f"Val:   {len(df_val):,} muestras ({len(val_subjects)} sujetos)")
    print(f"Test:  {len(df_test):,} muestras ({len(test_subjects)} sujetos)")

    X_train, y_train, meta_train = segment_dataframe(df_train, window_size, window_step)
    X_val,   y_val,   meta_val   = segment_dataframe(df_val,   window_size, window_step)
    X_test,  y_test,  meta_test  = segment_dataframe(df_test,  window_size, window_step)

    print(f"Ventanas train: {len(X_train)}, val: {len(X_val)}, test: {len(X_test)}")

    # Normalización (stats solo de train)
    mean, std = compute_stats(X_train)
    X_train = normalize(X_train, mean, std)
    X_val   = normalize(X_val,   mean, std)
    X_test  = normalize(X_test,  mean, std)

    # Guardar stats
    stats = {
        "mean": mean.tolist(),
        "std":  std.tolist(),
        "channels": CHANNELS
    }
    with open(output_dir / "stats.json", "w") as f:
        json.dump(stats, f, indent=2)

    # Guardar .npz
    for split, X, y, meta in [
        ("train", X_train, y_train, meta_train),
        ("val",   X_val,   y_val,   meta_val),
        ("test",  X_test,  y_test,  meta_test)
    ]:
        np.savez(
            output_dir / f"windows_{split}.npz",
            X=X, y=y,
            subject_ids=[m["subject_id"] for m in meta],
            trial_ids=[m["trial_id"] for m in meta],
            sensor_ids=[m["sensor_id"] for m in meta]
        )
        print(f"Guardado windows_{split}.npz: X={X.shape}, y={y.shape}")
        vals, cnts = np.unique(y, return_counts=True)
        for v, c in zip(vals, cnts):
            lbl = {0:"stance",1:"swing",2:"default"}[v]
            print(f"  {lbl}: {c} ({100*c/len(y):.1f}%)")
```

---

## Pseudocódigo: train_baselines.py

```python
# training/src/train_baselines.py

import numpy as np
import joblib
from pathlib import Path
from sklearn.linear_model import LogisticRegression
from sklearn.ensemble import RandomForestClassifier
from sklearn.neural_network import MLPClassifier
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline

from features import extract_features_batch

def load_split(processed_dir, split="train"):
    data = np.load(f"{processed_dir}/windows_{split}.npz", allow_pickle=True)
    return data["X"], data["y"]

def train_and_save(model, name, X_train_feat, y_train, models_dir):
    model.fit(X_train_feat, y_train)
    path = Path(models_dir) / f"{name}.pkl"
    joblib.dump(model, path)
    print(f"Guardado: {path}")
    return model

def run_baseline_training(processed_dir, models_dir):
    X_tr, y_tr = load_split(processed_dir, "train")
    X_va, y_va = load_split(processed_dir, "val")

    # Extraer features manuales: shape (N, num_features)
    F_tr = extract_features_batch(X_tr)
    F_va = extract_features_batch(X_va)

    models = {
        "logistic_regression": Pipeline([
            ("scaler", StandardScaler()),
            ("clf", LogisticRegression(max_iter=1000, multi_class="multinomial"))
        ]),
        "random_forest": RandomForestClassifier(
            n_estimators=100, max_depth=15, random_state=42, n_jobs=-1
        ),
        "mlp": Pipeline([
            ("scaler", StandardScaler()),
            ("clf", MLPClassifier(hidden_layer_sizes=(64, 32),
                                  max_iter=300, random_state=42))
        ]),
    }

    for name, model in models.items():
        print(f"\n--- {name} ---")
        trained = train_and_save(model, name, F_tr, y_tr, models_dir)
        val_score = trained.score(F_va, y_va)
        print(f"  Val accuracy: {val_score:.3f}")
```

---

## Pseudocódigo: evaluate.py

```python
# training/src/evaluate.py

import numpy as np
import pandas as pd
import json
import matplotlib.pyplot as plt
from sklearn.metrics import (
    accuracy_score, f1_score, confusion_matrix, classification_report
)
from pathlib import Path

CLASS_NAMES = ["stance", "swing", "default"]

def evaluate_model(model, X_test, y_test, model_name, report_dir):
    """Evaluación completa de un modelo sobre el test set."""
    import time

    # Predicción con medición de latencia
    t0 = time.perf_counter()
    y_pred = model.predict(X_test)
    latency_ms = (time.perf_counter() - t0) / len(X_test) * 1000

    acc  = accuracy_score(y_test, y_pred)
    f1   = f1_score(y_test, y_pred, average="macro")
    cm   = confusion_matrix(y_test, y_pred)
    report = classification_report(y_test, y_pred, target_names=CLASS_NAMES)

    print(f"\n=== {model_name} ===")
    print(f"Accuracy: {acc:.4f} | F1 macro: {f1:.4f} | Latencia: {latency_ms:.2f} ms/muestra")
    print(report)

    # Guardar matriz de confusión
    report_dir = Path(report_dir)
    report_dir.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(6, 5))
    im = ax.imshow(cm, cmap="Blues")
    ax.set_xticks(range(3)); ax.set_xticklabels(CLASS_NAMES, rotation=45)
    ax.set_yticks(range(3)); ax.set_yticklabels(CLASS_NAMES)
    for i in range(3):
        for j in range(3):
            ax.text(j, i, cm[i, j], ha="center", va="center",
                    color="white" if cm[i,j] > cm.max()/2 else "black")
    ax.set_xlabel("Predicho"); ax.set_ylabel("Real")
    ax.set_title(f"{model_name}\nAcc={acc:.3f} F1={f1:.3f}")
    plt.tight_layout()
    plt.savefig(report_dir / f"cm_{model_name}.png", dpi=120)
    plt.close()

    return {"model": model_name, "accuracy": acc, "f1_macro": f1,
            "latency_ms": latency_ms, "confusion_matrix": cm.tolist()}


def compare_all_models(results: list, output_path: str):
    """Genera tabla comparativa de todos los modelos."""
    df = pd.DataFrame([{k: v for k, v in r.items()
                        if k != "confusion_matrix"}
                       for r in results])
    df = df.sort_values("f1_macro", ascending=False)
    print("\n=== TABLA COMPARATIVA ===")
    print(df.to_string(index=False))
    df.to_csv(output_path, index=False)
    print(f"\nGuardado: {output_path}")
    return df
```

---

## Pseudocódigo: export_tflite.py

```python
# training/src/export_tflite.py

import tensorflow as tf
import numpy as np
import json
import subprocess
from pathlib import Path


def representative_dataset_gen(X_sample: np.ndarray):
    """
    Genera muestras representativas para calibración de cuantización INT8.
    Se usan ~100–500 muestras del set de entrenamiento.
    """
    for i in range(min(500, len(X_sample))):
        yield [X_sample[i:i+1].astype(np.float32)]


def export_to_tflite(keras_model_path: str,
                     X_train_sample: np.ndarray,
                     output_tflite_path: str,
                     output_header_path: str):
    """
    Exporta un modelo Keras a TFLite INT8 y genera el header C.
    """
    print(f"Cargando modelo: {keras_model_path}")
    model = tf.keras.models.load_model(keras_model_path)

    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]

    # Dataset representativo para calibración INT8
    converter.representative_dataset = lambda: representative_dataset_gen(X_train_sample)

    # Cuantización completa INT8
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type  = tf.int8
    converter.inference_output_type = tf.int8

    print("Convirtiendo a TFLite INT8...")
    tflite_model = converter.convert()

    # Guardar .tflite
    output_path = Path(output_tflite_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "wb") as f:
        f.write(tflite_model)
    print(f"Modelo guardado: {output_path} ({len(tflite_model)/1024:.1f} KB)")

    # Generar header C con xxd
    header_path = Path(output_header_path)
    header_path.parent.mkdir(parents=True, exist_ok=True)
    var_name = output_path.stem.replace("-", "_").replace(".", "_")
    result = subprocess.run(
        ["xxd", "-i", str(output_path)],
        capture_output=True, text=True
    )
    # Renombrar variable a g_model_data
    header_content = result.stdout.replace(
        output_path.stem.replace("/", "_").replace(".", "_"),
        "g_model_data"
    )
    with open(header_path, "w") as f:
        f.write("#pragma once\n\n")
        f.write(header_content)
    print(f"Header C guardado: {header_path}")

    # Verificar accuracy del modelo cuantizado
    interpreter = tf.lite.Interpreter(model_path=str(output_path))
    interpreter.allocate_tensors()
    inp_details = interpreter.get_input_details()[0]
    out_details = interpreter.get_output_details()[0]

    print(f"\nDetalles del modelo TFLite:")
    print(f"  Input:  shape={inp_details['shape']}, dtype={inp_details['dtype']}")
    print(f"  Output: shape={out_details['shape']}, dtype={out_details['dtype']}")
    print(f"  Input scale/zp:  {inp_details['quantization']}")
    print(f"  Output scale/zp: {out_details['quantization']}")

    return tflite_model


def generate_normalization_header(stats_json_path: str,
                                  output_path: str):
    """
    Lee stats.json y genera firmware/common/normalization.h
    """
    with open(stats_json_path) as f:
        stats = json.load(f)

    mean = stats["mean"]
    std  = stats["std"]
    channels = stats.get("channels", ["ax","ay","az","gx","gy","gz"])

    lines = ["#pragma once\n", "\n",
             f"// Auto-generado por export_tflite.py desde {stats_json_path}\n",
             "// NO editar manualmente\n\n",
             f"const float CHANNEL_MEAN[{len(mean)}] = " + "{"]
    for i, (m, ch) in enumerate(zip(mean, channels)):
        comma = "," if i < len(mean)-1 else ""
        lines.append(f"\n  {m:.6f}f{comma}  // {ch}_mean")
    lines.append("\n};\n\n")

    lines.append(f"const float CHANNEL_STD[{len(std)}] = " + "{")
    for i, (s, ch) in enumerate(zip(std, channels)):
        comma = "," if i < len(std)-1 else ""
        lines.append(f"\n  {s:.6f}f{comma}  // {ch}_std")
    lines.append("\n};\n")

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        f.writelines(lines)
    print(f"normalization.h generado: {output_path}")
```
