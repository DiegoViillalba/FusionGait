#pragma once

// ── Identidad ────────────────────────────────────────────────────────────
#define SENSOR_ID        2
#define PLACEMENT        "pierna"
#define IMU_REV2         1

// ── BLE Central ──────────────────────────────────────────────────────────
#define SLAVE_NAME       "GaitSlave_3"
#define BLE_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_IMU_UUID     "19B10002-E8F2-537E-4F6C-D104768A1214"

// ── Pipeline IMU ─────────────────────────────────────────────────────────
// Frecuencia de muestreo real medida en datos de entrenamiento: 16 ms/muestra = 62.5 Hz
#define SAMPLE_MS        16
#define SAMPLE_HZ        62             // 1000/16 = 62.5 → 62 (int)
#define WINDOW_SIZE      120
#define STEP_SIZE        10
#define N_AXES           6

// ── Extractor espectral ───────────────────────────────────────────────────
#define N_FFT_BINS              10
#define SPECTRAL_DIM_PER_SENSOR 84    // 6 × (10 + 4)
#define SPECTRAL_DIM_TOTAL      168   // 2 sensores × 84

// ── Modelo TinyML ─────────────────────────────────────────────────────────
#define N_CLASSES        4
// Tensor arena para MLP 168→128→64→32→4 int8 (~10 KB usado):
#define TENSOR_ARENA_KB  32

#define SERIAL_BAUD      115200

// Clases: 0=loading | 1=midstance | 2=terminal | 3=swing
