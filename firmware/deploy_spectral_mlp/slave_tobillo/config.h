#pragma once

// ── Identidad ────────────────────────────────────────────────────────────
#define SENSOR_ID        3
#define PLACEMENT        "tobillo"
#define IMU_REV2         1        // 0=LSM9DS1  1=BMI270 (Rev2)

// ── BLE ──────────────────────────────────────────────────────────────────
// El slave espectral anuncia con el mismo nombre que el esclavo raw,
// pero la característica envía 84 floats en vez de 6.
#define BLE_DEVICE_NAME  "GaitSlave_3"
#define BLE_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_IMU_UUID     "19B10002-E8F2-537E-4F6C-D104768A1214"  // UUID distinto!

// ── Pipeline IMU ─────────────────────────────────────────────────────────
// Frecuencia de muestreo real medida en datos de entrenamiento: 16 ms/muestra = 62.5 Hz
#define SAMPLE_MS        16             // 62.5 Hz
#define SAMPLE_HZ        62             // 1000/16 truncado
#define WINDOW_SIZE      120    // muestras por ventana (1.2 s @ 100 Hz)
#define STEP_SIZE        10     // muestras entre envíos (envía cada 100 ms)
#define N_AXES           6      // ax,ay,az,gx,gy,gz

// ── Extractor espectral ───────────────────────────────────────────────────
// Debe coincidir exactamente con el notebook:
//   SPECTRAL_DIM_PER_SENSOR = N_AXES * (N_FFT_BINS + 4)
//                           = 6 * (10 + 4) = 84
#define N_FFT_BINS              10
#define SPECTRAL_DIM_PER_SENSOR 84   // features que envía este slave
