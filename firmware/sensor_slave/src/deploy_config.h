// deploy_config.h — generado automáticamente
#pragma once

// ── Arquitectura distribuida ──────────────────────────────────────────
// MASTER (pierna)  : sensor_id=2, ejecuta el modelo completo
// SLAVE  (tobillo) : sensor_id=3, extrae features y los envía por BLE

#define WINDOW_SIZE       50   // muestras por ventana
#define STEP_SIZE         10          // hop entre ventanas
#define SAMPLE_HZ         100     // Hz del IMU
#define N_SENSORS         2               // master + 1 slave
#define N_AXES            6               // ax,ay,az,gx,gy,gz
#define N_FFT_BINS        10    // bins FFT por canal
#define SPECTRAL_DIM      168  // features totales del extractor
#define N_CLASSES         4     // fases de marcha
#define MASTER_SENSOR_ID  2
#define SLAVE_SENSOR_ID   3

// Clases
// 0 = loading | 1 = midstance | 2 = terminal | 3 = swing
