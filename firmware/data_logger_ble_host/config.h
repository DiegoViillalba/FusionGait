#pragma once

// ─── Nodo hub (sensor 1 = pelvis) ────────────────────────────────────────────
#define SENSOR_ID        1
#define PLACEMENT        "pelvis"
#define HUB_DEVICE_NAME  "GaitHub"

// 0 = Nano 33 BLE Sense original (LSM9DS1)
// 1 = Nano 33 BLE Sense Rev2    (BMI270)
#define IMU_REV2  1

// ─── Servicio Hub → PC ───────────────────────────────────────────────────────
#define HUB_SERVICE_UUID  "56780000-cafe-4b0b-a5b8-c2e8c9e9d1a0"
#define HUB_IMU1_UUID     "56780001-cafe-4b0b-a5b8-c2e8c9e9d1a0"  // pelvis (propio)
#define HUB_IMU2_UUID     "56780002-cafe-4b0b-a5b8-c2e8c9e9d1a0"  // thigh  (relay)
#define HUB_IMU3_UUID     "56780003-cafe-4b0b-a5b8-c2e8c9e9d1a0"  // ankle  (relay)
#define HUB_CMD_UUID      "56780004-cafe-4b0b-a5b8-c2e8c9e9d1a0"  // PC → hub
#define HUB_STS_UUID      "56780005-cafe-4b0b-a5b8-c2e8c9e9d1a0"  // status

// ─── Servicio nodo esclavo (debe coincidir con data_logger_ble/config.h) ────
#define NODE_SERVICE_UUID  "12340000-cafe-4b0b-a5b8-c2e8c9e9d1a0"
#define NODE_IMU_UUID      "12340001-cafe-4b0b-a5b8-c2e8c9e9d1a0"
#define NODE_CMD_UUID      "12340002-cafe-4b0b-a5b8-c2e8c9e9d1a0"

#define SLAVE2_NAME  "GaitNode_2"
#define SLAVE3_NAME  "GaitNode_3"

// Tiempo máximo buscando cada esclavo antes de saltarlo.
// Permite usar hub + 1 esclavo (o hub solo) sin colgar el arranque.
#define SLAVE_SCAN_TIMEOUT_MS  10000UL   // 10 s (reducido para pruebas)

// ─── Sampling ────────────────────────────────────────────────────────────────
#define BAUD_RATE        115200
#define SAMPLE_HZ        100
#define SAMPLE_PERIOD_US (1000000UL / SAMPLE_HZ)
#define BATCH_SIZE       5
#define PKT_SIZE         (3 + BATCH_SIZE * 16)   // 83 bytes
