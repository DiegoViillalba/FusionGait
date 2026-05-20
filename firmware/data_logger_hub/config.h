#pragma once

// ── Identidad del hub ─────────────────────────────────────────────────────
// El hub ocupa sensor_id=1 y lleva la IMU en su propio cuerpo.
#define SENSOR_ID        1
#define PLACEMENT        "tobillo"   // ajustar al segmento real

// ── IMU ───────────────────────────────────────────────────────────────────
// 0 = Nano 33 BLE Sense original (LSM9DS1)
// 1 = Nano 33 BLE Sense Rev2    (BMI270)
#define IMU_REV2         1

// ── Número de esclavos BLE que buscar ────────────────────────────────────
// 1 → Hub(tobillo) + Sensor2(pierna)
// 2 → Hub(tobillo) + Sensor2(pierna) + Sensor3(cadera)
#define N_SLAVES         2

// Nombres BLE, sensor_ids y ubicaciones de cada esclavo.
// Deben coincidir con los config.h de data_logger_ble_sensor.
#define SLAVE2_NAME      "GaitNode_2"
#define SLAVE2_ID        2
#define SLAVE2_PLACEMENT "pierna"

#define SLAVE3_NAME      "GaitNode_3"
#define SLAVE3_ID        3
#define SLAVE3_PLACEMENT "cadera"

// ── UUIDs BLE ─────────────────────────────────────────────────────────────
// Deben coincidir con data_logger_ble_sensor/config.h
#define BLE_SERVICE_UUID "19B20000-E8F2-537E-4F6C-D104768A1214"
#define BLE_IMU_UUID     "19B20001-E8F2-537E-4F6C-D104768A1214"

// Tiempo máximo buscando cada esclavo antes de saltar al siguiente.
// Permite usar el hub con 1 esclavo (o sin ninguno) sin colgar el arranque.
#define SLAVE_SCAN_TIMEOUT_MS  15000UL   // 15 s

// ── Sampling ──────────────────────────────────────────────────────────────
#define BAUD_RATE        115200
#define SAMPLE_HZ        100
#define SAMPLE_PERIOD_US (1000000UL / SAMPLE_HZ)   // 10 000 µs
