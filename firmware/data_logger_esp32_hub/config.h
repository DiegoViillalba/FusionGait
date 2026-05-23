#pragma once

// ── Identidad del hub ESP32 ───────────────────────────────────────────────
// El ESP32-S3 actúa como hub puro (relay BLE → USB Serial).
// No tiene IMU propio: solo reenvía datos de los sensores Arduino.
// sensor_id=1 se reserva para el hub (aunque no emita muestras propias).
#define HUB_SENSOR_ID    1

// ── Número de sensores BLE esclavos a buscar ─────────────────────────────
// 1 → solo GaitNode_2
// 2 → GaitNode_2 + GaitNode_3
#define N_SLAVES         2

// Nombres BLE y configuración de cada esclavo
// (deben coincidir con data_logger_ble_sensor/config.h)
#define SLAVE1_NAME      "GaitNode_2"
#define SLAVE1_ID        2
#define SLAVE1_PLACEMENT "pierna"

#define SLAVE2_NAME      "GaitNode_3"
#define SLAVE2_ID        3
#define SLAVE2_PLACEMENT "cadera"

// ── UUIDs BLE ─────────────────────────────────────────────────────────────
// Deben coincidir con data_logger_ble_sensor/config.h
#define BLE_SERVICE_UUID "19B20000-E8F2-537E-4F6C-D104768A1214"
#define BLE_IMU_UUID     "19B20001-E8F2-537E-4F6C-D104768A1214"

// Tiempo máximo buscando cada esclavo antes de saltarlo y seguir.
#define SLAVE_SCAN_TIMEOUT_MS  15000UL

// ── Serial ────────────────────────────────────────────────────────────────
#define BAUD_RATE  115200

// ── LED integrado ESP32-S3 DevKit ─────────────────────────────────────────
// Ajusta si tu placa tiene LED en otro pin (o pon -1 para deshabilitar).
#define LED_PIN    2
