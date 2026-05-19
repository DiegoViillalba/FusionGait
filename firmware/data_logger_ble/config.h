#pragma once

// ─── EDITAR ANTES DE SUBIR A CADA ARDUINO ───────────────────────────────────
#define SENSOR_ID  2
#define PLACEMENT  "thigh"
// ─────────────────────────────────────────────────────────────────────────────

// ─── VERSIÓN DE HARDWARE ─────────────────────────────────────────────────────
// 0 = Nano 33 BLE Sense original (LSM9DS1)
// 1 = Nano 33 BLE Sense Rev2    (BMI270)
#define IMU_REV2  1
// ─────────────────────────────────────────────────────────────────────────────

#define BAUD_RATE        115200   // para debug Serial (opcional)
#define SAMPLE_HZ        100
#define SAMPLE_PERIOD_US (1000000UL / SAMPLE_HZ)

// Nombre BLE del dispositivo — debe ser único por nodo
// El script Python busca dispositivos con prefijo "GaitNode"
// Cambia el sufijo según SENSOR_ID: GaitNode_1, GaitNode_2, GaitNode_3
#define BLE_DEVICE_NAME  "GaitNode_2"

// ─── UUIDs GATT (no modificar) ───────────────────────────────────────────────
// Servicio principal
#define SERVICE_UUID    "12340000-cafe-4b0b-a5b8-c2e8c9e9d1a0"
// Característica: paquete de datos IMU (Notify)
#define CHAR_IMU_UUID   "12340001-cafe-4b0b-a5b8-c2e8c9e9d1a0"
// Característica: comandos START/STOP (Write Without Response)
#define CHAR_CMD_UUID   "12340002-cafe-4b0b-a5b8-c2e8c9e9d1a0"
// Característica: status (Read)
#define CHAR_STS_UUID   "12340003-cafe-4b0b-a5b8-c2e8c9e9d1a0"

// ─── Batching ────────────────────────────────────────────────────────────────
// Cuántas muestras se empaquetan en cada notificación BLE.
// 5 muestras × 16 bytes = 80 bytes + 3 bytes header = 83 bytes.
// Esto reduce la tasa de notify a 20 Hz (BLE cómodo) con 100 Hz efectivos.
#define BATCH_SIZE  5
