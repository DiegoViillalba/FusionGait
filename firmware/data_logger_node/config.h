#pragma once

// ─── EDITAR ANTES DE SUBIR A CADA ARDUINO ───────────────────────────────────
// Cambiar estos dos valores según el nodo que estás programando:
//   Nodo 1: SENSOR_ID 1, PLACEMENT "pelvis"
//   Nodo 2: SENSOR_ID 2, PLACEMENT "thigh"
//   Nodo 3: SENSOR_ID 3, PLACEMENT "ankle"
#define SENSOR_ID  1
#define PLACEMENT  "pelvis"
// ─────────────────────────────────────────────────────────────────────────────

// ─── VERSIÓN DEL HARDWARE ─────────────────────────────────────────────────────
// Nano 33 BLE Sense original (LSM9DS1)  → #define IMU_REV2 0
// Nano 33 BLE Sense Rev2   (BMI270)     → #define IMU_REV2 1
//
// Cómo saber cuál tienes: mira la serigrafía del PCB.
// Si pone "BLE Sense" sin más → original.  Si pone "BLE Sense Rev2" → Rev2.
// También puedes subir el firmware, abrir Serial Monitor y ver si responde
// con ERROR:IMU_NOT_FOUND (tienes Rev2 y debes poner 1 aquí).
#define IMU_REV2  1
// ─────────────────────────────────────────────────────────────────────────────

#define BAUD_RATE        115200
#define SAMPLE_HZ        100
#define SAMPLE_PERIOD_US (1000000UL / SAMPLE_HZ)   // 10 000 µs

// LED RGB del Nano 33 BLE Sense (activo en LOW)
#define LED_IDLE     LEDB    // azul  → esperando START
#define LED_CAPTURE  LEDG    // verde → capturando
#define LED_ERROR    LEDR    // rojo  → error hardware
