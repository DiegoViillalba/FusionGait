/**
 * inference_slave.ino — FusionGait BLE Slave (Sensor 2 / Pierna)
 *
 * Hardware : Arduino Nano 33 BLE Sense Rev2 (nRF52840 + BMI270)
 * Role     : BLE Peripheral — notifica 6 floats (ax,ay,az,gx,gy,gz) al Master
 *            a 100 Hz sobre una característica BLE personalizada.
 *
 * LEDs (activos en bajo):
 *   Azul  → anunciando (esperando conexión)
 *   Verde → conectado al Master, enviando datos
 *   Rojo  → error de IMU
 *
 * Subir ANTES que el Master. El Master busca el nombre "GaitSlave_2".
 */

#include <ArduinoBLE.h>
#include "config.h"

#if IMU_REV2
  #include <Arduino_BMI270_BMM150.h>
#else
  #include <Arduino_LSM9DS1.h>
#endif

// ── BLE service & characteristic ─────────────────────────────────────────
BLEService     imuService(BLE_SERVICE_UUID);
// 6 floats = 24 bytes, notificable y legible
BLECharacteristic imuChar(BLE_IMU_UUID,
                           BLENotify | BLERead,
                           6 * sizeof(float));

// ── Timing ───────────────────────────────────────────────────────────────
unsigned long lastSampleMs = 0;

// ── Helpers ───────────────────────────────────────────────────────────────
inline void setLED(bool r, bool g, bool b) {
    // RGB LEDs en el Nano 33 BLE son activos-en-bajo
    digitalWrite(LEDR, r ? LOW : HIGH);
    digitalWrite(LEDG, g ? LOW : HIGH);
    digitalWrite(LEDB, b ? LOW : HIGH);
}

void haltError(const char* msg) {
    Serial.println(msg);
    while (true) {
        setLED(true, false, false);
        delay(200);
        setLED(false, false, false);
        delay(200);
    }
}

// ── setup() ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    setLED(false, false, false);

    // IMU
    if (!IMU.begin()) haltError("ERROR:IMU_INIT");
    Serial.print("IMU OK — acc ODR: ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");

    // BLE
    if (!BLE.begin()) haltError("ERROR:BLE_INIT");

    BLE.setLocalName(BLE_DEVICE_NAME);
    BLE.setAdvertisedService(imuService);
    imuService.addCharacteristic(imuChar);
    BLE.addService(imuService);

    // Valor inicial vacío
    float zeros[6] = {0};
    imuChar.writeValue((uint8_t*)zeros, sizeof(zeros));

    BLE.advertise();
    Serial.print("STATUS:ADVERTISING as ");
    Serial.println(BLE_DEVICE_NAME);
    setLED(false, false, true);   // azul = anunciando
}

// ── loop() ────────────────────────────────────────────────────────────────
void loop() {
    BLEDevice central = BLE.central();

    if (!central) {
        // Sin conexión — parpadeo azul suave
        delay(SAMPLE_MS);
        return;
    }

    // ── Central conectado ─────────────────────────────────────────────────
    Serial.print("CONNECTED:");
    Serial.println(central.address());
    setLED(false, true, false);   // verde = conectado

    while (central.connected()) {
        unsigned long now = millis();
        if (now - lastSampleMs < SAMPLE_MS) continue;
        lastSampleMs = now;

        if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) continue;

        float ax, ay, az, gx, gy, gz;
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);

        float payload[6] = {ax, ay, az, gx, gy, gz};
        imuChar.writeValue((uint8_t*)payload, sizeof(payload));
    }

    // Desconectado — volver a anunciar
    Serial.println("DISCONNECTED");
    BLE.advertise();
    setLED(false, false, true);   // azul = anunciando de nuevo
}
