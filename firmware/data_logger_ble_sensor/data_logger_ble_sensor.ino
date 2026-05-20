/**
 * data_logger_ble_sensor.ino — FusionGait Sensor BLE de Adquisición
 *
 * Hardware : Arduino Nano 33 BLE Sense Rev2 (nRF52840 + BMI270)
 * Role     : BLE Peripheral — anuncia como "GaitNode_<N>" y envía
 *            6 floats (24 bytes: ax,ay,az,gx,gy,gz) vía BLE Notify
 *            a 100 Hz al Hub conectado (data_logger_hub).
 *
 * No requiere comandos: emite datos en cuanto el Hub se conecta.
 * El Hub controla el inicio y fin de la grabación CSV.
 *
 * LEDs (activos en bajo):
 *   Azul  → anunciando, esperando conexión del Hub
 *   Verde → conectado al Hub, enviando datos IMU
 *   Rojo  → error de IMU
 *
 * Config en config.h:
 *   SENSOR_ID, PLACEMENT, BLE_DEVICE_NAME ("GaitNode_2" o "GaitNode_3")
 *   Para el Sensor 3: copiar config_s3.h → config.h y recompilar.
 *
 * Subir ANTES que el Hub. Los sensores deben estar anunciándose
 * (LED azul) cuando el Hub arranca y escanea.
 */

#include <ArduinoBLE.h>
#include "config.h"

#if IMU_REV2
  #include <Arduino_BMI270_BMM150.h>
#else
  #include <Arduino_LSM9DS1.h>
#endif

// ── GATT ─────────────────────────────────────────────────────────────────
BLEService        imuService(BLE_SERVICE_UUID);
BLECharacteristic imuChar(BLE_IMU_UUID,
                           BLENotify | BLERead,
                           6 * sizeof(float));

// ── Timing ────────────────────────────────────────────────────────────────
static unsigned long last_sample_ms = 0;

// ── LED helpers ───────────────────────────────────────────────────────────
static inline void set_led(bool r, bool g, bool b) {
    digitalWrite(LEDR, r ? LOW : HIGH);
    digitalWrite(LEDG, g ? LOW : HIGH);
    digitalWrite(LEDB, b ? LOW : HIGH);
}

static void halt_error(const char* msg) {
    Serial.println(msg);
    while (true) {
        set_led(true, false, false); delay(200);
        set_led(false, false, false); delay(200);
    }
}

// ── setup() ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    set_led(false, false, false);

    // IMU
    if (!IMU.begin()) halt_error("ERROR:IMU_INIT");
    Serial.print("IMU OK — acc ODR: ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");

    // BLE
    if (!BLE.begin()) halt_error("ERROR:BLE_INIT");

    BLE.setLocalName(BLE_DEVICE_NAME);
    BLE.setAdvertisedService(imuService);
    imuService.addCharacteristic(imuChar);
    BLE.addService(imuService);

    // Valor inicial neutro
    float zeros[6] = {};
    imuChar.writeValue((uint8_t*)zeros, sizeof(zeros));

    BLE.advertise();
    Serial.print("STATUS:ADVERTISING as "); Serial.println(BLE_DEVICE_NAME);
    Serial.print("Sensor ID="); Serial.print(SENSOR_ID);
    Serial.print("  Placement="); Serial.println(PLACEMENT);
    set_led(false, false, true);   // azul = anunciando
}

// ── loop() ────────────────────────────────────────────────────────────────
void loop() {
    BLEDevice hub = BLE.central();

    if (!hub) {
        delay(SAMPLE_MS);
        return;
    }

    // Hub conectado
    Serial.print("STATUS:CONNECTED "); Serial.println(hub.address());
    set_led(false, true, false);   // verde = conectado

    while (hub.connected()) {
        unsigned long now = millis();
        if (now - last_sample_ms < SAMPLE_MS) continue;
        last_sample_ms = now;

        if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) continue;

        float ax, ay, az, gx, gy, gz;
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);

        float payload[6] = { ax, ay, az, gx, gy, gz };
        imuChar.writeValue((uint8_t*)payload, sizeof(payload));
    }

    // Desconectado → volver a anunciar
    Serial.println("STATUS:DISCONNECTED — re-advertising");
    BLE.advertise();
    set_led(false, false, true);   // azul = anunciando
}
