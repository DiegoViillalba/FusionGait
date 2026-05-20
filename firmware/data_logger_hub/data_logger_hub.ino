/**
 * data_logger_hub.ino — FusionGait Hub de Adquisición (USB Serial)
 *
 * Hardware : Arduino Nano 33 BLE Sense Rev2 (nRF52840 + BMI270)
 * Role     : BLE Central — se conecta a N_SLAVES sensores BLE,
 *            lee su propia IMU y reenvía todo al PC vía USB Serial
 *            en el mismo formato CSV que data_logger_node:
 *              timestamp_ms,sensor_id,placement,ax,ay,az,gx,gy,gz
 *
 * Comandos USB Serial (idénticos a data_logger_node):
 *   STATUS           → estado del hub y esclavos conectados
 *   START            → comenzar grabación (hub + esclavos)
 *   STOP             → detener grabación
 *   SET_TRIAL:<id>   → fijar id de trial (se refleja en STATUS)
 *   SET_SUBJECT:<id> → fijar id de sujeto
 *   SYNC             → responde SYNC_ACK:<millis()>
 *
 * Salida CSV durante CAPTURING (una línea por muestra):
 *   timestamp_ms,sensor_id,placement,ax,ay,az,gx,gy,gz
 *
 * LEDs (activos en bajo):
 *   Rojo parpadeante → escaneando esclavos BLE
 *   Azul fijo        → READY, esperando START
 *   Verde fijo       → CAPTURING
 *   Rojo fijo        → error de IMU
 *
 * Orden de arranque:
 *   1. Subir data_logger_ble_sensor a los sensores esclavos (se anuncian como
 *      GaitNode_2, GaitNode_3) y encenderlos primero.
 *   2. Encender el hub → conecta automáticamente a los esclavos en orden.
 *   3. Abrir la GUI y pulsar Conectar (modo Hub BLE).
 */

#include <ArduinoBLE.h>
#include "config.h"

#if IMU_REV2
  #include <Arduino_BMI270_BMM150.h>
#else
  #include <Arduino_LSM9DS1.h>
#endif

// ── Tabla de esclavos ──────────────────────────────────────────────────────
struct SlaveInfo {
    const char* name;
    uint8_t     sensor_id;
    const char* placement;
};

static const SlaveInfo SLAVES[N_SLAVES] = {
    { SLAVE2_NAME, SLAVE2_ID, SLAVE2_PLACEMENT },
#if N_SLAVES >= 2
    { SLAVE3_NAME, SLAVE3_ID, SLAVE3_PLACEMENT },
#endif
};

// ── BLE Central: una entrada por esclavo ─────────────────────────────────
static BLEDevice         slave_devs[N_SLAVES];
static BLECharacteristic slave_chars[N_SLAVES];
static volatile float    slave_imu[N_SLAVES][6];
static volatile bool     slave_ready[N_SLAVES];

// ── Callbacks BLE (ArduinoBLE no admite lambdas con captura) ─────────────
static void storePayload(BLECharacteristic& ch, int idx) {
    if (ch.valueLength() != 6 * (int)sizeof(float)) return;
    memcpy((void*)slave_imu[idx], ch.value(), 6 * sizeof(float));
    slave_ready[idx] = true;
}
void onSlave0(BLEDevice, BLECharacteristic ch) { storePayload(ch, 0); }
#if N_SLAVES >= 2
void onSlave1(BLEDevice, BLECharacteristic ch) { storePayload(ch, 1); }
#endif

typedef void (*SlaveCB)(BLEDevice, BLECharacteristic);
static SlaveCB SLAVE_CBS[N_SLAVES] = {
    onSlave0,
#if N_SLAVES >= 2
    onSlave1,
#endif
};

// ── Estado general ────────────────────────────────────────────────────────
enum HubState { CONN_SCANNING, CONN_READY };
static HubState hub_state     = CONN_SCANNING;
static int      scan_target   = 0;
static unsigned long scan_start_ms = 0;
static bool     slave_active[N_SLAVES] = {};  // se conectó al menos una vez

enum CapState { CAP_IDLE, CAP_RUNNING };
static CapState cap_state = CAP_IDLE;

static uint32_t last_sample_us = 0;
static uint32_t sample_count   = 0;

static String   trial_id   = "t000";
static String   subject_id = "s000";
static String   cmd_buf    = "";

// ── LED helpers ───────────────────────────────────────────────────────────
static void set_led(bool r, bool g, bool b) {
    digitalWrite(LEDR, r ? LOW : HIGH);
    digitalWrite(LEDG, g ? LOW : HIGH);
    digitalWrite(LEDB, b ? LOW : HIGH);
}

// ── Emitir una línea CSV por USB Serial ───────────────────────────────────
static void emit_csv(uint32_t ts, uint8_t sid, const char* placement,
                     float ax, float ay, float az,
                     float gx, float gy, float gz) {
    // Formato: timestamp_ms,sensor_id,placement,ax,ay,az,gx,gy,gz
    Serial.print(ts);          Serial.print(',');
    Serial.print(sid);         Serial.print(',');
    Serial.print(placement);   Serial.print(',');
    Serial.print(ax, 4);       Serial.print(',');
    Serial.print(ay, 4);       Serial.print(',');
    Serial.print(az, 4);       Serial.print(',');
    Serial.print(gx, 4);       Serial.print(',');
    Serial.print(gy, 4);       Serial.print(',');
    Serial.println(gz, 4);
}

// ── Procesar comando USB Serial ───────────────────────────────────────────
static void process_command(const String& cmd) {
    if (cmd == "START" && hub_state == CONN_READY && cap_state == CAP_IDLE) {
        cap_state      = CAP_RUNNING;
        last_sample_us = micros();
        sample_count   = 0;
        for (int i = 0; i < N_SLAVES; i++) slave_ready[i] = false;
        set_led(false, true, false);   // verde = capturando
        Serial.println("ACK:START");

    } else if (cmd == "STOP" && cap_state == CAP_RUNNING) {
        cap_state = CAP_IDLE;
        set_led(false, false, true);   // azul = idle
        Serial.print("ACK:STOP:"); Serial.println(sample_count);

    } else if (cmd.startsWith("SET_TRIAL:")) {
        trial_id = cmd.substring(10); trial_id.trim();
        Serial.print("ACK:SET_TRIAL:"); Serial.println(trial_id);

    } else if (cmd.startsWith("SET_SUBJECT:")) {
        subject_id = cmd.substring(12); subject_id.trim();
        Serial.print("ACK:SET_SUBJECT:"); Serial.println(subject_id);

    } else if (cmd == "SYNC") {
        Serial.print("SYNC_ACK:"); Serial.println(millis());

    } else if (cmd == "STATUS") {
        Serial.print("STATUS:hub,sensor_id="); Serial.print(SENSOR_ID);
        Serial.print(",placement=");           Serial.print(PLACEMENT);
        Serial.print(",trial=");               Serial.print(trial_id);
        Serial.print(",subject=");             Serial.print(subject_id);
        Serial.print(",capturing=");           Serial.print(cap_state == CAP_RUNNING ? 1 : 0);
        Serial.print(",samples=");             Serial.print(sample_count);
        Serial.print(",ready=");               Serial.print(hub_state == CONN_READY ? 1 : 0);
        for (int i = 0; i < N_SLAVES; i++) {
            Serial.print(",s"); Serial.print(SLAVES[i].sensor_id);
            Serial.print("="); Serial.print(slave_devs[i] && slave_devs[i].connected() ? 1 : 0);
        }
        Serial.println();
    }
}

// ── BLE: intentar conectar al esclavo actual (scan_target) ────────────────
// Devuelve true si encontró y procesó (conectado o fallido), false si aún busca.
static bool tryConnectNext() {
    BLEDevice found = BLE.available();
    bool timed_out  = (millis() - scan_start_ms) >= SLAVE_SCAN_TIMEOUT_MS;

    if (!found && !timed_out) return false;   // seguir esperando

    BLE.stopScan();
    int idx = scan_target;

    if (found) {
        Serial.print("STATUS:FOUND "); Serial.println(SLAVES[idx].name);

        if (found.connect() && found.discoverAttributes()) {
            BLECharacteristic ch = found.characteristic(BLE_IMU_UUID);
            if (ch && ch.canSubscribe()) {
                ch.setEventHandler(BLEUpdated, SLAVE_CBS[idx]);
                ch.subscribe();
                slave_devs[idx]  = found;
                slave_chars[idx] = ch;
                slave_active[idx] = true;
                Serial.print("STATUS:SLAVE_CONNECTED ");
                Serial.println(SLAVES[idx].name);
            } else {
                found.disconnect();
                Serial.print("STATUS:CHAR_NOT_FOUND "); Serial.println(SLAVES[idx].name);
            }
        } else {
            found.disconnect();
            Serial.print("STATUS:CONNECT_FAILED "); Serial.println(SLAVES[idx].name);
        }
    } else {
        Serial.print("STATUS:SCAN_TIMEOUT "); Serial.println(SLAVES[idx].name);
    }

    // Avanzar al siguiente esclavo o declarar READY
    scan_target++;
    if (scan_target < N_SLAVES) {
        Serial.print("STATUS:SCANNING "); Serial.println(SLAVES[scan_target].name);
        BLE.scanForName(SLAVES[scan_target].name);
        scan_start_ms = millis();
    } else {
        hub_state = CONN_READY;
        int n_conn = 0;
        for (int i = 0; i < N_SLAVES; i++)
            if (slave_active[i] && slave_devs[i].connected()) n_conn++;
        Serial.print("STATUS:READY slaves_connected="); Serial.println(n_conn);
        set_led(false, false, true);   // azul = listo
        Serial.println("READY");
    }
    return true;
}

// ── setup() ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(BAUD_RATE);

    pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
    set_led(false, false, false);

    // IMU
    if (!IMU.begin()) {
        set_led(true, false, false);
        while (true) {
            if (Serial.available()) {
                String s = Serial.readStringUntil('\n');
                if (s.indexOf("STATUS") >= 0) Serial.println("ERROR:IMU_NOT_FOUND");
            }
            delay(300);
        }
    }
    Serial.print("IMU OK — "); Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");

    // BLE
    if (!BLE.begin()) {
        Serial.println("ERROR:BLE_INIT");
        while (true) delay(500);
    }

    // Iniciar escaneo del primer esclavo
    Serial.print("STATUS:SCANNING "); Serial.println(SLAVES[0].name);
    BLE.scanForName(SLAVES[0].name);
    scan_start_ms = millis();
    set_led(true, false, false);   // rojo = escaneando
}

// ── loop() ────────────────────────────────────────────────────────────────
void loop() {
    // ── 1. Leer comandos USB Serial ─────────────────────────────────────
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            cmd_buf.trim();
            if (cmd_buf.length() > 0) process_command(cmd_buf);
            cmd_buf = "";
        } else if (c != '\r') {
            cmd_buf += c;
            if (cmd_buf.length() > 64) cmd_buf = "";
        }
    }

    // ── 2. Fase de escaneo/conexión ──────────────────────────────────────
    if (hub_state == CONN_SCANNING) {
        // Parpadeo rojo mientras busca
        static unsigned long last_blink = 0;
        if (millis() - last_blink > 400) {
            last_blink = millis();
            static bool led_on = false;
            led_on = !led_on;
            set_led(led_on, false, false);
        }
        tryConnectNext();
        return;
    }

    // ── 3. Verificar reconexión de esclavos (solo en IDLE) ───────────────
    if (cap_state == CAP_IDLE) {
        for (int i = 0; i < N_SLAVES; i++) {
            if (slave_active[i] && slave_devs[i] && !slave_devs[i].connected()) {
                Serial.print("STATUS:SLAVE_LOST "); Serial.println(SLAVES[i].name);
                Serial.print("STATUS:SCANNING "); Serial.println(SLAVES[i].name);
                BLE.scanForName(SLAVES[i].name);
                scan_start_ms = millis();
                scan_target   = i;
                hub_state     = CONN_SCANNING;
                set_led(true, false, false);
                return;
            }
        }
    }

    // ── 4. Procesar notificaciones BLE de esclavos ───────────────────────
    BLE.poll();
    for (int i = 0; i < N_SLAVES; i++) {
        if (!slave_active[i] || !slave_devs[i].connected()) continue;
        slave_devs[i].poll();

        // Si en CAPTURING hay datos nuevos del esclavo, emitir CSV
        if (cap_state == CAP_RUNNING && slave_ready[i]) {
            slave_ready[i] = false;
            float ax = slave_imu[i][0], ay = slave_imu[i][1], az = slave_imu[i][2];
            float gx = slave_imu[i][3], gy = slave_imu[i][4], gz = slave_imu[i][5];
            emit_csv(millis(), SLAVES[i].sensor_id, SLAVES[i].placement,
                     ax, ay, az, gx, gy, gz);
            sample_count++;
        } else {
            // Descartar datos en IDLE para no acumular stale
            slave_ready[i] = false;
        }
    }

    // ── 5. IMU propio: emitir a SAMPLE_HZ Hz ────────────────────────────
    if (cap_state != CAP_RUNNING) return;

    uint32_t now = micros();
    if ((uint32_t)(now - last_sample_us) < SAMPLE_PERIOD_US) return;
    last_sample_us += SAMPLE_PERIOD_US;

    if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return;

    float ax, ay, az, gx, gy, gz;
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    emit_csv(millis(), SENSOR_ID, PLACEMENT, ax, ay, az, gx, gy, gz);
    sample_count++;
}
