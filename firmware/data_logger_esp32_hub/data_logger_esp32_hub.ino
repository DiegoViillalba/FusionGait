/**
 * data_logger_esp32_hub.ino — FusionGait Hub BLE para ESP32-S3
 *
 * Hardware : ESP32-S3 (cualquier DevKit con BLE)
 * Role     : Hub puro BLE → USB Serial.
 *            Se conecta a N_SLAVES sensores Arduino (GaitNode_2, GaitNode_3),
 *            recibe sus datos IMU via BLE Notify y los reenvía al PC como CSV
 *            por el puerto USB Serial nativo del ESP32-S3.
 *
 * Sin IMU propio — el ESP32 es un relay; no emite muestras con sensor_id=1.
 *
 * Salida CSV (mismo formato que data_logger_hub y data_logger_node):
 *   timestamp_ms,sensor_id,placement,ax,ay,az,gx,gy,gz
 *
 * Comandos USB Serial:
 *   STATUS  → estado de conexión y captura
 *   START   → comenzar grabación (reenvío de datos)
 *   STOP    → detener grabación
 *   SET_TRIAL:<id>
 *   SET_SUBJECT:<id>
 *   SYNC    → SYNC_ACK:<millis()>
 *
 * LEDs (LED_PIN, activo en alto en la mayoría de ESP32 DevKit):
 *   Parpadeante lento → escaneando esclavos BLE
 *   Encendido fijo    → listo / capturando
 *
 * Librería BLE usada: NimBLE-Arduino (más eficiente en RAM que BLE genérico)
 * Instalar via Arduino Library Manager: "NimBLE-Arduino"
 */

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "config.h"

// ── Tabla de esclavos ──────────────────────────────────────────────────────
struct SlaveInfo {
    const char* name;
    uint8_t     sensor_id;
    const char* placement;
};

static const SlaveInfo SLAVES[N_SLAVES] = {
    { SLAVE1_NAME, SLAVE1_ID, SLAVE1_PLACEMENT },
#if N_SLAVES >= 2
    { SLAVE2_NAME, SLAVE2_ID, SLAVE2_PLACEMENT },
#endif
};

// ── Estado BLE ─────────────────────────────────────────────────────────────
static NimBLEClient*     clients[N_SLAVES]    = {};
static volatile float    slave_imu[N_SLAVES][6];
static volatile bool     slave_ready[N_SLAVES];
static bool              slave_active[N_SLAVES] = {};

// ── Estado de captura ──────────────────────────────────────────────────────
enum CapState { CAP_IDLE, CAP_RUNNING };
static CapState cap_state  = CAP_IDLE;
static uint32_t sample_cnt = 0;
static String   trial_id   = "t000";
static String   subject_id = "s000";
static String   cmd_buf    = "";

// ── LED ────────────────────────────────────────────────────────────────────
static void led(bool on) {
#if LED_PIN >= 0
    digitalWrite(LED_PIN, on ? HIGH : LOW);
#endif
}

// ── CSV output ─────────────────────────────────────────────────────────────
static void emit_csv(uint8_t sid, const char* placement,
                     float ax, float ay, float az,
                     float gx, float gy, float gz) {
    Serial.print(millis());      Serial.print(',');
    Serial.print(sid);           Serial.print(',');
    Serial.print(placement);     Serial.print(',');
    Serial.print(ax, 4);         Serial.print(',');
    Serial.print(ay, 4);         Serial.print(',');
    Serial.print(az, 4);         Serial.print(',');
    Serial.print(gx, 4);         Serial.print(',');
    Serial.print(gy, 4);         Serial.print(',');
    Serial.println(gz, 4);
}

// ── Procesar comando USB ────────────────────────────────────────────────────
static void process_command(const String& cmd) {
    if (cmd == "START" && cap_state == CAP_IDLE) {
        cap_state  = CAP_RUNNING;
        sample_cnt = 0;
        for (int i = 0; i < N_SLAVES; i++) slave_ready[i] = false;
        led(true);
        Serial.println("ACK:START");

    } else if (cmd == "STOP" && cap_state == CAP_RUNNING) {
        cap_state = CAP_IDLE;
        Serial.print("ACK:STOP:"); Serial.println(sample_cnt);

    } else if (cmd.startsWith("SET_TRIAL:")) {
        trial_id = cmd.substring(10); trial_id.trim();
        Serial.print("ACK:SET_TRIAL:"); Serial.println(trial_id);

    } else if (cmd.startsWith("SET_SUBJECT:")) {
        subject_id = cmd.substring(12); subject_id.trim();
        Serial.print("ACK:SET_SUBJECT:"); Serial.println(subject_id);

    } else if (cmd == "SYNC") {
        Serial.print("SYNC_ACK:"); Serial.println(millis());

    } else if (cmd == "STATUS") {
        Serial.print("STATUS:esp32hub,sensor_id="); Serial.print(HUB_SENSOR_ID);
        Serial.print(",trial=");      Serial.print(trial_id);
        Serial.print(",subject=");    Serial.print(subject_id);
        Serial.print(",capturing=");  Serial.print(cap_state == CAP_RUNNING ? 1 : 0);
        Serial.print(",samples=");    Serial.print(sample_cnt);
        for (int i = 0; i < N_SLAVES; i++) {
            Serial.print(",s"); Serial.print(SLAVES[i].sensor_id);
            Serial.print("=");  Serial.print(
                (clients[i] && clients[i]->isConnected()) ? 1 : 0);
        }
        Serial.println();
    }
}

// ── Callbacks de notificación BLE ────────────────────────────────────────
// NimBLE 2.5.0 instalado: notify_callback es
//   std::function<void(NimBLERemoteCharacteristic*, uint8_t*, size_t, bool)>
// (misma firma que NimBLE 1.x)

static void notifyCb0(NimBLERemoteCharacteristic*,
                      uint8_t* data, size_t len, bool) {
    if (len == 24) {
        memcpy((void*)slave_imu[0], data, 24);
        slave_ready[0] = true;
    }
}
#if N_SLAVES >= 2
static void notifyCb1(NimBLERemoteCharacteristic*,
                      uint8_t* data, size_t len, bool) {
    if (len == 24) {
        memcpy((void*)slave_imu[1], data, 24);
        slave_ready[1] = true;
    }
}
#endif

typedef void (*NotifyCbFn)(NimBLERemoteCharacteristic*,
                            uint8_t*, size_t, bool);
static NotifyCbFn NOTIFY_CBS[N_SLAVES] = {
    notifyCb0,
#if N_SLAVES >= 2
    notifyCb1,
#endif
};

// ── Scan helper: busca por nombre usando callback (NimBLE 2.x) ────────────
// NimBLE 2.x cambió: start() devuelve bool; los resultados se leen via
// getResults() después del scan o via callback.

static NimBLEAddress g_found_addr;
static bool          g_found_flag = false;
static const char*   g_scan_name  = nullptr;

class HubScanCb : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* dev) override {
        if (g_scan_name && dev->getName() == g_scan_name) {
            g_found_addr = dev->getAddress();
            g_found_flag = true;
            NimBLEDevice::getScan()->stop();
        }
    }
} g_scan_cb;

// ── Conectar a un esclavo ──────────────────────────────────────────────────
static bool connectSlave(int idx) {
    Serial.print("STATUS:SCANNING "); Serial.println(SLAVES[idx].name);

    g_scan_name  = SLAVES[idx].name;
    g_found_flag = false;

    NimBLEScan* scanner = NimBLEDevice::getScan();
    scanner->setScanCallbacks(&g_scan_cb, false);
    scanner->setActiveScan(true);
    scanner->setInterval(100);   // ms
    scanner->setWindow(80);      // ms

    // En NimBLE 2.x start() toma ms y es NO bloqueante → esperamos en loop
    scanner->start(SLAVE_SCAN_TIMEOUT_MS, false);
    uint32_t t0 = millis();
    while (!g_found_flag && (millis() - t0 < SLAVE_SCAN_TIMEOUT_MS + 500)) {
        delay(10);
    }
    scanner->stop();

    if (!g_found_flag) {
        Serial.print("STATUS:SCAN_TIMEOUT "); Serial.println(SLAVES[idx].name);
        scanner->clearResults();
        return false;
    }
    scanner->clearResults();

    Serial.print("STATUS:FOUND "); Serial.print(SLAVES[idx].name);
    Serial.print(" @ "); Serial.println(g_found_addr.toString().c_str());

    NimBLEClient* client = NimBLEDevice::createClient();
    client->setConnectionParams(12, 12, 0, 51);   // ~15 ms interval, ~800 ms timeout

    if (!client->connect(g_found_addr)) {
        Serial.print("STATUS:CONNECT_FAILED "); Serial.println(SLAVES[idx].name);
        NimBLEDevice::deleteClient(client);
        return false;
    }

    NimBLERemoteService* svc = client->getService(BLE_SERVICE_UUID);
    if (!svc) {
        Serial.print("STATUS:SERVICE_NOT_FOUND "); Serial.println(SLAVES[idx].name);
        client->disconnect();
        NimBLEDevice::deleteClient(client);
        return false;
    }

    NimBLERemoteCharacteristic* ch = svc->getCharacteristic(BLE_IMU_UUID);
    if (!ch || !ch->canNotify()) {
        Serial.print("STATUS:CHAR_NOT_FOUND "); Serial.println(SLAVES[idx].name);
        client->disconnect();
        NimBLEDevice::deleteClient(client);
        return false;
    }

    ch->subscribe(true, NOTIFY_CBS[idx]);
    clients[idx]      = client;
    slave_active[idx] = true;
    Serial.print("STATUS:SLAVE_CONNECTED "); Serial.println(SLAVES[idx].name);
    return true;
}

// ── setup() ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(BAUD_RATE);
    // Esperar a que el host USB CDC abra el puerto (máx 3 s)
    // Si no hay host en 3 s, continuamos de todas formas.
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}
    delay(200);

#if LED_PIN >= 0
    pinMode(LED_PIN, OUTPUT);
    led(false);
#endif

    Serial.println("STATUS:BOOT esp32hub");
    Serial.flush();

    NimBLEDevice::init("");   // nombre vacío: no anunciamos
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);  // máxima potencia TX

    // Conectar a cada esclavo en orden (con timeout por esclavo)
    int connected = 0;
    for (int i = 0; i < N_SLAVES; i++) {
        // Parpadeo durante escaneo (1.2 s visible)
        for (int b = 0; b < 6; b++) { led(b % 2); delay(200); }
        Serial.flush();
        if (connectSlave(i)) connected++;
        Serial.flush();
    }

    Serial.print("STATUS:READY esp32hub connected="); Serial.println(connected);
    Serial.flush();
    Serial.println("READY");
    Serial.flush();
    led(connected > 0);
}

// ── loop() ───────────────────────────────────────────────────────────────
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

    // ── 2. Verificar conexiones (solo en IDLE para no interrumpir trial) ──
    if (cap_state == CAP_IDLE) {
        for (int i = 0; i < N_SLAVES; i++) {
            if (slave_active[i] && clients[i] && !clients[i]->isConnected()) {
                Serial.print("STATUS:SLAVE_LOST "); Serial.println(SLAVES[i].name);
                NimBLEDevice::deleteClient(clients[i]);
                clients[i] = nullptr;
                led(false);
                // Intentar reconectar una vez
                if (connectSlave(i)) led(true);
            }
        }
    }

    // ── 3. Reenviar datos BLE como CSV ────────────────────────────────────
    if (cap_state != CAP_RUNNING) return;

    for (int i = 0; i < N_SLAVES; i++) {
        if (!slave_active[i]) continue;
        if (!slave_ready[i])  continue;

        // Leer atómicamente (desactivar interrupciones no es posible en ESP32
        // de forma sencilla, pero el volatile + copia es suficientemente seguro
        // para float de 4 bytes en Xtensa/RISC-V alineado)
        float buf[6];
        slave_ready[i] = false;
        memcpy(buf, (const void*)slave_imu[i], sizeof(buf));

        emit_csv(SLAVES[i].sensor_id, SLAVES[i].placement,
                 buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
        sample_cnt++;
    }
}
