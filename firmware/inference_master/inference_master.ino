/**
 * inference_master.ino — FusionGait Master + TFLite Inference
 *
 * Hardware : Arduino Nano 33 BLE Sense Rev2 (nRF52840 + BMI270)
 * Role     : BLE Central — se conecta a N_SLAVES esclavos, recibe sus
 *            datos IMU, combina con el propio, normaliza y corre inferencia
 *            TFLite int8 cada STEP muestras nuevas.
 *
 * Configuración en config.h:
 *   N_SLAVES 1  → Master + Slave2          (N_FEATURES=12)
 *   N_SLAVES 2  → Master + Slave2 + Slave3 (N_FEATURES=18)
 *
 * Archivos requeridos (generados por scripts/export_to_arduino.py):
 *   gait_model_Xs.h  — modelo int8 (X = N_SLAVES+1 sensores)
 *   scaler.h         — SCALER_MEAN[N_FEATURES], SCALER_STD[N_FEATURES]
 *
 * Salida Serial 115200:
 *   INFER,<fase>,<p0>,<p1>,<p2>,<p3>   — resultado de inferencia
 *   STATUS:<msg>                        — estado conexión BLE
 *   ARENA:<bytes>                       — arena TFLite usada al arrancar
 *   ERROR:<msg>                         — error fatal
 *
 * LEDs:
 *   Rojo parpadeante → escaneando BLE
 *   Azul fijo        → esclavos conectados, buffer llenándose
 *   Verde fijo       → inferencia activa
 */

#include <ArduinoBLE.h>
#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/schema/schema_generated.h>

#include "config.h"
#include "gait_model.h"    // generado por export_to_arduino.py (cualquier nombre)
#include "scaler.h"        // generado por export_to_arduino.py

#if IMU_REV2
  #include <Arduino_BMI270_BMM150.h>
#else
  #include <Arduino_LSM9DS1.h>
#endif

// ── Nombres de fase (orden = PHASE2ID del notebook) ──────────────────────
static const char* PHASE_NAMES[N_CLASSES] = {
    "loading", "midstance", "terminal", "swing"
};

// ── Nombres y callbacks de esclavos (tabla fija, sin heap) ───────────────
static const char* SLAVE_NAMES[N_SLAVES] = {
#if N_SLAVES >= 1
    SLAVE2_NAME,
#endif
#if N_SLAVES >= 2
    SLAVE3_NAME,
#endif
};

// Datos IMU recibidos de cada esclavo: slave_imu[esclavo][ax,ay,az,gx,gy,gz]
static volatile float slave_imu[N_SLAVES][6];
static volatile bool  slave_ready[N_SLAVES];

// Escribe 24 bytes del payload BLE al slot de esclavo `idx`
static void copySlavePayload(BLECharacteristic& ch, int idx) {
    if (ch.valueLength() != 6 * (int)sizeof(float)) return;
    memcpy((void*)slave_imu[idx], ch.value(), 6 * sizeof(float));
    slave_ready[idx] = true;
}

// Callbacks individuales (ArduinoBLE no admite closures/lambdas con captura)
void onSlave0Notify(BLEDevice, BLECharacteristic ch) { copySlavePayload(ch, 0); }
#if N_SLAVES >= 2
void onSlave1Notify(BLEDevice, BLECharacteristic ch) { copySlavePayload(ch, 1); }
#endif

typedef void (*SlaveNotifyCB)(BLEDevice, BLECharacteristic);
static SlaveNotifyCB SLAVE_CBS[N_SLAVES] = {
    onSlave0Notify,
#if N_SLAVES >= 2
    onSlave1Notify,
#endif
};

// ── Objetos BLE ───────────────────────────────────────────────────────────
static BLEDevice        slave_devs[N_SLAVES];
static BLECharacteristic slave_chars[N_SLAVES];

// Máquina de estados de conexión
enum ConnState { CONN_SCANNING, CONN_READY };
static ConnState conn_state  = CONN_SCANNING;
static int       scan_target = 0;   // índice del esclavo que se está buscando

// ── Tensor arena ──────────────────────────────────────────────────────────
static uint8_t tensor_arena[TENSOR_ARENA_KB * 1024];

// ── TFLite globals ────────────────────────────────────────────────────────
static const tflite::Model*      tfl_model   = nullptr;
static tflite::MicroInterpreter* interpreter = nullptr;
static TfLiteTensor*             input_tens  = nullptr;
static TfLiteTensor*             output_tens = nullptr;

// ── Ring buffer ───────────────────────────────────────────────────────────
static float ring_buf[WINDOW_SIZE][N_FEATURES];
static int   buf_head          = 0;
static int   buf_count         = 0;
static int   samples_since_inf = 0;

// ── LED helpers ───────────────────────────────────────────────────────────
inline void setLED(bool r, bool g, bool b) {
    digitalWrite(LEDR, r ? LOW : HIGH);
    digitalWrite(LEDG, g ? LOW : HIGH);
    digitalWrite(LEDB, b ? LOW : HIGH);
}

void haltError(const char* msg) {
    Serial.println(msg);
    while (true) { setLED(true,false,false); delay(300); setLED(false,false,false); delay(300); }
}

// ── Normalización Z-score por canal ──────────────────────────────────────
static inline void normalise(float* sample) {
    for (int i = 0; i < N_FEATURES; i++)
        sample[i] = (sample[i] - SCALER_MEAN[i]) / (SCALER_STD[i] + 1e-8f);
}

// ── Inferencia TFLite int8 ────────────────────────────────────────────────
// Devuelve índice de clase predicha (-1 si error).
// out_probs[N_CLASSES] recibe las probabilidades dequantizadas.
static int runInference(float out_probs[N_CLASSES]) {
    int8_t*  inp   = input_tens->data.int8;
    float    scale = input_tens->params.scale;
    int32_t  zp    = input_tens->params.zero_point;

    // Llenar tensor desde el ring buffer (más antiguo primero)
    // Shape esperado: (1, WINDOW_SIZE, N_FEATURES, 1) — CNN2D
    for (int t = 0; t < WINDOW_SIZE; t++) {
        int row = (buf_head + t) % WINDOW_SIZE;
        for (int f = 0; f < N_FEATURES; f++) {
            float q = ring_buf[row][f] / scale + (float)zp;
            inp[t * N_FEATURES + f] = (int8_t)constrain(q, -128.0f, 127.0f);
        }
    }

    if (interpreter->Invoke() != kTfLiteOk) return -1;

    int8_t*  out     = output_tens->data.int8;
    float    o_scale = output_tens->params.scale;
    int32_t  o_zp    = output_tens->params.zero_point;
    int      best    = 0;
    float    best_v  = -1e9f;

    for (int c = 0; c < N_CLASSES; c++) {
        float v      = (out[c] - o_zp) * o_scale;
        out_probs[c] = v;
        if (v > best_v) { best_v = v; best = c; }
    }
    return best;
}

// ── Conectar al siguiente esclavo pendiente ───────────────────────────────
static bool tryConnectNext() {
    BLEDevice found = BLE.available();
    if (!found) return false;

    BLE.stopScan();
    int idx = scan_target;
    slave_devs[idx] = found;

    Serial.print("STATUS:FOUND "); Serial.println(SLAVE_NAMES[idx]);

    if (!slave_devs[idx].connect()) {
        Serial.print("STATUS:CONNECT_FAILED "); Serial.println(SLAVE_NAMES[idx]);
        BLE.scanForName(SLAVE_NAMES[idx]);
        return false;
    }
    if (!slave_devs[idx].discoverAttributes()) {
        Serial.print("STATUS:DISCOVER_FAILED "); Serial.println(SLAVE_NAMES[idx]);
        slave_devs[idx].disconnect();
        BLE.scanForName(SLAVE_NAMES[idx]);
        return false;
    }

    slave_chars[idx] = slave_devs[idx].characteristic(BLE_IMU_UUID);
    if (!slave_chars[idx] || !slave_chars[idx].canSubscribe()) {
        Serial.print("STATUS:CHAR_NOT_FOUND "); Serial.println(SLAVE_NAMES[idx]);
        slave_devs[idx].disconnect();
        BLE.scanForName(SLAVE_NAMES[idx]);
        return false;
    }

    slave_chars[idx].setEventHandler(BLEValueUpdated, SLAVE_CBS[idx]);
    slave_chars[idx].subscribe();
    Serial.print("STATUS:SLAVE_CONNECTED "); Serial.println(SLAVE_NAMES[idx]);

    // Avanzar al siguiente esclavo o pasar a READY
    scan_target++;
    if (scan_target < N_SLAVES) {
        BLE.scanForName(SLAVE_NAMES[scan_target]);
        Serial.print("STATUS:SCANNING "); Serial.println(SLAVE_NAMES[scan_target]);
    } else {
        conn_state = CONN_READY;
        Serial.println("STATUS:ALL_SLAVES_CONNECTED");
        setLED(false, false, true);   // azul = todos conectados
    }
    return true;
}

// ── Verificar que todos los esclavos siguen conectados ────────────────────
static bool allSlavesConnected() {
    for (int i = 0; i < N_SLAVES; i++)
        if (!slave_devs[i] || !slave_devs[i].connected()) return false;
    return true;
}

// ── setup() ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000);

    pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
    setLED(false, false, false);

    // ── IMU ──────────────────────────────────────────────────────────────
    if (!IMU.begin()) haltError("ERROR:IMU_INIT");
    Serial.print("IMU OK — "); Serial.print(IMU.accelerationSampleRate()); Serial.println(" Hz");

    // ── TFLite ───────────────────────────────────────────────────────────
    tfl_model = tflite::GetModel(gait_model);
    if (tfl_model->version() != TFLITE_SCHEMA_VERSION)
        haltError("ERROR:TFLITE_SCHEMA_MISMATCH");

    // Ops usadas por GaitCNN2D
    static tflite::MicroMutableOpResolver<6> resolver;
    resolver.AddConv2D();
    resolver.AddMean();             // GlobalAveragePooling2D
    resolver.AddFullyConnected();   // Dense
    resolver.AddSoftmax();
    resolver.AddReshape();
    resolver.AddQuantize();

    static tflite::MicroInterpreter static_interp(
        tfl_model, resolver, tensor_arena, sizeof(tensor_arena));
    interpreter = &static_interp;

    if (interpreter->AllocateTensors() != kTfLiteOk)
        haltError("ERROR:ARENA_TOO_SMALL — sube TENSOR_ARENA_KB en config.h");

    input_tens  = interpreter->input(0);
    output_tens = interpreter->output(0);

    Serial.print("ARENA:"); Serial.print(interpreter->arena_used_bytes()); Serial.println(" B");
    Serial.print("N_SLAVES="); Serial.print(N_SLAVES);
    Serial.print("  N_FEATURES="); Serial.println(N_FEATURES);

    // Verificar forma: debe ser (1, WINDOW_SIZE, N_FEATURES, 1)
    Serial.print("INPUT_SHAPE:(");
    for (int i = 0; i < input_tens->dims->size; i++) {
        if (i) Serial.print(",");
        Serial.print(input_tens->dims->data[i]);
    }
    Serial.println(")");

    // ── BLE Central ──────────────────────────────────────────────────────
    if (!BLE.begin()) haltError("ERROR:BLE_INIT");

    BLE.scanForName(SLAVE_NAMES[0]);
    Serial.print("STATUS:SCANNING "); Serial.println(SLAVE_NAMES[0]);
    setLED(true, false, false);   // rojo = escaneando
}

// ── loop() ────────────────────────────────────────────────────────────────
void loop() {
    // ── 1. Gestionar conexiones BLE ──────────────────────────────────────
    if (conn_state == CONN_SCANNING) {
        // Parpadeo rojo mientras busca
        static unsigned long lastBlink = 0;
        if (millis() - lastBlink > 400) {
            lastBlink = millis();
            static bool ledOn = false;
            ledOn = !ledOn;
            setLED(ledOn, false, false);
        }
        tryConnectNext();
        return;
    }

    // Verificar que todos siguen conectados
    if (!allSlavesConnected()) {
        Serial.println("STATUS:SLAVE_LOST — reconnecting");
        conn_state   = CONN_SCANNING;
        scan_target  = 0;
        buf_count    = 0;
        buf_head     = 0;
        samples_since_inf = 0;
        // Reconectar desde el primer esclavo perdido
        for (int i = 0; i < N_SLAVES; i++) {
            if (!slave_devs[i] || !slave_devs[i].connected()) {
                scan_target = i;
                BLE.scanForName(SLAVE_NAMES[i]);
                Serial.print("STATUS:SCANNING "); Serial.println(SLAVE_NAMES[i]);
                break;
            }
        }
        setLED(true, false, false);
        return;
    }

    // ── 2. Leer IMU del Master ───────────────────────────────────────────
    if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return;

    float ax1, ay1, az1, gx1, gy1, gz1;
    IMU.readAcceleration(ax1, ay1, az1);
    IMU.readGyroscope(gx1, gy1, gz1);

    // ── 3. Esperar datos de todos los esclavos ───────────────────────────
    for (int i = 0; i < N_SLAVES; i++)
        if (!slave_ready[i]) return;

    // Consumir flags atómicamente
    for (int i = 0; i < N_SLAVES; i++) slave_ready[i] = false;

    // ── 4. Construir muestra: [master | slave0 | slave1 | ...] ───────────
    float sample[N_FEATURES];
    sample[0] = ax1; sample[1] = ay1; sample[2] = az1;
    sample[3] = gx1; sample[4] = gy1; sample[5] = gz1;
    for (int s = 0; s < N_SLAVES; s++) {
        for (int f = 0; f < 6; f++)
            sample[6 + s * 6 + f] = (float)slave_imu[s][f];
    }
    normalise(sample);

    // ── 5. Ring buffer ───────────────────────────────────────────────────
    memcpy(ring_buf[buf_head], sample, sizeof(sample));
    buf_head = (buf_head + 1) % WINDOW_SIZE;
    if (buf_count < WINDOW_SIZE) buf_count++;
    samples_since_inf++;

    if (buf_count == WINDOW_SIZE && samples_since_inf == 1)
        setLED(false, true, false);   // verde = buffer lleno, inferencia activa

    // ── 6. Inferir cada STEP muestras nuevas (buffer completo) ───────────
    if (buf_count < WINDOW_SIZE || samples_since_inf < STEP) return;
    samples_since_inf = 0;

    float probs[N_CLASSES];
    int phase = runInference(probs);
    if (phase < 0) { Serial.println("ERROR:INFERENCE_FAILED"); return; }

    // Formato: INFER,<fase>,<p0>,<p1>,<p2>,<p3>
    Serial.print("INFER,"); Serial.print(PHASE_NAMES[phase]);
    for (int c = 0; c < N_CLASSES; c++) {
        Serial.print(","); Serial.print(probs[c], 3);
    }
    Serial.println();
}
