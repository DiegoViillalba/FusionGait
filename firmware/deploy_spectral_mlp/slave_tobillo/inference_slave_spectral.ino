/**
 * inference_slave_spectral.ino — FusionGait BLE Slave con extracción espectral
 *
 * Hardware : Arduino Nano 33 BLE Sense Rev2 (nRF52840 + BMI270)
 * Role     : BLE Peripheral — acumula ventana local, extrae 84 features
 *            espectrales (FFT + estadísticos) y los envía al Master.
 *
 * Diferencia con inference_slave.ino (raw):
 *   RAW      → envía 6 floats/muestra @ 100 Hz  = 24 bytes cada 10 ms
 *   SPECTRAL → envía 84 floats/ventana @ STEP   = 336 bytes cada 100 ms
 *
 * Extractor (replica del notebook, celda spectral_features):
 *   Por cada canal IMU (6 canales):
 *     - 4 estadísticos:  media, std, RMS, pico-a-pico
 *     - N_FFT_BINS=10 energías FFT en bandas uniformes 0–50 Hz
 *   Total: 6 × (4 + 10) = 84 floats
 *
 * LEDs:
 *   Azul parpadea → anunciando (sin conexión)
 *   Verde fijo    → conectado, extrayendo y enviando features
 *   Rojo fijo     → error de IMU
 */

#include <ArduinoBLE.h>
#include "config.h"

#if IMU_REV2
  #include <Arduino_BMI270_BMM150.h>
#else
  #include <Arduino_LSM9DS1.h>
#endif

// ── BLE ──────────────────────────────────────────────────────────────────
// SPECTRAL_PAYLOAD_BYTES = 84 * 4 = 336 bytes
// El MTU de ArduinoBLE soporta hasta 512 bytes, pero negociado ~244.
// Enviamos en dos chunks si es necesario (ver sendSpectralFeatures).
static const int SPECTRAL_PAYLOAD = SPECTRAL_DIM_PER_SENSOR * sizeof(float); // 336

BLEService        imuService(BLE_SERVICE_UUID);
BLECharacteristic spectralChar(BLE_IMU_UUID,
                                BLENotify | BLERead,
                                SPECTRAL_PAYLOAD);

// ── Ring buffer local ─────────────────────────────────────────────────────
static float ring[WINDOW_SIZE][N_AXES];
static int   buf_head  = 0;
static int   buf_count = 0;
static int   step_cnt  = 0;

// ── Timing ───────────────────────────────────────────────────────────────
static unsigned long last_sample_ms = 0;

// ── LED helper ────────────────────────────────────────────────────────────
inline void setLED(bool r, bool g, bool b) {
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

// ── FFT de Goertzel simplificado (sin float[] dinámico) ───────────────────
// Para Nano 33 BLE (256 KB SRAM) es más seguro que arm_rfft.
// Calcula la energía en una banda de frecuencia [f_lo, f_hi) Hz.
static float bandEnergy(const float* sig, int n, float f_lo, float f_hi, float fs) {
    float energy = 0.0f;
    float df     = fs / (float)n;            // Hz por bin FFT
    int   lo     = (int)(f_lo / df + 0.5f);
    int   hi     = (int)(f_hi / df + 0.5f);
    if (hi > n / 2 + 1) hi = n / 2 + 1;
    if (lo >= hi) return 0.0f;

    // DFT directa para los bins en [lo, hi)
    for (int k = lo; k < hi; k++) {
        float re = 0.0f, im = 0.0f;
        float angle = 2.0f * 3.14159265f * (float)k / (float)n;
        for (int i = 0; i < n; i++) {
            re += sig[i] * cosf(angle * i);
            im -= sig[i] * sinf(angle * i);
        }
        energy += (re * re + im * im) / (float)(n * n);
    }
    return energy;
}

// ── Extractor espectral (replica del notebook) ────────────────────────────
// Resultado: features[SPECTRAL_DIM_PER_SENSOR]
// Layout por canal: [mean, std, rms, p2p, bin0, bin1, ..., bin9]
static void extractSpectralFeatures(float features[SPECTRAL_DIM_PER_SENSOR]) {
    const float FS  = (float)SAMPLE_HZ;
    const float BW  = FS / 2.0f / (float)N_FFT_BINS;   // ancho de banda por bin

    // Buffer temporal de señal con media removida (ventana alineada)
    static float sig[WINDOW_SIZE];

    int feat_idx = 0;
    for (int ch = 0; ch < N_AXES; ch++) {
        // Extraer canal de ring buffer (orden cronológico)
        float mu = 0.0f;
        for (int t = 0; t < WINDOW_SIZE; t++) {
            int row = (buf_head + t) % WINDOW_SIZE;
            sig[t]  = ring[row][ch];
            mu     += sig[t];
        }
        mu /= (float)WINDOW_SIZE;

        // Estadísticos temporales
        float variance = 0.0f, rms_sum = 0.0f;
        float vmin =  1e9f, vmax = -1e9f;
        for (int t = 0; t < WINDOW_SIZE; t++) {
            sig[t]    -= mu;   // DC-free para FFT
            variance  += sig[t] * sig[t];
            rms_sum   += (sig[t] + mu) * (sig[t] + mu);
            if (sig[t] + mu < vmin) vmin = sig[t] + mu;
            if (sig[t] + mu > vmax) vmax = sig[t] + mu;
        }
        float sigma = sqrtf(variance / (float)WINDOW_SIZE);
        float rms   = sqrtf(rms_sum  / (float)WINDOW_SIZE);
        float p2p   = vmax - vmin;

        features[feat_idx++] = mu;
        features[feat_idx++] = sigma;
        features[feat_idx++] = rms;
        features[feat_idx++] = p2p;

        // Energía espectral por banda
        for (int b = 0; b < N_FFT_BINS; b++) {
            float f_lo = (float)b       * BW;
            float f_hi = (float)(b + 1) * BW;
            features[feat_idx++] = bandEnergy(sig, WINDOW_SIZE, f_lo, f_hi, FS);
        }
    }
    // feat_idx debe ser == SPECTRAL_DIM_PER_SENSOR = 84
}

// ── Enviar features por BLE ───────────────────────────────────────────────
static float feat_buf[SPECTRAL_DIM_PER_SENSOR];

static void sendSpectralFeatures() {
    extractSpectralFeatures(feat_buf);
    spectralChar.writeValue((uint8_t*)feat_buf, SPECTRAL_PAYLOAD);
}

// ── setup() ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    setLED(false, false, false);

    if (!IMU.begin()) haltError("ERROR:IMU_INIT");
    Serial.print("IMU OK — ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");

    if (!BLE.begin()) haltError("ERROR:BLE_INIT");
    BLE.setLocalName(BLE_DEVICE_NAME);
    BLE.setAdvertisedService(imuService);
    imuService.addCharacteristic(spectralChar);
    BLE.addService(imuService);

    float zeros[SPECTRAL_DIM_PER_SENSOR] = {};
    spectralChar.writeValue((uint8_t*)zeros, SPECTRAL_PAYLOAD);

    BLE.advertise();
    Serial.print("STATUS:ADVERTISING as ");
    Serial.println(BLE_DEVICE_NAME);
    Serial.print("SPECTRAL_DIM_PER_SENSOR=");
    Serial.println(SPECTRAL_DIM_PER_SENSOR);
    setLED(false, false, true);
}

// ── loop() ───────────────────────────────────────────────────────────────
void loop() {
    // ── Parpadeo azul sin conexión ────────────────────────────────────────
    BLEDevice central = BLE.central();
    if (!central) {
        static unsigned long blink_t = 0;
        static bool led_on = false;
        if (millis() - blink_t > 500) {
            blink_t = millis();
            led_on = !led_on;
            setLED(false, false, led_on);
        }
        return;
    }

    Serial.print("CONNECTED:");
    Serial.println(central.address());
    setLED(false, true, false);   // verde = conectado

    while (central.connected()) {
        unsigned long now = millis();
        if (now - last_sample_ms < (unsigned long)SAMPLE_MS) continue;
        last_sample_ms = now;

        if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) continue;

        float ax, ay, az, gx, gy, gz;
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);

        // Almacenar en ring buffer
        ring[buf_head][0] = ax; ring[buf_head][1] = ay; ring[buf_head][2] = az;
        ring[buf_head][3] = gx; ring[buf_head][4] = gy; ring[buf_head][5] = gz;
        buf_head = (buf_head + 1) % WINDOW_SIZE;
        if (buf_count < WINDOW_SIZE) buf_count++;
        step_cnt++;

        // Enviar features cada STEP_SIZE muestras nuevas (buffer lleno)
        if (buf_count >= WINDOW_SIZE && step_cnt >= STEP_SIZE) {
            step_cnt = 0;
            sendSpectralFeatures();
        }
    }

    Serial.println("DISCONNECTED");
    BLE.advertise();
    setLED(false, false, true);
}
