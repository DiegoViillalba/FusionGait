#include "config.h"

// Seleccionar librería IMU según la versión del hardware
#if IMU_REV2
  #include <Arduino_BMI270_BMM150.h>   // Nano 33 BLE Sense Rev2
#else
  #include <Arduino_LSM9DS1.h>          // Nano 33 BLE Sense original
#endif

// ─── Estado ──────────────────────────────────────────────────────────────────
enum State { IDLE, CAPTURING };
static State      state          = IDLE;
static String     trial_id       = "t000";
static String     subject_id     = "s000";
static uint32_t   last_sample_us = 0;
static uint32_t   sample_count   = 0;
static String     cmd_buf        = "";   // acumulador de comando en curso

// ─── Prototipos ───────────────────────────────────────────────────────────────
static void process_command(const String& cmd);
static void send_sample();
static void set_led(bool r, bool g, bool b);

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(BAUD_RATE);

  // Los LEDs RGB son activos-LOW: iniciar apagados
  pinMode(LEDR, OUTPUT); digitalWrite(LEDR, HIGH);
  pinMode(LEDG, OUTPUT); digitalWrite(LEDG, HIGH);
  pinMode(LEDB, OUTPUT); digitalWrite(LEDB, HIGH);

  // No esperamos Serial: en el Nano 33 BLE Sense sobre mbed el puerto USB CDC
  // siempre está disponible. Bloquear aquí haría que el sketch nunca arrancara
  // si no hay un terminal abierto, y retrasaría el READY en el script Python.

  Serial.setTimeout(100);   // readStringUntil no bloqueará más de 100 ms

  if (!IMU.begin()) {
    set_led(true, false, false);   // rojo
    // Bucle parpadeante: el LED rojo indica error de IMU
    while (true) {
      if (Serial.available()) {
        // Responder STATUS aunque la IMU falle, para que Python pueda diagnosticar
        String s = Serial.readStringUntil('\n');
        if (s.indexOf("STATUS") >= 0)
          Serial.println("ERROR:IMU_NOT_FOUND");
      }
      delay(300);
    }
  }

  set_led(false, false, true);    // azul = listo, esperando START
  Serial.println("READY");        // útil si Python abre el puerto ANTES del boot
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  // 1. Leer Serial byte a byte (no-bloqueante)
  //    Acumula en cmd_buf hasta recibir '\n', luego procesa.
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      cmd_buf.trim();
      if (cmd_buf.length() > 0) process_command(cmd_buf);
      cmd_buf = "";
    } else if (c != '\r') {
      cmd_buf += c;
      if (cmd_buf.length() > 64) cmd_buf = "";  // anti-desbordamiento
    }
  }

  // 2. Captura periódica a SAMPLE_HZ Hz
  if (state == CAPTURING) {
    uint32_t now = micros();
    // Resta sin signo: funciona correctamente aunque micros() desborde
    if ((uint32_t)(now - last_sample_us) >= SAMPLE_PERIOD_US) {
      // Acumulativo para no derivar con el tiempo
      last_sample_us += SAMPLE_PERIOD_US;
      send_sample();
    }
  }
}

// ─── Procesar comando desde el PC ─────────────────────────────────────────────
static void process_command(const String& cmd) {
  if (cmd == "START") {
    state          = CAPTURING;
    last_sample_us = micros();
    sample_count   = 0;
    set_led(false, true, false);   // verde = capturando
    Serial.println("ACK:START");

  } else if (cmd == "STOP") {
    state = IDLE;
    set_led(false, false, true);   // azul = idle
    Serial.print("ACK:STOP:");
    Serial.println(sample_count);

  } else if (cmd.startsWith("SET_TRIAL:")) {
    trial_id = cmd.substring(10);
    trial_id.trim();
    Serial.print("ACK:SET_TRIAL:"); Serial.println(trial_id);

  } else if (cmd.startsWith("SET_SUBJECT:")) {
    subject_id = cmd.substring(12);
    subject_id.trim();
    Serial.print("ACK:SET_SUBJECT:"); Serial.println(subject_id);

  } else if (cmd == "SYNC") {
    Serial.print("SYNC_ACK:"); Serial.println(millis());

  } else if (cmd == "STATUS") {
    Serial.print("STATUS:sensor_id=");  Serial.print(SENSOR_ID);
    Serial.print(",placement=");        Serial.print(PLACEMENT);
    Serial.print(",trial=");            Serial.print(trial_id);
    Serial.print(",subject=");          Serial.print(subject_id);
    Serial.print(",capturing=");        Serial.print(state == CAPTURING ? 1 : 0);
    Serial.print(",samples=");          Serial.println(sample_count);
  }
}

// ─── Leer IMU y enviar línea CSV ──────────────────────────────────────────────
static void send_sample() {
  float ax, ay, az, gx, gy, gz;

  // Si el dato no está listo todavía, saltamos este ciclo (no bloqueamos)
  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return;

  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);

  // Formato: timestamp_ms,sensor_id,placement,ax,ay,az,gx,gy,gz
  Serial.print(millis());         Serial.print(',');
  Serial.print(SENSOR_ID);        Serial.print(',');
  Serial.print(PLACEMENT);        Serial.print(',');
  Serial.print(ax, 4);            Serial.print(',');
  Serial.print(ay, 4);            Serial.print(',');
  Serial.print(az, 4);            Serial.print(',');
  Serial.print(gx, 4);            Serial.print(',');
  Serial.print(gy, 4);            Serial.print(',');
  Serial.println(gz, 4);          // println añade \n

  sample_count++;
}

// ─── Control de LEDs RGB (activos-LOW) ────────────────────────────────────────
static void set_led(bool r, bool g, bool b) {
  digitalWrite(LEDR, r ? LOW : HIGH);
  digitalWrite(LEDG, g ? LOW : HIGH);
  digitalWrite(LEDB, b ? LOW : HIGH);
}
