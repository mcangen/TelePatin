/*
 * ============================================================================
 *  TelePatin - PRUEBA SIN RADIO (alimentación con batería)
 *  Placa: Heltec WiFi LoRa 32 (V3) o cualquier ESP32 / ESP32-S3
 * ============================================================================
 *
 *  Igual que Prueba_Enlace_TX pero SIN WiFi ni ESP-NOW: la radio nunca se
 *  enciende. Sirve para saber si la placa falla con la LiPo por los picos de
 *  corriente de la radio o por otra causa (polaridad, conector, arranque).
 *
 *  Funciones que SÍ tiene:
 *  - GY-91 (MPU + BMP280) con tara al arrancar. Botón PRG = tara y rearme.
 *  - GPS (lectura en segundo plano, como carga real de consumo).
 *  - Servo MG90S: autoprueba al arrancar y disparo al bajar 1.2 m desde el
 *    máximo tras haber subido más de 2 m (modo a mano, sin condición de
 *    aceleración, igual que Prueba_Enlace_TX).
 *
 *  Como no hay telemetría ni pantalla, TODO SE LEE EN EL LED BLANCO:
 *    Al arrancar:
 *      1 parpadeo largo     = arranque normal
 *      5 parpadeos rápidos  = se reinició por BAJO VOLTAJE
 *      3 parpadeos medios   = fallo del programa (panic / watchdog)
 *    Luego el servo hace su autoprueba: abre 1 s y vuelve a cerrar.
 *    En funcionamiento:
 *      destello corto cada 1 s   = ESPERA (programa vivo)
 *      parpadeo rápido           = SUBIENDO (pasó de 2 m)
 *      encendido fijo            = DISPARO (servo abierto)
 *      apagado ~2 s              = calibrando tara (no mover)
 *  Con USB conectado, el Monitor Serie (115200) muestra lo mismo en texto.
 *
 *  Conexiones (Heltec V3): GY-91 SDA 41 / SCL 42 / VIN 3V3,
 *  GPS TX->7 / RX->5, servo señal GPIO 4 (5 V del LM2596, GND común).
 *  Librerías: TinyGPSPlus, Adafruit BMP280 Library; U8g2 solo en la Heltec.
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>

#if defined(WIFI_LoRa_32_V3)
  #define ES_HELTEC 1
  #include <U8g2lib.h>
  #define PIN_OLED_SDA 17
  #define PIN_OLED_SCL 18
  #define PIN_OLED_RST 21
  #define PIN_VEXT     36
  #define PIN_LED      35
  #define PIN_GPS_RX   7    // Al TX del GPS
  #define PIN_GPS_TX   5    // Al RX del GPS
  #define PIN_SENSOR_SDA 41
  #define PIN_SENSOR_SCL 42
  #define PIN_SERVO      4
#else
  #define ES_HELTEC 0
  #define PIN_LED      2
  #define PIN_GPS_RX   16
  #define PIN_GPS_TX   17
  #define PIN_SENSOR_SDA 21
  #define PIN_SENSOR_SCL 22
  #define PIN_SERVO      13
#endif
#define PIN_BOTON_TARA 0

// ============================================================================
//  PARÁMETROS
// ============================================================================
const float    PRUEBA_SUBIDA_M       = 2.0;
const float    MARGEN_APOGEO_M       = 1.2;   // El mismo margen que el cohete
const uint8_t  MUESTRAS_CONFIRMACION = 3;
const int      SERVO_CERRADO = 0;
const int      SERVO_ABIERTO = 90;
const uint32_t PERIODO_SENSORES_MS = 20;
const uint32_t PERIODO_DIAG_MS     = 1000;
const uint32_t PERIODO_OLED_MS     = 500;
const float    ALFA_FILTRO_ALT     = 0.3;

// Servo con el PWM nativo (LEDC): 50 Hz, pulso de 500 a 2400 us para 0-180°
const uint32_t SERVO_FREQ_HZ      = 50;
const uint8_t  SERVO_RES_BITS     = 14;
const uint32_t SERVO_PULSO_MIN_US = 500;
const uint32_t SERVO_PULSO_MAX_US = 2400;

// ============================================================================
//  OBJETOS Y ESTADO
// ============================================================================
#if ES_HELTEC
U8G2_SSD1306_128X64_NONAME_F_2ND_HW_I2C display(U8G2_R0, PIN_OLED_RST);
#endif

TinyGPSPlus     gps;
HardwareSerial& gpsSerial = Serial2;
Adafruit_BMP280 bmp(&Wire);

bool    mpuOk = false, bmpOk = false;
uint8_t mpuAddr = 0x68;
float   aMagG = 0;
float   presionSuelo = 101325.0;
float   altFilt = 0, altMax = 0;

enum EstadoPrueba { P_ESPERA, P_SUBIENDO, P_DISPARO };
const char* NOMBRE_PRUEBA[] = { "ESPERA", "SUBIENDO", "DISPARO" };
EstadoPrueba estadoApo = P_ESPERA;
uint8_t  cuentaSubida = 0, cuentaApogeo = 0;

const uint32_t GPS_BAUDIOS[] = { 9600, 38400, 57600, 115200 };
uint8_t  gpsIndiceBaud = 0;
bool     gpsBaudOk = false;
uint32_t tCambioBaud = 0;

uint32_t tSensores = 0, tDiag = 0, tOled = 0;

// ============================================================================
//  SERVO
// ============================================================================
void servoMover(int grados) {
  grados = constrain(grados, 0, 180);
  uint32_t us   = map(grados, 0, 180, SERVO_PULSO_MIN_US, SERVO_PULSO_MAX_US);
  uint32_t duty = us * ((1UL << SERVO_RES_BITS) - 1) / (1000000UL / SERVO_FREQ_HZ);
  ledcWrite(PIN_SERVO, duty);
}

// ============================================================================
//  LED
// ============================================================================
void parpadear(int veces, int ms) {
  for (int i = 0; i < veces; i++) {
    digitalWrite(PIN_LED, HIGH); delay(ms);
    digitalWrite(PIN_LED, LOW);  delay(ms);
  }
}

void parpadearMotivoReinicio() {
  esp_reset_reason_t r = esp_reset_reason();
  Serial.printf("# Motivo del reinicio: %d\n", (int)r);
  if (r == ESP_RST_BROWNOUT) {
    Serial.println("# -> BAJO VOLTAJE");
    parpadear(5, 100);
  } else if (r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT) {
    Serial.println("# -> FALLO DEL PROGRAMA");
    parpadear(3, 250);
  } else {
    Serial.println("# -> arranque normal");
    parpadear(1, 600);
  }
  delay(500);
}

// Indica el estado sin bloquear el programa
void actualizarLed(uint32_t ahora) {
  bool encendido;
  switch (estadoApo) {
    case P_DISPARO:  encendido = true; break;                  // Fijo
    case P_SUBIENDO: encendido = (ahora / 100) % 2; break;     // Rápido
    default:         encendido = (ahora % 1000) < 60; break;   // Destello cada 1 s
  }
  digitalWrite(PIN_LED, encendido ? HIGH : LOW);
}

// ============================================================================
//  GY-91  (igual que Prueba_Enlace_TX)
// ============================================================================
bool mpuEscribir(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(valor);
  return Wire.endTransmission() == 0;
}

bool mpuIniciar() {
  const uint8_t direcciones[] = { 0x68, 0x69 };
  for (uint8_t dir : direcciones) {
    Wire.beginTransmission(dir);
    if (Wire.endTransmission() != 0) continue;
    mpuAddr = dir;
    mpuEscribir(0x6B, 0x80); delay(100);  // Reset
    mpuEscribir(0x6B, 0x01); delay(50);   // Despertar, reloj PLL
    mpuEscribir(0x1A, 0x03);              // DLPF giroscopio 41 Hz
    mpuEscribir(0x1B, 0x18);              // Giroscopio +-2000 dps
    mpuEscribir(0x1C, 0x18);              // Acelerómetro +-16 g
    return mpuEscribir(0x1D, 0x03);       // DLPF acelerómetro 41 Hz
  }
  return false;
}

bool mpuLeerAcel() {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, (uint8_t)6) != 6) return false;

  uint8_t b[6];
  for (uint8_t i = 0; i < 6; i++) b[i] = Wire.read();

  const float ESCALA = 9.80665f / 2048.0f;  // +-16 g -> m/s^2
  float ax = (int16_t)((b[0] << 8) | b[1]) * ESCALA;
  float ay = (int16_t)((b[2] << 8) | b[3]) * ESCALA;
  float az = (int16_t)((b[4] << 8) | b[5]) * ESCALA;
  aMagG = sqrtf(ax * ax + ay * ay + az * az) / 9.80665f;
  return true;
}

bool bmpIniciar() {
  const uint8_t direcciones[] = { 0x76, 0x77 };
  const uint8_t chipIds[]     = { 0x58, 0x60 };  // BMP280 / BME280
  for (uint8_t dir : direcciones) {
    for (uint8_t id : chipIds) {
      if (bmp.begin(dir, id)) {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X2,
                        Adafruit_BMP280::SAMPLING_X8,
                        Adafruit_BMP280::FILTER_X4,
                        Adafruit_BMP280::STANDBY_MS_1);
        return true;
      }
    }
  }
  return false;
}

float altitudDesdePresion(float pPa) {
  return 44330.0f * (1.0f - powf(pPa / presionSuelo, 0.1903f));
}

// Tara y rearme: altitud 0 aquí, prueba en ESPERA, servo trabado
void calibrarTara() {
  digitalWrite(PIN_LED, LOW);
  servoMover(SERVO_CERRADO);
  estadoApo = P_ESPERA;
  cuentaSubida = cuentaApogeo = 0;
  if (!bmpOk) return;

  Serial.println("# Calibrando tara... no mover");
  for (uint8_t i = 0; i < 10; i++) { bmp.readPressure(); delay(30); }
  double suma = 0;
  uint16_t n = 0;
  for (uint16_t i = 0; i < 60; i++) {
    float p = bmp.readPressure();
    if (!isnan(p) && p > 30000 && p < 110000) { suma += p; n++; }
    delay(25);
  }
  if (n > 0) presionSuelo = suma / n;
  altFilt = 0;
  altMax  = 0;
  Serial.printf("# Tara lista: P0 = %.2f Pa\n", presionSuelo);
}

// ============================================================================
//  PRUEBA DE APOGEO (a mano)
// ============================================================================
void actualizarPruebaApogeo() {
  if (!bmpOk) return;
  switch (estadoApo) {
    case P_ESPERA:
      cuentaSubida = (altFilt > PRUEBA_SUBIDA_M) ? cuentaSubida + 1 : 0;
      if (cuentaSubida >= MUESTRAS_CONFIRMACION) {
        estadoApo = P_SUBIENDO;
        cuentaApogeo = 0;
        Serial.println("# SUBIENDO");
      }
      break;
    case P_SUBIENDO:
      cuentaApogeo = (altFilt < altMax - MARGEN_APOGEO_M) ? cuentaApogeo + 1 : 0;
      if (cuentaApogeo >= MUESTRAS_CONFIRMACION) {
        servoMover(SERVO_ABIERTO);
        estadoApo = P_DISPARO;
        Serial.printf("# DISPARO alt=%.2f max=%.2f caida=%.2f m\n", altFilt, altMax, altMax - altFilt);
      }
      break;
    case P_DISPARO:
      break;
  }
}

void leerSensores() {
  if (mpuOk) mpuLeerAcel();
  if (bmpOk) {
    float p = bmp.readPressure();
    if (!isnan(p) && p > 30000 && p < 110000) {
      altFilt = ALFA_FILTRO_ALT * altitudDesdePresion(p) + (1.0f - ALFA_FILTRO_ALT) * altFilt;
      if (altFilt > altMax) altMax = altFilt;
    }
  }
  actualizarPruebaApogeo();
}

// ============================================================================
//  GPS (no bloqueante, con auto-detección de baudios)
// ============================================================================
void atenderGps(uint32_t ahora) {
  while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());
  if (!gpsBaudOk) {
    if (gps.passedChecksum() > 0) {
      gpsBaudOk = true;
      Serial.printf("# GPS conectado a %lu baudios\n", (unsigned long)GPS_BAUDIOS[gpsIndiceBaud]);
    } else if (ahora - tCambioBaud >= 3000) {
      tCambioBaud = ahora;
      gpsIndiceBaud = (gpsIndiceBaud + 1) % 4;
      gpsSerial.end();
      gpsSerial.begin(GPS_BAUDIOS[gpsIndiceBaud], SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    }
  }
}

// ============================================================================
//  OLED (solo Heltec; si la pantalla está dañada no afecta a nada)
// ============================================================================
void actualizarOled() {
#if ES_HELTEC
  char l[32];
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 10, "PRUEBA SIN RADIO");
  snprintf(l, sizeof(l), "MPU:%s BMP:%s %.2fg", mpuOk ? "OK" : "--", bmpOk ? "OK" : "--", aMagG);
  display.drawStr(0, 24, l);
  snprintf(l, sizeof(l), "Alt %.1f Max %.1f m", altFilt, altMax);
  display.drawStr(0, 38, l);
  display.drawStr(0, 52, NOMBRE_PRUEBA[estadoApo]);
  display.sendBuffer();
#endif
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);

  // Servo trabado desde el arranque
  ledcAttach(PIN_SERVO, SERVO_FREQ_HZ, SERVO_RES_BITS);
  servoMover(SERVO_CERRADO);

  parpadearMotivoReinicio();
  Serial.println("# TELEPATIN - PRUEBA SIN RADIO (WiFi apagado)");

#if ES_HELTEC
  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);
  delay(100);
  Wire1.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  display.begin();
#endif
  pinMode(PIN_BOTON_TARA, INPUT_PULLUP);

  // Autoprueba del servo: abre 1 s y cierra (se ve que la placa vive y el
  // servo recibe señal y alimentación)
  Serial.println("# Autoprueba del servo");
  servoMover(SERVO_ABIERTO);
  delay(1000);
  servoMover(SERVO_CERRADO);
  delay(500);

  Wire.begin(PIN_SENSOR_SDA, PIN_SENSOR_SCL, 400000);
  delay(250);
  mpuOk = mpuIniciar();
  bmpOk = bmpIniciar();
  Serial.printf("# GY-91 -> MPU:%s BMP:%s\n", mpuOk ? "OK" : "NO", bmpOk ? "OK" : "NO");
  if (!bmpOk) parpadear(10, 50);  // Aviso: sin barómetro no hay prueba de apogeo
  calibrarTara();

  gpsSerial.setRxBufferSize(1024);
  gpsSerial.begin(GPS_BAUDIOS[gpsIndiceBaud], SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  tCambioBaud = millis();
}

// ============================================================================
//  LOOP
// ============================================================================
void loop() {
  uint32_t ahora = millis();

  atenderGps(ahora);

  if (ahora - tSensores >= PERIODO_SENSORES_MS) {
    tSensores = ahora;
    leerSensores();
  }

  // Botón PRG: tara y rearme (vuelve a ESPERA y traba el servo)
  if (digitalRead(PIN_BOTON_TARA) == LOW) {
    delay(50);
    if (digitalRead(PIN_BOTON_TARA) == LOW) {
      calibrarTara();
      while (digitalRead(PIN_BOTON_TARA) == LOW) delay(10);
    }
  }

  actualizarLed(ahora);

  if (ahora - tDiag >= PERIODO_DIAG_MS) {
    tDiag = ahora;
    Serial.printf("# %s alt=%.2f max=%.2f |a|=%.2fg gps=%s sats=%d\n",
                  NOMBRE_PRUEBA[estadoApo], altFilt, altMax, aMagG,
                  gpsBaudOk ? "OK" : "--",
                  gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  }

  if (ES_HELTEC && ahora - tOled >= PERIODO_OLED_MS) {
    tOled = ahora;
    actualizarOled();
  }
}
