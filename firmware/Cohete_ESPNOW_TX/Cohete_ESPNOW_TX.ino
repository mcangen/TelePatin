/*
 * ============================================================================
 *  TelePatin - COHETE (TRANSMISOR ESP-NOW)
 *  Placa: Heltec WiFi LoRa 32 (V3), ESP32 DevKit (WROOM) o ESP32-S3 DevKitC
 *  Arduino IDE: "Heltec WiFi LoRa 32(V3)", "ESP32 Dev Module" o
 *               "ESP32S3 Dev Module" (core esp32 3.x)
 * ============================================================================
 *
 *  Versión de reemplazo de Cohete_TX: misma lógica de vuelo y mismo JSON, pero
 *  el enlace es ESP-NOW (radio WiFi directa, sin router) en modo Long Range.
 *  La estación terrena debe ser EstacionTerrena_ESPNOW_RX.
 *
 *  Librerías (Gestor de librerías del Arduino IDE):
 *    - Adafruit BMP280 Library (+ Adafruit Unified Sensor, Adafruit BusIO)
 *    - DHT sensor library     (Adafruit)
 *    (El servo usa el PWM nativo del ESP32 (LEDC), sin ESP32Servo: en el
 *     ESP32-S3 esa librería usa el driver MCPWM antiguo y deja sin
 *     transmisión al ESP-NOW.)
 *    - TinyGPSPlus            (Mikal Hart)
 *    - U8g2                   (olikraus)  -> solo en la Heltec (pantalla)
 *
 *  ---------------------------------------------------------------------------
 *  CONEXIONES (los pines se eligen solos según la placa)
 *  ---------------------------------------------------------------------------
 *   Señal                    | Heltec V3  | ESP32 DevKit | ESP32-S3 DevKitC
 *   -------------------------+------------+--------------+------------------
 *   GY-91 SDA                |     41     |      21      |        8
 *   GY-91 SCL                |     42     |      22      |        9
 *   DHT11 DATA (pull-up 10k) |      2     |       4      |        4   (solo con USAR_DHT = 1)
 *   Servo señal              |      4     |      13      |        5
 *   GPS: TX del GPS      ->  |   7 (RX)   |  16 (RX)     |   6 (RX)
 *   GPS: RX del GPS      <-  |   5 (TX)   |  17 (TX)     |   7 (TX)
 *   Botón de tara            |  0 (PRG)   |  0 (BOOT)    |   0 (BOOT)
 *   -------------------------+------------+--------------+------------------
 *  ALIMENTACIÓN:
 *   - Heltec: LiPo 1S al conector JST de la placa. El servo, desde 5V aparte
 *     (elevador o BEC), nunca desde 3V3.
 *   - DevKit: LiPo 1S + elevador a 5V -> pin 5V/VIN y al servo.
 *   Todas las GND unidas.
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <TinyGPS++.h>

// 1 = hay un DHT11 conectado (temperatura y humedad). 0 = no se usa: la
// temperatura sale del BMP280 y la humedad va como null.
#define USAR_DHT 0

// WIFI_LoRa_32_V3 lo define el core al elegir la placa Heltec V3
#if defined(WIFI_LoRa_32_V3)
  #define ES_HELTEC 1
  #include <U8g2lib.h>
#else
  #define ES_HELTEC 0
#endif

// ============================================================================
//  PINES (automáticos según la placa)
// ============================================================================
#if ES_HELTEC
  #define PIN_SENSOR_SDA 41
  #define PIN_SENSOR_SCL 42
  #define PIN_DHT        2    // Solo si USAR_DHT = 1
  #define PIN_SERVO      4
  #define PIN_GPS_RX     7    // Al TX del GPS
  #define PIN_GPS_TX     5    // Al RX del GPS
  // Internos de la Heltec V3
  #define PIN_OLED_SDA   17
  #define PIN_OLED_SCL   18
  #define PIN_OLED_RST   21
  #define PIN_VEXT       36   // LOW = enciende la OLED
  #define PIN_LED        35
#elif CONFIG_IDF_TARGET_ESP32S3
  #define PIN_SENSOR_SDA 8
  #define PIN_SENSOR_SCL 9
  #define PIN_DHT        4
  #define PIN_SERVO      5
  #define PIN_GPS_RX     6
  #define PIN_GPS_TX     7
#else
  #define PIN_SENSOR_SDA 21
  #define PIN_SENSOR_SCL 22
  #define PIN_DHT        4
  #define PIN_SERVO      13
  #define PIN_GPS_RX     16
  #define PIN_GPS_TX     17
#endif
#define PIN_BOTON_TARA 0

// ============================================================================
//  CONFIGURACIÓN ESP-NOW  (DEBE SER IDÉNTICA EN LA ESTACIÓN TERRENA)
// ============================================================================
#define ESPNOW_CANAL      1
#define USAR_LONG_RANGE   0   // 1 = modo LR (más alcance). Activarlo SOLO cuando el enlace ya funcione, y en ambos lados.
uint8_t DIRECCION_BROADCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// Potencia de transmisión (2 a 21 dBm). A 21 dBm cada envío pide picos de
// ~350-400 mA: con una LiPo 1S por el JST el voltaje se hunde y la placa se
// reinicia o la radio falla. 15 dBm alcanza ~100-200 m con vista despejada.
const int POTENCIA_TX_DBM = 15;

// ============================================================================
//  PARÁMETROS DE VUELO  (los mismos de Cohete_TX)
// ============================================================================
const float    MARGEN_APOGEO_M           = 1.2;   // Caída bajo la altitud máxima para declarar apogeo
const float    MARGEN_APOGEO_SOLO_BARO_M = 4.0;
const float    UMBRAL_CAIDA_G            = 0.8;
const float    UMBRAL_LANZAMIENTO_G      = 2.5;
const float    UMBRAL_LANZAMIENTO_ALT_M  = 3.0;
const uint8_t  MUESTRAS_CONFIRMACION     = 3;
const uint32_t BLOQUEO_POST_LANZ_MS      = 500;
const uint32_t VENTANA_FALSO_LANZ_MS     = 1500;
const float    ALT_MIN_VUELO_REAL_M      = 1.5;
const uint32_t TIMEOUT_APOGEO_MS         = 6000;
const float    ALT_ATERRIZAJE_M          = 3.0;
const uint32_t TIEMPO_QUIETO_ATERRIZ_MS  = 3000;

const int SERVO_CERRADO = 0;
const int SERVO_ABIERTO = 90;

// ============================================================================
//  TEMPORIZACIÓN
// ============================================================================
const uint32_t PERIODO_SENSORES_MS = 20;
const uint32_t PERIODO_TX_MS       = 150;
const uint32_t PERIODO_DHT_MS      = 2000;
const uint32_t GPS_MAX_EDAD_MS     = 2000;  // Posición más vieja que esto = sin fix
const uint32_t GPS_CAMBIO_BAUD_MS  = 3000;
const float    ALFA_FILTRO_ALT     = 0.3;
const uint32_t PERIODO_OLED_MS     = 500;

// ============================================================================
//  OBJETOS
// ============================================================================
#if ES_HELTEC
// OLED por I2C hardware en Wire1; Wire queda para los sensores.
U8G2_SSD1306_128X64_NONAME_F_2ND_HW_I2C display(U8G2_R0, PIN_OLED_RST);
#endif
Adafruit_BMP280 bmp(&Wire);
DHT             dht(PIN_DHT, DHT11);

// Servo con el PWM nativo (LEDC): 50 Hz, pulso de 500 a 2400 us para 0-180°
const uint32_t SERVO_FREQ_HZ      = 50;
const uint8_t  SERVO_RES_BITS     = 14;
const uint32_t SERVO_PULSO_MIN_US = 500;
const uint32_t SERVO_PULSO_MAX_US = 2400;

bool servoIniciar() {
  return ledcAttach(PIN_SERVO, SERVO_FREQ_HZ, SERVO_RES_BITS);
}

void servoMover(int grados) {
  grados = constrain(grados, 0, 180);
  uint32_t us   = map(grados, 0, 180, SERVO_PULSO_MIN_US, SERVO_PULSO_MAX_US);
  uint32_t duty = us * ((1UL << SERVO_RES_BITS) - 1) / (1000000UL / SERVO_FREQ_HZ);
  ledcWrite(PIN_SERVO, duty);
}
TinyGPSPlus     gps;
HardwareSerial& gpsSerial = Serial2;

// ============================================================================
//  ESTADO GLOBAL
// ============================================================================
enum EstadoVuelo { ESPERA, ASCENSO, DESCENSO, ATERRIZADO };
const char* NOMBRE_ESTADO[] = { "ESPERA", "ASCENSO", "DESCENSO", "ATERRIZADO" };

EstadoVuelo estado = ESPERA;

bool radioOk = false, bmpOk = false, mpuOk = false;
uint8_t mpuAddr = 0x68;

float presionSuelo = 101325.0;
float presionPa    = NAN;
float altFilt      = 0.0;
float altMax       = 0.0;

float ax = 0, ay = 0, az = 0;
float aMagG = 1.0;

float temperatura = NAN, humedad = NAN;

bool        paracaidas = false;
const char* motivoDespliegue = "";
uint32_t    tLanzamiento = 0, tQuieto = 0;
uint8_t     cuentaLanz = 0, cuentaApogeo = 0, cuentaBaro = 0;

// GPS: auto-detección de baudios (por si el módulo fue reconfigurado)
const uint32_t GPS_BAUDIOS[] = { 9600, 115200, 38400, 57600 };
uint8_t  gpsIndiceBaud = 0;
bool     gpsBaudOk = false;
uint32_t tCambioBaud = 0;

uint32_t paquetesEnviados = 0;
uint32_t tSensores = 0, tTx = 0, tDht = 0, tOled = 0;

// Tara remota: botón "Fijar cero" de la página -> receptor -> ESP-NOW -> aquí.
const uint32_t PERIODO_ESTADO_TARA_MS = 1000;
const uint32_t QUIETO_MIN_MS = 1000;   // Tiempo quieto exigido antes de la tara
const float    QUIETO_TOL_G  = 0.08;   // |a| debe estar en 1 g +- esto
volatile bool     cmdTaraPendiente = false;
volatile uint32_t cmdTaraId = 0;
uint32_t ultimoIdTara = 0, tUltimaTara = 0, tUltimoMovimiento = 0, tEstadoTara = 0;
uint8_t  resultadoTara = 0;  // 0 ninguna, 1 OK, 2 en movimiento, 3 no está en ESPERA, 4 sin barómetro

// Prueba de servo desde la página ("Probar servo"): abre, espera y vuelve a cerrar.
const uint32_t SERVO_PRUEBA_MS = 2000;
// Despliegue manual de emergencia ("Desplegar paracaídas" en la página):
// se acepta en CUALQUIER estado, incluso en pleno vuelo, y deja el servo abierto.
volatile bool     cmdDesplegarPendiente = false;
volatile uint32_t cmdDesplegarId = 0;
bool              huboLanzamiento = false;   // Para no informar tiempos "tras el lanzamiento" sin lanzamiento
volatile bool     cmdServoPendiente = false;
volatile uint32_t cmdServoId = 0;
uint32_t ultimoIdServo = 0, tServoPrueba = 0, tEstadoServo = 0;
bool     servoEnPrueba = false;
uint8_t  resultadoServo = 0;  // 0 ninguna, 1 prueba hecha, 3 no está en ESPERA, 5 prueba en curso

// Registro del despliegue para el informe en la página
float    altDespliegue = NAN, altMaxDespliegue = NAN;  // Congelados en el despliegue
uint32_t msDespliegue = 0;    // ms desde el lanzamiento

// ============================================================================
//  OLED (solo Heltec; en el DevKit estas funciones no hacen nada)
// ============================================================================
bool gpsConFix();

void mostrarMensaje(const char* l1, const char* l2 = "", const char* l3 = "") {
#if ES_HELTEC
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, l1);
  display.drawStr(0, 28, l2);
  display.drawStr(0, 44, l3);
  display.sendBuffer();
#endif
}

void actualizarOled() {
#if ES_HELTEC
  char l[32];
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);

  snprintf(l, sizeof(l), "COHETE  %s", NOMBRE_ESTADO[estado]);
  display.drawStr(0, 10, l);
  snprintf(l, sizeof(l), "Alt %.1f  Max %.1f m", altFilt, altMax);
  display.drawStr(0, 22, l);
  snprintf(l, sizeof(l), "|a| %.2f g", aMagG);
  display.drawStr(0, 34, l);
  snprintf(l, sizeof(l), "Paraca: %s %s", paracaidas ? "FUERA" : "TRABADO", motivoDespliegue);
  display.drawStr(0, 46, l);
  if (!gpsBaudOk) {
    snprintf(l, sizeof(l), "GPS: buscando... TX:%lu", (unsigned long)paquetesEnviados);
  } else {
    snprintf(l, sizeof(l), "GPS:%s Sat:%d TX:%lu", gpsConFix() ? "FIX" : "--",
             gps.satellites.isValid() ? (int)gps.satellites.value() : 0,
             (unsigned long)paquetesEnviados);
  }
  display.drawStr(0, 58, l);

  display.sendBuffer();
#endif
}

// ============================================================================
//  MPU9250 / MPU6500 (registros directos)
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

    Wire.beginTransmission(mpuAddr);
    Wire.write(0x75);
    Wire.endTransmission(false);
    Wire.requestFrom(mpuAddr, (uint8_t)1);
    uint8_t who = Wire.available() ? Wire.read() : 0;
    Serial.printf("# MPU en 0x%02X, WHO_AM_I=0x%02X\n", mpuAddr, who);

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
  ax = (int16_t)((b[0] << 8) | b[1]) * ESCALA;
  ay = (int16_t)((b[2] << 8) | b[3]) * ESCALA;
  az = (int16_t)((b[4] << 8) | b[5]) * ESCALA;
  aMagG = sqrtf(ax * ax + ay * ay + az * az) / 9.80665f;
  return true;
}

// ============================================================================
//  BMP280 + TARA
// ============================================================================
bool bmpIniciar() {
  const uint8_t direcciones[] = { 0x76, 0x77 };
  const uint8_t chipIds[]     = { 0x58, 0x60 };
  for (uint8_t dir : direcciones) {
    for (uint8_t id : chipIds) {
      if (bmp.begin(dir, id)) {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X2,
                        Adafruit_BMP280::SAMPLING_X8,
                        Adafruit_BMP280::FILTER_X4,
                        Adafruit_BMP280::STANDBY_MS_1);
        Serial.printf("# BMP en 0x%02X (chip 0x%02X)\n", dir, id);
        return true;
      }
    }
  }
  return false;
}

float altitudDesdePresion(float pPa) {
  return 44330.0f * (1.0f - powf(pPa / presionSuelo, 0.1903f));
}

void calibrarTara() {
  if (!bmpOk) return;
  Serial.println("# Calibrando tara barometrica... no mover el cohete");
  mostrarMensaje("CALIBRANDO TARA", "No mover el cohete...");

  for (uint8_t intento = 0; intento < 3; intento++) {
    for (uint8_t i = 0; i < 10; i++) { bmp.readPressure(); delay(30); }

    double suma = 0;
    float pMin = 1e9, pMax = 0;
    uint16_t n = 0;
    for (uint16_t i = 0; i < 60; i++) {
      float p = bmp.readPressure();
      if (!isnan(p) && p > 30000 && p < 110000) {
        suma += p; n++;
        pMin = min(pMin, p);
        pMax = max(pMax, p);
      }
      delay(25);
    }
    if (n < 40) continue;

    presionSuelo = suma / n;
    if (pMax - pMin < 12.0f || intento == 2) break;
    Serial.printf("# Tara inestable (%.1f Pa), repitiendo\n", pMax - pMin);
  }

  altFilt = 0.0;
  altMax  = 0.0;
  tUltimaTara = millis();
  Serial.printf("# Tara lista: P0 = %.2f Pa\n", presionSuelo);
  mostrarMensaje("TARA LISTA", "Altitud 0 fijada");
}

// ============================================================================
//  PARACAÍDAS Y MÁQUINA DE ESTADOS  (idéntica a Cohete_TX)
// ============================================================================
void desplegarParacaidas(const char* motivo) {
  servoMover(SERVO_ABIERTO);
  paracaidas = true;
  servoEnPrueba = false;  // Una prueba en curso nunca debe volver a cerrar el servo
  motivoDespliegue = motivo;
  altDespliegue = altFilt;
  altMaxDespliegue = altMax;
  msDespliegue = millis() - tLanzamiento;
  estado = DESCENSO;
  tQuieto = 0;
  Serial.printf("# DESPLIEGUE (%s) alt=%.2f max=%.2f t=%lu ms\n",
                motivo, altFilt, altMax, millis() - tLanzamiento);
}

void actualizarMaquinaEstados(uint32_t ahora) {
  switch (estado) {
    case ESPERA: {
      bool porAcel = mpuOk && aMagG > UMBRAL_LANZAMIENTO_G;
      bool porAlt  = bmpOk && altFilt > UMBRAL_LANZAMIENTO_ALT_M;
      cuentaLanz = (porAcel || porAlt) ? cuentaLanz + 1 : 0;
      if (cuentaLanz >= MUESTRAS_CONFIRMACION) {
        estado = ASCENSO;
        tLanzamiento = ahora;
        huboLanzamiento = true;
        altMax = max(0.0f, altFilt);
        cuentaApogeo = cuentaBaro = 0;
        Serial.println("# LANZAMIENTO detectado");
      }
      break;
    }

    case ASCENSO: {
      if (altFilt > altMax) altMax = altFilt;
      uint32_t dt = ahora - tLanzamiento;
      if (dt < BLOQUEO_POST_LANZ_MS) break;

      if (bmpOk && dt > VENTANA_FALSO_LANZ_MS && altMax < ALT_MIN_VUELO_REAL_M) {
        estado = ESPERA;
        cuentaLanz = 0;
        altMax = 0;
        Serial.println("# Falso lanzamiento, volviendo a ESPERA");
        break;
      }

      bool bajoMargen = bmpOk && altFilt < altMax - MARGEN_APOGEO_M;
      bool acelCaida  = !mpuOk || aMagG < UMBRAL_CAIDA_G;
      cuentaApogeo = (bajoMargen && acelCaida) ? cuentaApogeo + 1 : 0;

      bool caidaClara = bmpOk && altFilt < altMax - MARGEN_APOGEO_SOLO_BARO_M;
      cuentaBaro = caidaClara ? cuentaBaro + 1 : 0;

      if (cuentaApogeo >= MUESTRAS_CONFIRMACION)    desplegarParacaidas("APOGEO");
      else if (cuentaBaro >= MUESTRAS_CONFIRMACION) desplegarParacaidas("BARO");
      else if (dt > TIMEOUT_APOGEO_MS)              desplegarParacaidas("TIMEOUT");
      break;
    }

    case DESCENSO: {
      bool quieto = (!bmpOk || altFilt < ALT_ATERRIZAJE_M) &&
                    (!mpuOk || fabsf(aMagG - 1.0f) < 0.15f);
      if (!quieto) {
        tQuieto = 0;
      } else if (tQuieto == 0) {
        tQuieto = ahora;
      } else if (ahora - tQuieto > TIEMPO_QUIETO_ATERRIZ_MS) {
        estado = ATERRIZADO;
        Serial.println("# ATERRIZADO");
      }
      break;
    }

    case ATERRIZADO:
      break;  // Se sigue transmitiendo como baliza (con GPS)
  }
}

// ============================================================================
//  LECTURA DE SENSORES
// ============================================================================
void leerSensores(uint32_t ahora) {
  if (mpuOk && mpuLeerAcel() && fabsf(aMagG - 1.0f) > QUIETO_TOL_G) tUltimoMovimiento = ahora;

  if (bmpOk) {
    float p = bmp.readPressure();
    if (!isnan(p) && p > 30000 && p < 110000) {
      presionPa = p;
      float altCruda = altitudDesdePresion(p);
      altFilt = ALFA_FILTRO_ALT * altCruda + (1.0f - ALFA_FILTRO_ALT) * altFilt;
    }
  }

  // El DHT11 bloquea ~25 ms: no se lee durante el ascenso.
  if (estado != ASCENSO && ahora - tDht >= PERIODO_DHT_MS) {
    tDht = ahora;
#if USAR_DHT
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t)) temperatura = t;
    if (!isnan(h)) humedad = h;
#else
    // Sin DHT11: temperatura del BMP280 (mide su propio chip, referencia aproximada)
    if (bmpOk) {
      float t = bmp.readTemperature();
      if (!isnan(t)) temperatura = t;
    }
#endif
  }

  actualizarMaquinaEstados(ahora);
}

// ============================================================================
//  GPS NEO-M8N (no bloqueante)
// ============================================================================
void atenderGps(uint32_t ahora) {
  while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());

  // Solo una frase NMEA con checksum correcto confirma la velocidad. Contar
  // caracteres no sirve: a una velocidad equivocada también llega basura.
  if (!gpsBaudOk) {
    if (gps.passedChecksum() > 0) {
      gpsBaudOk = true;
      Serial.printf("# GPS conectado a %lu baudios\n", (unsigned long)GPS_BAUDIOS[gpsIndiceBaud]);
    } else if (ahora - tCambioBaud >= GPS_CAMBIO_BAUD_MS) {
      tCambioBaud = ahora;
      gpsIndiceBaud = (gpsIndiceBaud + 1) % 4;
      gpsSerial.end();
      gpsSerial.begin(GPS_BAUDIOS[gpsIndiceBaud], SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    }
  }
}

bool gpsConFix() {
  return gps.location.isValid() && gps.location.age() < GPS_MAX_EDAD_MS;
}

// ============================================================================
//  TELEMETRÍA (JSON por ESP-NOW)
// ============================================================================
void numeroONull(char* dst, size_t tam, float v, uint8_t decimales) {
  if (isnan(v)) snprintf(dst, tam, "null");
  else          snprintf(dst, tam, "%.*f", decimales, v);
}

void enviarTelemetria(uint32_t ahora) {
  char sTemp[12], sHum[12], sPres[16], sLat[16], sLon[16];
  numeroONull(sTemp, sizeof(sTemp), temperatura, 1);
  numeroONull(sHum,  sizeof(sHum),  humedad, 1);
  numeroONull(sPres, sizeof(sPres), isnan(presionPa) ? NAN : presionPa / 100.0f, 2);

  bool fix = gpsConFix();
  numeroONull(sLat, sizeof(sLat), fix ? gps.location.lat() : NAN, 6);
  numeroONull(sLon, sizeof(sLon), fix ? gps.location.lng() : NAN, 6);
  int sats = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;

  // Mismas claves que Cohete_TX + lat/lon/sats (el dashboard ya usa lat/lon)
  static char json[ESP_NOW_MAX_DATA_LEN];
  int len = snprintf(json, sizeof(json),
      "{\"n\":%lu,\"t\":%.2f,\"alt\":%.2f,\"altMax\":%.2f,\"temp\":%s,\"hum\":%s,"
      "\"pres\":%s,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"lat\":%s,\"lon\":%s,"
      "\"sats\":%d,\"para\":%d,\"est\":\"%s\"}",
      (unsigned long)paquetesEnviados, ahora / 1000.0f, altFilt, altMax, sTemp, sHum,
      sPres, ax, ay, az, sLat, sLon, sats, paracaidas ? 1 : 0, NOMBRE_ESTADO[estado]);
  if (len <= 0 || len >= (int)sizeof(json)) return;

  Serial.println(json);  // Copia local para pruebas en banco

  if (!radioOk) return;
  // esp_now_send no bloquea: encola la trama y vuelve en microsegundos.
  if (esp_now_send(DIRECCION_BROADCAST, (const uint8_t*)json, len) == ESP_OK) {
    paquetesEnviados++;
#ifdef PIN_LED
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
#endif
  }
}

// ============================================================================
//  ESP-NOW
// ============================================================================
// Órdenes desde la estación (tarea WiFi: solo copiar y levantar la bandera)
// Formato: "CMD:TARA:<id>", "CMD:SERVO:<id>" o "CMD:DESPLEGAR:<id>"
void alRecibirOrden(const esp_now_recv_info_t* info, const uint8_t* datos, int len) {
  if (len < 6 || len > 24 || memcmp(datos, "CMD:", 4) != 0) return;
  char txt[25];
  memcpy(txt, datos, len);
  txt[len] = '\0';
  if (strncmp(txt + 4, "TARA:", 5) == 0) {
    cmdTaraId = strtoul(txt + 9, nullptr, 10);
    cmdTaraPendiente = true;
  } else if (strncmp(txt + 4, "SERVO:", 6) == 0) {
    cmdServoId = strtoul(txt + 10, nullptr, 10);
    cmdServoPendiente = true;
  } else if (strncmp(txt + 4, "DESPLEGAR:", 10) == 0) {
    cmdDesplegarId = strtoul(txt + 14, nullptr, 10);
    cmdDesplegarPendiente = true;
  }
}

// Arranque según el ejemplo oficial del core 3.x: esperar a que el WiFi
// arranque ANTES de fijar el canal; si no, el canal puede no aplicarse.
bool espNowIniciar() {
  WiFi.mode(WIFI_STA);
  uint32_t t0 = millis();
  while (!WiFi.STA.started() && millis() - t0 < 3000) delay(10);
#if USAR_LONG_RANGE
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
#endif
  WiFi.setChannel(ESPNOW_CANAL);
  esp_wifi_set_max_tx_power(constrain(POTENCIA_TX_DBM, 2, 21) * 4);  // En unidades de 0.25 dBm

  uint8_t canal; wifi_second_chan_t sec;
  esp_wifi_get_channel(&canal, &sec);
  Serial.printf("# WiFi canal real %u (esperado %u), LR=%d\n", canal, ESPNOW_CANAL, USAR_LONG_RANGE);

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(alRecibirOrden);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, DIRECCION_BROADCAST, 6);
  peer.channel = 0;  // 0 = el canal actual del WiFi
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

// ============================================================================
//  TARA REMOTA ("Fijar cero" desde la página)
// ============================================================================
// Mensaje aparte del JSON de telemetría (que ya ronda los 250 bytes). La
// página lo reconoce por "tipo":"tara" y no lo cuenta como paquete de datos.
void enviarEstadoTara() {
  char sP0[16] = "null", sHace[16] = "null";
  if (bmpOk) snprintf(sP0, sizeof(sP0), "%.2f", presionSuelo / 100.0f);
  if (tUltimaTara) snprintf(sHace, sizeof(sHace), "%lu", (unsigned long)((millis() - tUltimaTara) / 1000));

  char msg[96];
  int len = snprintf(msg, sizeof(msg), "{\"tipo\":\"tara\",\"id\":%lu,\"res\":%u,\"p0\":%s,\"hace\":%s}",
                     (unsigned long)ultimoIdTara, resultadoTara, sP0, sHace);
  if (len <= 0 || len >= (int)sizeof(msg)) return;
  Serial.println(msg);
  if (radioOk) esp_now_send(DIRECCION_BROADCAST, (const uint8_t*)msg, len);
}

// Seguridad: la tara solo se acepta en ESPERA y con el cohete quieto. En
// vuelo nunca se toca el cero.
void atenderTaraRemota() {
  if (cmdTaraPendiente) {
    cmdTaraPendiente = false;
    uint32_t id = cmdTaraId;
    if (id != ultimoIdTara) {  // El receptor repite la orden: se atiende una vez
      ultimoIdTara = id;
      if (estado != ESPERA) {
        resultadoTara = 3;
      } else if (!bmpOk) {
        resultadoTara = 4;
      } else if (mpuOk && millis() - tUltimoMovimiento < QUIETO_MIN_MS) {
        resultadoTara = 2;
      } else {
        calibrarTara();
        cuentaLanz = 0;
        resultadoTara = 1;
      }
      Serial.printf("# Tara remota id=%lu -> resultado %u\n", (unsigned long)id, resultadoTara);
      tEstadoTara = 0;  // Confirmar de inmediato
    }
  }
  // En ASCENSO no se envía: no interrumpir la telemetría del vuelo.
  if (estado != ASCENSO && millis() - tEstadoTara >= PERIODO_ESTADO_TARA_MS) {
    tEstadoTara = millis();
    enviarEstadoTara();
  }
}

// ============================================================================
//  SERVO: PRUEBA REMOTA E INFORME DEL DESPLIEGUE
// ============================================================================
void enviarEstadoServo() {
  char sAlt[16] = "null", sMs[16] = "null";
  if (paracaidas && !isnan(altDespliegue)) snprintf(sAlt, sizeof(sAlt), "%.2f", altDespliegue);
  if (paracaidas && huboLanzamiento) snprintf(sMs, sizeof(sMs), "%lu", (unsigned long)msDespliegue);

  char msg[160];
  int len = snprintf(msg, sizeof(msg),
      "{\"tipo\":\"servo\",\"id\":%lu,\"res\":%u,\"para\":%d,\"prueba\":%d,\"motivo\":\"%s\","
      "\"altDisp\":%s,\"altMax\":%.2f,\"msLanz\":%s,\"margen\":%.1f}",
      (unsigned long)ultimoIdServo, resultadoServo, paracaidas ? 1 : 0, servoEnPrueba ? 1 : 0,
      motivoDespliegue, sAlt, paracaidas ? altMaxDespliegue : altMax, sMs, MARGEN_APOGEO_M);
  if (len <= 0 || len >= (int)sizeof(msg)) return;
  Serial.println(msg);
  if (radioOk) esp_now_send(DIRECCION_BROADCAST, (const uint8_t*)msg, len);
}

// Seguridad: la prueba solo se acepta en ESPERA (nunca en vuelo ni con el
// paracaídas ya liberado). El cierre se hace sin bloquear el programa.
// El despliegue manual de emergencia, en cambio, se acepta SIEMPRE.
void atenderServoRemoto() {
  if (cmdDesplegarPendiente) {
    cmdDesplegarPendiente = false;
    uint32_t id = cmdDesplegarId;
    if (id != ultimoIdServo) {   // La estación repite cada orden: se atiende una vez
      ultimoIdServo = id;
      if (!paracaidas) {
        desplegarParacaidas("MANUAL");   // Abre y deja abierto; pasa a DESCENSO
        resultadoServo = 6;
      } else {
        servoMover(SERVO_ABIERTO);       // Ya estaba liberado: insistir por si se trabó
        resultadoServo = 7;
      }
      Serial.printf("# DESPLIEGUE MANUAL id=%lu -> resultado %u\n", (unsigned long)id, resultadoServo);
      tEstadoServo = 0;
    }
  }

  if (cmdServoPendiente) {
    cmdServoPendiente = false;
    uint32_t id = cmdServoId;
    if (id != ultimoIdServo) {
      ultimoIdServo = id;
      if (estado != ESPERA || paracaidas) {
        resultadoServo = 3;
      } else if (servoEnPrueba) {
        resultadoServo = 5;
      } else {
        servoMover(SERVO_ABIERTO);
        servoEnPrueba = true;
        tServoPrueba = millis();
        resultadoServo = 1;
      }
      Serial.printf("# Prueba de servo id=%lu -> resultado %u\n", (unsigned long)id, resultadoServo);
      tEstadoServo = 0;
    }
  }

  if (servoEnPrueba && millis() - tServoPrueba >= SERVO_PRUEBA_MS) {
    servoEnPrueba = false;
    if (!paracaidas) servoMover(SERVO_CERRADO);
    tEstadoServo = 0;
  }

  if (estado != ASCENSO && millis() - tEstadoServo >= PERIODO_ESTADO_TARA_MS) {
    tEstadoServo = millis();
    enviarEstadoServo();
  }
}

// ============================================================================
//  SETUP
// ============================================================================
#ifdef PIN_LED
// Código de parpadeos del LED al arrancar (por si la pantalla no funciona):
//   1 parpadeo largo    = arranque normal (encendido o botón RST)
//   5 parpadeos rápidos = se reinició por BAJO VOLTAJE
//   3 parpadeos medios  = fallo del programa (panic / watchdog)
void parpadearMotivoReinicio() {
  esp_reset_reason_t r = esp_reset_reason();
  Serial.printf("# Motivo del reinicio: %d\n", (int)r);
  int veces = 1, ms = 600;
  if (r == ESP_RST_BROWNOUT) { veces = 5; ms = 100; }
  else if (r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT) { veces = 3; ms = 250; }
  for (int i = 0; i < veces; i++) {
    digitalWrite(PIN_LED, HIGH); delay(ms);
    digitalWrite(PIN_LED, LOW);  delay(ms);
  }
}
#endif

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BOTON_TARA, INPUT_PULLUP);

  // 1. Servo primero: trabar el paracaídas lo antes posible
  if (!servoIniciar()) Serial.println("# ERROR: no se pudo iniciar el PWM del servo");
  servoMover(SERVO_CERRADO);

#ifdef PIN_LED
  pinMode(PIN_LED, OUTPUT);
  parpadearMotivoReinicio();
#endif

#if ES_HELTEC
  // Solo Heltec: encender la OLED (Vext) y arrancarla en el bus Wire1
  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);
  delay(100);
  Wire1.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  display.setBusClock(400000);
  display.begin();
#endif
  mostrarMensaje("TELEPATIN - COHETE", "ESP-NOW", "Iniciando...");

  delay(500);
  Serial.println("# TELEPATIN - COHETE (ESP-NOW)");

  // 2. Sensores I2C (GY-91)
  Wire.begin(PIN_SENSOR_SDA, PIN_SENSOR_SCL, 400000);
  delay(250);
  mpuOk = mpuIniciar();
  bmpOk = bmpIniciar();
#if USAR_DHT
  dht.begin();
#endif
  Serial.printf("# MPU:%s BMP:%s\n", mpuOk ? "OK" : "FALLO", bmpOk ? "OK" : "FALLO");

  // 3. GPS: buffer grande para no perder frases NMEA
  gpsSerial.setRxBufferSize(1024);
  gpsSerial.begin(GPS_BAUDIOS[gpsIndiceBaud], SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  // 4. Radio. Si falla, el paracaídas sigue funcionando.
  radioOk = espNowIniciar();
  Serial.printf("# ESP-NOW: %s  MAC %s\n", radioOk ? "OK" : "FALLO", WiFi.macAddress().c_str());

  char l2[32], l3[32];
  snprintf(l2, sizeof(l2), "MPU:%s BMP:%s", mpuOk ? "OK" : "X", bmpOk ? "OK" : "X");
  snprintf(l3, sizeof(l3), "ESP-NOW:%s", radioOk ? "OK" : "FALLO");
  mostrarMensaje("SENSORES", l2, l3);
  delay(1500);

  // 5. Tara: altitud 0 en la rampa
  calibrarTara();

  uint32_t ahora = millis();
  tSensores = tTx = tDht = tCambioBaud = tOled = ahora;
}

// ============================================================================
//  LOOP (no bloqueante)
// ============================================================================
void loop() {
  uint32_t ahora = millis();

  atenderGps(ahora);

  if (ahora - tSensores >= PERIODO_SENSORES_MS) {
    tSensores = ahora;
    leerSensores(ahora);
  }

  if (ahora - tTx >= PERIODO_TX_MS) {
    tTx = ahora;
    enviarTelemetria(ahora);
  }

  // La OLED no se refresca en ascenso: ahí cada milisegundo cuenta.
  if (ES_HELTEC && estado != ASCENSO && ahora - tOled >= PERIODO_OLED_MS) {
    tOled = ahora;
    actualizarOled();
  }

  atenderTaraRemota();
  atenderServoRemoto();

  // Botón BOOT/PRG: repetir la tara antes del lanzamiento
  if (estado == ESPERA && digitalRead(PIN_BOTON_TARA) == LOW) {
    delay(50);
    if (digitalRead(PIN_BOTON_TARA) == LOW) {
      calibrarTara();
      while (digitalRead(PIN_BOTON_TARA) == LOW) delay(10);
    }
  }
}
