/*
 * ============================================================================
 *  TelePatin - PRUEBA DE ENLACE (TRANSMISOR ESP-NOW + GPS + GY-91)
 *  Placa: Heltec WiFi LoRa 32 (V3) o cualquier ESP32 / ESP32-S3
 * ============================================================================
 *
 *  Envía cada 150 ms un JSON con el mismo formato que Cohete_ESPNOW_TX.
 *  Cada sensor que esté conectado manda datos REALES; el que falte se simula,
 *  así la prueba funciona con o sin módulos:
 *  - GY-91 (MPU9250 + BMP280): altitud (con tara al arrancar), presión,
 *    temperatura y aceleración reales. Sin él: datos simulados.
 *  - GPS (GY-NEO6MV2 / NEO-M8N): posición real. Sin fix: lat/lon = null.
 *
 *  Campos de estado del JSON:
 *    "gps":  0 = no llegan datos del módulo, 1 = sin fix, 2 = con fix
 *    "imu":  1 = aceleración real del MPU,   0 = simulada
 *    "baro": 1 = altitud real del BMP280,    0 = simulada
 *
 *  Conexión del GY-91 (Heltec V3):
 *    VIN -> 3V3     GND -> GND     SDA -> GPIO 41     SCL -> GPIO 42
 *    (SAO/SDO, NCS y CSB sin conectar)
 *  (En un ESP32 DevKit: SDA -> 21, SCL -> 22)
 *  Al arrancar se hace la TARA: dejar el sensor quieto ~2 s. Botón PRG = tara.
 *  También se puede fijar el cero desde la página ("Fijar cero"): solo se
 *  acepta si el sensor lleva 1 s quieto. El resultado vuelve como un mensaje
 *  {"tipo":"tara",...} cada segundo.
 *
 *  Servo del paracaídas (MG90S) - señal en GPIO 4 (DevKit: 13), alimentado
 *  con 5 V externos (LM2596), GND común. 0° = trabado, 90° = liberado.
 *  - "Probar servo" en la página: abre 2 s y cierra (solo en ESPERA).
 *  - Prueba de apogeo a mano: subir el sensor > 2 m y bajarlo 1.2 m desde
 *    el máximo -> DISPARO. "Fijar cero" rearma la prueba.
 *
 *  Conexión del GPS (Heltec V3):
 *    VCC -> 3V3 (NEO-6M) o 5V (NEO-M8N, consume más)     GND -> GND
 *    TX del GPS -> GPIO 6     RX del GPS -> GPIO 7
 *  El RX del GPS SÍ hace falta: por ahí se le pide al M8N que active NMEA.
 *  (En un ESP32 DevKit: TX del GPS -> 16, RX del GPS -> 17)
 *
 *  Receptor: EstacionTerrena_ESPNOW_RX (sin cambios).
 *  En la página: panel "Prueba de enlace" tras pulsar "Conectar receptor".
 *  Librerías: TinyGPSPlus, Adafruit BMP280 Library; U8g2 solo en la Heltec.
 *  El servo se controla con el PWM nativo del ESP32 (LEDC), sin ESP32Servo:
 *  en el ESP32-S3 esa librería usa el driver MCPWM antiguo y deja sin
 *  transmisión al ESP-NOW.
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
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
  #define PIN_GPS_RX   6    // Al TX del módulo GPS
  #define PIN_GPS_TX   7    // Al RX del módulo GPS
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
#define PIN_BOTON_TARA 0    // PRG en la Heltec, BOOT en el DevKit

// ============================================================================
//  CONFIGURACIÓN ESP-NOW  (DEBE SER IDÉNTICA EN LA ESTACIÓN TERRENA)
// ============================================================================
#define ESPNOW_CANAL      1
#define USAR_LONG_RANGE   0   // 1 = modo LR. Activarlo SOLO cuando el enlace ya funcione, y en ambos lados.
uint8_t DIRECCION_BROADCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// Potencia de transmisión (2 a 21 dBm). A 21 dBm cada envío pide picos de
// ~350-400 mA: con USB no pasa nada, pero con una LiPo 1S por el JST el
// voltaje se hunde y la placa se reinicia o la radio falla. 15 dBm reduce
// mucho esos picos y sigue alcanzando ~100-200 m con vista despejada.
const int POTENCIA_TX_DBM = 15;

// 0 = el programa no genera señal para el servo (útil para descartar que el
// servo cause reinicios). El resto de la prueba funciona igual.
#define USAR_SERVO        1

const uint32_t PERIODO_TX_MS    = 150;
const uint32_t PERIODO_OLED_MS  = 500;
const uint32_t PERIODO_DIAG_MS  = 2000;
const uint32_t GPS_MAX_EDAD_MS  = 2000;  // Posición más vieja que esto = sin fix
const uint32_t GPS_CAMBIO_BAUD_MS = 3000;
const uint32_t PERIODO_SENSORES_MS = 20;   // 50 Hz, igual que en el cohete
const float    ALFA_FILTRO_ALT     = 0.3;

#if ES_HELTEC
U8G2_SSD1306_128X64_NONAME_F_2ND_HW_I2C display(U8G2_R0, PIN_OLED_RST);
#endif

TinyGPSPlus     gps;
HardwareSerial& gpsSerial = Serial2;
Adafruit_BMP280 bmp(&Wire);

// GY-91
bool    mpuOk = false, bmpOk = false;
uint8_t mpuAddr = 0x68;
float   ax = 0, ay = 0, az = 0, aMagG = 0;   // m/s^2 y módulo en g
float   presionSuelo = 101325.0, presionPa = NAN, tempBmp = NAN;
float   altFilt = 0, altMax = 0;
uint32_t tSensores = 0;

// Tara remota: botón "Fijar cero" de la página -> receptor -> ESP-NOW -> aquí.
// El receptor repite la orden varias veces; el id evita atenderla dos veces.
const uint32_t PERIODO_ESTADO_TARA_MS = 1000;  // Estado de la tara hacia la página
const uint32_t QUIETO_MIN_MS = 1000;           // Tiempo quieto exigido antes de la tara
const float    QUIETO_TOL_G  = 0.08;           // |a| debe estar en 1 g +- esto
volatile bool     cmdTaraPendiente = false;
volatile uint32_t cmdTaraId = 0;
uint32_t ultimoIdTara = 0, tUltimaTara = 0, tUltimoMovimiento = 0, tEstadoTara = 0;
uint8_t  resultadoTara = 0;  // 0 ninguna, 1 OK, 2 en movimiento, 3 no está en ESPERA, 4 sin barómetro

// ----------------------------------------------------------------------------
//  MODO PRUEBA DE APOGEO (para probar a mano, p. ej. por una escalera)
// ----------------------------------------------------------------------------
// Igual que el cohete pero SIN la condición de aceleración (en la mano siempre
// marca ~1 g): sube más de 2 m -> SUBIENDO; baja 1.2 m del máximo -> DISPARO.
// "Fijar cero" (o el botón PRG) rearma la prueba y vuelve a trabar el servo.
const float   PRUEBA_SUBIDA_M      = 2.0;
const float   MARGEN_APOGEO_M      = 1.2;   // El mismo margen que Cohete_ESPNOW_TX
const uint8_t MUESTRAS_CONFIRMACION = 3;
const int     SERVO_CERRADO = 0;
const int     SERVO_ABIERTO = 90;
const uint32_t SERVO_PRUEBA_MS = 2000;

enum EstadoPrueba { P_ESPERA, P_SUBIENDO, P_DISPARO };
const char* NOMBRE_PRUEBA[] = { "ESPERA", "SUBIENDO", "DISPARO" };
EstadoPrueba estadoApo = P_ESPERA;
uint8_t  cuentaSubida = 0, cuentaApogeo = 0;
bool     paracaidas = false;
const char* motivoDespliegue = "";
float    altDespliegue = NAN, altMaxDespliegue = NAN;  // Congelados en el disparo
uint32_t tSubida = 0, msDespliegue = 0;

// Servo con el PWM nativo (LEDC): 50 Hz, pulso de 500 a 2400 us para 0-180°
const uint32_t SERVO_FREQ_HZ     = 50;
const uint8_t  SERVO_RES_BITS    = 14;
const uint32_t SERVO_PULSO_MIN_US = 500;
const uint32_t SERVO_PULSO_MAX_US = 2400;

bool servoIniciar() {
  return ledcAttach(PIN_SERVO, SERVO_FREQ_HZ, SERVO_RES_BITS);
}

// Todas las órdenes al servo pasan por aquí (respeta USAR_SERVO)
void servoMover(int grados) {
#if USAR_SERVO
  grados = constrain(grados, 0, 180);
  uint32_t us   = map(grados, 0, 180, SERVO_PULSO_MIN_US, SERVO_PULSO_MAX_US);
  uint32_t duty = us * ((1UL << SERVO_RES_BITS) - 1) / (1000000UL / SERVO_FREQ_HZ);
  ledcWrite(PIN_SERVO, duty);
#endif
}

volatile bool     cmdServoPendiente = false;
volatile uint32_t cmdServoId = 0;
uint32_t ultimoIdServo = 0, tServoPrueba = 0, tEstadoServo = 0;
bool     servoEnPrueba = false;
uint8_t  resultadoServo = 0;  // 0 ninguna, 1 prueba hecha, 3 no está en ESPERA, 5 prueba en curso

bool     radioOk = false;
uint8_t  canalReal = 0;
uint32_t paquetesEnviados = 0, fallos = 0, tTx = 0, tOled = 0, tDiag = 0;
esp_err_t ultimoErrorEnvio = ESP_OK;
volatile uint32_t alAireOk = 0, alAireFallo = 0;
float    altSim = 0;

// GPS: auto-detección de baudios. El NEO-6M viene a 9600; los NEO-M8N "para
// drones" suelen venir a 38400/57600/115200 y a veces solo en binario UBX.
const uint32_t GPS_BAUDIOS[] = { 9600, 38400, 57600, 115200, 4800, 19200, 230400 };
const uint8_t  GPS_N_BAUDIOS = sizeof(GPS_BAUDIOS) / sizeof(GPS_BAUDIOS[0]);
uint8_t  gpsIndiceBaud = 0;
bool     gpsBaudOk = false;
uint32_t tCambioBaud = 0;

// Diagnóstico de la ventana de prueba de cada velocidad
uint32_t gpsBytesVentana = 0, gpsBytesUltimaVentana = 0;
bool     gpsVioUbx = false;
uint8_t  gpsByteAnterior = 0;

// ============================================================================
//  GPS (no bloqueante)
// ============================================================================
// Envía un mensaje binario UBX al u-blox (cabecera B5 62 + checksum Fletcher)
void ubxEnviar(uint8_t clase, uint8_t id, const uint8_t* datos, uint16_t len) {
  uint8_t ckA = 0, ckB = 0;
  uint8_t cab[4] = { clase, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
  gpsSerial.write(0xB5);
  gpsSerial.write(0x62);
  for (uint8_t b : cab)          { gpsSerial.write(b); ckA += b; ckB += ckA; }
  for (uint16_t i = 0; i < len; i++) { gpsSerial.write(datos[i]); ckA += datos[i]; ckB += ckA; }
  gpsSerial.write(ckA);
  gpsSerial.write(ckB);
}

// UBX-CFG-PRT: activa la salida NMEA en el UART1, manteniendo la velocidad
// actual. Solo cambia la RAM del módulo (no se guarda): al apagarlo vuelve a
// su configuración original, y este programa lo repite en cada arranque.
void gpsActivarNmea(uint32_t baud) {
  uint8_t p[20] = { 0 };
  p[0]  = 1;                       // Puerto UART1
  p[4]  = 0xD0; p[5] = 0x08;       // 8 bits, sin paridad, 1 stop
  p[8]  = baud & 0xFF;
  p[9]  = (baud >> 8) & 0xFF;
  p[10] = (baud >> 16) & 0xFF;
  p[11] = (baud >> 24) & 0xFF;
  p[12] = 0x07;                    // Entrada: UBX + NMEA + RTCM
  p[14] = 0x03;                    // Salida:  UBX + NMEA
  ubxEnviar(0x06, 0x00, p, sizeof(p));
}

void gpsAbrir(uint8_t indice) {
  gpsSerial.end();
  gpsSerial.begin(GPS_BAUDIOS[indice], SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  delay(20);
  gpsActivarNmea(GPS_BAUDIOS[indice]);
  gpsBytesVentana = 0;
  gpsVioUbx = false;
}

void atenderGps(uint32_t ahora) {
  while (gpsSerial.available() > 0) {
    uint8_t c = gpsSerial.read();
    gps.encode(c);
    gpsBytesVentana++;
    if (gpsByteAnterior == 0xB5 && c == 0x62) gpsVioUbx = true;  // Cabecera UBX
    gpsByteAnterior = c;
  }

  // Solo una frase NMEA con checksum correcto confirma la velocidad.
  if (!gpsBaudOk) {
    if (gps.passedChecksum() > 0) {
      gpsBaudOk = true;
      Serial.printf("# GPS conectado a %lu baudios\n", (unsigned long)GPS_BAUDIOS[gpsIndiceBaud]);
    } else if (ahora - tCambioBaud >= GPS_CAMBIO_BAUD_MS) {
      Serial.printf("# GPS a %lu baud: %lu bytes, %s\n",
                    (unsigned long)GPS_BAUDIOS[gpsIndiceBaud], (unsigned long)gpsBytesVentana,
                    gpsBytesVentana == 0 ? "no llega nada"
                    : gpsVioUbx ? "llega binario UBX (sin NMEA)" : "basura (velocidad incorrecta)");
      gpsBytesUltimaVentana = gpsBytesVentana;
      tCambioBaud = ahora;
      gpsIndiceBaud = (gpsIndiceBaud + 1) % GPS_N_BAUDIOS;
      gpsAbrir(gpsIndiceBaud);
    }
  }
}

// ============================================================================
//  GY-91: MPU9250/MPU6500 por registros + BMP280  (igual que Cohete_ESPNOW_TX)
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

void mostrarMensaje(const char* l1, const char* l2);

// Promedia la presión en reposo y la toma como altitud 0
void calibrarTara() {
  if (!bmpOk) return;
  mostrarMensaje("CALIBRANDO TARA", "No mover el sensor...");
  Serial.println("# Calibrando tara barometrica...");
  for (uint8_t i = 0; i < 10; i++) { bmp.readPressure(); delay(30); }  // Descartar

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
  tUltimaTara = millis();
  Serial.printf("# Tara lista: P0 = %.2f Pa\n", presionSuelo);

  // Rearmar la prueba de apogeo
  estadoApo = P_ESPERA;
  cuentaSubida = cuentaApogeo = 0;
  paracaidas = false;
  motivoDespliegue = "";
  altDespliegue = NAN;
  servoEnPrueba = false;
  servoMover(SERVO_CERRADO);
}

void desplegarParacaidas(const char* motivo) {
  servoMover(SERVO_ABIERTO);
  paracaidas = true;
  servoEnPrueba = false;
  motivoDespliegue = motivo;
  altDespliegue = altFilt;
  altMaxDespliegue = altMax;
  msDespliegue = millis() - tSubida;
  estadoApo = P_DISPARO;
  tEstadoServo = 0;  // Informar de inmediato a la página
  Serial.printf("# DISPARO (%s) alt=%.2f max=%.2f caida=%.2f m\n",
                motivo, altFilt, altMax, altMax - altFilt);
}

void actualizarPruebaApogeo() {
  if (!bmpOk) return;
  switch (estadoApo) {
    case P_ESPERA:
      cuentaSubida = (altFilt > PRUEBA_SUBIDA_M) ? cuentaSubida + 1 : 0;
      if (cuentaSubida >= MUESTRAS_CONFIRMACION) {
        estadoApo = P_SUBIENDO;
        tSubida = millis();
        cuentaApogeo = 0;
        Serial.println("# Prueba de apogeo: SUBIENDO");
      }
      break;
    case P_SUBIENDO:
      cuentaApogeo = (altFilt < altMax - MARGEN_APOGEO_M) ? cuentaApogeo + 1 : 0;
      if (cuentaApogeo >= MUESTRAS_CONFIRMACION) desplegarParacaidas("APOGEO");
      break;
    case P_DISPARO:
      break;
  }
}

void leerSensores() {
  if (mpuOk && mpuLeerAcel() && fabsf(aMagG - 1.0f) > QUIETO_TOL_G) tUltimoMovimiento = millis();
  if (bmpOk) {
    float p = bmp.readPressure();
    if (!isnan(p) && p > 30000 && p < 110000) {
      presionPa = p;
      altFilt = ALFA_FILTRO_ALT * altitudDesdePresion(p) + (1.0f - ALFA_FILTRO_ALT) * altFilt;
      if (altFilt > altMax) altMax = altFilt;
    }
  }
  actualizarPruebaApogeo();
}

bool gpsConFix() {
  return gps.location.isValid() && gps.location.age() < GPS_MAX_EDAD_MS;
}

// 0 = no llegan datos, 1 = responde sin fix, 2 = con fix
int gpsEstado() {
  if (!gpsBaudOk) return 0;
  return gpsConFix() ? 2 : 1;
}

int gpsSatelites() {
  return gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
}

// Confirmación del driver WiFi: el paquete salió (o no) al aire
void alEnviar(const esp_now_send_info_t* info, esp_now_send_status_t estado) {
  if (estado == ESP_NOW_SEND_SUCCESS) alAireOk++;
  else                                alAireFallo++;
}

// Órdenes desde la estación (tarea WiFi: solo copiar y levantar la bandera)
// Formato: "CMD:TARA:<id>" o "CMD:SERVO:<id>"
void alRecibir(const esp_now_recv_info_t* info, const uint8_t* datos, int len) {
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

  wifi_second_chan_t sec;
  esp_wifi_get_channel(&canalReal, &sec);
  Serial.printf("# WiFi STA %s, canal real %u (esperado %u), LR=%d\n",
                WiFi.STA.started() ? "OK" : "SIN ARRANCAR", canalReal, ESPNOW_CANAL, USAR_LONG_RANGE);

  esp_err_t e = esp_now_init();
  if (e != ESP_OK) { Serial.printf("# esp_now_init: %s\n", esp_err_to_name(e)); return false; }
  esp_now_register_send_cb(alEnviar);
  esp_now_register_recv_cb(alRecibir);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, DIRECCION_BROADCAST, 6);
  peer.channel = 0;  // 0 = el canal actual del WiFi
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  e = esp_now_add_peer(&peer);
  if (e != ESP_OK) { Serial.printf("# esp_now_add_peer: %s\n", esp_err_to_name(e)); return false; }
  return true;
}

void enviarPrueba(uint32_t ahora) {
  float t = ahora / 1000.0f;

  // Barómetro: real si hay BMP280; si no, la altitud sube y baja (0-30 m)
  // en ciclos de 20 s. La humedad no tiene sensor en esta prueba.
  float alt, altMaxEnv, pres, temp, hum;
  if (bmpOk && !isnan(presionPa)) {
    alt = altFilt;
    altMaxEnv = altMax;
    pres = presionPa / 100.0f;
    tempBmp = bmp.readTemperature();
    temp = tempBmp;
    hum = NAN;
  } else {
    altSim = 15.0f - 15.0f * cosf(t * 2.0f * PI / 20.0f);
    alt = altSim;
    altMaxEnv = 30.0f;
    pres = 1010.0f - altSim * 0.12f;
    temp = 28.0f + 0.5f * sinf(t / 7.0f);
    hum  = 65.0f + 3.0f * sinf(t / 11.0f);
  }

  // Acelerómetro: real si hay MPU; si no, simulado (reposo con vibración)
  float axE = ax, ayE = ay, azE = az;
  if (!mpuOk) {
    axE = 0.3f * sinf(t * 3.0f);
    ayE = 0.3f * cosf(t * 3.0f);
    azE = 9.81f;
  }

  char sHum[12] = "null";
  if (!isnan(hum)) snprintf(sHum, sizeof(sHum), "%.1f", hum);

  // Posición real del GPS, o null si no hay fix
  char sLat[16] = "null", sLon[16] = "null";
  if (gpsConFix()) {
    snprintf(sLat, sizeof(sLat), "%.6f", gps.location.lat());
    snprintf(sLon, sizeof(sLon), "%.6f", gps.location.lng());
  }

  static char json[ESP_NOW_MAX_DATA_LEN];
  int len = snprintf(json, sizeof(json),
      "{\"n\":%lu,\"t\":%.2f,\"alt\":%.2f,\"altMax\":%.2f,\"temp\":%.1f,\"hum\":%s,"
      "\"pres\":%.2f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"lat\":%s,\"lon\":%s,"
      "\"sats\":%d,\"gps\":%d,\"imu\":%d,\"baro\":%d,\"para\":%d,\"est\":\"%s\"}",
      (unsigned long)paquetesEnviados, t, alt, altMaxEnv, temp, sHum, pres, axE, ayE, azE,
      sLat, sLon, gpsSatelites(), gpsEstado(), mpuOk ? 1 : 0, bmpOk ? 1 : 0,
      paracaidas ? 1 : 0, NOMBRE_PRUEBA[estadoApo]);
  if (len <= 0 || len >= (int)sizeof(json)) return;

  Serial.println(json);

  esp_err_t e = radioOk ? esp_now_send(DIRECCION_BROADCAST, (const uint8_t*)json, len) : ESP_FAIL;
  if (e == ESP_OK) {
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  } else {
    if (e != ultimoErrorEnvio) Serial.printf("# esp_now_send fallo: %s\n", esp_err_to_name(e));
    ultimoErrorEnvio = e;
    fallos++;
  }
  // El contador avanza siempre: si el receptor ve saltos en "n", hubo pérdidas
  paquetesEnviados++;
}

// ============================================================================
//  TARA REMOTA
// ============================================================================
// Mensaje aparte (no cabe en el JSON de telemetría, que ronda los 250 bytes).
// La página lo reconoce por "tipo":"tara" y no lo cuenta como paquete de datos.
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

void atenderTaraRemota(uint32_t ahora) {
  if (cmdTaraPendiente) {
    cmdTaraPendiente = false;
    uint32_t id = cmdTaraId;
    if (id != ultimoIdTara) {
      ultimoIdTara = id;
      if (!bmpOk) {
        resultadoTara = 4;
      } else if (mpuOk && millis() - tUltimoMovimiento < QUIETO_MIN_MS) {
        // millis() y no "ahora": leerSensores() pudo actualizar tUltimoMovimiento
        // después de tomar "ahora", y la resta daría negativo (número enorme).
        resultadoTara = 2;  // Se está moviendo: la tara saldría mal
      } else {
        calibrarTara();
        resultadoTara = 1;
      }
      Serial.printf("# Tara remota id=%lu -> resultado %u\n", (unsigned long)id, resultadoTara);
      tEstadoTara = 0;  // Confirmar de inmediato
    }
  }
  if (millis() - tEstadoTara >= PERIODO_ESTADO_TARA_MS) {
    tEstadoTara = millis();
    enviarEstadoTara();
  }
}

// ============================================================================
//  SERVO: PRUEBA REMOTA E INFORME DEL DISPARO  (mismo formato que el cohete)
// ============================================================================
void enviarEstadoServo() {
  char sAlt[16] = "null", sMs[16] = "null";
  if (paracaidas && !isnan(altDespliegue)) snprintf(sAlt, sizeof(sAlt), "%.2f", altDespliegue);
  if (paracaidas) snprintf(sMs, sizeof(sMs), "%lu", (unsigned long)msDespliegue);

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

// La prueba de servo solo se acepta en ESPERA (no subiendo ni disparado)
void atenderServoRemoto() {
  if (cmdServoPendiente) {
    cmdServoPendiente = false;
    uint32_t id = cmdServoId;
    if (id != ultimoIdServo) {
      ultimoIdServo = id;
      if (estadoApo != P_ESPERA || paracaidas) {
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

  if (millis() - tEstadoServo >= PERIODO_ESTADO_TARA_MS) {
    tEstadoServo = millis();
    enviarEstadoServo();
  }
}

void mostrarMensaje(const char* l1, const char* l2) {
#if ES_HELTEC
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, l1);
  display.drawStr(0, 28, l2);
  display.sendBuffer();
#endif
}

void actualizarOled() {
#if ES_HELTEC
  char l[32];
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  snprintf(l, sizeof(l), "NOW:%s C%u TX:%lu", radioOk ? "OK" : "X", canalReal,
           (unsigned long)paquetesEnviados);
  display.drawStr(0, 10, l);
  snprintf(l, sizeof(l), "Aire:%lu Fallos:%lu", (unsigned long)alAireOk,
           (unsigned long)(fallos + alAireFallo));
  display.drawStr(0, 22, l);
  snprintf(l, sizeof(l), "MPU:%s BMP:%s %.2fg", mpuOk ? "OK" : "--", bmpOk ? "OK" : "--",
           mpuOk ? aMagG : 0.0f);
  display.drawStr(0, 34, l);
  if (bmpOk) {
    snprintf(l, sizeof(l), "%.1f/%.1fm %s", altFilt, altMax,
             servoEnPrueba ? "SERVO" : NOMBRE_PRUEBA[estadoApo]);
  } else {
    snprintf(l, sizeof(l), "Sin GY-91: simulado");
  }
  display.drawStr(0, 46, l);
  switch (gpsEstado()) {
    case 0:
      // Distingue "no llega nada" (cables/alimentación) de "llegan bytes que
      // aún no se entienden" (velocidad o protocolo, se está probando).
      if (gpsBytesVentana > 0 || gpsBytesUltimaVentana > 0)
        snprintf(l, sizeof(l), "GPS: bytes %lubd", (unsigned long)GPS_BAUDIOS[gpsIndiceBaud]);
      else
        snprintf(l, sizeof(l), "GPS: SIN DATOS %lubd", (unsigned long)GPS_BAUDIOS[gpsIndiceBaud]);
      break;
    case 1:  snprintf(l, sizeof(l), "GPS: buscando Sat:%d", gpsSatelites()); break;
    default: snprintf(l, sizeof(l), "GPS: FIX Sat:%d", gpsSatelites()); break;
  }
  display.drawStr(0, 60, l);
  display.sendBuffer();
#endif
}

// Motivo del último reinicio: si sale "BAJO VOLTAJE", la placa se reinició por
// una caída de alimentación (típico cuando el servo arranca).
const char* motivoReinicio() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "ENCENDIDO";
    case ESP_RST_EXT:      return "BOTON RST";
    case ESP_RST_SW:       return "SOFTWARE";
    case ESP_RST_BROWNOUT: return "BAJO VOLTAJE!";
    case ESP_RST_PANIC:    return "FALLO (PANIC)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "WATCHDOG";
    default:               return "OTRO";
  }
}

// Código de parpadeos del LED al arrancar (por si la pantalla no funciona):
//   1 parpadeo largo   = arranque normal (encendido o botón RST)
//   5 parpadeos rápidos = se reinició por BAJO VOLTAJE
//   3 parpadeos medios = fallo del programa (panic / watchdog)
void parpadearMotivoReinicio() {
  esp_reset_reason_t r = esp_reset_reason();
  int veces = 1, ms = 600;
  if (r == ESP_RST_BROWNOUT) { veces = 5; ms = 100; }
  else if (r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT) { veces = 3; ms = 250; }
  for (int i = 0; i < veces; i++) {
    digitalWrite(PIN_LED, HIGH); delay(ms);
    digitalWrite(PIN_LED, LOW);  delay(ms);
  }
  delay(500);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  parpadearMotivoReinicio();

#if USAR_SERVO
  // Servo primero: trabado desde el arranque
  if (!servoIniciar()) Serial.println("# ERROR: no se pudo iniciar el PWM del servo");
  servoMover(SERVO_CERRADO);
#endif

#if ES_HELTEC
  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);
  delay(100);
  Wire1.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  display.setBusClock(400000);
  display.begin();
#endif

  char lr[32];
  snprintf(lr, sizeof(lr), "Reinicio: %s", motivoReinicio());
  Serial.printf("# %s  (servo %s)\n", lr, USAR_SERVO ? "ACTIVO" : "DESACTIVADO");
  mostrarMensaje(lr, USAR_SERVO ? "Servo: activo" : "Servo: DESACTIVADO");
  delay(1500);
  pinMode(PIN_BOTON_TARA, INPUT_PULLUP);

  // GY-91 en el bus I2C principal. Si no está, se sigue con datos simulados.
  Wire.begin(PIN_SENSOR_SDA, PIN_SENSOR_SCL, 400000);
  delay(250);
  mpuOk = mpuIniciar();
  bmpOk = bmpIniciar();
  Serial.printf("# GY-91 -> MPU:%s BMP:%s\n", mpuOk ? "OK" : "NO", bmpOk ? "OK" : "NO");
  char l2[32];
  snprintf(l2, sizeof(l2), "MPU:%s BMP:%s", mpuOk ? "OK" : "NO", bmpOk ? "OK" : "NO");
  mostrarMensaje("GY-91", l2);
  delay(1000);
  calibrarTara();

  // GPS: buffer grande para no perder frases NMEA
  gpsSerial.setRxBufferSize(1024);
  gpsAbrir(gpsIndiceBaud);
  tCambioBaud = millis();

  radioOk = espNowIniciar();
  Serial.printf("# PRUEBA TX ESP-NOW: %s  MAC %s\n", radioOk ? "OK" : "FALLO",
                WiFi.macAddress().c_str());
}

void loop() {
  uint32_t ahora = millis();

  atenderGps(ahora);

  if (ahora - tSensores >= PERIODO_SENSORES_MS) {
    tSensores = ahora;
    leerSensores();
  }

  atenderTaraRemota(ahora);
  atenderServoRemoto();

  // Botón PRG/BOOT: repetir la tara (altitud 0 donde esté el sensor)
  if (digitalRead(PIN_BOTON_TARA) == LOW) {
    delay(50);
    if (digitalRead(PIN_BOTON_TARA) == LOW) {
      calibrarTara();
      while (digitalRead(PIN_BOTON_TARA) == LOW) delay(10);
    }
  }

  if (ahora - tTx >= PERIODO_TX_MS) {
    tTx = ahora;
    enviarPrueba(ahora);
  }

  if (ahora - tDiag >= PERIODO_DIAG_MS) {
    tDiag = ahora;
    Serial.printf("# enviados=%lu al_aire=%lu fallo_aire=%lu fallo_send=%lu canal=%u radio=%d tx=%ddBm ultimo_error=%s\n",
                  (unsigned long)paquetesEnviados, (unsigned long)alAireOk,
                  (unsigned long)alAireFallo, (unsigned long)fallos, canalReal, radioOk,
                  POTENCIA_TX_DBM, esp_err_to_name(ultimoErrorEnvio));
    Serial.printf("# gy91 mpu=%d bmp=%d alt=%.2f max=%.2f |a|=%.2fg\n",
                  mpuOk, bmpOk, altFilt, altMax, aMagG);
    Serial.printf("# gps estado=%d sats=%d caracteres=%lu frases_ok=%lu frases_mal=%lu\n",
                  gpsEstado(), gpsSatelites(), (unsigned long)gps.charsProcessed(),
                  (unsigned long)gps.passedChecksum(), (unsigned long)gps.failedChecksum());
  }

  if (ES_HELTEC && ahora - tOled >= PERIODO_OLED_MS) {
    tOled = ahora;
    actualizarOled();
  }
}
