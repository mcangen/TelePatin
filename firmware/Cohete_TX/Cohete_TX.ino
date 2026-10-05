/*
 * ============================================================================
 *  TelePatin - COHETE (TRANSMISOR LoRa)
 *  Placa: Heltec WiFi LoRa 32 (V3)  ->  ESP32-S3 + SX1262
 *  Arduino IDE: Placa "Heltec WiFi LoRa 32(V3)" (core esp32 de Espressif)
 * ============================================================================
 *
 *  Librerías (Gestor de librerías del Arduino IDE):
 *    - RadioLib               (jgromes)   -> radio SX1262 de la Heltec V3
 *    - Adafruit BMP280 Library (+ Adafruit Unified Sensor, Adafruit BusIO)
 *    - DHT sensor library     (Adafruit)
 *    - ESP32Servo             (Kevin Harrington)
 *    - U8g2                   (olikraus)  -> pantalla OLED integrada
 *
 *  NOTA: el MPU9250 del GY-91 se lee por registros directos (sin librería).
 *  Muchos GY-91 traen en realidad un MPU6500/MPU9255 (WHO_AM_I distinto) y las
 *  librerías de MPU9250 fallan con ellos; la lectura directa funciona con todos.
 *
 *  ---------------------------------------------------------------------------
 *  CONEXIONES RECOMENDADAS (Heltec V3)
 *  ---------------------------------------------------------------------------
 *   Señal              | GPIO | Notas
 *   -------------------+------+------------------------------------------------
 *   GY-91 SDA          |  41  | I2C principal (Wire). SDA por defecto del core
 *   GY-91 SCL          |  42  | I2C principal (Wire). SCL por defecto del core
 *   GY-91 VIN / GND    | 3V3  | El GY-91 tiene regulador; 3V3 o 5V sirven
 *   DHT11 DATA         |   5  | Resistencia pull-up 10k a 3V3 (si el módulo
 *                      |      | no la trae ya)
 *   Servo señal        |   6  | Alimentar el servo con 5V EXTERNOS, GND común
 *   Botón PRG (placa)  |   0  | Repetir la tara en ESPERA (ya está en la placa)
 *   -------------------+------+------------------------------------------------
 *   Reservados internamente (NO usar): 8-14 (LoRa), 17/18/21 (OLED),
 *   35 (LED), 36 (Vext), 37 (ADC batería), 19/20 (USB), 43/44 (UART0).
 *   Evitar pines de arranque: 0, 3, 45, 46.
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RadioLib.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <U8g2lib.h>

// ============================================================================
//  PINES
// ============================================================================
// --- Sensores externos ---
#define PIN_SENSOR_SDA 41
#define PIN_SENSOR_SCL 42
#define PIN_DHT        5
#define PIN_SERVO      6
#define PIN_BOTON_PRG  0

// --- Internos de la Heltec V3 (no cambiar) ---
#define PIN_OLED_SDA   17
#define PIN_OLED_SCL   18
#define PIN_OLED_RST   21
#define PIN_VEXT       36   // LOW = enciende la alimentación de la OLED
#define PIN_LED        35

#define LORA_NSS       8
#define LORA_SCK       9
#define LORA_MOSI      10
#define LORA_MISO      11
#define LORA_RST       12
#define LORA_BUSY      13
#define LORA_DIO1      14

// ============================================================================
//  CONFIGURACIÓN LoRa  (DEBE SER IDÉNTICA EN LA ESTACIÓN TERRENA)
// ============================================================================
#define LORA_FREQ_MHZ   915.0  // 915 = América (Colombia). 868 en Europa.
#define LORA_BW_KHZ     500.0  // Ancho de banda alto -> paquetes rápidos
#define LORA_SF         7      // SF7: ~85 ms de aire para ~150 bytes
#define LORA_CR         5      // Coding rate 4/5
#define LORA_SYNC_WORD  0x12   // Red privada
#define LORA_TX_DBM     17     // Potencia de transmisión (máx. 22)
#define LORA_PREAMBULO  8
#define LORA_TCXO_V     1.8    // TCXO de la Heltec V3

// ============================================================================
//  PARÁMETROS DE VUELO  (ajustar según el cohete)
// ============================================================================
const float    MARGEN_APOGEO_M           = 1.5;   // Caída bajo alt. máx. para declarar apogeo
const float    MARGEN_APOGEO_SOLO_BARO_M = 4.0;   // Respaldo: caída solo barométrica
const float    UMBRAL_CAIDA_G            = 0.8;   // |a| < esto = cuasi caída libre (sin empuje)
const float    UMBRAL_LANZAMIENTO_G      = 2.5;   // |a| > esto = empuje del agua
const float    UMBRAL_LANZAMIENTO_ALT_M  = 3.0;   // Respaldo de lanzamiento por altitud
const uint8_t  MUESTRAS_CONFIRMACION     = 3;     // Muestras seguidas para confirmar evento
const uint32_t BLOQUEO_POST_LANZ_MS      = 500;   // Ignorar apogeo durante el empuje
const uint32_t VENTANA_FALSO_LANZ_MS     = 1500;  // Si tras esto no subió...
const float    ALT_MIN_VUELO_REAL_M      = 1.5;   // ...al menos esto, fue falsa alarma
const uint32_t TIMEOUT_APOGEO_MS         = 6000;  // Respaldo: desplegar sí o sí tras el lanzamiento
const float    ALT_ATERRIZAJE_M          = 3.0;
const uint32_t TIEMPO_QUIETO_ATERRIZ_MS  = 3000;

const int SERVO_CERRADO = 0;    // Grados con el paracaídas trabado
const int SERVO_ABIERTO = 90;   // Grados para expulsarlo

// ============================================================================
//  TEMPORIZACIÓN
// ============================================================================
const uint32_t PERIODO_SENSORES_MS = 20;    // 50 Hz para la máquina de estados
const uint32_t PERIODO_TX_MS       = 150;   // Telemetría cada 150 ms
const uint32_t PERIODO_DHT_MS      = 2000;  // El DHT11 no admite lecturas más rápidas
const uint32_t PERIODO_OLED_MS     = 500;
const uint32_t TIMEOUT_TX_MS       = 500;   // Si el radio no confirma, se libera
const float    ALFA_FILTRO_ALT     = 0.3;   // Filtro exponencial de altitud (0..1)

// ============================================================================
//  OBJETOS
// ============================================================================
// OLED por I2C HARDWARE en el segundo bus (Wire1): Wire queda libre para los
// sensores y el refresco es mucho más rápido que por software.
U8G2_SSD1306_128X64_NONAME_F_2ND_HW_I2C display(U8G2_R0, PIN_OLED_RST);
// Si la pantalla no enciende, usar en su lugar la versión por software:
// U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, PIN_OLED_SCL, PIN_OLED_SDA, PIN_OLED_RST);

SX1262          radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
Adafruit_BMP280 bmp(&Wire);
DHT             dht(PIN_DHT, DHT11);
Servo           servo;

// ============================================================================
//  ESTADO GLOBAL
// ============================================================================
enum EstadoVuelo { ESPERA, ASCENSO, DESCENSO, ATERRIZADO };
const char* NOMBRE_ESTADO[] = { "ESPERA", "ASCENSO", "DESCENSO", "ATERRIZADO" };

EstadoVuelo estado = ESPERA;

bool radioOk = false, bmpOk = false, mpuOk = false;
uint8_t mpuAddr = 0x68;

// Barómetro
float presionSuelo = 101325.0;  // Pa, se fija en la tara
float presionPa    = NAN;
float altFilt      = 0.0;       // Altitud filtrada sobre el suelo (m)
float altMax       = 0.0;

// Acelerómetro
float ax = 0, ay = 0, az = 0;   // m/s^2
float aMagG = 1.0;              // Módulo en g

// DHT11
float temperatura = NAN, humedad = NAN;

// Paracaídas / eventos
bool        paracaidas = false;
const char* motivoDespliegue = "";
uint32_t    tLanzamiento = 0, tQuieto = 0;
uint8_t     cuentaLanz = 0, cuentaApogeo = 0, cuentaBaro = 0;

// Radio
volatile bool flagTx = false;
bool     transmitiendo = false;
uint32_t tInicioTx = 0, paquetesEnviados = 0;

uint32_t tSensores = 0, tTx = 0, tDht = 0, tOled = 0;

void IRAM_ATTR isrTxTerminado() { flagTx = true; }

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
  const uint8_t direcciones[] = { 0x68, 0x69 };  // AD0 a GND / a VCC
  for (uint8_t dir : direcciones) {
    Wire.beginTransmission(dir);
    if (Wire.endTransmission() != 0) continue;
    mpuAddr = dir;

    // WHO_AM_I: 0x71 MPU9250, 0x73 MPU9255, 0x70 MPU6500
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
    mpuEscribir(0x1C, 0x18);              // Acelerómetro +-16 g (el empuje supera 2 g)
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
  for (uint8_t i = 0; i < 6; i++) b[i] = Wire.read();  // Orden de lectura garantizado

  const float ESCALA = 9.80665f / 2048.0f;  // +-16 g -> 2048 LSB/g, salida en m/s^2
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
  const uint8_t chipIds[]     = { 0x58, 0x60 };  // BMP280 / BME280
  for (uint8_t dir : direcciones) {
    for (uint8_t id : chipIds) {
      if (bmp.begin(dir, id)) {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X2,    // Temperatura
                        Adafruit_BMP280::SAMPLING_X8,    // Presión
                        Adafruit_BMP280::FILTER_X4,      // Poco retardo para el apogeo
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

void mostrarMensaje(const char* l1, const char* l2 = "", const char* l3 = "");

// Promedia la presión en reposo y la toma como altitud 0 (nivel del suelo).
void calibrarTara() {
  if (!bmpOk) return;
  mostrarMensaje("CALIBRANDO TARA", "No mover el cohete...");
  Serial.println("# Calibrando tara barometrica...");

  for (uint8_t intento = 0; intento < 3; intento++) {
    for (uint8_t i = 0; i < 10; i++) { bmp.readPressure(); delay(30); }  // Descartar

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
    // 12 Pa ~ 1 m. Si el rango es mayor hubo movimiento o ráfagas: repetir.
    if (pMax - pMin < 12.0f || intento == 2) break;
    Serial.printf("# Tara inestable (%.1f Pa), repitiendo\n", pMax - pMin);
  }

  altFilt = 0.0;
  altMax  = 0.0;
  Serial.printf("# Tara lista: P0 = %.2f Pa\n", presionSuelo);
  mostrarMensaje("TARA LISTA", "Altitud 0 fijada");
  delay(500);
}

// ============================================================================
//  PARACAÍDAS
// ============================================================================
void desplegarParacaidas(const char* motivo) {
  servo.write(SERVO_ABIERTO);
  paracaidas = true;
  motivoDespliegue = motivo;
  estado = DESCENSO;
  tQuieto = 0;
  Serial.printf("# DESPLIEGUE (%s) alt=%.2f max=%.2f t=%lu ms\n",
                motivo, altFilt, altMax, millis() - tLanzamiento);
}

// ============================================================================
//  MÁQUINA DE ESTADOS
// ============================================================================
void actualizarMaquinaEstados(uint32_t ahora) {
  switch (estado) {
    case ESPERA: {
      bool porAcel = mpuOk && aMagG > UMBRAL_LANZAMIENTO_G;
      bool porAlt  = bmpOk && altFilt > UMBRAL_LANZAMIENTO_ALT_M;
      cuentaLanz = (porAcel || porAlt) ? cuentaLanz + 1 : 0;
      if (cuentaLanz >= MUESTRAS_CONFIRMACION) {
        estado = ASCENSO;
        tLanzamiento = ahora;
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

      // Golpe en la rampa: no subió nada, volver a ESPERA.
      if (bmpOk && dt > VENTANA_FALSO_LANZ_MS && altMax < ALT_MIN_VUELO_REAL_M) {
        estado = ESPERA;
        cuentaLanz = 0;
        altMax = 0;
        Serial.println("# Falso lanzamiento, volviendo a ESPERA");
        break;
      }

      // Criterio principal: bajó del máximo - margen Y no hay empuje (cuasi
      // caída libre). Sin MPU se usa solo el barómetro.
      bool bajoMargen = bmpOk && altFilt < altMax - MARGEN_APOGEO_M;
      bool acelCaida  = !mpuOk || aMagG < UMBRAL_CAIDA_G;
      cuentaApogeo = (bajoMargen && acelCaida) ? cuentaApogeo + 1 : 0;

      // Respaldo 1: el barómetro marca una caída clara aunque el
      // acelerómetro no la confirme (p. ej. cohete girando).
      bool caidaClara = bmpOk && altFilt < altMax - MARGEN_APOGEO_SOLO_BARO_M;
      cuentaBaro = caidaClara ? cuentaBaro + 1 : 0;

      if (cuentaApogeo >= MUESTRAS_CONFIRMACION)    desplegarParacaidas("APOGEO");
      else if (cuentaBaro >= MUESTRAS_CONFIRMACION) desplegarParacaidas("BARO");
      else if (dt > TIMEOUT_APOGEO_MS)              desplegarParacaidas("TIMEOUT");  // Respaldo 2
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
      break;  // Se sigue transmitiendo como baliza
  }
}

// ============================================================================
//  LECTURA DE SENSORES
// ============================================================================
void leerSensores(uint32_t ahora) {
  if (mpuOk) mpuLeerAcel();

  if (bmpOk) {
    float p = bmp.readPressure();
    if (!isnan(p) && p > 30000 && p < 110000) {
      presionPa = p;
      float altCruda = altitudDesdePresion(p);
      altFilt = ALFA_FILTRO_ALT * altCruda + (1.0f - ALFA_FILTRO_ALT) * altFilt;
    }
  }

  // El DHT11 bloquea ~25 ms: no se lee durante el ascenso para no retrasar
  // la detección del apogeo. Se conserva el último valor válido.
  if (estado != ASCENSO && ahora - tDht >= PERIODO_DHT_MS) {
    tDht = ahora;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t)) temperatura = t;
    if (!isnan(h)) humedad = h;
  }

  actualizarMaquinaEstados(ahora);
}

// ============================================================================
//  TELEMETRÍA (JSON por LoRa)
// ============================================================================
// Escribe un número o "null" (JSON válido cuando el sensor no tiene dato).
void numeroONull(char* dst, size_t tam, float v, uint8_t decimales) {
  if (isnan(v)) snprintf(dst, tam, "null");
  else          snprintf(dst, tam, "%.*f", decimales, v);
}

void enviarTelemetria(uint32_t ahora) {
  char sTemp[12], sHum[12], sPres[16];
  numeroONull(sTemp, sizeof(sTemp), temperatura, 1);
  numeroONull(sHum,  sizeof(sHum),  humedad, 1);
  numeroONull(sPres, sizeof(sPres), isnan(presionPa) ? NAN : presionPa / 100.0f, 2);

  // Claves compatibles con el dashboard: t (s), alt, temp, pres (hPa), ax/ay/az (m/s^2)
  static char json[220];
  int len = snprintf(json, sizeof(json),
      "{\"n\":%lu,\"t\":%.2f,\"alt\":%.2f,\"altMax\":%.2f,\"temp\":%s,\"hum\":%s,"
      "\"pres\":%s,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"para\":%d,\"est\":\"%s\"}",
      (unsigned long)paquetesEnviados, ahora / 1000.0f, altFilt, altMax, sTemp, sHum,
      sPres, ax, ay, az, paracaidas ? 1 : 0, NOMBRE_ESTADO[estado]);
  if (len <= 0 || len >= (int)sizeof(json)) return;

  Serial.println(json);  // Copia local para pruebas en banco

  if (!radioOk) return;
  // Envío NO bloqueante: el lazo de sensores sigue corriendo mientras el
  // paquete sale al aire (~85 ms).
  int st = radio.startTransmit((uint8_t*)json, len);
  if (st == RADIOLIB_ERR_NONE) {
    transmitiendo = true;
    tInicioTx = ahora;
    paquetesEnviados++;
    digitalWrite(PIN_LED, HIGH);
  } else {
    Serial.printf("# Error TX %d\n", st);
  }
}

void atenderRadio(uint32_t ahora) {
  if (!transmitiendo) return;
  if (flagTx || ahora - tInicioTx > TIMEOUT_TX_MS) {
    flagTx = false;
    radio.finishTransmit();
    transmitiendo = false;
    digitalWrite(PIN_LED, LOW);
  }
}

// ============================================================================
//  OLED
// ============================================================================
void mostrarMensaje(const char* l1, const char* l2, const char* l3) {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, l1);
  display.drawStr(0, 28, l2);
  display.drawStr(0, 44, l3);
  display.sendBuffer();
}

void actualizarOled() {
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
  snprintf(l, sizeof(l), "%s%s%s TX:%lu", radioOk ? "R" : "-", bmpOk ? "B" : "-",
           mpuOk ? "M" : "-", (unsigned long)paquetesEnviados);
  display.drawStr(0, 58, l);

  display.sendBuffer();
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BOTON_PRG, INPUT_PULLUP);

  // 1. Servo primero: trabar el paracaídas lo antes posible
  ESP32PWM::allocateTimer(0);
  servo.setPeriodHertz(50);
  servo.attach(PIN_SERVO, 500, 2400);
  servo.write(SERVO_CERRADO);

  // 2. Encender la OLED (Vext) y arrancarla en el bus Wire1
  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);
  delay(100);
  Wire1.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  display.setBusClock(400000);
  display.begin();
  mostrarMensaje("TELEPATIN - COHETE", "Iniciando...");

  // 3. Bus I2C de sensores (GY-91)
  Wire.begin(PIN_SENSOR_SDA, PIN_SENSOR_SCL, 400000);
  delay(250);
  mpuOk = mpuIniciar();
  bmpOk = bmpIniciar();
  dht.begin();
  Serial.printf("# MPU:%s BMP:%s\n", mpuOk ? "OK" : "FALLO", bmpOk ? "OK" : "FALLO");

  // 4. Radio LoRa. Si falla, el cohete sigue funcionando (el paracaídas no
  //    depende del radio).
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                       LORA_SYNC_WORD, LORA_TX_DBM, LORA_PREAMBULO, LORA_TCXO_V);
  if (st == RADIOLIB_ERR_NONE) {
    radio.setDio2AsRfSwitch(true);
    radio.setPacketSentAction(isrTxTerminado);
    radioOk = true;
  }
  Serial.printf("# LoRa: %s (%d)\n", radioOk ? "OK" : "FALLO", st);

  char l2[32], l3[32];
  snprintf(l2, sizeof(l2), "MPU:%s BMP:%s", mpuOk ? "OK" : "X", bmpOk ? "OK" : "X");
  snprintf(l3, sizeof(l3), "LoRa:%s", radioOk ? "OK" : "FALLO");
  mostrarMensaje("SENSORES", l2, l3);
  delay(1500);

  // 5. Tara: altitud 0 en la rampa
  calibrarTara();

  uint32_t ahora = millis();
  tSensores = tTx = tDht = tOled = ahora;
}

// ============================================================================
//  LOOP (no bloqueante)
// ============================================================================
void loop() {
  uint32_t ahora = millis();

  atenderRadio(ahora);

  if (ahora - tSensores >= PERIODO_SENSORES_MS) {
    tSensores = ahora;
    leerSensores(ahora);
  }

  if (!transmitiendo && ahora - tTx >= PERIODO_TX_MS) {
    tTx = ahora;
    enviarTelemetria(ahora);
  }

  // La OLED no se refresca en ascenso: ahí cada milisegundo cuenta.
  if (estado != ASCENSO && ahora - tOled >= PERIODO_OLED_MS) {
    tOled = ahora;
    actualizarOled();
  }

  // Botón PRG: repetir la tara antes del lanzamiento
  if (estado == ESPERA && digitalRead(PIN_BOTON_PRG) == LOW) {
    delay(50);
    if (digitalRead(PIN_BOTON_PRG) == LOW) {
      calibrarTara();
      while (digitalRead(PIN_BOTON_PRG) == LOW) delay(10);
    }
  }
}
