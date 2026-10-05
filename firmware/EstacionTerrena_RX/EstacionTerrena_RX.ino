/*
 * ============================================================================
 *  TelePatin - ESTACIÓN TERRENA (RECEPTOR LoRa)
 *  Placa: Heltec WiFi LoRa 32 (V3)  ->  ESP32-S3 + SX1262
 *  Arduino IDE: Placa "Heltec WiFi LoRa 32(V3)" (core esp32 de Espressif)
 * ============================================================================
 *
 *  - Recibe los paquetes LoRa del cohete.
 *  - Reenvía cada JSON EXACTAMENTE como llega, una línea por paquete, por
 *    Serial a 115200 baudios (lo lee el dashboard web vía Web Serial).
 *  - Muestra altitud, estado, paracaídas y RSSI en la OLED integrada.
 *
 *  Las líneas de diagnóstico propias empiezan con '#', para que el dashboard
 *  las ignore.
 *
 *  Librerías: RadioLib (jgromes), U8g2 (olikraus).
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RadioLib.h>
#include <U8g2lib.h>

// ============================================================================
//  PINES INTERNOS DE LA HELTEC V3
// ============================================================================
#define PIN_OLED_SDA   17
#define PIN_OLED_SCL   18
#define PIN_OLED_RST   21
#define PIN_VEXT       36
#define PIN_LED        35

#define LORA_NSS       8
#define LORA_SCK       9
#define LORA_MOSI      10
#define LORA_MISO      11
#define LORA_RST       12
#define LORA_BUSY      13
#define LORA_DIO1      14

// ============================================================================
//  CONFIGURACIÓN LoRa  (DEBE SER IDÉNTICA A LA DEL COHETE)
// ============================================================================
#define LORA_FREQ_MHZ   915.0
#define LORA_BW_KHZ     500.0
#define LORA_SF         7
#define LORA_CR         5
#define LORA_SYNC_WORD  0x12
#define LORA_TX_DBM     17
#define LORA_PREAMBULO  8
#define LORA_TCXO_V     1.8

const uint32_t PERIODO_OLED_MS     = 250;
const uint32_t SIN_SENAL_MS        = 2000;  // Aviso si no llega nada en este tiempo

// ============================================================================
//  OBJETOS Y ESTADO
// ============================================================================
U8G2_SSD1306_128X64_NONAME_F_2ND_HW_I2C display(U8G2_R0, PIN_OLED_RST);
// Alternativa por software si la pantalla no enciende:
// U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, PIN_OLED_SCL, PIN_OLED_SDA, PIN_OLED_RST);

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

volatile bool flagRx = false;
bool radioOk = false;

char     paquete[256];
uint32_t paquetesOk = 0, erroresCrc = 0, tUltimoPaquete = 0, tOled = 0;
float    rssi = 0, snr = 0;

// Últimos valores extraídos para la pantalla
float altActual = NAN, altMaxima = NAN;
int   estadoPara = -1;
char  estadoVuelo[12] = "---";

void IRAM_ATTR isrRx() { flagRx = true; }

// ============================================================================
//  EXTRACCIÓN SIMPLE DE CAMPOS JSON (solo para la OLED)
// ============================================================================
bool jsonNumero(const char* json, const char* clave, float& out) {
  char patron[20];
  snprintf(patron, sizeof(patron), "\"%s\":", clave);
  const char* p = strstr(json, patron);
  if (!p) return false;
  p += strlen(patron);
  if (strncmp(p, "null", 4) == 0) return false;
  out = atof(p);
  return true;
}

bool jsonTexto(const char* json, const char* clave, char* out, size_t tam) {
  char patron[20];
  snprintf(patron, sizeof(patron), "\"%s\":\"", clave);
  const char* p = strstr(json, patron);
  if (!p) return false;
  p += strlen(patron);
  const char* fin = strchr(p, '"');
  if (!fin) return false;
  size_t n = min((size_t)(fin - p), tam - 1);
  memcpy(out, p, n);
  out[n] = '\0';
  return true;
}

// ============================================================================
//  OLED
// ============================================================================
void mostrarMensaje(const char* l1, const char* l2 = "") {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, l1);
  display.drawStr(0, 28, l2);
  display.sendBuffer();
}

void actualizarOled(uint32_t ahora) {
  char l[32];
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);

  snprintf(l, sizeof(l), "EST. TERRENA  #%lu", (unsigned long)paquetesOk);
  display.drawStr(0, 10, l);

  if (!radioOk) {
    display.drawStr(0, 30, "ERROR DE RADIO LoRa");
    display.sendBuffer();
    return;
  }

  if (paquetesOk == 0) {
    display.drawStr(0, 30, "Esperando cohete...");
  } else {
    display.setFont(u8g2_font_helvB12_tf);
    if (isnan(altActual)) snprintf(l, sizeof(l), "ALT --- m");
    else                  snprintf(l, sizeof(l), "ALT %.1f m", altActual);
    display.drawStr(0, 27, l);

    display.setFont(u8g2_font_6x10_tf);
    snprintf(l, sizeof(l), "Max %.1f m  %s", isnan(altMaxima) ? 0.0f : altMaxima, estadoVuelo);
    display.drawStr(0, 39, l);
    snprintf(l, sizeof(l), "Paracaidas: %s",
             estadoPara == 1 ? "DESPLEGADO" : (estadoPara == 0 ? "TRABADO" : "?"));
    display.drawStr(0, 50, l);
  }

  uint32_t silencio = ahora - tUltimoPaquete;
  if (paquetesOk > 0 && silencio > SIN_SENAL_MS) {
    snprintf(l, sizeof(l), "SIN SENAL hace %lus", (unsigned long)(silencio / 1000));
  } else {
    snprintf(l, sizeof(l), "RSSI %.0f SNR %.1f", rssi, snr);
  }
  display.drawStr(0, 62, l);

  display.sendBuffer();
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);

  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);  // Enciende la OLED
  delay(100);
  Wire1.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  display.setBusClock(400000);
  display.begin();
  mostrarMensaje("TELEPATIN - TIERRA", "Iniciando LoRa...");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                       LORA_SYNC_WORD, LORA_TX_DBM, LORA_PREAMBULO, LORA_TCXO_V);
  if (st == RADIOLIB_ERR_NONE) {
    radio.setDio2AsRfSwitch(true);
    radio.setPacketReceivedAction(isrRx);
    st = radio.startReceive();
    radioOk = (st == RADIOLIB_ERR_NONE);
  }
  Serial.printf("# LoRa RX: %s (%d)\n", radioOk ? "OK" : "FALLO", st);
}

// ============================================================================
//  LOOP
// ============================================================================
void loop() {
  uint32_t ahora = millis();

  if (flagRx) {
    flagRx = false;

    size_t len = radio.getPacketLength();
    if (len > sizeof(paquete) - 1) len = sizeof(paquete) - 1;
    int st = radio.readData((uint8_t*)paquete, len);

    if (st == RADIOLIB_ERR_NONE && len > 0) {
      paquete[len] = '\0';

      // Reenvío EXACTO del JSON recibido + salto de línea
      Serial.write((const uint8_t*)paquete, len);
      Serial.write('\n');

      rssi = radio.getRSSI();
      snr  = radio.getSNR();
      paquetesOk++;
      tUltimoPaquete = ahora;

      float v;
      if (jsonNumero(paquete, "alt", v))    altActual = v;
      if (jsonNumero(paquete, "altMax", v)) altMaxima = v;
      if (jsonNumero(paquete, "para", v))   estadoPara = (int)v;
      jsonTexto(paquete, "est", estadoVuelo, sizeof(estadoVuelo));

      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    } else if (st == RADIOLIB_ERR_CRC_MISMATCH) {
      erroresCrc++;  // Paquete corrupto: se descarta, no llega al dashboard
    }

    radio.startReceive();
  }

  if (ahora - tOled >= PERIODO_OLED_MS) {
    tOled = ahora;
    actualizarOled(ahora);
  }
}
