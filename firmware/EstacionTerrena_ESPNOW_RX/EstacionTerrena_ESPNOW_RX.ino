/*
 * ============================================================================
 *  TelePatin - ESTACIÓN TERRENA (RECEPTOR ESP-NOW)
 *  Placa: ESP32 DevKit (WROOM) o Heltec WiFi LoRa 32 (V3)
 *  Arduino IDE: "ESP32 Dev Module" o "Heltec WiFi LoRa 32(V3)" (core esp32 3.x)
 * ============================================================================
 *
 *  Pareja de Cohete_ESPNOW_TX. Recibe por ESP-NOW y:
 *  - Reenvía cada JSON EXACTAMENTE como llega, una línea por paquete, por
 *    Serial a 115200 baudios (lo lee el dashboard web vía Web Serial).
 *  - En la Heltec, además, muestra altitud, estado, paracaídas, GPS y RSSI en
 *    la OLED. En el ESP32 DevKit (sin pantalla) el LED cambia con cada paquete.
 *
 *  - Recibe órdenes de la página por Serial ("TARA <id>" = botón "Fijar
 *    cero", "SERVO <id>" = botón "Probar servo") y las reenvía al cohete
 *    por ESP-NOW.
 *
 *  Las líneas de diagnóstico propias empiezan con '#'.
 *  Librerías: ESP-NOW viene en el core. U8g2 (olikraus) solo para la Heltec.
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

// WIFI_LoRa_32_V3 lo define el core al elegir la placa Heltec V3
#if defined(WIFI_LoRa_32_V3)
  #define ES_HELTEC 1
  #include <Wire.h>
  #include <U8g2lib.h>
  #define PIN_OLED_SDA 17
  #define PIN_OLED_SCL 18
  #define PIN_OLED_RST 21
  #define PIN_VEXT     36
  #define PIN_LED      35
#else
  #define ES_HELTEC 0
  #define PIN_LED      2    // LED azul de la mayoría de DevKit ESP32
#endif

// ============================================================================
//  CONFIGURACIÓN ESP-NOW  (DEBE SER IDÉNTICA A LA DEL COHETE)
// ============================================================================
#define ESPNOW_CANAL      1
#define USAR_LONG_RANGE   0   // 1 = modo LR. Activarlo SOLO cuando el enlace ya funcione, y en ambos lados.

const uint32_t PERIODO_OLED_MS = 250;
const uint32_t SIN_SENAL_MS    = 2000;
const uint32_t AVISO_SERIAL_MS = 5000;  // Aviso "# sin senal" por Serial
const uint32_t PERIODO_ENLACE_MS = 1000; // Línea "#ENLACE {...}" para el panel de la página

// ============================================================================
//  OBJETOS Y ESTADO
// ============================================================================
#if ES_HELTEC
U8G2_SSD1306_128X64_NONAME_F_2ND_HW_I2C display(U8G2_R0, PIN_OLED_RST);
#endif

// Los paquetes llegan en la tarea WiFi; se pasan al loop() por una cola.
struct Paquete {
  uint8_t len;
  int8_t  rssi;
  char    datos[ESP_NOW_MAX_DATA_LEN + 1];
};
QueueHandle_t colaRx;

bool radioOk = false;
uint8_t canalReal = 0;
uint8_t DIRECCION_BROADCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// Órdenes de la página ("Fijar cero", "Probar servo")
const uint8_t  CMD_REPETICIONES = 5;
const uint32_t CMD_PERIODO_MS   = 80;
const char*    cmdNombre = "TARA";
char     lineaCmd[32];
uint8_t  lenCmd = 0, cmdRepeticiones = 0;
uint32_t cmdId = 0, tCmd = 0;
volatile uint32_t descartados = 0;  // Tramas ESP-NOW que no eran nuestro JSON

uint32_t paquetesOk = 0, tUltimoPaquete = 0, tOled = 0, tAviso = 0, tEnlace = 0;
float    rssi = 0;

// Últimos valores extraídos para la pantalla
float altActual = NAN, altMaxima = NAN, sats = NAN, lat = NAN;
int   estadoPara = -1;
char  estadoVuelo[12] = "---";

// ============================================================================
//  RECEPCIÓN ESP-NOW (se ejecuta en la tarea WiFi: solo copiar y encolar)
// ============================================================================
void alRecibir(const esp_now_recv_info_t* info, const uint8_t* datos, int len) {
  if (len <= 0 || len > ESP_NOW_MAX_DATA_LEN || datos[0] != '{') { descartados++; return; }
  Paquete p;
  p.len  = (uint8_t)len;
  p.rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
  memcpy(p.datos, datos, len);
  p.datos[len] = '\0';
  xQueueSend(colaRx, &p, 0);  // Si la cola está llena, se descarta
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

  wifi_second_chan_t sec;
  esp_wifi_get_channel(&canalReal, &sec);
  Serial.printf("# WiFi STA %s, canal real %u (esperado %u), LR=%d\n",
                WiFi.STA.started() ? "OK" : "SIN ARRANCAR", canalReal, ESPNOW_CANAL, USAR_LONG_RANGE);

  esp_err_t e = esp_now_init();
  if (e != ESP_OK) { Serial.printf("# esp_now_init: %s\n", esp_err_to_name(e)); return false; }
  if (esp_now_register_recv_cb(alRecibir) != ESP_OK) return false;

  // Par broadcast para poder enviar órdenes al cohete ("Fijar cero")
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, DIRECCION_BROADCAST, 6);
  peer.channel = 0;  // 0 = el canal actual del WiFi
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

// ============================================================================
//  ÓRDENES DESDE LA PÁGINA  (línea "TARA <id>" o "SERVO <id>" por Serial)
// ============================================================================
// ESP-NOW broadcast no confirma la entrega, así que la orden se repite unas
// veces; el cohete la atiende una sola vez gracias al id.
void atenderPagina(uint32_t ahora) {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      lineaCmd[lenCmd] = '\0';
      const char* nombre = nullptr;
      if (strncmp(lineaCmd, "TARA", 4) == 0)       nombre = "TARA";
      else if (strncmp(lineaCmd, "SERVO", 5) == 0) nombre = "SERVO";
      if (nombre) {
        cmdNombre = nombre;
        cmdId = strtoul(lineaCmd + strlen(nombre), nullptr, 10);
        cmdRepeticiones = CMD_REPETICIONES;
        tCmd = 0;
        Serial.printf("#CMD %s %lu enviando al cohete\n", cmdNombre, (unsigned long)cmdId);
      }
      lenCmd = 0;
    } else if (lenCmd < sizeof(lineaCmd) - 1) {
      lineaCmd[lenCmd++] = c;
    }
  }

  if (cmdRepeticiones > 0 && ahora - tCmd >= CMD_PERIODO_MS) {
    tCmd = ahora;
    cmdRepeticiones--;
    char msg[24];
    int len = snprintf(msg, sizeof(msg), "CMD:%s:%lu", cmdNombre, (unsigned long)cmdId);
    if (radioOk) esp_now_send(DIRECCION_BROADCAST, (const uint8_t*)msg, len);
  }
}

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
//  OLED (solo Heltec)
// ============================================================================
#if ES_HELTEC
void mostrarMensaje(const char* l1, const char* l2) {
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

  snprintf(l, sizeof(l), "TIERRA ESPNOW #%lu", (unsigned long)paquetesOk);
  display.drawStr(0, 10, l);

  if (!radioOk) {
    display.drawStr(0, 30, "ERROR ESP-NOW");
    display.sendBuffer();
    return;
  }

  if (paquetesOk == 0) {
    display.drawStr(0, 30, "Esperando cohete...");
  } else {
    display.setFont(u8g2_font_helvB12_tf);
    if (isnan(altActual)) snprintf(l, sizeof(l), "ALT --- m");
    else                  snprintf(l, sizeof(l), "ALT %.1f m", altActual);
    display.drawStr(0, 26, l);

    display.setFont(u8g2_font_6x10_tf);
    snprintf(l, sizeof(l), "Max %.1f m  %s", isnan(altMaxima) ? 0.0f : altMaxima, estadoVuelo);
    display.drawStr(0, 37, l);
    snprintf(l, sizeof(l), "Para:%s GPS:%s",
             estadoPara == 1 ? "FUERA" : (estadoPara == 0 ? "TRABADO" : "?"),
             !isnan(lat) ? "OK" : "--");
    display.drawStr(0, 48, l);
  }

  uint32_t silencio = ahora - tUltimoPaquete;
  if (paquetesOk > 0 && silencio > SIN_SENAL_MS) {
    snprintf(l, sizeof(l), "SIN SENAL hace %lus", (unsigned long)(silencio / 1000));
  } else {
    snprintf(l, sizeof(l), "RSSI %.0f dBm  Sat %d", rssi, isnan(sats) ? 0 : (int)sats);
  }
  display.drawStr(0, 62, l);

  display.sendBuffer();
}
#endif

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);

#if ES_HELTEC
  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);  // Enciende la OLED
  delay(100);
  Wire1.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  display.setBusClock(400000);
  display.begin();
  mostrarMensaje("TELEPATIN - TIERRA", "Iniciando ESP-NOW...");
#endif

  colaRx = xQueueCreate(8, sizeof(Paquete));
  radioOk = espNowIniciar();
  Serial.printf("# ESP-NOW RX: %s  MAC %s\n", radioOk ? "OK" : "FALLO", WiFi.macAddress().c_str());
  tAviso = millis();
}

// ============================================================================
//  LOOP
// ============================================================================
void loop() {
  uint32_t ahora = millis();

  atenderPagina(ahora);

  Paquete p;
  while (xQueueReceive(colaRx, &p, 0) == pdTRUE) {
    // Reenvío EXACTO del JSON recibido + salto de línea
    Serial.write((const uint8_t*)p.datos, p.len);
    Serial.write('\n');

    rssi = p.rssi;
    paquetesOk++;
    tUltimoPaquete = ahora;

    float v;
    if (jsonNumero(p.datos, "alt", v))    altActual = v;
    if (jsonNumero(p.datos, "altMax", v)) altMaxima = v;
    if (jsonNumero(p.datos, "para", v))   estadoPara = (int)v;
    if (jsonNumero(p.datos, "sats", v))   sats = v;
    lat = jsonNumero(p.datos, "lat", v) ? v : NAN;
    jsonTexto(p.datos, "est", estadoVuelo, sizeof(estadoVuelo));

    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }

  // Sin pantalla, el diagnóstico va por Serial (el dashboard ignora '#')
  if (ahora - tUltimoPaquete > AVISO_SERIAL_MS && ahora - tAviso >= AVISO_SERIAL_MS) {
    tAviso = ahora;
    Serial.printf("# sin senal (%lu paquetes recibidos)\n", (unsigned long)paquetesOk);
  }

  // Estado del enlace para el panel "Prueba de enlace" de la página. Empieza
  // por '#', así que no se mezcla con los datos de vuelo.
  if (ahora - tEnlace >= PERIODO_ENLACE_MS) {
    tEnlace = ahora;
    if (paquetesOk > 0) {
      Serial.printf("#ENLACE {\"radio\":%d,\"canal\":%u,\"rx\":%lu,\"rssi\":%.0f,\"silencio_ms\":%lu,\"descartados\":%lu}\n",
                    radioOk ? 1 : 0, canalReal, (unsigned long)paquetesOk, rssi,
                    (unsigned long)(ahora - tUltimoPaquete), (unsigned long)descartados);
    } else {
      Serial.printf("#ENLACE {\"radio\":%d,\"canal\":%u,\"rx\":0,\"rssi\":null,\"silencio_ms\":null,\"descartados\":%lu}\n",
                    radioOk ? 1 : 0, canalReal, (unsigned long)descartados);
    }
  }

#if ES_HELTEC
  if (ahora - tOled >= PERIODO_OLED_MS) {
    tOled = ahora;
    actualizarOled(ahora);
  }
#endif
}
