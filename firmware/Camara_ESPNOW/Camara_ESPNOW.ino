/*
 * ============================================================================
 *  TelePatin - CÁMARA CONTROLADA DESDE EL DASHBOARD (ESP32-CAM + ESP-NOW)
 *  Arduino IDE: placa "AI Thinker ESP32-CAM" (core esp32 3.x)
 * ============================================================================
 *
 *  Al recibir los 5 V queda EN ESPERA (no graba). Desde el dashboard:
 *    "Grabar"  -> crea /cam_001 (o la siguiente libre) y empieza a grabar
 *    "Detener" -> cierra los archivos y vuelve a EN ESPERA
 *  Cada grabación es una carpeta nueva; nunca se borra nada.
 *
 *  Camino de las órdenes (el mismo que "Fijar cero" y "Probar servo"):
 *    Dashboard -> USB -> ESP32 estación -> ESP-NOW -> esta cámara
 *  La cámara responde cada segundo con su estado:
 *    {"tipo":"cam","id":..,"res":..,"rec":1,"dir":"cam_003","seg":12.3,
 *     "img":296,"fps":24.0,"mb":11.8,"libre":28900,"err":0}
 *
 *  Archivos de cada grabación:
 *    /cam_003/video.mjpeg   imágenes JPEG una tras otra (se parte en
 *                           video_2.mjpeg, video_3... cada ~3.9 GB por FAT32)
 *    /cam_003/tiempos.csv   imagen, ms desde que empezó la grabación, bytes
 *    /cam_003/info.txt      resolución, calidad, duración, fps promedio
 *  Si se corta la energía grabando, solo se pierde ~1 s (se asegura cada 1 s).
 *
 *  LED rojo de la placa (GPIO 33):
 *    destello corto cada 2 s = EN ESPERA
 *    parpadeo cada 0.5 s     = GRABANDO
 *    encendido fijo          = ERROR (sin microSD o sin cámara)
 *  Sin microSD, en espera la vuelve a buscar cada 3 s: basta con meterla y
 *  en unos segundos el error desaparece solo, sin pulsar RST.
 *
 *  Conexión en el cohete: 5V -> 5.2 V del LM2596 (tras el interruptor),
 *  GND -> GND común, y un condensador de 470 µF entre 5V y GND junto a la
 *  placa (la radio + la cámara dan picos de corriente).
 *  MicroSD de 32 GB o menos en FAT32.
 * ============================================================================
 */

#include <Arduino.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

// ============================================================================
//  AJUSTES DE GRABACIÓN (la configuración con la que se probó: 800x600, 12)
// ============================================================================
const framesize_t RESOLUCION       = FRAMESIZE_SVGA;  // 800x600
const char*       RESOLUCION_TXT   = "800x600";
const int         CALIDAD_JPEG     = 12;              // 10-63: más bajo = mejor imagen
const uint32_t    FLUSH_MS         = 1000;            // Asegurar datos cada 1 s
const uint32_t    DESCARTAR_INICIO = 5;               // Imágenes al empezar (exposición)
const uint32_t    MAX_BYTES_ARCHIVO = 3900000000UL;   // Límite FAT32 (4 GB) con margen
const uint32_t    REINTENTO_SD_MS  = 3000;            // Sin microSD: volver a buscarla cada 3 s
const bool        GIRAR_180        = false;           // true si va montada boca abajo

// ============================================================================
//  ESP-NOW (DEBE COINCIDIR CON LA ESTACIÓN Y EL COHETE)
// ============================================================================
#define ESPNOW_CANAL      1
const int POTENCIA_TX_DBM = 15;
const uint32_t PERIODO_ESTADO_MS = 1000;
uint8_t DIRECCION_BROADCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ============================================================================
//  PINES DEL ESP32-CAM AI-THINKER (fijos de la placa)
// ============================================================================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define PIN_LED_ROJO      33   // LED pequeño de la placa (se enciende con LOW)
#define PIN_FLASH          4   // LED de flash: siempre apagado

// ============================================================================
//  ESTADO
// ============================================================================
bool camOk = false, sdOk = false, radioOk = false;
uint8_t errorCodigo = 0;          // 0 ok, 1 sin microSD, 2 sin cámara, 3 error al escribir

bool grabando = false;
File archivoVideo, archivoTiempos;
char carpeta[24] = "";
uint8_t  parteVideo = 1;
uint32_t bytesParte = 0;
uint32_t imagenes = 0, descartadas = 0;
uint64_t bytesTotales = 0;
uint32_t tInicioGrab = 0, tFlush = 0, tLed = 0, tEstado = 0, tReintentoSd = 0;

// Órdenes desde el dashboard: "CMD:CAM_ON:<id>" / "CMD:CAM_OFF:<id>"
enum { ORDEN_NINGUNA = 0, ORDEN_GRABAR = 1, ORDEN_DETENER = 2 };
volatile uint8_t  ordenPendiente = ORDEN_NINGUNA;
volatile uint32_t ordenId = 0;
uint32_t ultimoIdOrden = 0;
// Resultado de la última orden: 1 grabación iniciada, 2 detenida,
// 3 ya estaba grabando, 4 no estaba grabando, 5 no se puede grabar (error)
uint8_t resultadoOrden = 0;

void ledRojo(bool encendido) { digitalWrite(PIN_LED_ROJO, encendido ? LOW : HIGH); }

// ============================================================================
//  CÁMARA
// ============================================================================
bool iniciarCamara() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk  = XCLK_GPIO_NUM;
  c.pin_pclk  = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href  = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn  = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = RESOLUCION;
  c.jpeg_quality = CALIDAD_JPEG;
  c.fb_count     = 2;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t e = esp_camera_init(&c);
  if (e != ESP_OK) {
    Serial.printf("esp_camera_init: 0x%x (revisar la cinta de la cámara)\n", e);
    return false;
  }

  // Ajustes para vuelo: exterior, mucha luz, mucho movimiento
  sensor_t* s = esp_camera_sensor_get();
  s->set_brightness(s, 0);
  s->set_contrast(s, 1);
  s->set_saturation(s, 0);
  s->set_special_effect(s, 0);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 1);                    // Sunny
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);
  s->set_ae_level(s, -1);
  s->set_gain_ctrl(s, 1);
  s->set_gainceiling(s, GAINCEILING_4X);
  s->set_bpc(s, 1);
  s->set_wpc(s, 1);
  s->set_raw_gma(s, 1);
  s->set_lenc(s, 1);
  s->set_dcw(s, 1);
  s->set_hmirror(s, GIRAR_180 ? 1 : 0);
  s->set_vflip(s, GIRAR_180 ? 1 : 0);
  return true;
}

// ============================================================================
//  MICROSD Y GRABACIÓN
// ============================================================================
bool iniciarTarjeta() {
  // Modo 1-bit: no usa el flash (GPIO 4) y deja libres los GPIO 12 y 13
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("No se pudo montar la microSD (¿32 GB o menos y FAT32?)");
    return false;
  }
  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("No hay microSD");
    return false;
  }
  return true;
}

// Volver a montar la microSD: sirve para meterla (o cambiarla) sin reiniciar
// la cámara. Solo se llama cuando NO se está grabando.
bool montarTarjetaDeNuevo() {
  bool antes = sdOk;
  SD_MMC.end();
  sdOk = iniciarTarjeta();
  if (sdOk && errorCodigo == 1) errorCodigo = 0;      // Se resolvió el "sin microSD"
  if (!sdOk && errorCodigo == 0) errorCodigo = 1;
  if (sdOk && !antes) {
    Serial.printf("microSD detectada (%llu MB)\n", SD_MMC.cardSize() / (1024 * 1024));
    tEstado = 0;                                       // Avisar al dashboard de inmediato
  }
  return sdOk;
}

uint32_t mbLibres() {
  if (!sdOk) return 0;
  return (uint32_t)((SD_MMC.totalBytes() - SD_MMC.usedBytes()) / (1024 * 1024));
}

bool abrirParteVideo() {
  char ruta[48];
  if (parteVideo == 1) snprintf(ruta, sizeof(ruta), "%s/video.mjpeg", carpeta);
  else                 snprintf(ruta, sizeof(ruta), "%s/video_%u.mjpeg", carpeta, parteVideo);
  archivoVideo = SD_MMC.open(ruta, FILE_WRITE);
  bytesParte = 0;
  return (bool)archivoVideo;
}

// info.txt: se escribe al empezar y se reescribe al terminar con los totales
void escribirInfo(bool final) {
  char ruta[48];
  snprintf(ruta, sizeof(ruta), "%s/info.txt", carpeta);
  File f = SD_MMC.open(ruta, FILE_WRITE);
  if (!f) return;
  float seg = (millis() - tInicioGrab) / 1000.0f;
  f.printf("carpeta=%s\n", carpeta + 1);
  f.printf("resolucion=%s\n", RESOLUCION_TXT);
  f.printf("calidad_jpeg=%d\n", CALIDAD_JPEG);
  f.printf("inicio_ms_desde_encendido=%lu\n", (unsigned long)tInicioGrab);
  f.printf("estado=%s\n", final ? "completa" : "grabando (si queda asi, se corto la energia)");
  if (final) {
    f.printf("duracion_s=%.2f\n", seg);
    f.printf("imagenes=%lu\n", (unsigned long)imagenes);
    f.printf("fps_promedio=%.2f\n", seg > 0 ? imagenes / seg : 0.0f);
    f.printf("partes_video=%u\n", parteVideo);
    f.printf("megabytes=%.1f\n", bytesTotales / 1048576.0);
  }
  f.close();
}

// Crea la siguiente carpeta libre (/cam_001, /cam_002...)
bool crearCarpeta() {
  for (int i = 1; i < 1000; i++) {
    snprintf(carpeta, sizeof(carpeta), "/cam_%03d", i);
    if (!SD_MMC.exists(carpeta)) break;
  }
  return SD_MMC.mkdir(carpeta);
}

uint8_t iniciarGrabacion() {
  if (grabando) return 3;
  if (!camOk) return 5;
  if (!sdOk) montarTarjetaDeNuevo();       // ¿La acaban de meter?
  if (!sdOk) return 5;

  // Si falla, puede que la hayan sacado y vuelto a meter: remontar y reintentar
  if (!crearCarpeta()) {
    if (!montarTarjetaDeNuevo() || !crearCarpeta()) { errorCodigo = 3; return 5; }
  }

  parteVideo = 1;
  char ruta[48];
  snprintf(ruta, sizeof(ruta), "%s/tiempos.csv", carpeta);
  archivoTiempos = SD_MMC.open(ruta, FILE_WRITE);
  if (!abrirParteVideo() || !archivoTiempos) { errorCodigo = 3; return 5; }
  archivoTiempos.println("imagen,ms,bytes");

  imagenes = descartadas = 0;
  bytesTotales = 0;
  tInicioGrab = tFlush = millis();
  escribirInfo(false);
  grabando = true;
  errorCodigo = 0;
  Serial.printf("GRABANDO en %s\n", carpeta);
  return 1;
}

uint8_t detenerGrabacion() {
  if (!grabando) return 4;
  grabando = false;
  archivoVideo.flush();   archivoVideo.close();
  archivoTiempos.flush(); archivoTiempos.close();
  escribirInfo(true);
  Serial.printf("DETENIDA %s: %lu imagenes\n", carpeta, (unsigned long)imagenes);
  return 2;
}

void grabarImagen() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;

  // Las primeras imágenes pueden salir mal expuestas
  if (descartadas < DESCARTAR_INICIO) {
    descartadas++;
    esp_camera_fb_return(fb);
    return;
  }

  // Archivo cerca del límite de FAT32: seguir en la parte siguiente
  if (bytesParte + fb->len > MAX_BYTES_ARCHIVO) {
    archivoVideo.flush();
    archivoVideo.close();
    parteVideo++;
    if (!abrirParteVideo()) {
      esp_camera_fb_return(fb);
      errorCodigo = 3;
      detenerGrabacion();
      return;
    }
  }

  uint32_t ms = millis() - tInicioGrab;
  size_t largo = fb->len;
  size_t escritos = archivoVideo.write(fb->buf, largo);
  esp_camera_fb_return(fb);

  if (escritos != largo) {
    // Tarjeta llena, retirada o dañada
    Serial.println("Error escribiendo en la microSD");
    errorCodigo = 3;
    detenerGrabacion();
    return;
  }

  imagenes++;
  bytesParte += largo;
  bytesTotales += largo;
  archivoTiempos.printf("%lu,%lu,%u\n", (unsigned long)imagenes, (unsigned long)ms, (unsigned)largo);

  if (millis() - tFlush >= FLUSH_MS) {
    tFlush = millis();
    archivoVideo.flush();
    archivoTiempos.flush();
  }
}

// ============================================================================
//  ESP-NOW
// ============================================================================
// Se ejecuta en la tarea WiFi: solo copiar la orden y levantar la bandera.
// Ignora todo lo demás (telemetría del cohete, órdenes de tara o servo).
void alRecibir(const esp_now_recv_info_t* info, const uint8_t* datos, int len) {
  if (len < 12 || len > 24 || memcmp(datos, "CMD:CAM_", 8) != 0) return;
  char txt[25];
  memcpy(txt, datos, len);
  txt[len] = '\0';
  if (strncmp(txt + 8, "ON:", 3) == 0) {
    ordenId = strtoul(txt + 11, nullptr, 10);
    ordenPendiente = ORDEN_GRABAR;
  } else if (strncmp(txt + 8, "OFF:", 4) == 0) {
    ordenId = strtoul(txt + 12, nullptr, 10);
    ordenPendiente = ORDEN_DETENER;
  }
}

bool espNowIniciar() {
  WiFi.mode(WIFI_STA);
  uint32_t t0 = millis();
  while (!WiFi.STA.started() && millis() - t0 < 3000) delay(10);
  WiFi.setChannel(ESPNOW_CANAL);
  esp_wifi_set_max_tx_power(constrain(POTENCIA_TX_DBM, 2, 21) * 4);

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(alRecibir);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, DIRECCION_BROADCAST, 6);
  peer.channel = 0;
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

void enviarEstado() {
  float seg = grabando ? (millis() - tInicioGrab) / 1000.0f : 0;
  float fps = (grabando && seg > 1) ? imagenes / seg : 0;
  char msg[200];
  int len = snprintf(msg, sizeof(msg),
      "{\"tipo\":\"cam\",\"id\":%lu,\"res\":%u,\"rec\":%d,\"dir\":\"%s\",\"seg\":%.1f,"
      "\"img\":%lu,\"fps\":%.1f,\"mb\":%.1f,\"libre\":%lu,\"err\":%u,\"resol\":\"%s\"}",
      (unsigned long)ultimoIdOrden, resultadoOrden, grabando ? 1 : 0,
      carpeta[0] ? carpeta + 1 : "", seg, (unsigned long)imagenes, fps,
      bytesTotales / 1048576.0, (unsigned long)mbLibres(), errorCodigo, RESOLUCION_TXT);
  if (len <= 0 || len >= (int)sizeof(msg)) return;
  Serial.println(msg);
  if (radioOk) esp_now_send(DIRECCION_BROADCAST, (const uint8_t*)msg, len);
}

void atenderOrdenes() {
  uint8_t orden = ordenPendiente;
  if (orden == ORDEN_NINGUNA) return;
  ordenPendiente = ORDEN_NINGUNA;
  uint32_t id = ordenId;
  if (id == ultimoIdOrden) return;   // La estación repite cada orden varias veces
  ultimoIdOrden = id;
  resultadoOrden = (orden == ORDEN_GRABAR) ? iniciarGrabacion() : detenerGrabacion();
  Serial.printf("Orden %s id=%lu -> resultado %u\n", orden == ORDEN_GRABAR ? "GRABAR" : "DETENER",
                (unsigned long)id, resultadoOrden);
  tEstado = 0;  // Confirmar de inmediato
}

// ============================================================================
//  LED
// ============================================================================
void actualizarLed(uint32_t ahora) {
  if (errorCodigo == 1 || errorCodigo == 2) { ledRojo(true); return; }      // Error fijo
  if (grabando) { ledRojo((ahora / 500) % 2); return; }                     // Cada 0.5 s
  ledRojo((ahora % 2000) < 80);                                             // Destello cada 2 s
}

// ============================================================================
//  SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_ROJO, OUTPUT);
  ledRojo(false);
  pinMode(PIN_FLASH, OUTPUT);
  digitalWrite(PIN_FLASH, LOW);
  Serial.println("\nTELEPATIN - CAMARA ESP-NOW (800x600, calidad 12)");

  if (!psramFound()) Serial.println("AVISO: no se detecta la PSRAM");
  camOk = iniciarCamara();
  sdOk  = iniciarTarjeta();
  if (!sdOk)  errorCodigo = 1;
  if (!camOk) errorCodigo = 2;
  digitalWrite(PIN_FLASH, LOW);

  radioOk = espNowIniciar();
  Serial.printf("Camara:%s  microSD:%s (%lu MB libres)  ESP-NOW:%s\n",
                camOk ? "OK" : "FALLO", sdOk ? "OK" : "FALLO", (unsigned long)mbLibres(),
                radioOk ? "OK" : "FALLO");
  Serial.println("EN ESPERA: pulsa \"Grabar\" en el dashboard");
}

void loop() {
  uint32_t ahora = millis();

  atenderOrdenes();
  if (grabando) grabarImagen();
  else delay(5);  // En espera no hace falta correr

  // Sin microSD y en espera: buscarla de nuevo cada pocos segundos, para que
  // al meterla funcione sin pulsar RST
  if (!grabando && !sdOk && millis() - tReintentoSd >= REINTENTO_SD_MS) {
    tReintentoSd = millis();
    montarTarjetaDeNuevo();
  }

  if (millis() - tEstado >= PERIODO_ESTADO_MS) {
    tEstado = millis();
    enviarEstado();
  }
  actualizarLed(ahora);
}
