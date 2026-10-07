/*
 * ============================================================================
 *  TelePatin - CÁMARA GRABADORA 1280x720 (ESP32-CAM AI-Thinker + OV2640)
 *  Arduino IDE: placa "AI Thinker ESP32-CAM" (core esp32 3.x)
 * ============================================================================
 *
 *  Graba el vuelo en la microSD, SIN WiFi ni internet. Funciona sola: al
 *  encenderse empieza a grabar y no depende de la aviónica, así que si la
 *  cámara falla, la telemetría y el paracaídas no se enteran.
 *
 *  Cada encendido crea una carpeta nueva (nunca borra vuelos anteriores):
 *    /hd_001/video.mjpeg   imágenes JPEG una tras otra (video MJPEG)
 *    /hd_001/tiempos.csv   imagen, milisegundos desde el arranque, bytes
 *
 *  ¿Por qué MJPEG y no AVI/MP4? Si la batería se desconecta de golpe (al
 *  aterrizar, al apagar), un AVI queda dañado porque su cabecera se escribe
 *  al final. El MJPEG se puede leer hasta la última imagen guardada.
 *  Los datos se aseguran en la tarjeta cada segundo.
 *
 *  Ver el video en el PC:
 *    - VLC: abrir video.mjpeg directamente.
 *    - Convertir a MP4 con ffmpeg (usar los fps que muestra el Monitor Serie):
 *        ffmpeg -framerate 12 -i video.mjpeg -c:v libx264 -pix_fmt yuv420p vuelo.mp4
 *
 *  LED rojo pequeño de la placa (GPIO 33):
 *    parpadeo lento (1 s)  = grabando
 *    encendido fijo        = ERROR (sin cámara o sin microSD). Ver Monitor Serie.
 *
 *  Alimentación: 5 V (en el cohete, del LM2596 tras el interruptor) y GND
 *  común. MicroSD de 32 GB o menos en FAT32.
 * ============================================================================
 */

#include <Arduino.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

// ============================================================================
//  AJUSTES DE GRABACIÓN
// ============================================================================
const framesize_t RESOLUCION    = FRAMESIZE_HD;  // 1280x720, ~12-15 fps aprox.
const int         CALIDAD_JPEG  = 10;              // 10-63: más bajo = mejor imagen, más pesado
const uint32_t    FLUSH_MS      = 1000;            // Asegurar datos en la tarjeta cada 1 s
const uint32_t    DESCARTAR_INICIO = 15;           // Imágenes iniciales mientras se ajusta la exposición
const bool        GIRAR_180     = false;           // true si la cámara va montada boca abajo

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
File archivoVideo, archivoTiempos;
char carpeta[24];
uint32_t imagenes = 0, descartadas = 0, bytesTotales = 0;
uint32_t tFlush = 0, tLed = 0, tInfo = 0, tInicio = 0;

void ledRojo(bool encendido) { digitalWrite(PIN_LED_ROJO, encendido ? LOW : HIGH); }

// Error irrecuperable: LED fijo y se queda esperando (reiniciar para reintentar)
void fallar(const char* motivo) {
  Serial.printf("ERROR: %s\n", motivo);
  ledRojo(true);
  while (true) delay(1000);
}

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
  c.fb_count     = 2;                      // Doble búfer: captura mientras se escribe
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;     // Siempre la imagen más reciente

  esp_err_t e = esp_camera_init(&c);
  if (e != ESP_OK) {
    Serial.printf("esp_camera_init: 0x%x (revisar la cinta de la cámara)\n", e);
    return false;
  }

  // Ajustes recomendados para vuelo: exterior, mucha luz, mucho movimiento
  sensor_t* s = esp_camera_sensor_get();
  s->set_brightness(s, 0);
  s->set_contrast(s, 1);
  s->set_saturation(s, 0);
  s->set_special_effect(s, 0);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 1);                    // 1 = Sunny: colores estables al aire libre
  s->set_exposure_ctrl(s, 1);              // Exposición automática
  s->set_aec2(s, 1);
  s->set_ae_level(s, -1);                  // Cielo brillante: un poco más oscuro
  s->set_gain_ctrl(s, 1);
  s->set_gainceiling(s, GAINCEILING_4X);   // Menos ruido y exposiciones más cortas
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
//  MICROSD
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
  Serial.printf("microSD: %llu MB libres de %llu MB\n",
                (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / (1024 * 1024),
                SD_MMC.cardSize() / (1024 * 1024));

  // Primera carpeta libre: /vuelo_001, /vuelo_002, ...
  for (int i = 1; i < 1000; i++) {
    snprintf(carpeta, sizeof(carpeta), "/hd_%03d", i);
    if (!SD_MMC.exists(carpeta)) break;
  }
  if (!SD_MMC.mkdir(carpeta)) {
    Serial.printf("No se pudo crear %s\n", carpeta);
    return false;
  }

  char ruta[48];
  snprintf(ruta, sizeof(ruta), "%s/video.mjpeg", carpeta);
  archivoVideo = SD_MMC.open(ruta, FILE_WRITE);
  snprintf(ruta, sizeof(ruta), "%s/tiempos.csv", carpeta);
  archivoTiempos = SD_MMC.open(ruta, FILE_WRITE);
  if (!archivoVideo || !archivoTiempos) {
    Serial.println("No se pudieron crear los archivos");
    return false;
  }
  archivoTiempos.println("imagen,ms,bytes");
  Serial.printf("Grabando en %s\n", carpeta);
  return true;
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_ROJO, OUTPUT);
  ledRojo(false);
  pinMode(PIN_FLASH, OUTPUT);
  digitalWrite(PIN_FLASH, LOW);
  Serial.println("\nTELEPATIN - CAMARA GRABADORA 1280x720 calidad 10 (sin WiFi)");

  if (!psramFound()) fallar("no se detecta la PSRAM");
  if (!iniciarCamara()) fallar("camara");
  if (!iniciarTarjeta()) fallar("microSD");

  // La microSD en modo 1-bit no usa el GPIO 4: asegurar el flash apagado
  digitalWrite(PIN_FLASH, LOW);
  tInicio = tFlush = tInfo = millis();
}

// ============================================================================
//  LOOP
// ============================================================================
void loop() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Captura fallida");
    delay(10);
    return;
  }

  // Las primeras imágenes salen mal expuestas mientras el sensor se ajusta
  if (descartadas < DESCARTAR_INICIO) {
    descartadas++;
    esp_camera_fb_return(fb);
    return;
  }

  uint32_t ahora = millis();
  size_t escritos = archivoVideo.write(fb->buf, fb->len);
  size_t largo = fb->len;
  esp_camera_fb_return(fb);

  if (escritos != largo) {
    // Tarjeta llena, retirada o dañada
    Serial.println("Error escribiendo en la microSD");
    ledRojo(true);
    delay(500);
    return;
  }

  imagenes++;
  bytesTotales += largo;
  archivoTiempos.printf("%lu,%lu,%u\n", (unsigned long)imagenes, (unsigned long)ahora, (unsigned)largo);

  // Asegurar lo escrito: si se corta la energía solo se pierde ~1 s
  if (ahora - tFlush >= FLUSH_MS) {
    tFlush = ahora;
    archivoVideo.flush();
    archivoTiempos.flush();
  }

  // LED: parpadeo lento mientras graba
  if (ahora - tLed >= 500) {
    tLed = ahora;
    digitalWrite(PIN_LED_ROJO, !digitalRead(PIN_LED_ROJO));
  }

  // Resumen en el Monitor Serie cada 5 s
  if (ahora - tInfo >= 5000) {
    float seg = (ahora - tInicio) / 1000.0f;
    Serial.printf("%s  imagenes=%lu  %.1f fps  %.1f MB\n", carpeta, (unsigned long)imagenes,
                  imagenes / seg, bytesTotales / 1048576.0f);
    tInfo = ahora;
  }
}
