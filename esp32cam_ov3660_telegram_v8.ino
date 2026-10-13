/*
  esp32cam_ov3660_telegram_v8

  AI-Thinker ESP32-CAM + OV3660 -> Telegram photo every 3 s, sent with AsyncTelegram2
  (the same sendPhoto() call as in your SYNC RTC and OV2640 timelapse sketches)

  - LED GPIO33 (active LOW): fast blink while connecting to WiFi, solid on when connected
  - Brownout detector disabled
  - Caption: "im awake", sketch name, local time, RSSI, exposure mode, light readings, JPEG size

  Selectable below: resolution, white balance, vertical/horizontal flip, special effect.

  Exposure logic (light measurement from v5, no hour gate):

    AUTO  : normal full-auto exposure. Every photo the sketch reads what the sensor reports:
              0x56A1 AVG READOUT (average image level), 0x3500-0x3502 exposure, 0x350A-0x350B gain.
            If the auto gain reaches GAIN_TRIGGER the AEC is running out of light -> DARK.
    DARK  : manual long exposure at a low gain, closed loop on the average image level.
            The frame is lengthened (frame height, reg 0x380E/0x380F) so the exposure really fits,
            and every photo the exposure x gain is corrected until the average image level
            is near DARK_TARGET_AVG. Back to AUTO when the scene gets bright again.

  Why a closed loop: the OV3660 only exposes for longer than one frame if the frame itself is
  longer. The measured frame time is checked after the first long exposure; if the frame did
  not lengthen, the other method (reg 0x350C/0x350D) is tried, and if that fails too the sketch
  falls back to short exposures with more gain and says so in the caption.
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
WiFiClientSecure client;
#include <AsyncTelegram2.h>  // https://github.com/cotestatnt/AsyncTelegram2
AsyncTelegram2 myBot(client);
#include <time.h>
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

const char* SKETCH_NAME = "esp32cam_ov3660_telegram_v8";   // shown in the photo caption

// ---- WiFi / Telegram ----
const char* ssid  = "YOUR_WIFI_SSID";
const char* pass  = "YOUR_WIFI_PASSWORD";
const char* token = "YOUR_TELEGRAM_BOT_TOKEN";
int64_t userid = YOUR_TELEGRAM_USER_ID;                       // your chat id
#define MYTZ "WET0WEST,M3.5.0/1,M10.5.0"          // time zone, for the time shown in the caption

// 0 = connect without checking Telegram's certificate (as in your OV2640 sketch)
// 1 = check it with telegram_cert (as in your SYNC RTC sketch; needs correct time and a library that has it)
#define USE_TELEGRAM_CERT 0

const uint8_t  LED_PIN       = 33;     // red LED, active LOW
const uint32_t BLINK_MS      = 100;
const uint32_t SEND_INTERVAL = 10000;

// =====================================================================================
//  IMAGE OPTIONS
// =====================================================================================

// ---- resolution (pick one) ----
#define RES_VGA    0     //  640 x 480
#define RES_HD     1     // 1280 x 720   (720p)
#define RES_FHD    2     // 1920 x 1080  (1080p)
#define RES_UXGA   3     // 1600 x 1200
#define RES_QXGA   4     // 2048 x 1536
#define RESOLUTION RES_VGA

#define JPEG_QUALITY 10  // 0-63, lower = better quality and bigger files

// ---- white balance: 0 auto, 1 sunny, 2 cloudy, 3 office, 4 home ----
#define WB_MODE       0
#define WB_MODE_DARK -1  // -1 = same as WB_MODE, otherwise the mode used in DARK (v4 used 2 = cloudy)

// ---- special effect: 0 none, 1 negative, 2 grayscale, 3 red tint, 4 green tint, 5 blue tint, 6 sepia ----
#define SPECIAL_EFFECT 0

// ---- orientation (1 = on) ----
#define HMIRROR 1        // horizontal mirror (v4/v5 used 1)
#define VFLIP   0        // vertical flip

// ---- auto exposure brightness target, -5..5 (0 = driver default, about 55 on the 0..255 scale).
//      +2 gives about 75, which matches DARK_TARGET_AVG so there is no brightness jump between modes ----
#define AE_LEVEL 0

// =====================================================================================
//  EXPOSURE TUNING
// =====================================================================================
const float    GAIN_TRIGGER    = 2.0f;    // auto gain at/above this -> DARK
const float    GAIN_TARGET     = 2.0f;    // gain aimed for in DARK (exposure time is stretched instead)
const float    GAIN_MAX        = 16.0f;   // highest gain in DARK when frames can be lengthened
const float    GAIN_MAX_SHORT  = 31.0f;   // highest gain if frames cannot be lengthened (fallback)
const float    DARK_TARGET_AVG = 80.0f;   // average image level aimed for in DARK (higher = brighter)
const float    MAX_ENTRY_RATIO = 16.0f;   // largest brightening applied when entering DARK
const float    DEADBAND        = 0.20f;   // DARK: no correction if the level is within +/-20% of the target
const float    MAX_STEP_UP     = 4.0f;    // DARK: largest correction per photo (brighter)
const float    MAX_STEP_DOWN   = 0.4f;    // DARK: largest correction per photo (darker)
const float    EXIT_FRACTION   = 0.5f;    // back to AUTO when the needed exposure x gain falls below
                                          // this fraction of what AUTO was using when DARK started
const uint32_t MAX_EXPOSURE_MS = 1000;    // longest exposure (frame) allowed; the camera driver times out at 4000 ms
const uint8_t  WARMUP_FRAMES   = 6;       // AUTO frames discarded before measuring (boot / after DARK)
const uint8_t  SETTLE_FRAMES   = 2;       // frames discarded after changing the manual exposure

// AI-Thinker ESP32-CAM pin map
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// 16-bit SCCB register access through the esp32-camera sensor layer
#define OV3660_REG(s, addr, mask, val)  (s)->set_reg((s), (addr), (mask), (val))
#define OV3660_GETREG(s, addr)          (s)->get_reg((s), (addr), 0xFF)

#if RESOLUTION == RES_VGA
  #define FRAME_SIZE FRAMESIZE_VGA
#elif RESOLUTION == RES_HD
  #define FRAME_SIZE FRAMESIZE_HD
#elif RESOLUTION == RES_FHD
  #define FRAME_SIZE FRAMESIZE_FHD
#elif RESOLUTION == RES_UXGA
  #define FRAME_SIZE FRAMESIZE_UXGA
#else
  #define FRAME_SIZE FRAMESIZE_QXGA
#endif

uint32_t lastBlink = 0;
uint32_t lastSend  = 0;
bool     ledState  = false;

// ---- state ----
bool     sensorIsOV3660 = false;
bool     darkMode       = false;     // false = AUTO, true = DARK (manual long exposure)
uint32_t photoCount     = 0;

int      baseVts   = 0;              // normal frame height in rows, read after init
float    rowMs     = 0.05f;          // measured time of one sensor row
bool     lengthWorks    = true;      // can the frame be lengthened at all?
bool     lengthChecked  = false;     // has that been verified?
int      lengthMethod   = 0;         // 0 = frame height (0x380E/F), 1 = "add VTS" (0x350C/D)

float    curExpRows = 0;             // DARK: exposure written (rows)
float    curGain    = 1;             // DARK: gain written
float    entryProduct = 0;           // AUTO exposure x gain when DARK started

// last readings (also used for the caption)
int      mAvg  = -1;
float    mExp  = 0;
float    mGain = 0;
int      mVts  = 0;

// caption info
const char* lastMode = "auto";
bool        measuredNow = false;

uint32_t sentOk = 0, sentFail = 0;

void ledOn()  { digitalWrite(LED_PIN, LOW);  }
void ledOff() { digitalWrite(LED_PIN, HIGH); }

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, pass);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastBlink >= BLINK_MS) {
      lastBlink = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? LOW : HIGH);
    }
    delay(1);
  }
  ledOn();
  Serial.printf("\nConnected, IP: %s\n", WiFi.localIP().toString().c_str());
}

void discardFrames(uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    camera_fb_t* f = esp_camera_fb_get();
    if (f) esp_camera_fb_return(f);
  }
}

// Average time of n consecutive frames in ms (taken back to back, so each call waits for
// the next complete frame).
float measureFrameMs(uint8_t n) {
  discardFrames(1);                         // synchronise to a frame boundary
  uint32_t t0 = millis();
  discardFrames(n);
  return (millis() - t0) / (float)n;
}

// ---- reading the sensor ----

// Reads 0x56A1 (average image level), the exposure and gain the sensor is using,
// and the frame height. Results go to mAvg / mExp / mGain / mVts.
bool readStats(sensor_t* s) {
  int a  = OV3660_GETREG(s, 0x56A1);
  int e0 = OV3660_GETREG(s, 0x3500), e1 = OV3660_GETREG(s, 0x3501), e2 = OV3660_GETREG(s, 0x3502);
  int g0 = OV3660_GETREG(s, 0x350A), g1 = OV3660_GETREG(s, 0x350B);
  int v0 = OV3660_GETREG(s, 0x380E), v1 = OV3660_GETREG(s, 0x380F);
  if (e0 < 0 || e1 < 0 || e2 < 0 || g0 < 0 || g1 < 0) return false;
  mAvg  = a;
  mExp  = (((e0 & 0x0F) << 16) | (e1 << 8) | e2) / 16.0f;
  mGain = (((g0 & 0x03) << 8) | g1) / 16.0f;
  mVts  = (v0 >= 0 && v1 >= 0) ? ((v0 << 8) | v1) : 0;
  return true;
}

// ---- writing the sensor ----

void setWb(sensor_t* s, int mode) {
  s->set_wb_mode(s, mode);
}

void setVts(sensor_t* s, long vts) {
  if (vts < 1) vts = 1;
  if (vts > 0xFFFF) vts = 0xFFFF;
  OV3660_REG(s, 0x380E, 0xFF, (vts >> 8) & 0xFF);
  OV3660_REG(s, 0x380F, 0xFF, vts & 0xFF);
}

void setAddVts(sensor_t* s, long add) {
  if (add < 0) add = 0;
  if (add > 0xFFFF) add = 0xFFFF;
  OV3660_REG(s, 0x350C, 0xFF, (add >> 8) & 0xFF);
  OV3660_REG(s, 0x350D, 0xFF, add & 0xFF);
}

// Back to full auto with the normal frame length.
void resetToAuto(sensor_t* s) {
  OV3660_REG(s, 0x3503, 0x03, 0x00);        // AEC + AGC auto
  setAddVts(s, 0);
  if (baseVts > 0) setVts(s, baseVts);
  OV3660_REG(s, 0x3A00, 0x04, 0x00);        // night mode off
  OV3660_REG(s, 0x5308, 0x40, 0x00);        // auto sharpness
  setWb(s, WB_MODE);
  darkMode = false;
}

// Manual exposure (rows) and gain (1.0 = 1x). The frame is lengthened so the exposure fits.
void applyManual(sensor_t* s, float expRows, float gain) {
  uint32_t ev = (uint32_t)(expRows * 16.0f);
  if (ev > 0xFFFFF) ev = 0xFFFFF;
  uint32_t gv = (uint32_t)(gain * 16.0f);
  if (gv > 0x3FF) gv = 0x3FF;
  long need = (long)expRows + 8;            // frame height the exposure needs

  OV3660_REG(s, 0x3503, 0x03, 0x03);        // manual AEC + AGC
  if (lengthMethod == 0) {
    setAddVts(s, 0);
    setVts(s, need > baseVts ? need : baseVts);
  } else {
    setVts(s, baseVts);
    setAddVts(s, need > baseVts ? need - baseVts : 0);
  }
  OV3660_REG(s, 0x3500, 0x0F, (ev >> 16) & 0x0F);
  OV3660_REG(s, 0x3501, 0xFF, (ev >> 8) & 0xFF);
  OV3660_REG(s, 0x3502, 0xFF, ev & 0xFF);
  OV3660_REG(s, 0x350A, 0x03, (gv >> 8) & 0x03);
  OV3660_REG(s, 0x350B, 0xFF, gv & 0xFF);
  OV3660_REG(s, 0x5308, 0xFF, 0x45);        // low sharpness, value from your working sketch
#if WB_MODE_DARK >= 0
  setWb(s, WB_MODE_DARK);
#endif
  darkMode = true;
}

// Chooses exposure and gain for a wanted exposure x gain and writes them.
void setManualFromProduct(sensor_t* s, float product) {
  float maxRows = lengthWorks ? (MAX_EXPOSURE_MS / rowMs) : (float)(baseVts - 8);
  if (maxRows > 60000.0f) maxRows = 60000.0f;
  if (maxRows < 16.0f)    maxRows = 16.0f;
  float gmax = lengthWorks ? GAIN_MAX : GAIN_MAX_SHORT;

  float g = GAIN_TARGET;
  float e = product / g;
  if (e > maxRows) { e = maxRows; g = product / e; }   // time budget reached: make up with gain
  if (g > gmax)    g = gmax;
  if (g < 1.0f)    { g = 1.0f; e = product / g; }
  if (e < 16.0f)   e = 16.0f;
  if (e > maxRows) e = maxRows;

  curExpRows = e;
  curGain    = g;
  applyManual(s, e, g);
  Serial.printf("manual: exposure=%.0f rows (%.0f ms) gain=%.1fx (product %.0f)\n",
                e, e * rowMs, g, e * g);
}

// After the first long exposure: did the frame really get longer? If not, try the other method;
// if that fails too, stay with short exposures (more gain).
void verifyLengthening(sensor_t* s) {
  if (lengthChecked) return;
  if (curExpRows <= baseVts - 8) return;    // exposure fits in a normal frame: nothing to check
  float expected = (curExpRows + 8) * rowMs;

  discardFrames(SETTLE_FRAMES);
  float got = measureFrameMs(1);
  Serial.printf("length check, method %d: frame %.0f ms, expected %.0f ms\n", lengthMethod, got, expected);
  if (got >= 0.6f * expected) { lengthChecked = true; lengthWorks = true; return; }

  lengthMethod = lengthMethod ? 0 : 1;      // try the other register
  Serial.printf("frame did not lengthen, trying method %d\n", lengthMethod);
  applyManual(s, curExpRows, curGain);
  discardFrames(SETTLE_FRAMES);
  got = measureFrameMs(1);
  Serial.printf("length check, method %d: frame %.0f ms, expected %.0f ms\n", lengthMethod, got, expected);
  lengthChecked = true;
  if (got >= 0.6f * expected) { lengthWorks = true; return; }

  lengthWorks = false;                      // neither works: short exposure, more gain
  lengthMethod = 0;
  Serial.println("frame cannot be lengthened: DARK will use short exposures and more gain");
  resetToAuto(s);
  discardFrames(WARMUP_FRAMES);
}

// ---- AUTO <-> DARK ----

void enterDark(sensor_t* s) {
  entryProduct = mExp * mGain;
  float ratio = 1.0f;
  if (mAvg > 0) ratio = DARK_TARGET_AVG / (float)mAvg;
  if (ratio < 1.0f) ratio = 1.0f;
  if (ratio > MAX_ENTRY_RATIO) ratio = MAX_ENTRY_RATIO;
  Serial.printf("entering DARK: auto product %.0f x%.1f\n", entryProduct, ratio);

  setManualFromProduct(s, entryProduct * ratio);
  lastMode = "dark";
  if (!lengthChecked && curExpRows > baseVts - 8) {
    verifyLengthening(s);                   // may switch method or fall back to AUTO
    if (!darkMode) { lastMode = "auto"; return; }
    if (!lengthWorks) {                     // fell back: redo with short exposure and more gain
      setManualFromProduct(s, entryProduct * ratio);
    }
  }
  discardFrames(SETTLE_FRAMES);
}

// DARK: correct exposure x gain from the average image level of the last frames.
void trackDark(sensor_t* s) {
  if (!readStats(s)) { lastMode = "read-error"; return; }
  if (mAvg <= 0) { lastMode = "dark (no avg)"; return; }

  float ratio = DARK_TARGET_AVG / (float)mAvg;
  if (ratio > 1.0f - DEADBAND && ratio < 1.0f + DEADBAND) {
    lastMode = "dark (hold)";
    return;
  }
  if (ratio > MAX_STEP_UP)   ratio = MAX_STEP_UP;
  if (ratio < MAX_STEP_DOWN) ratio = MAX_STEP_DOWN;

  float wanted = curExpRows * curGain * ratio;
  Serial.printf("DARK: AVG=%d target=%.0f -> exposure x gain %.0f -> %.0f\n",
                mAvg, DARK_TARGET_AVG, curExpRows * curGain, wanted);

  if (wanted < EXIT_FRACTION * entryProduct) {
    Serial.println("bright again: back to AUTO");
    resetToAuto(s);
    discardFrames(WARMUP_FRAMES);
    readStats(s);
    lastMode = "auto";
    return;
  }

  float before = curExpRows * curGain;
  setManualFromProduct(s, wanted);
  if (curExpRows * curGain != before) {
    lastMode = (ratio > 1.0f) ? "dark (brighter)" : "dark (darker)";
    discardFrames(SETTLE_FRAMES);
  } else {
    lastMode = "dark (limit)";              // already at the exposure/gain limits
  }
}

void adaptToLight() {
  sensor_t* s = esp_camera_sensor_get();
  measuredNow = true;

  if (photoCount == 0) {                    // first photo after boot: let the AEC settle
    resetToAuto(s);
    discardFrames(WARMUP_FRAMES);
  }

  if (!darkMode) {
    if (!readStats(s)) { lastMode = "read-error"; return; }
    Serial.printf("auto: AVG=%d exposure=%.0f rows gain=%.1fx VTS=%d\n", mAvg, mExp, mGain, mVts);
    if (mGain < GAIN_TRIGGER) { lastMode = "auto"; return; }
    enterDark(s);
  } else {
    trackDark(s);
  }
}

// ---- camera ----

bool initCamera() {
  camera_config_t config = {};              // zero-init so no field holds garbage
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.jpeg_quality = JPEG_QUALITY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  if (psramFound()) {
    config.frame_size = FRAME_SIZE;
    config.fb_count   = 2;
  } else {
    Serial.println("No PSRAM found: using VGA with one frame buffer");
    config.frame_size = FRAMESIZE_VGA;
    config.fb_count   = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    sensorIsOV3660 = true;
    Serial.println("OV3660 detected");
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, WB_MODE);
    s->set_hmirror(s, HMIRROR);
    s->set_vflip(s, VFLIP);
    s->set_special_effect(s, SPECIAL_EFFECT);
#if AE_LEVEL != 0
    s->set_ae_level(s, AE_LEVEL);
#endif

    // normal frame height and the time of one row, used to turn "ms" into "rows"
    int v0 = OV3660_GETREG(s, 0x380E), v1 = OV3660_GETREG(s, 0x380F);
    baseVts = (v0 >= 0 && v1 >= 0) ? ((v0 << 8) | v1) : 0;
    discardFrames(WARMUP_FRAMES);
    float frameMs = measureFrameMs(3);
    if (baseVts > 0 && frameMs > 0) rowMs = frameMs / (float)baseVts;
    Serial.printf("frame height %d rows, frame %.0f ms, row %.4f ms\n", baseVts, frameMs, rowMs);
    if (baseVts <= 0) {
      Serial.println("WARNING: frame height unreadable, long exposures disabled");
      lengthWorks = false;
      lengthChecked = true;
      baseVts = 1000;
    }
  } else {
    Serial.printf("WARNING: sensor PID 0x%04X is not an OV3660, exposure logic disabled\n",
                  s->id.PID);
  }
  return true;
}

// Reads width/height from the JPEG header (the SOF marker). Returns false if it is not found.
bool jpegSize(const uint8_t* b, size_t n, int& w, int& h) {
  if (n < 4 || b[0] != 0xFF || b[1] != 0xD8) return false;
  size_t i = 2;
  while (i + 9 < n) {
    if (b[i] != 0xFF) { i++; continue; }
    uint8_t m = b[i + 1];
    if (m == 0xFF) { i++; continue; }
    if (m == 0xD8 || m == 0x01 || (m >= 0xD0 && m <= 0xD7)) { i += 2; continue; }
    size_t len = ((size_t)b[i + 2] << 8) | b[i + 3];
    if (m >= 0xC0 && m <= 0xCF && m != 0xC4 && m != 0xC8 && m != 0xCC) {
      h = (b[i + 5] << 8) | b[i + 6];
      w = (b[i + 7] << 8) | b[i + 8];
      return true;
    }
    i += 2 + len;
  }
  return false;
}

bool sendPicture() {
  measuredNow = false;
  if (sensorIsOV3660) {
    adaptToLight();
  }
  photoCount++;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed");
    return false;
  }

  // the average level of the frame just captured (used by the next correction step)
  if (sensorIsOV3660) {
    int a = OV3660_GETREG(esp_camera_sensor_get(), 0x56A1);
    if (a >= 0) mAvg = a;
  }

  int jw = 0, jh = 0;
  bool haveSize = jpegSize(fb->buf, fb->len, jw, jh);
  Serial.printf("Sensor JPEG: %dx%d, %u bytes (fb reports %dx%d)\n",
                jw, jh, (unsigned)fb->len, (int)fb->width, (int)fb->height);

  // multi-line caption (AsyncTelegram2 also uses it as the file name, as in your other sketches)
  char caption[260];
  int n = snprintf(caption, sizeof(caption), " \n%s\n", SKETCH_NAME);

  time_t now = time(nullptr);
  if (now > 1700000000 && n > 0 && n < (int)sizeof(caption)) {
    struct tm tInfo;
    localtime_r(&now, &tInfo);
    n += strftime(caption + n, sizeof(caption) - n, "%d/%m/%Y %H:%M:%S\n", &tInfo);
  }
  if (n > 0 && n < (int)sizeof(caption))
    n += snprintf(caption + n, sizeof(caption) - n, "RSSI: %d dBm\nMode: %s%s\n",
                  (int)WiFi.RSSI(), lastMode, measuredNow ? "" : " (held)");
  if (sensorIsOV3660 && n > 0 && n < (int)sizeof(caption)) {
    if (darkMode)
      n += snprintf(caption + n, sizeof(caption) - n, "AVG=%d E=%.0f ms G=%.1fx%s\n",
                    mAvg, curExpRows * rowMs, curGain, lengthWorks ? "" : " (no long frames)");
    else
      n += snprintf(caption + n, sizeof(caption) - n, "AVG=%d E=%.0f rows G=%.1fx\n",
                    mAvg, mExp, mGain);
  }
  if (n > 0 && n < (int)sizeof(caption)) {
    if (haveSize)
      snprintf(caption + n, sizeof(caption) - n, "JPEG: %dx%d, %u KB", jw, jh, (unsigned)(fb->len / 1024));
    else
      snprintf(caption + n, sizeof(caption) - n, "JPEG: size unknown, %u KB", (unsigned)(fb->len / 1024));
  }

  Serial.println(caption);

  size_t len = fb->len;
  bool queued = myBot.sendPhoto(userid, fb->buf, fb->len, caption);
  esp_camera_fb_return(fb);

  Serial.printf("Photo %u bytes -> %s\n", (unsigned)len, queued ? "sent" : "NOT sent");
  return queued;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);   // disable brownout detector

  Serial.begin(115200);
  Serial.println();
  Serial.println(SKETCH_NAME);
  pinMode(LED_PIN, OUTPUT);
  ledOff();

  if (!initCamera()) {
    Serial.println("Restarting in 3 s...");
    delay(3000);
    ESP.restart();
  }

  connectWiFi();

  // time from NTP (shown in the caption, needed for the certificate check)
  configTzTime(MYTZ, "time.google.com", "time.windows.com", "pool.ntp.org");
  struct tm ti;
  bool timeOk = getLocalTime(&ti, 10000);
  if (timeOk) Serial.println(&ti, "Local time: %d/%m/%Y %H:%M:%S");
  else        Serial.println("NTP time not available yet");

#if USE_TELEGRAM_CERT
  client.setCACert(telegram_cert);
#else
  client.setInsecure();
#endif

  myBot.addSentCallback([](bool sent) {
    if (sent) sentOk++; else sentFail++;
    Serial.println(sent ? "Telegram: picture delivered" : "Telegram: picture NOT delivered");
  }, 3000);

  myBot.setUpdateTime(1000);
  myBot.setTelegramToken(token);

  Serial.print("Test Telegram connection... ");
  Serial.println(myBot.begin() ? "OK" : "NOK");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  // lets AsyncTelegram2 read the replies to the photos (and runs the "sent" callback)
  TBMessage msg;
  myBot.getNewMessage(msg);

  if (millis() - lastSend >= SEND_INTERVAL) {
    lastSend = millis();
    sendPicture();
  }
}
