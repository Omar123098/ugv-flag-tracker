// main.cpp
// ESP32-CAM (AI-Thinker): all vision logic lives in loop().
// Sends one line per frame to the Arduino car on GPIO14 (9600 baud):
//   <COLOR,DIR,LOCK>\n     e.g. <RED,L,true>  <BLUE,C,false>  <NONE,N,true>
//   COLOR: RED, GREEN, BLUE, or NONE
//   DIR  : L = left, R = right, C = centred, N = no flag
//   LOCK : true  = car must stand still / spin (flag not centred or not found)
//          false = flag is centred, car can drive forward
//
// TEST_MODE 1: test with just your laptop, no control board needed.
//   - Open http://<ip>/ in a browser to see the camera with the detection drawn on it
//     (yellow lines = center zone, box red = LOCKED/spinning, green = UNLOCKED/free)
//   - Serial Monitor (9600) prints a readable status line
// TEST_MODE 0: competition, no WiFi, only the UART messages.
#define TEST_MODE 1

#include <Arduino.h>
#include "esp_camera.h"

#if TEST_MODE
#include <WiFi.h>
#include "img_converters.h"
#include "esp_http_server.h"
#include "secrets.h"  // WIFI_SSID / WIFI_PASS, see include/secrets.h.example
void startWebServer();
#endif

// UART to control ESP32: CAM GPIO14 (TX) -> control RX, plus common GND
#define LINK_TX_PIN 14 //transmit
#define LINK_RX_PIN 15 //recieve

// AI-Thinker pin map dol kol el pins el 3ndi 3la el chip nafsaha bs eh kol da tf 
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define FLASH_LED_GPIO 4

// competition colors: only red, green, blue
#define RED 0
#define GREEN 1
#define BLUE 2
#define NUM_COLORS 3
#define NO_COLOR -1  // white background, gray, black, or any other hue

#define CENTER_ZONE 27  // pixels on each side of the center column that count as "centered" (160/6: middle third of the screen)
#define STEP 2          // scan every 2nd pixel in x and y (1 vote = 4 real pixels)
const char *COLOR_NAMES[NUM_COLORS] = {"RED", "GREEN", "BLUE"};

// bright, clean R,G,B sent to the control team's LED (same order as the defines)
const uint8_t LED_RGB[NUM_COLORS][3] = {
  {255, 0, 0},    // RED
  {0, 255, 0},    // GREEN
  {0, 0, 255}     // BLUE
};

// Tune these with the Serial prints
#define MIN_V 40        // brightness below this = too dark to trust the hue, no vote
#define MIN_SAT 45      // colorfulness below this = white/gray background (was 90: too strict for a washed-out camera)
#define MIN_CHROMA 20   // max(R,G,B) - min(R,G,B) below this = white/pink glare, no vote
#define MIN_VOTES 200    // a color needs at least this many pixels to win

// hue bands (0..359). Anything outside them gets no vote.
#define RED_HUE_LOW 320    // red wraps around 0: hue >= 320 OR hue < RED_HUE_HIGH
#define RED_HUE_HIGH 30
#define GREEN_HUE_LOW 65
#define GREEN_HUE_HIGH 170
#define BLUE_HUE_LOW 170
#define BLUE_HUE_HIGH 275

// camera image tuning, each -2..2
#define CAM_SATURATION 2   // +2 = strongest colors, the main fix for "flag looks white"
#define CAM_AE_LEVEL -2    // negative = darker exposure, fewer blown-out white areas
#define CAM_BRIGHTNESS -1
#define CAM_CONTRAST 1

#if TEST_MODE
#define SHOW_CLASSES 1  // 1 = browser shows each pixel painted in its detected color
// display color for each class, same order as the defines (RGB565: RRRRR GGGGGG BBBBB)
//   0xF800 = 11111 000000 00000 = full red
//   0x07E0 = 00000 111111 00000 = full green
//   0x001F = 00000 000000 11111 = full blue
const uint16_t CLASS_RGB565[NUM_COLORS] = {0xF800, 0x07E0, 0x001F};
#endif

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);  // must match the baud rate on the Arduino car
  pinMode(FLASH_LED_GPIO, OUTPUT);
  digitalWrite(FLASH_LED_GPIO, LOW);

  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;  // raw pixels so we can read colors
  config.frame_size = FRAMESIZE_QQVGA;     // 160x120
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_count = psramFound() ? 2 : 1;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) delay(1000);
  }

  // push the sensor toward strong colors and away from overexposure
  sensor_t *s = esp_camera_sensor_get();
  s->set_saturation(s, CAM_SATURATION);
  s->set_ae_level(s, CAM_AE_LEVEL);
  s->set_brightness(s, CAM_BRIGHTNESS);
  s->set_contrast(s, CAM_CONTRAST);
  s->set_whitebal(s, 1);                 // auto white balance on
  s->set_awb_gain(s, 1);
  s->set_gainceiling(s, GAINCEILING_4X); // less analog gain = less washed-out noise
  // throw away the first frames while auto exposure / white balance settle
  for (int i = 0; i < 10; i++) {
    camera_fb_t *f = esp_camera_fb_get();
    if (f) esp_camera_fb_return(f);
  }

#if TEST_MODE
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setSleep(false);
  Serial.print("Connecting to WiFi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) { delay(250); Serial.print("."); }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nTEST MODE: open http://%s/ on your laptop\n", WiFi.localIP().toString().c_str());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("UGV-CAM", "ugv12345");
    Serial.printf("\nTEST MODE: no WiFi. Join hotspot UGV-CAM (pass ugv12345), open http://%s/\n",
                  WiFi.softAPIP().toString().c_str());
  }
  startWebServer();
#endif

}
void readRGB(const camera_fb_t *fb, int x, int y,
             uint8_t &r, uint8_t &g, uint8_t &b) {
  int p = y * fb->width + x;
  uint8_t *frames = fb->buf;
  // RRRRRGGG GGGBBBBB
  uint16_t pixel = (frames[p * 2] << 8) | (frames[p * 2 + 1]); //hna e7na bnlza2 el 16 bit m3 b3d fl pixel dy 
  uint8_t r5 = (pixel >> 11) & 0x1F;  // 0..31
  uint8_t g6 = (pixel >> 5) & 0x3F;   // 0..63
  uint8_t b5 = pixel & 0x1F;          // 0..31
  r = (r5 << 3) | (r5 >> 2);
  g = (g6 << 2) | (g6 >> 4);
  b = (b5 << 3) | (b5 >> 2);
}
// RGB -> hue 0..359, sat 0..255, val 0..255
void rgbToHSV(uint8_t r, uint8_t g, uint8_t b, int &hue, int &sat, int &val) {
  int mx = max(r, max(g, b));
  int mn = min(r, min(g, b));
  int d = mx - mn;
  val = mx;
  sat = mx ? d * 255 / mx : 0;
  if (d == 0) { hue = 0; return; }        // gray: no hue
  if (mx == r)      hue = 60 * (g - b) / d;
  else if (mx == g) hue = 120 + 60 * (b - r) / d;
  else              hue = 240 + 60 * (r - g) / d;
  if (hue < 0) hue += 360;
}

// which competition color is this pixel? NO_COLOR = no vote
int pixelColor(uint8_t r, uint8_t g, uint8_t b) {
  int hue, sat, val;
  rgbToHSV(r, g, b, hue, sat, val);
  int chroma = max(r, max(g, b)) - min(r, min(g, b));
  if (val < MIN_V) return NO_COLOR;        // too dark: black or deep shadow
  if (sat < MIN_SAT) return NO_COLOR;      // white paper, gray shadow
  if (chroma < MIN_CHROMA) return NO_COLOR; // bright near-white glare
  if (hue >= RED_HUE_LOW || hue < RED_HUE_HIGH) return RED;
  if (hue >= GREEN_HUE_LOW && hue < GREEN_HUE_HIGH) return GREEN;
  if (hue >= BLUE_HUE_LOW && hue < BLUE_HUE_HIGH) return BLUE;
  return NO_COLOR;                         // orange/yellow or purple/pink gaps
}

const char *colorName(int c) { return c == NO_COLOR ? "NONE" : COLOR_NAMES[c]; }

#if TEST_MODE
void setRGB565(camera_fb_t *fb, int x, int y, uint16_t color) {
  if (x < 0 || y < 0 || x >= fb->width || y >= fb->height) return;
  int p = y * fb->width + x;
  fb->buf[p * 2] = color >> 8;
  fb->buf[p * 2 + 1] = color & 0xFF;
}

// white crosshair on the detected center
void drawCross(camera_fb_t *fb, int cx, int cy) {
  for (int d = -8; d <= 8; d++) {
    setRGB565(fb, cx + d, cy, 0xFFFF);
    setRGB565(fb, cx, cy + d, 0xFFFF);
  }
}
#define YELLOW 0xFFE0   // 11111 111111 00000 = full red + full green

void drawVLine(camera_fb_t *fb, int x, uint16_t color) {
  for (int y = 0; y < fb->height; y++) setRGB565(fb, x, y, color);
}

void fillRect(camera_fb_t *fb, int x0, int y0, int w, int h, uint16_t color) {
  for (int y = y0; y < y0 + h; y++)
    for (int x = x0; x < x0 + w; x++) setRGB565(fb, x, y, color);
}

void drawBorder(camera_fb_t *fb, uint16_t color) {
  for (int x = 0; x < fb->width; x++) {
    setRGB565(fb, x, 0, color);
    setRGB565(fb, x, 1, color);
    setRGB565(fb, x, fb->height - 1, color);
    setRGB565(fb, x, fb->height - 2, color);
  }
  for (int y = 0; y < fb->height; y++) {
    setRGB565(fb, 0, y, color);
    setRGB565(fb, 1, y, color);
    setRGB565(fb, fb->width - 1, y, color);
    setRGB565(fb, fb->width - 2, y, color);
  }
} //the center borders: the left border is at 53 : 80-center_zone and the right is at 107 : 80+27 (3 equal thirds)

uint16_t rgbTo565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// direction as a word. If nothing is found, "NONE" (dir = 0 would wrongly say CENTER)
const char *dirName(bool found, int dir) {
  if (!found) return "NONE";
  if (dir < 0) return "LEFT";
  if (dir > 0) return "RIGHT";
  return "CENTER";
}

// three boxes at the bottom of the video: [LEFT] [CENTER] [RIGHT]
// the active one lights up cyan, the others stay dark gray
void drawDirection(camera_fb_t *fb, bool found, int dir) {
  const uint16_t ON = 0x07FF;   // cyan
  const uint16_t OFF = 0x4208;  // dark gray
  int y = fb->height - 14;      // sits just above the border
  fillRect(fb, 10, y, 40, 8, (found && dir < 0)  ? ON : OFF);   // left box
  fillRect(fb, 60, y, 40, 8, (found && dir == 0) ? ON : OFF);   // center box
  fillRect(fb, 110, y, 40, 8, (found && dir > 0) ? ON : OFF);   // right box
}

// The web server runs in its own tasks (esp_http_server), so a slow or dead
// browser connection never blocks loop(). loop() only publishes the latest JPEG.
//   port 80: GET /    -> page showing the stream
//            GET /jpg -> single frame as JPEG
//            GET /status -> one line of text (color, avg RGB, err, LOCKED/FREE)
//   port 81: GET /stream -> MJPEG stream (own server so it doesn't block port 80)
SemaphoreHandle_t jpgMutex;
uint8_t *latestJpg = nullptr;
size_t latestLen = 0;
uint32_t latestId = 0;
volatile uint32_t lastWebRequest = 0;  // loop() only encodes JPEGs while someone is watching
char statusText[128] = "waiting...";

void publishJpg(camera_fb_t *fb) {
  uint8_t *jpg = nullptr;
  size_t len = 0;
  if (!frame2jpg(fb, 80, &jpg, &len)) return;
  xSemaphoreTake(jpgMutex, portMAX_DELAY);
  free(latestJpg);
  latestJpg = jpg;
  latestLen = len;
  latestId++;
  xSemaphoreGive(jpgMutex);
}

// copies the latest JPEG into buf if it is newer than lastId; returns its length or 0
size_t copyLatestJpg(uint8_t *&buf, size_t &cap, uint32_t &lastId) {
  size_t len = 0;
  xSemaphoreTake(jpgMutex, portMAX_DELAY);
  if (latestJpg && latestId != lastId) {
    if (latestLen > cap) {
      free(buf);
      buf = (uint8_t *)malloc(latestLen);
      cap = buf ? latestLen : 0;
    }
    if (buf) {
      memcpy(buf, latestJpg, latestLen);
      len = latestLen;
      lastId = latestId;
    }
  }
  xSemaphoreGive(jpgMutex);
  return len;
}

esp_err_t indexHandler(httpd_req_t *req) {
  static const char page[] =
      "<html><body style='background:#222;margin:0'>"
      "<img id=c style='width:640px;image-rendering:pixelated'>"
      "<div id=s style='color:#fff;font:24px monospace;padding:8px'></div>"
      "<script>"
      "document.getElementById('c').src='http://'+location.hostname+':81/stream';"
      "setInterval(()=>fetch('/status').then(r=>r.text())"
      ".then(t=>document.getElementById('s').textContent=t),250);"
      "</script></body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

esp_err_t jpgHandler(httpd_req_t *req) {
  uint8_t *buf = nullptr;
  size_t cap = 0, len = 0;
  uint32_t lastId = 0;
  for (int i = 0; i < 50 && !len; i++) {  // wait up to ~1s for loop() to publish one
    lastWebRequest = millis();
    len = copyLatestJpg(buf, cap, lastId);
    if (!len) vTaskDelay(pdMS_TO_TICKS(20));
  }
  if (!len) {
    free(buf);
    return httpd_resp_send_500(req);
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t res = httpd_resp_send(req, (const char *)buf, len);
  free(buf);
  return res;
}
esp_err_t statusHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, statusText, HTTPD_RESP_USE_STRLEN);
}

// each viewer gets its own task, so several viewers can watch at once
void streamTask(void *arg) {
  httpd_req_t *req = (httpd_req_t *)arg;
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  uint8_t *buf = nullptr;
  size_t cap = 0;
  uint32_t lastId = 0;
  esp_err_t res = ESP_OK;
  while (res == ESP_OK) {
    lastWebRequest = millis();
    size_t len = copyLatestJpg(buf, cap, lastId);
    if (!len) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    char hdr[80];
    int n = snprintf(hdr, sizeof(hdr), "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", len);
    res = httpd_resp_send_chunk(req, hdr, n);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)buf, len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n", 2);
  }
  free(buf);  // viewer went away
  httpd_req_async_handler_complete(req);
  vTaskDelete(nullptr);
}

esp_err_t streamHandler(httpd_req_t *req) {
  httpd_req_t *asyncReq = nullptr;
  if (httpd_req_async_handler_begin(req, &asyncReq) != ESP_OK) return ESP_FAIL;
  if (xTaskCreate(streamTask, "stream", 4096, asyncReq, 5, nullptr) != pdPASS) {
    httpd_req_async_handler_complete(asyncReq);
    return ESP_FAIL;
  }
  return ESP_OK;  // the server is free for the next request right away
}

void startWebServer() {
  jpgMutex = xSemaphoreCreateMutex();

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.lru_purge_enable = true;  // drop the oldest connection instead of refusing new ones
  config.send_wait_timeout = 2;    // give up on a stalled viewer after 2 s
  httpd_handle_t mainServer = nullptr;
  httpd_uri_t indexUri = {.uri = "/", .method = HTTP_GET, .handler = indexHandler, .user_ctx = nullptr};
  httpd_uri_t jpgUri = {.uri = "/jpg", .method = HTTP_GET, .handler = jpgHandler, .user_ctx = nullptr};
  httpd_uri_t statusUri = {.uri = "/status", .method = HTTP_GET, .handler = statusHandler, .user_ctx = nullptr};
  if (httpd_start(&mainServer, &config) == ESP_OK) {
    httpd_register_uri_handler(mainServer, &indexUri);
    httpd_register_uri_handler(mainServer, &jpgUri);
    httpd_register_uri_handler(mainServer, &statusUri);
  }

  config.server_port = 81;
  config.ctrl_port += 1;
  httpd_handle_t streamServer = nullptr;
  httpd_uri_t streamUri = {.uri = "/stream", .method = HTTP_GET, .handler = streamHandler, .user_ctx = nullptr};
  if (httpd_start(&streamServer, &config) == ESP_OK) {
    httpd_register_uri_handler(streamServer, &streamUri);
  }
}
#endif

bool flag_existed = false; //hia dy el 7eta el hab3atha along with the rgb values, since eni hab3atha 3la led
void loop() {
  uint32_t tStart = micros();
  camera_fb_t *fb = esp_camera_fb_get();
  uint32_t tGot = micros();
  if (!fb) return;
  int rows = fb->height;
  int cols = fb->width;
  int avgX = 0; 
  int avgY = 0;
  int count = 0; //hna bardo el 7agat el hazbotha wab3atha 

  // one ballot box per color
  int votes[NUM_COLORS] = {0};
  int sumX[NUM_COLORS] = {0};
  int sumY[NUM_COLORS] = {0};
  int sumR[NUM_COLORS] = {0};
  int sumG[NUM_COLORS] = {0};
  int sumB[NUM_COLORS] = {0};

#if TEST_MODE
  uint8_t cr, cg, cb;
  readRGB(fb, cols / 2, rows / 2, cr, cg, cb);  // read before the overlay paints over it
  bool viewing = millis() - lastWebRequest < 2000;
#endif
  for (int j = 0; j < rows; j += STEP) {
    for (int i = 0; i < cols; i += STEP) {
      uint8_t r, g, b;
      readRGB(fb, i, j, r, g, b);
      int c = pixelColor(r, g, b);
      if (c != NO_COLOR) {
        // the next scanned pixel must agree, so single noise dots don't vote
        // (a real flag is a solid area, noise is scattered single pixels)
        uint8_t r2, g2, b2;
        readRGB(fb, min(i + STEP, cols - 1), j, r2, g2, b2);
        if (pixelColor(r2, g2, b2) != c) c = NO_COLOR;
      }
      if (c != NO_COLOR) {  // this pixel votes for color c
        votes[c]++;
        sumX[c] += i;
        sumY[c] += j;
        sumR[c] += r;
        sumG[c] += g;
        sumB[c] += b;
      }
#if TEST_MODE && SHOW_CLASSES
      // only paint pixels that voted, so you still see the real camera image underneath
      if (viewing && c != NO_COLOR) setRGB565(fb, i, j, CLASS_RGB565[c]);
#endif
    }
  }
  uint32_t tScanned = micros();

  // the color with the most votes wins (if it has enough)
  int winner = NO_COLOR;
  for (int c = 0; c < NUM_COLORS; c++) { 
    if (votes[c] < MIN_VOTES) continue;
    if (winner == NO_COLOR || votes[c] > votes[winner]) winner = c;
  }

  flag_existed = winner != NO_COLOR;  // true if some color passed MIN_VOTES
  int avgR = 0, avgG = 0, avgB = 0;
  if (flag_existed) {
    count = votes[winner];
    avgX = sumX[winner] / count;  // mean of x location of the winning color only
    avgY = sumY[winner] / count;
    avgR = sumR[winner] / count; 
    avgG = sumG[winner] / count;
    avgB = sumB[winner] / count;
  } else {
    avgX = -1;
    avgY = -1;
  }

  // message fields: must come AFTER the winner exists
  int err = flag_existed ? (avgX - cols / 2) : 0;  // negative = target on the left, positive = right
  int dir = 0;                                     // -1 = left, 0 = centered, +1 = right
  if (flag_existed && err < -CENTER_ZONE) dir = -1;
  else if (flag_existed && err > CENTER_ZONE) dir = 1;
  int area = flag_existed ? count * STEP * STEP : 0;  // each vote stands for STEP*STEP real pixels
  bool locked = !(flag_existed && dir == 0);       // true = car stands still / spins, false = flag centred, car drives

  // bright LED color for the winner (avgR/G/B stay only for test-mode display)
  int ledR = flag_existed ? LED_RGB[winner][0] : 0;
  int ledG = flag_existed ? LED_RGB[winner][1] : 0;
  int ledB = flag_existed ? LED_RGB[winner][2] : 0;

#if TEST_MODE
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    int h, s, v;
    rgbToHSV(cr, cg, cb, h, s, v);
    Serial.printf("center RGB=%3d,%3d,%3d HSV=%3d,%3d,%3d -> %-6s | winner=%-6s count=%5d avgX=%3d avgY=%3d\n",
                  cr, cg, cb, h, s, v, colorName(pixelColor(cr, cg, cb)),
                  colorName(winner), count, avgX, avgY);
    Serial.print("votes:");
    for (int c = 0; c < NUM_COLORS; c++) Serial.printf(" %s=%d", COLOR_NAMES[c], votes[c]);
    Serial.println();
  }
  uint32_t tJpg0 = micros();
  if (viewing) {
    snprintf(statusText, sizeof(statusText), "%s  avgRGB=%d,%d,%d  err=%d  DIR=%s  %s",
             colorName(winner), avgR, avgG, avgB, err,
             dirName(flag_existed, dir), locked ? "LOCKED" : "FREE");
    int mid = cols / 2;
    drawVLine(fb, mid - CENTER_ZONE, YELLOW);
    drawVLine(fb, mid + CENTER_ZONE, YELLOW);
    drawBorder(fb, locked ? 0xF800 : 0x07E0);   // red = locked, green = free
    drawDirection(fb, flag_existed, dir);       // [LEFT] [CENTER] [RIGHT] indicator
    if (flag_existed) {
      drawCross(fb, avgX, avgY);
      fillRect(fb, 4, 4, 14, 14, rgbTo565(avgR, avgG, avgB));  // swatch of the detected color
    }
    publishJpg(fb);
  }
  static uint32_t lastTiming = 0;
  if (millis() - lastTiming > 1000) {
    lastTiming = millis();
    Serial.printf("TIMING get=%lu scan=%lu jpg=%lu us rssi=%d dBm\n",
                  tGot - tStart, tScanned - tGot, micros() - tJpg0, WiFi.RSSI());
  }
#endif

  // message for the Arduino car: <COLOR,DIR,LOCK>   e.g. <RED,L,true>
  const char *dirChar = !flag_existed ? "N" : (dir < 0 ? "L" : (dir > 0 ? "R" : "C"));
  Serial1.printf("<%s,%s,%s>\n", colorName(winner), dirChar, locked ? "true" : "false");
  esp_camera_fb_return(fb);
}