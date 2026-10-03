/*
 * ESP32-Based Smart Environmental Monitoring System
 * Author : B. Aathi Varaghan
 *
 * Sensors : DHT22 (temperature, humidity), BMP280 (pressure), MQ-135 (air quality), LDR (light)
 * Output  : SSD1306 OLED, buzzer, status LED, MQTT (JSON) over Wi-Fi
 * RTOS    : sensorTask -> alertTask -> (shared latest reading) -> displayTask / mqttTask
 *
 * Libraries (Arduino Library Manager):
 *   "DHT sensor library" + "Adafruit Unified Sensor"   (Adafruit)
 *   "Adafruit BMP280 Library"
 *   "Adafruit SSD1306" + "Adafruit GFX Library"
 *   "MQTT" by Joel Gaehwiler (arduino-mqtt)
 * Board: "ESP32 Dev Module" (Arduino-ESP32 core)
 *
 * NOTE: pins, thresholds and the MQ-135 calibration value are starting points.
 * Adjust them to match your own wiring and calibration.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MQTT.h>
#include <esp_task_wdt.h>
#include "secrets.h"

// ---------------- Pin map (see docs/pin_connections.png) ----------------
#define PIN_DHT     4
#define PIN_SDA     21
#define PIN_SCL     22
#define PIN_MQ135   34      // ADC1_CH6 (input only)
#define PIN_LDR     35      // ADC1_CH7 (input only)
#define PIN_BUZZER  25
#define PIN_LED     2

// ---------------- Settings ----------------
#define DEVICE_ID          "esp32-env-01"
#define MQTT_TOPIC         "esp32/env/telemetry"
#define SENSOR_PERIOD_MS   2000     // DHT22 needs >= 2 s between reads
#define DISPLAY_PERIOD_MS  1000
#define PUBLISH_PERIOD_MS  5000
#define ADC_SAMPLES        10       // moving average length for analog sensors
#define OFFLINE_BUFFER     50       // readings kept while offline

// Alert thresholds
#define TEMP_MAX_C         35.0f
#define HUM_MAX_PCT        90.0f
#define AQ_MAX_PPM         1000.0f

// MQ-135 (approximate CO2-equivalent, relative indicator only)
#define MQ_RL_KOHM         10.0f    // load resistor on the module
#define MQ_R0_KOHM         76.63f   // calibrate in clean air after pre-heating!
#define MQ_VCC             5.0f
#define MQ_DIVIDER_RATIO   1.5f     // 10k/20k divider: Vsensor = Vadc * 1.5

struct Reading {
  float tempC, humPct, pressHpa, aqPpm, lightPct;
  bool  alert;
};

// ---------------- Globals ----------------
DHT dht(PIN_DHT, DHT22);
Adafruit_BMP280 bmp;
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
WiFiClient net;
MQTTClient mqtt(256);

QueueHandle_t     qReadings;        // sensorTask -> alertTask (5 items)
SemaphoreHandle_t i2cMutex;         // protects the shared I2C bus
SemaphoreHandle_t dataMutex;        // protects latest
Reading latest = {NAN, NAN, NAN, 0, 0, false};
bool    haveData = false;
volatile bool wifiOk = false;
volatile bool mqttOk = false;

static Reading ring[OFFLINE_BUFFER];   // circular buffer for offline readings
static int ringHead = 0, ringCount = 0;
static char payload[256];              // static JSON buffer (no heap use in loop)

// ---------------- Helpers ----------------
float readAdcAvgMv(int pin) {
  uint32_t sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) { sum += analogReadMilliVolts(pin); delay(2); }
  return sum / (float)ADC_SAMPLES;
}

float mq135Ppm() {
  float vSensor = (readAdcAvgMv(PIN_MQ135) / 1000.0f) * MQ_DIVIDER_RATIO;
  if (vSensor < 0.05f) return 0;
  float rs = ((MQ_VCC / vSensor) - 1.0f) * MQ_RL_KOHM;
  return 116.6020682f * powf(rs / MQ_R0_KOHM, -2.769034857f);   // common MQ-135 CO2 curve
}

float ldrPercent() {
  return constrain(readAdcAvgMv(PIN_LDR) / 3300.0f * 100.0f, 0.0f, 100.0f);
}

void buildPayload(const Reading &r) {
  snprintf(payload, sizeof(payload),
    "{\"device\":\"%s\",\"temp_c\":%.1f,\"hum_pct\":%.1f,\"press_hpa\":%.1f,"
    "\"aq_ppm\":%.0f,\"light_pct\":%.0f,\"alert\":%d}",
    DEVICE_ID, r.tempC, r.humPct, r.pressHpa, r.aqPpm, r.lightPct, r.alert ? 1 : 0);
}

// ---------------- Tasks ----------------
void sensorTask(void *) {
  esp_task_wdt_add(NULL);                         // watchdog: reset if this task hangs
  Reading r = {NAN, NAN, NAN, 0, 0, false};
  for (;;) {
    float t = dht.readTemperature(), h = dht.readHumidity();
    if (!isnan(t)) r.tempC = t;                   // keep last valid value on DHT error
    if (!isnan(h)) r.humPct = h;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
      r.pressHpa = bmp.readPressure() / 100.0f;
      xSemaphoreGive(i2cMutex);
    }
    r.aqPpm = mq135Ppm();
    r.lightPct = ldrPercent();
    xQueueSend(qReadings, &r, 0);
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

void alertTask(void *) {
  Reading r;
  for (;;) {
    if (xQueueReceive(qReadings, &r, portMAX_DELAY) == pdTRUE) {
      r.alert = (r.tempC > TEMP_MAX_C) || (r.humPct > HUM_MAX_PCT) || (r.aqPpm > AQ_MAX_PPM);
      digitalWrite(PIN_BUZZER, r.alert);
      digitalWrite(PIN_LED, r.alert);
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      latest = r; haveData = true;
      xSemaphoreGive(dataMutex);
    }
  }
}

void displayTask(void *) {
  for (;;) {
    Reading r; bool ok;
    xSemaphoreTake(dataMutex, portMAX_DELAY); r = latest; ok = haveData; xSemaphoreGive(dataMutex);
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(SSD1306_WHITE);
      oled.setCursor(0, 0);
      if (ok) {
        oled.printf("Temp : %.1f C\n", r.tempC);
        oled.printf("Humid: %.1f %%\n", r.humPct);
        oled.printf("Press: %.1f hPa\n", r.pressHpa);
        oled.printf("AQ   : %.0f ppm\n", r.aqPpm);
        oled.printf("Light: %.0f %%\n", r.lightPct);
        oled.printf("WiFi:%s MQTT:%s %s", wifiOk ? "OK" : "--", mqttOk ? "OK" : "--", r.alert ? "ALERT" : "");
      } else {
        oled.print("Starting sensors...");
      }
      oled.display();
      xSemaphoreGive(i2cMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
  }
}

void ringPush(const Reading &r) {
  ring[(ringHead + ringCount) % OFFLINE_BUFFER] = r;
  if (ringCount < OFFLINE_BUFFER) ringCount++;
  else ringHead = (ringHead + 1) % OFFLINE_BUFFER;      // overwrite oldest
}

void connectNetwork() {
  if (WiFi.status() != WL_CONNECTED) {
    wifiOk = false; mqttOk = false;
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) vTaskDelay(pdMS_TO_TICKS(500));
  }
  wifiOk = (WiFi.status() == WL_CONNECTED);
  if (wifiOk && !mqtt.connected()) {
    mqttOk = mqtt.connect(DEVICE_ID, MQTT_USER, MQTT_PASS);
  }
  mqttOk = wifiOk && mqtt.connected();
}

void mqttTask(void *) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);                        // modem sleep between transmissions
  mqtt.begin(MQTT_HOST, MQTT_PORT, net);
  mqtt.setKeepAlive(60);
  for (;;) {
    if (!mqtt.connected() || WiFi.status() != WL_CONNECTED) connectNetwork();
    mqtt.loop();

    Reading r; bool ok;
    xSemaphoreTake(dataMutex, portMAX_DELAY); r = latest; ok = haveData; xSemaphoreGive(dataMutex);
    if (ok) {
      if (mqttOk) {
        while (ringCount > 0 && mqttOk) {      // flush buffered readings first
          buildPayload(ring[ringHead]);
          if (mqtt.publish(MQTT_TOPIC, payload, false, 1)) {
            ringHead = (ringHead + 1) % OFFLINE_BUFFER; ringCount--;
          } else mqttOk = false;
        }
        buildPayload(r);
        if (!mqtt.publish(MQTT_TOPIC, payload, false, 1)) { mqttOk = false; ringPush(r); }
      } else {
        ringPush(r);                            // offline: keep reading in RAM
      }
    }
    vTaskDelay(pdMS_TO_TICKS(PUBLISH_PERIOD_MS));
  }
}

// ---------------- Arduino entry points ----------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUZZER, OUTPUT); pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW); digitalWrite(PIN_LED, LOW);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MQ135, ADC_11db);
  analogSetPinAttenuation(PIN_LDR, ADC_11db);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  dht.begin();
  if (!bmp.begin(0x76)) Serial.println("BMP280 not found (check wiring / address)");
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("OLED not found (check wiring / address)");

  qReadings = xQueueCreate(5, sizeof(Reading));
  i2cMutex  = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();

  //                      function      name       stack  arg  prio  handle
  xTaskCreate(sensorTask,  "sensorTask",  4096, NULL, 2, NULL);
  xTaskCreate(alertTask,   "alertTask",   3072, NULL, 3, NULL);
  xTaskCreate(displayTask, "displayTask", 4096, NULL, 1, NULL);
  xTaskCreate(mqttTask,    "mqttTask",    6144, NULL, 2, NULL);
}

void loop() {
  vTaskDelay(portMAX_DELAY);   // all work is done in FreeRTOS tasks
}
