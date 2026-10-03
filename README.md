# ESP32-Based Smart Environmental Monitoring System

An ESP32 node that measures **temperature, humidity, barometric pressure, air quality and light intensity**, shows the values on an OLED display, raises a buzzer/LED alert when a threshold is crossed, and publishes the data as JSON over **MQTT** to an IoT dashboard.

Developed as part of an Industrial Internship (upskill Campus / The IoT Academy with UniConverge Technologies Pvt Ltd).

**Author:** B. Aathi Varaghan, B.E. Electronics and Communication Engineering, SCAD College of Engineering and Technology (Anna University)

## Features
- Five parameters in one low-cost node
- Local OLED display and buzzer/LED alert
- Wi-Fi + MQTT (QoS 1) JSON telemetry
- FreeRTOS tasks: sensing, alert logic, display and MQTT run independently
- Automatic Wi-Fi/MQTT reconnection with a 50-reading offline RAM buffer
- Watchdog on the sensor task

## Hardware
| Component | ESP32 pin | Notes |
|---|---|---|
| DHT22 | GPIO4 | 10 kΩ pull-up to 3.3 V |
| BMP280 (I2C 0x76) | GPIO21 SDA / GPIO22 SCL | shared I2C bus |
| SSD1306 OLED (I2C 0x3C) | GPIO21 SDA / GPIO22 SCL | 0.96", 128x64 |
| MQ-135 | GPIO34 (ADC1_CH6) | output scaled to 3.3 V max with a voltage divider |
| LDR + 10 kΩ divider | GPIO35 (ADC1_CH7) | |
| Buzzer | GPIO25 | via BC547 transistor, 1 kΩ base resistor |
| Status LED | GPIO2 | 220 Ω series resistor |

Diagrams are in [`docs/`](docs): high-level block diagram, pin connections and firmware flow chart.

## Getting started
1. Install the Arduino IDE and the **ESP32** board package.
2. Install these libraries from the Library Manager: *DHT sensor library*, *Adafruit Unified Sensor*, *Adafruit BMP280 Library*, *Adafruit SSD1306*, *Adafruit GFX Library*, *MQTT* (by Joel Gaehwiler).
3. In `firmware/esp32_env_monitor/`, copy `secrets.h.example` to `secrets.h` and enter your Wi-Fi and MQTT broker details.
4. Select **ESP32 Dev Module**, open `esp32_env_monitor.ino`, upload, and open the Serial Monitor at 115200 baud.
5. Pre-heat the MQ-135 and calibrate `MQ_R0_KOHM` in clean air for sensible air-quality values.

## MQTT payload
Topic: `esp32/env/telemetry`
```json
{"device":"esp32-env-01","temp_c":28.4,"hum_pct":61.2,"press_hpa":1008.6,"aq_ppm":412,"light_pct":73,"alert":0}
```

## Repository layout
```
firmware/esp32_env_monitor/   Arduino sketch and secrets template
docs/                         Block, wiring and flow diagrams
report/                       Internship project report (PDF and Word)
```

## Notes
- The MQ-135 gives a relative CO2-equivalent estimate suited to trends and alerts, not lab-grade measurement.
- Pins, thresholds and calibration constants are defined at the top of the sketch; change them to match your build.

## License
MIT, see [LICENSE](LICENSE).
