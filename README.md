# Smart Home Safety & Energy Automation System

An ESP32-based IoT system that senses temperature, gas, motion, and light, automates safety and comfort responses locally, and streams live data to an Adafruit IO dashboard over MQTT. Built for 3707ICT (Automation and IoT).

## Features

- Monitors temperature, humidity, gas/smoke, motion, and light
- Runs four automation rules: smart lighting, temperature-based ventilation, gas/smoke safety, and away-mode security
- Computes a Home Safety Index — a 0–100 risk score fusing all sensors — to drive adaptive sensor/cloud polling
- Supports Home, Night, and Away modes that change how motion is interpreted
- Publishes live and historical data to Adafruit IO over MQTT

## Hardware

- ESP32 DevKit
- DHT22 (temperature + humidity)
- MQ-2 (gas/smoke)
- PIR motion sensor
- LDR (light level)
- RGB LED, buzzer, servo

# Running it

### In Wokwi (simulation)

1. Create a new ESP32 project at [wokwi.com](https://wokwi.com).
2. Copy in `sketch.ino` and `diagram.json`.
3. Add the libraries listed in `libraries.txt` via the Library Manager.
4. Fill in your own Adafruit IO username and key (get these from [io.adafruit.com](https://io.adafruit.com)  --> My Key).
5. Click Play. The Serial Monitor confirms Wi-Fi and MQTT connection; feeds appear automatically in your Adafruit IO account once data starts publishing.

 https://wokwi.com/projects/476135628216453121
