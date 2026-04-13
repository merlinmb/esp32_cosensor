# ESP32 CO2 Sensor Monitor

An ESP32-based CO2 monitoring device using a Sensirion SCD4x sensor, with a 256x64 OLED display, MQTT telemetry, and a built-in web server.

## Features

- CO2, temperature, and humidity readings via Sensirion SCD4x (I2C)
- 256x64 OLED display (SSD1322) showing live readings and a sparkline history chart
- MQTT publishing of sensor data to a home automation broker
- Built-in web server for status, manual refresh, and OTA firmware updates
- mDNS support (`espCO2Sensor.local`)
- Display brightness control via MQTT
- Configurable reading interval (default: 5 minutes)

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | ESP32 DOIT DevKit V1 |
| CO2 Sensor | Sensirion SCD4x (I2C) |
| Display | ER-OLEDM032-1W — 256x64 white OLED, SSD1322 controller, 3.2" diagonal, 4-wire HW SPI |

### Pin Connections

| Signal | ESP32 Pin |
|--------|-----------|
| Display CS | GPIO 5 |
| Display DC | GPIO 17 |
| Display Reset | GPIO 16 |
| SCD4x SDA/SCL | Default I2C pins |

## Software / Build Environment

Built with [PlatformIO](https://platformio.org/) targeting the `esp32doit-devkit-v1` board.

### Dependencies

| Library | Source |
|---------|--------|
| PubSubClient | `knolleary/PubSubClient@^2.8` |
| NTPClient | `https://github.com/taranais/NTPClient` |
| Time | `PaulStoffregen/Time` |
| Sensirion I2C SCD4x | `https://github.com/Sensirion/arduino-i2c-scd4x` |
| u8g2 | `olikraus/u8g2` |
| WiFiClientSecure, WebServer, ArduinoOTA | ESP32 Arduino framework |

### Local Libraries

The project also depends on private local libraries located in `../libraries/merlinIncludes`:

- `merlinNetwork` — WiFi connection management with dual AP fallback
- `merlinUpdateWebServer` — shared web server scaffolding and OTA helpers

These are not included in this repository.

## Configuration

Connection credentials are stored in `src/connectionDetails.h` (not committed). Create this file with your network and MQTT settings:

```cpp
// src/connectionDetails.h
#define WIFI_ACCESSPOINT   "your-ssid"
#define WIFI_PASSWORD      "your-password"

#define MQTT_SERVER_IP     "192.168.x.x"
#define MQTT_SERVER_PORT   1883
#define MQTT_SERVERADDRESS "192.168.x.x"
#define MQTT_CLIENTNAME    "espCO2Sensor"
```

## MQTT Topics

| Topic | Direction | Description |
|-------|-----------|-------------|
| `stat/espCO2Sensor/CO2Value` | publish | CO2 reading in ppm |
| `stat/espCO2Sensor/Temperature` | publish | Temperature in °C |
| `stat/espCO2Sensor/Humidity` | publish | Relative humidity % |
| `stat/espCO2Sensor/delay` | publish | Current poll interval in seconds |
| `cmnd/espCO2Sensor/sendstat` | subscribe | Re-read sensor and publish stats |
| `cmnd/espCO2Sensor/reset` | subscribe | Reboot the device |
| `cmnd/mcmddevices/brightness` | subscribe | Set display brightness (0–255) |
| `cmnd/mcmddevices/brightnesspercentage` | subscribe | Set display brightness (0–100%) |

## Web Interface

Access via `http://espCO2Sensor.local` or the device IP.

| Endpoint | Description |
|----------|-------------|
| `/` | Status page with current readings and links |
| `/sendstat` | Trigger a sensor read and MQTT publish |
| `/getraw` | Read sensor and return JSON |
| `/updatedisplay` | Force a display refresh |
| `/firmware` | OTA firmware upload page |
| `/reset` | Reboot the device |

## Building and Flashing

```bash
# Install PlatformIO CLI or use the VS Code extension

# Build
pio run

# Upload (device on COM4 by default)
pio run --target upload

# Monitor serial output
pio device monitor
```
