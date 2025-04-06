# MicroPython-ESP32-Projects

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![MicroPython](https://img.shields.io/badge/MicroPython-1.19-brightgreen)](https://micropython.org/)
[![ESP32](https://img.shields.io/badge/ESP32-Compatible-blue)](https://www.espressif.com/en/products/socs/esp32)

A collection of MicroPython projects designed for ESP32 microcontrollers with various sensors, focusing on IoT applications with MQTT connectivity.

## 📋 Table of Contents

- [🔧 Projects](#-projects)
  - [BreakSense](#breaksense)
  - [TempSense](#tempsense)
  - [LightCont](#lightcont)
- [🔄 Shared Configuration](#-shared-configuration)
- [⬇️ Installation](#️-installation)
- [🚀 Usage](#-usage)
- [📁 Repository Structure](#-repository-structure)
- [🔒 Security Notes](#-security-notes)
- [📜 License](#-license)

## 🔧 Projects

### BreakSense

A digital brake state monitoring system for ESP32 microcontrollers.

**Features:**
- Wi-Fi and MQTT connectivity with automatic reconnection
- GPIO pin monitoring with software debouncing
- Configurable settings (debounce time, topic, pin)
- Unique client ID based on ESP32 MAC address
- MQTT Last Will and Testament for offline detection

**Technical Details:**
- Monitors a digital brake sensor on a configurable GPIO pin
- Publishes state changes (`HIGH`/`LOW`) to an MQTT topic
- Uses debouncing to prevent false triggers
- Configurable delay, pin, and debug options

### TempSense

A temperature and humidity monitoring system that uses I2C sensors with the ESP32.

**Features:**
- Wi-Fi and MQTT connectivity with automatic reconnection
- I2C sensor integration
- Exponential Moving Average (EMA) data smoothing
- Smart publishing (only when significant changes occur)
- Configurable thresholds and parameters

**Configuration Options:**
- `MQTT_TOPIC` (default: `'esp32/tempsense'`)
- `LOOP_DELAY` (default: `5` seconds)
- `EMA_ALPHA` smoothing factor (default: `0.1`)
- `TEMP_THRESHOLD` (default: `0.3` °C)
- `HUMIDITY_THRESHOLD` (default: `1.0` %RH)
- I2C pins and frequency settings

**How It Works:**
1. Connects to Wi-Fi and MQTT broker
2. Sets up LWT message for offline detection
3. Reads temperature and humidity from I2C sensor
4. Applies EMA smoothing algorithm to reduce noise
5. Publishes data to MQTT when changes exceed thresholds
6. Automatically reconnects if connections drop

### LightCont

> ℹ️ This project is in development. Documentation will be updated as the project progresses.

## 🔄 Shared Configuration

All projects use a common `secrets.py` file for sensitive configuration values.

### Required `secrets.py` Content

```python
# Wi-Fi credentials
WIFI_SSID = "your_wifi_ssid"
WIFI_PASSWORD = "your_wifi_password"

# MQTT broker details
MQTT_BROKER = "mqtt.example.com"  # IP address or hostname
MQTT_PORT = 1883                  # Default MQTT port
```

⚠️ **IMPORTANT**: This file is excluded from version control via `.gitignore`. Never commit or share this file.

## ⬇️ Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/magealexstra/MicroPython.git
   cd MicroPython-'Project'
   ```

2. **Create `secrets.py`**
   Create this file in the project root with your credentials (see [Shared Configuration](#-shared-configuration))

3. **Project-specific setup**
   - **BreakSense**: Connect a digital sensor to GPIO pin (default: GPIO2)
   - **TempSense**: Connect an I2C temperature/humidity sensor (e.g., SHT31)
     - Default pins: SCL=GPIO22, SDA=GPIO21
     - Default I2C address: 0x44 (SHT31)

4. **Upload to ESP32**
   Using tools like `ampy`, `rshell`, or Thonny IDE:
   ```bash
   # Example with ampy
   ampy --port /dev/ttyUSB0 put secrets.py
   ampy --port /dev/ttyUSB0 put TempSense/TempSense.py /TempSense.py
   ```

## 🚀 Usage

Each project runs independently on an ESP32. After uploading the files, you can:

1. **Start the script**
   ```python
   # On the ESP32 REPL
   import TempSense  # or BreakSense
   ```

2. **For automatic startup**
   Create a `main.py` file on your ESP32:
   ```python
   # Choose which project to run
   import TempSense  # or BreakSense
   ```

3. **Monitoring**
   - Use an MQTT client to subscribe to the respective topics:
     - BreakSense: `esp32/breaksense`
     - TempSense: `esp32/tempsense`

## 📁 Repository Structure

```
MicroPython-ESP32-Projects/
├── BreakSense/
│   └── BreakSense.py     # Brake sensor monitoring code
├── LightCont/
│   └── .gitkeep          # Placeholder for upcoming project
├── TempSense/
│   └── TempSense.py      # Temperature/humidity monitoring code
├── .gitignore            # Git ignore configuration
├── LICENSE               # MIT License file
└── README.md             # This documentation file
```

## 🔒 Security Notes

- Never commit `secrets.py` or any file containing credentials
- The `.gitignore` file is configured to exclude `secrets.py`
- Consider using environment-specific or per-device configurations for production deployments

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

Made with ❤️ for the MicroPython and ESP32 community
