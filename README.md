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
  - [LivingRoomIOT](#livingroomiot)
  - [WifiDetect](#wifidetect)
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

A relay-based light controller system for ESP32 with both MQTT and physical control options.

**Features:**
- Wi-Fi and MQTT connectivity with automatic reconnection
- Support for multiple relay-controlled lights/devices
- Unique MQTT topics per device based on MAC address (avoid conflicts in multi-device setups)
- Support for both types of physical controls:
  - Momentary push buttons (toggle on each press)
  - Static toggle switches (light state follows switch position)
- Light sensor support with automatic light control based on ambient brightness
- Flexible button-to-relay mapping (control specific lights with buttons)
- Topic-based control for individual relays
- MQTT state updates whenever relay states change
- Exponential backoff for connection retries
- Unique client ID based on ESP32 MAC address
- MQTT Last Will and Testament for offline detection

**Configuration Options:**
- `RELAY_PINS` - List of GPIO pins connected to relays
- `BUTTON_PINS` - List of GPIO pins connected to buttons/switches
- `BUTTON_RELAY_MAP` - Mapping dictionary defining which button controls which relay
- `BUTTON_TYPES` - Dictionary defining button type ("momentary" or "static") for each pin
- `MQTT_USE_DEVICE_ID` - Boolean to enable/disable unique device-specific topics (default: `True`)
- `LIGHT_SENSOR_PIN` - GPIO pin for analog light sensor (optional)
- `LIGHT_THRESHOLD` - Light level threshold for automatic control (default: `500`)
- `LIGHT_CHECK_INTERVAL` - Seconds between light sensor checks (default: `10`)
- `LIGHT_CONTROLLED_RELAYS` - List of relay pins to control automatically with light sensor
- `DEBOUNCE_MS` - Button debounce time in milliseconds (default: `300`)
- `MAX_CONNECTION_RETRIES` - Maximum reconnection attempts before device reset

**How It Works:**
1. Initializes relay outputs, button inputs (with pull-up resistors), and light sensor (if configured)
2. Connects to Wi-Fi and MQTT broker with Last Will message
3. Creates unique MQTT topics using the device's MAC address to avoid conflicts
4. Subscribes to command topics for each relay
5. Monitors for MQTT commands to control relays
6. Processes physical controls based on their type:
   - Momentary buttons: toggle relay state on press
   - Static switches: relay directly follows switch position
7. Checks light sensor (if configured) and automatically controls specified relays based on brightness
8. Publishes state changes to MQTT whenever a relay changes
9. Automatically reconnects with exponential backoff if connections drop

### LivingRoomIOT

A comprehensive living room automation system integrating temperature/humidity sensing, multi-relay control, button inputs, and door monitoring. Designed for ESP32-S3-N16R8 but adaptable.

**Features:**
- Combines functionalities of TempSense, LightCont, and BreakSense (for door).
- Wi-Fi and MQTT connectivity with robust reconnection logic (exponential backoff, LWT, unique ID).
- SHT31 temperature and humidity sensing via I2C.
- Exponential Moving Average (EMA) smoothing for sensor data.
- Threshold-based publishing for sensor data to reduce traffic.
- Control for multiple relays via MQTT commands.
- Physical momentary switch control for mapped relays with debouncing.
- Door sensor monitoring (e.g., magnetic reed switch) with debouncing.
- Publishes distinct MQTT topics for sensors, door status, and individual relay states.
- Uses `secrets.py` for sensitive configuration.
- Configurable pins, thresholds, debounce times, and loop delay.
- Automatic device reset after prolonged connection failures.

**Configuration Options (within `LivingRoomIOT.py`):**
- `DEBUG` flag for verbose output.
- I2C pins (`I2C_SCL_PIN`, `I2C_SDA_PIN`), frequency (`I2C_FREQ`), and sensor address (`SENSOR_ADDR`).
- Sensor smoothing (`EMA_ALPHA`) and publishing thresholds (`TEMP_THRESHOLD`, `HUMIDITY_THRESHOLD`).
- GPIO pins for relays (`RELAY_PINS`), buttons (`BUTTON_PINS`), and door sensor (`DOOR_SENSOR_PIN`).
- Debounce times (`BUTTON_DEBOUNCE_MS`, `DOOR_DEBOUNCE_MS`).
- Main loop delay (`LOOP_DELAY`).
- Connection retry limit (`MAX_CONNECTION_RETRIES`).

**MQTT Topics:**
- Sensor Data: `livingroom/temperature_humidity` (JSON payload or ONLINE/OFFLINE)
- Door Status: `livingroom/door` (OPEN/CLOSED)
- Relay Commands: `livingroom/relay/{pin_number}/command` (Subscribes, expects ON/OFF)
- Relay States: `livingroom/relay/{pin_number}/state` (Publishes ON/OFF)

**How It Works:**
1. Initializes hardware: I2C sensor, relays (OFF), buttons (PULL_UP), door sensor (PULL_UP).
2. Connects to Wi-Fi and MQTT broker (with LWT, subscriptions, initial state publishing).
3. Enters main loop:
    a. Checks and attempts reconnection for Wi-Fi and MQTT if needed, using exponential backoff and resetting if max retries exceeded.
    b. Reads sensor, applies EMA, publishes to `livingroom/temperature_humidity` if change exceeds thresholds.
    c. Checks buttons (debounced), toggles corresponding relay, publishes state to `livingroom/relay/{pin}/state`.
    d. Checks door sensor (debounced), publishes state change to `livingroom/door`.
    e. Checks for incoming MQTT messages (relay commands).
    f. Sleeps for `LOOP_DELAY`.

### WifiDetect

A lightweight WiFi network scanner for ESP8266 microcontroller with OLED display. Shows available 2.4GHz networks and their signal strengths.

**Features:**
- Continuous WiFi network scanning
- Real-time display of network names and signal strengths
- Signal strength shown as percentage
- Clean, minimalist display format
- Memory-optimized for ESP8266 hardware
- Display of up to 4 networks simultaneously
- 10-second scan interval

**Technical Details:**
- ESP8266 ESP-12F microcontroller (2.4GHz WiFi only)
- SSD1306 0.96" OLED display (I2C: SDA on D1/GPIO5, SCL on D2/GPIO4)
- Ultra-minimal memory footprint for stable operation
- Optimized for power efficiency

**How It Works:**
1. Initializes hardware (OLED display, WiFi)
2. Continuously scans for available 2.4GHz WiFi networks
3. Displays network names and signal strength percentages
4. Sorts networks by signal strength (strongest first)
5. Updates display information every 10 seconds

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
   - Copy the provided template: `cp secrets_example.py secrets.py`
   - Edit `secrets.py` with your actual credentials
   - See [Shared Configuration](#-shared-configuration) for required values

3. **Project-specific setup**
   - **BreakSense**: Connect a digital sensor to GPIO pin (default: GPIO2)
   - **TempSense**: Connect an I2C temperature/humidity sensor (e.g., SHT31)
     - Default pins: SCL=GPIO22, SDA=GPIO21
     - Default I2C address: 0x44 (SHT31)
   - **LightCont**: Connect relays and momentary switches to ESP32
     - Default relay pin: GPIO5
     - Default button pin: GPIO4 (with pull-up resistor)
     - Customize pin assignments in the script as needed
   - **LivingRoomIOT**: Connect SHT31 sensor, relays, buttons, and door sensor.
     - Default I2C: SCL=GPIO22, SDA=GPIO21
     - Default Relays: GPIO5, 18, 19, 23
     - Default Buttons: GPIO4, 15, 16, 17 (with pull-up resistors)
     - Default Door Sensor: GPIO2 (with pull-up resistor)
     - Customize pin assignments and other constants in `LivingRoomIOT.py`.
   - **WifiDetect**: Connect OLED display and set up ESP8266 ESP-12F board.
     - OLED Display: SDA on D1 (GPIO5), SCL on D2 (GPIO4)
     - Five buttons: FLATH (GPIO0), RSET (hardware), D5 (GPIO14), D6 (GPIO12), D7 (GPIO13)
     - Battery monitoring via ADC (A0)
     - Charging indicator on D4 (GPIO2)

4. **Upload to ESP32/ESP8266**
   Using tools like `ampy`, `rshell`, or Thonny IDE:
   ```bash
   # Example with ampy for LivingRoomIOT
   ampy --port /dev/ttyUSB0 put secrets.py
   ampy --port /dev/ttyUSB0 put LivingRoomIOT/LivingRoomIOT.py /LivingRoomIOT.py
   
   # Example with ampy for WifiDetect on ESP8266
   ampy --port /dev/ttyUSB0 put secrets.py
   ampy --port /dev/ttyUSB0 put WifiDetect/ssd1306.py /ssd1306.py  # OLED display driver
   ampy --port /dev/ttyUSB0 put WifiDetect/WifiDetect.py /WifiDetect.py
   ```

## 🚀 Usage

Each project runs independently on an ESP32. After uploading the files, you can:

1. **Start the script**
   ```python
   # On the ESP32/ESP8266 REPL
   import TempSense  # or BreakSense, LightCont, LivingRoomIOT, WifiDetect
   ```

2. **For automatic startup**
   Create a `main.py` file on your ESP32/ESP8266:
   ```python
   # Choose which project to run
   import TempSense  # or BreakSense, LightCont, LivingRoomIOT, WifiDetect
   ```

3. **Monitoring and Interaction**
   - For MQTT-based projects, use an MQTT client to subscribe to the respective topics:
     - BreakSense: `esp32/breaksense`
     - TempSense: `esp32/tempsense`
     - LightCont:
       - Individual relay command topics: `esp32/lightcontrol/{pin_number}/command`
       - Individual relay state topics: `esp32/lightcontrol/{pin_number}/state`
     - LivingRoomIOT:
       - Sensor: `livingroom/temperature_humidity`
       - Door: `livingroom/door`
       - Relay Commands: `livingroom/relay/{pin_number}/command`
       - Relay States: `livingroom/relay/{pin_number}/state`
   - For WifiDetect:
     - View the top WiFi networks on the OLED display
     - Networks are automatically sorted by signal strength
     - Display updates every 10 seconds without user interaction
     - Shows up to 4 networks simultaneously

## 📁 Repository Structure

```
MicroPython-ESP32-Projects/
├── BreakSense/
│   └── BreakSense.py     # Brake sensor monitoring code
├── LightCont/
│   └── LightCont.py      # Light controller with relay and button support
├── LivingRoomIOT/
│   ├── DesignPlan.md     # Design notes for LivingRoomIOT
│   └── LivingRoomIOT.py  # Integrated living room automation script
├── TempSense/
│   └── TempSense.py      # Temperature/humidity monitoring code
├── WifiDetect/
│   ├── WifiDetect.py     # WiFi network scanner with OLED display
│   └── ssd1306.py        # OLED display driver
├── .gitignore            # Git ignore configuration
├── LICENSE               # MIT License file
├── README.md             # This documentation file
└── secrets_example.py    # Template for creating secrets.py configuration
```

## 🔒 Security Notes

- Never commit `secrets.py` or any file containing credentials
- The `.gitignore` file is configured to exclude `secrets.py`
- Consider using environment-specific or per-device configurations for production deployments

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

Made with ❤️ for the MicroPython and ESP32 community
