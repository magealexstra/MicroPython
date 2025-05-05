# MicroPython-ESP32-Projects

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![MicroPython](https://img.shields.io/badge/MicroPython-~1.19+-brightgreen)](https://micropython.org/)
[![ESP32](https://img.shields.io/badge/ESP32-Compatible-blue)](https://www.espressif.com/en/products/socs/esp32)
[![ESP8266](https://img.shields.io/badge/ESP8266-Compatible%20(WifiDetect)-blue)](https://www.espressif.com/en/products/socs/esp8266)

A collection of MicroPython projects designed for ESP32 microcontrollers (and ESP8266 for WifiDetect) with various sensors, focusing on IoT applications with MQTT connectivity.

## 📋 Table of Contents

- [🔧 Projects](#-projects)
  - [BreakSense](#breaksense)
  - [TempSense](#tempsense)
  - [LightCont](#lightcont)
  - [LivingRoomIOT](#livingroomiot)
  - [WifiDetect](#wifidetect)
- [🔄 Shared Configuration (`secrets.py`)](#-shared-configuration-secretspy)
- [⬇️ Installation](#️-installation)
- [🚀 Usage](#-usage)
- [📁 Repository Structure](#-repository-structure)
- [🔒 Security Notes](#-security-notes)
- [📜 License](#-license)

## 🔧 Projects

### BreakSense

Monitors a digital input (like a break beam sensor or switch) and reports state changes via MQTT.

**Features:**
- 📶 **Connectivity:** Connects to Wi-Fi and MQTT broker.
- 🚦 **Digital Input Monitoring:** Reads the state of a configured GPIO pin.
- ⏳ **Debouncing:** Software debouncing prevents false triggers from noisy signals.
- 📢 **MQTT Publishing:** Publishes state changes (`HIGH`/`LOW`) to a specific topic.
- 🚨 **LWT Status:** Publishes `ONLINE`/`OFFLINE` status using MQTT Last Will and Testament.
- 🆔 **Unique Client ID:** Uses part of the device MAC address for a unique MQTT client ID.
- 🔄 **Auto-Reconnect:** Attempts to reconnect to Wi-Fi and MQTT if connections drop.
- ⚙️ **Configuration:** Easily configure GPIO pin, MQTT topic, debounce time, and loop delay.
- 🐛 **Debugging:** Optional debug messages via `DEBUG` flag.

**Configuration Options (within `BreakSense.py`):**
- `MQTT_TOPIC`: Base topic for state and status messages.
- `GPIO_PIN`: Pin connected to the digital sensor/switch.
- `DEBUG`: Enable/disable verbose logging.
- `DEBOUNCE_TIME_MS`: Debounce duration in milliseconds.
- `LOOP_DELAY_S`: Main loop delay in seconds.

**MQTT Topics:**
- `{MQTT_TOPIC}`: Publishes `HIGH` or `LOW` for sensor state, and `ONLINE`/`OFFLINE` for device status (LWT).

**How It Works:**
1. Initializes the configured GPIO pin as an input with a pull-up resistor.
2. Connects to Wi-Fi and the MQTT broker (setting LWT).
3. Enters main loop:
    a. Checks Wi-Fi/MQTT connections and attempts reconnection if needed.
    b. Reads the digital input pin state.
    c. Applies debouncing logic to filter out noise.
    d. If a stable state change is detected, publishes `HIGH` or `LOW` to the MQTT topic.
    e. Sleeps for `LOOP_DELAY_S`.

### TempSense

Reads temperature and humidity from an I2C sensor, smooths the data, and publishes it via MQTT when significant changes occur.

**Features:**
- 📶 **Connectivity:** Connects to Wi-Fi and MQTT broker.
- 🌡️ **I2C Sensing:** Reads Temperature & Humidity (example code for SHT31 provided).
- 💧 **Data Smoothing:** Applies Exponential Moving Average (EMA) to readings.
- 📉 **Threshold Publishing:** Only publishes data (JSON format) when changes exceed configured thresholds, reducing MQTT traffic.
- 🚨 **LWT Status:** Publishes `ONLINE`/`OFFLINE` status using MQTT Last Will and Testament.
- 🆔 **Unique Client ID:** Uses part of the device MAC address for a unique MQTT client ID.
- 🔄 **Auto-Reconnect:** Attempts to reconnect to Wi-Fi and MQTT if connections drop.
- ⚙️ **Configuration:** Configure I2C pins, sensor address, MQTT topic, smoothing factor, thresholds, and loop delay.
- 🐛 **Debugging:** Optional debug messages via `DEBUG` flag.

**Configuration Options (within `TempSense.py`):**
- `MQTT_TOPIC`: Base topic for sensor data and status messages.
- `DEBUG`: Enable/disable verbose logging.
- `LOOP_DELAY_S`: Main loop delay (sensor read interval).
- `EMA_ALPHA`: Smoothing factor (0.0-1.0).
- `TEMP_THRESHOLD`, `HUMIDITY_THRESHOLD`: Minimum change to trigger publish.
- `I2C_SCL_PIN`, `I2C_SDA_PIN`, `I2C_FREQ`: I2C bus settings.
- `SENSOR_I2C_ADDR`: I2C address of your sensor.

**MQTT Topics:**
- `{MQTT_TOPIC}`: Publishes JSON `{"temperature": T, "humidity": H}` for sensor data, and `ONLINE`/`OFFLINE` for device status (LWT).

**How It Works:**
1. Initializes the I2C bus and checks for the sensor.
2. Connects to Wi-Fi and the MQTT broker (setting LWT).
3. Enters main loop:
    a. Checks Wi-Fi/MQTT connections and attempts reconnection if needed.
    b. Reads temperature and humidity from the I2C sensor.
    c. Applies EMA smoothing to the readings.
    d. Compares smoothed values to the last published values.
    e. If the change exceeds `TEMP_THRESHOLD` or `HUMIDITY_THRESHOLD`, publishes the new data as a JSON payload to the MQTT topic.
    f. Sleeps for `LOOP_DELAY_S`.

### LightCont

Controls multiple relays via MQTT commands or physical buttons/switches, with optional automatic control based on an analog light sensor.

**Features:**
- 📶 **Connectivity:** Connects to Wi-Fi and MQTT broker.
- 💡 **Multi-Relay Control:** Manages multiple GPIO pins configured as relay outputs.
- 🔘 **Physical Inputs:** Supports both momentary push buttons (toggle) and static toggle switches (follow state), mapped to specific relays.
- ✨ **Optional Light Sensing:** Reads an analog light sensor.
- ⚙️ **Optional Auto Light Control:** Automatically turns specified relays ON/OFF based on a configurable light threshold.
- 🆔 **Unique Device Topics (Optional):** Can use device MAC address in MQTT topics (`MQTT_USE_DEVICE_ID`) to avoid conflicts.
- ⏳ **Debouncing:** Software debouncing for physical button/switch inputs.
- 🚨 **LWT Status:** Publishes `ONLINE`/`OFFLINE` status using MQTT Last Will and Testament.
- 🔄 **Connection Retries/Reset:** Attempts reconnection with exponential backoff and can reset the device after prolonged failures.
- ⚙️ **Configuration:** Define relay/button pins, mappings, button types, light sensor settings, MQTT structure, timing, etc.
- 🐛 **Debugging:** Optional debug messages via `DEBUG` flag.

**Configuration Options (within `LightCont.py`):**
- `DEBUG`: Enable/disable verbose logging.
- `MQTT_TOPIC_PREFIX`, `MQTT_USE_DEVICE_ID`: Control MQTT topic structure.
- `LOOP_DELAY`, `MAX_CONNECTION_RETRIES`, `DEBOUNCE_MS`: Timing and connection settings.
- `RELAY_PINS`: List of GPIO pins for relays.
- `BUTTON_PINS`: List of GPIO pins for buttons/switches.
- `BUTTON_RELAY_MAP`: Dictionary mapping button pins to the relay pins they control.
- `BUTTON_TYPES`: Dictionary specifying "momentary" or "static" for each button pin.
- `LIGHT_SENSOR_PIN`: Analog input pin for light sensor (set to `None` to disable).
- `LIGHT_THRESHOLD`: ADC value threshold for auto light control.
- `LIGHT_CHECK_INTERVAL`: How often (seconds) to check the light sensor.
- `LIGHT_CONTROLLED_RELAYS`: List of relay pins to be controlled by the light sensor.

**MQTT Topics (examples assume `MQTT_USE_DEVICE_ID=True`, base=`esp32/lightcontrol/{device_id}`):**
- Device Status: `{base}/state` (Publishes ONLINE/OFFLINE)
- Relay Command (All): `{base}/command` (Subscribes, expects ON/OFF) - *Only if `MQTT_USE_DEVICE_ID=True`*
- Relay Command (Specific): `{base}/{pin_number}/command` (Subscribes, expects ON/OFF)
- Relay State (Specific): `{base}/{pin_number}/state` (Publishes ON/OFF)
- *Note: If `MQTT_USE_DEVICE_ID=False`, topics use `esp32/lightcontrol/{pin_number}/...` structure.*

**How It Works:**
1. Initializes configured relays (OFF), buttons (input pull-up), and the light sensor ADC (if enabled).
2. Connects to Wi-Fi and the MQTT broker (sets LWT, subscribes to command topics, publishes initial relay states).
3. Enters main loop:
    a. Checks Wi-Fi/MQTT connections, attempts reconnection with exponential backoff, resets device if max retries exceeded.
    b. Checks physical buttons/switches (debounced), controls the mapped relay based on button type (momentary toggle or static follow), publishes new relay state.
    c. Checks light sensor periodically (if enabled), compares reading to `LIGHT_THRESHOLD`, and sets the state of `LIGHT_CONTROLLED_RELAYS` accordingly, publishing changes.
    d. Checks for incoming MQTT messages (main command or specific relay commands) and updates relay states.
    e. Sleeps for `LOOP_DELAY`.

### LivingRoomIOT

A comprehensive living room automation system integrating temperature/humidity sensing, multi-relay control, physical inputs (buttons/switches), generic open/close sensor monitoring, and optional light-based automation. Designed for ESP32 but adaptable.

**Features:**
- 🌡️ **AHT2x Sensing:** Reads temperature & humidity via I2C (requires external library).
- 💧 **Data Smoothing:** Applies Exponential Moving Average (EMA) to sensor readings.
- 📉 **Threshold Publishing:** Only publishes sensor data when changes exceed configured thresholds.
- 💡 **Relay Control:** Manages multiple relays via MQTT commands.
- 🔘 **Physical Inputs:** Supports both momentary push buttons and static toggle switches, mapped to specific relays.
- 🚪 **Generic Sensors:** Monitors multiple open/close sensors (e.g., door, window reed switches) using a flexible configuration.
- ✨ **Light Sensing:** Reads analog light sensor values (optional).
- 🌗 **Daylight Events:** Publishes DAY/NIGHT transition events based on light sensor and synchronized time (optional).
- ⚙️ **Automatic Light Control:** Controls specified relays based on ambient light level (optional).
- 📶 **Connectivity:** Robust Wi-Fi and MQTT connection management (LWT, unique ID option, exponential backoff, auto-reset on failure).
- 🕒 **Time Sync:** Synchronizes internal clock using NTP.
- ⚙️ **Configuration:** Highly configurable via constants within the script and `secrets.py`.
- 🐛 **Debugging:** Optional detailed logging.
- 🔘 **Button Events:** Publishes physical button press events via MQTT.

**Configuration Options (within `LivingRoomIOT.py`):**
- `DEBUG`: Enable/disable verbose logging.
- `MQTT_TOPIC_PREFIX`, `MQTT_USE_DEVICE_ID`: Control MQTT topic structure.
- `MQTT_TOPIC_SENSOR`, `MQTT_TOPIC_LIGHT_SENSOR`, `MQTT_TOPIC_DAYLIGHT_EVENT`: Sub-topic names.
- I2C settings: `I2C_SCL_PIN`, `I2C_SDA_PIN`, `I2C_FREQ`, `SENSOR_ADDR` (for AHT2x).
- AHT2x settings: `EMA_ALPHA`, `TEMP_THRESHOLD`, `HUMIDITY_THRESHOLD`.
- Light Sensor settings: `LIGHT_SENSOR_PIN` (set to `None` to disable), `LIGHT_THRESHOLD`, `LIGHT_CHECK_INTERVAL`, `LIGHT_CONTROLLED_RELAYS`.
- GPIO Pins: `RELAY_PINS`, `BUTTON_PINS`.
- Input Mapping: `BUTTON_RELAY_MAP` (button pin -> relay pin), `BUTTON_TYPES` (pin -> "momentary" or "static").
- Generic Sensors: `SENSOR_CONFIG` dictionary (sensor name -> {pin, topic, debounce_ms}). Set pin to `None` to disable a sensor.
- Debounce: `BUTTON_DEBOUNCE_MS` (per-sensor debounce defined in `SENSOR_CONFIG`).
- Timing/Connection: `LOOP_DELAY`, `MAX_CONNECTION_RETRIES`.

**MQTT Topics (assuming `MQTT_USE_DEVICE_ID = True`, base = `livingroom/{device_id}`):**
- Device Status: `{base}/state` (Publishes ONLINE/OFFLINE)
- AHT2x Data: `{base}/sensor` (Publishes JSON `{"temperature": T, "humidity": H}`)
- Generic Sensor State: `{base}/{sensor_topic}` (e.g., `{base}/door`, `{base}/window`. Publishes OPEN/CLOSED)
- Light Sensor Reading: `{base}/light` (Publishes raw ADC value, optional)
- Daylight Event: `{base}/daylight_event` (Publishes JSON `{"event": "DAY/NIGHT", "utc_timestamp": epoch}`)
- Relay Command (All): `{base}/command` (Subscribes, expects ON/OFF)
- Relay Command (Specific): `{base}/relay/{pin_number}/command` (Subscribes, expects ON/OFF)
- Relay State (Specific): `{base}/relay/{pin_number}/state` (Publishes ON/OFF)
- Button Event: `{base}/button_event` (Publishes JSON `{"button_pin": P, "button_type": T, "state": S}`)

**How It Works:**
1. Initializes hardware: Relays (OFF), Buttons (PULL_UP), Generic Sensors (PULL_UP), I2C AHT2x, ADC Light Sensor (if enabled).
2. Connects to Wi-Fi, syncs time via NTP.
3. Connects to MQTT broker (sets LWT, subscribes to command topics, publishes initial ONLINE status and sensor/relay states).
4. Enters main loop:
    a. Checks Wi-Fi/MQTT connections, attempts reconnection with exponential backoff, resets device if max retries exceeded.
    b. Reads AHT2x, applies EMA, publishes to `{base}/sensor` if change exceeds thresholds.
    c. Checks physical buttons/switches (debounced), controls mapped relay based on type (momentary/static), publishes state to `{base}/relay/{pin}/state`.
    d. Checks generic sensors (debounced), publishes state change (OPEN/CLOSED) to `{base}/{sensor_topic}`.
    e. Checks physical buttons/switches (debounced), publishes button event to `{base}/button_event` and controls the mapped relay based on button type (momentary toggle or static follow), publishing new relay state to `{base}/relay/{pin}/state`.
    f. Checks light sensor periodically (if enabled and time synced):
        i. Detects DAY/NIGHT transitions, publishes event to `{base}/daylight_event`.
        ii. Controls `LIGHT_CONTROLLED_RELAYS` based on `LIGHT_THRESHOLD`.
    g. Checks for incoming MQTT messages (relay commands).
    h. Sleeps for `LOOP_DELAY`.

### WifiDetect

A lightweight WiFi network scanner for ESP8266 microcontrollers with an OLED display.

**Features:**
- 📡 **2.4GHz WiFi Scanning:** Detects nearby Wi-Fi networks (ESP8266 limitation).
- 💻 **OLED Display Output:** Shows network information on a connected SSD1306 display.
- 📶 **Signal Strength:** Displays signal strength as a percentage (%).
- ✨ **Sorted Network List:** Shows the top 4 strongest networks detected.
- ⏱️ **Periodic Scanning:** Scans for networks at a configurable interval.
- 🗑️ **Memory Optimization:** Includes garbage collection calls for stability on ESP8266.

**Configuration Options (within `WifiDetect.py`):**
- `SDA_PIN`, `SCL_PIN`: GPIO pins for the I2C OLED display.
- `SCAN_INTERVAL`: Time in seconds between network scans.

**MQTT Topics:**
- None. This project uses the OLED display for output.

**How It Works:**
1. Initializes the I2C OLED display and the ESP8266's Wi-Fi interface.
2. Enters main loop:
    a. Periodically scans for available 2.4GHz Wi-Fi networks.
    b. Calculates signal strength percentage for each network.
    c. Sorts the detected networks by signal strength (strongest first).
    d. Clears the OLED display.
    e. Displays the SSID (truncated if necessary) and signal percentage for the top 4 networks found.
    f. Includes garbage collection calls to manage memory.
    g. Repeats after `SCAN_INTERVAL`.

## 🔄 Shared Configuration (`secrets.py`)

Most projects rely on a `secrets.py` file in the root directory of the microcontroller's filesystem for sensitive information.

### Required `secrets.py` Content

Create a file named `secrets.py` on your device with the following content, replacing the placeholder values with your actual credentials:

```python
# Wi-Fi Credentials
WIFI_SSID = "your_wifi_ssid"
WIFI_PASSWORD = "your_wifi_password"

# MQTT Broker Details (if using MQTT-based projects)
MQTT_BROKER = "your_mqtt_broker_ip_or_hostname"
MQTT_PORT = 1883  # Default MQTT port (change if needed, e.g., 8883 for TLS)

# Optional: MQTT Username/Password (uncomment and set if required by your broker)
# MQTT_USER = "your_mqtt_username"
# MQTT_PASSWORD = "your_mqtt_password"
```

⚠️ **IMPORTANT**: This file is excluded from version control via `.gitignore`. **Never commit or share your `secrets.py` file.**

## ⬇️ Installation

1.  **Clone the Repository:**
    ```bash
    git clone https://github.com/magealexstra/MicroPython.git
    cd MicroPython
    ```

2.  **Prepare `secrets.py`:**
    *   Copy the example file: `cp secrets_example.py secrets.py`
    *   Edit the new `secrets.py` file on your computer with your actual Wi-Fi and MQTT broker details (see [Shared Configuration](#-shared-configuration-secretspy)).

3.  **Connect Hardware:**
    *   Connect your ESP32 or ESP8266 board to your computer.
    *   Connect the required sensors, relays, buttons, or displays to the GPIO pins specified in the configuration section of the specific project script you intend to run (e.g., `LightCont.py`, `TempSense.py`). **Refer to the `[CONFIG]` comments within the `.py` file for default pin assignments.**

4.  **Upload Files to Device:**
    *   Use a tool like Thonny IDE, `rshell`, or `ampy` to transfer files to your microcontroller's filesystem.
    *   **Essential:** Upload the `secrets.py` file you prepared to the root directory of the device.
    *   Upload the main Python script for the project you want to run (e.g., `LightCont/LightCont.py`) to the root directory or a subdirectory on the device.
    *   **WifiDetect Specific:** Also upload the `WifiDetect/ssd1306.py` driver file to the root directory on the ESP8266.
    *   **LivingRoomIOT Specific:** Also upload the `aht.py` driver library file to the same directory as `LivingRoomIOT.py` on the device.

    *Example using Thonny:* Open the files (`secrets.py`, `ProjectName.py`, `ssd1306.py` if needed) and use `File > Save copy... > MicroPython device`.*

    *Example using `ampy` (replace `/dev/ttyUSB0` with your device's serial port):*
    ```bash
    # Upload secrets
    ampy --port /dev/ttyUSB0 put secrets.py secrets.py

    # Upload project script (e.g., LightCont)
    ampy --port /dev/ttyUSB0 put LightCont/LightCont.py LightCont.py

    # Upload driver if needed (e.g., for WifiDetect)
    # ampy --port /dev/ttyUSB0 put WifiDetect/ssd1306.py ssd1306.py
    ```

## 🚀 Usage

1.  **Running the Script:**
    *   **Manual Start (REPL):** Connect to your device's MicroPython REPL (using Thonny, `rshell`, `screen`, etc.) and import the script:
        ```python
        >>> import LightCont # Or BreakSense, TempSense, LivingRoomIOT, WifiDetect
        # The script's main() function will typically run automatically if __name__ == '__main__'
        ```
    *   **Automatic Start (on boot):** Rename the project script you uploaded to `main.py` on the device's root filesystem. The script will run automatically when the device boots or resets.
        *(Example using `ampy` to rename `LightCont.py` to `main.py`):*
        ```bash
        # ampy --port /dev/ttyUSB0 put LightCont/LightCont.py main.py
        ```

2.  **Interaction & Monitoring:**
    *   **MQTT Projects (BreakSense, TempSense, LightCont, LivingRoomIOT):**
        *   Use an MQTT client (like MQTT Explorer, mosquitto_sub, Node-RED) to connect to your `MQTT_BROKER`.
        *   Subscribe to the relevant topics listed in the project's "MQTT Topics" section above to monitor sensor data and device status.
        *   Publish messages (typically `ON` or `OFF`) to the command topics to control relays. Remember to use the correct topic structure based on your `MQTT_USE_DEVICE_ID` setting.
    *   **WifiDetect:**
        *   Observe the connected OLED display. It will automatically scan and show the strongest nearby 2.4GHz Wi-Fi networks and their signal strength percentages. The display updates every `SCAN_INTERVAL` seconds.
    *   **Physical Controls (LightCont, LivingRoomIOT):**
        *   Press connected momentary buttons or flip static switches to control the mapped relays.

## 📁 Repository Structure

```
MicroPython-ESP32-Projects/
├── BreakSense/
│   └── BreakSense.py     
├── LightCont/
│   └── LightCont.py      
├── LivingRoomIOT/
│   ├── LivingRoomIOT.py  
│   └── aht.py            # AHT2x sensor driver
├── TempSense/
│   └── TempSense.py      
├── WifiDetect/
│   ├── WifiDetect.py     
│   └── ssd1306.py        # OLED display driver library
├── .gitignore            # Git ignore configuration (excludes secrets.py)
├── LICENSE               
├── README.md             #
└── secrets_example.py    # Template for creating secrets.py
```
*(Note: Some directories might contain `.gitkeep` files which are placeholders for Git)*

## 🔒 Security Notes

- The `secrets.py` file contains sensitive credentials. **Do not share it or commit it to version control.**
- The `.gitignore` file is configured to prevent accidental commits of `secrets.py`.
- For production environments, consider more robust methods for managing secrets than a plain text file on the device filesystem.
- Ensure your Wi-Fi network and MQTT broker are secured appropriately.

## Credits

This project utilizes the aht.py driver, for which we thank Jonathan Fromentin. You can explore his other work at https://github.com/etno712.

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

Made with ❤️ for the MicroPython and ESP32/ESP8266 community
