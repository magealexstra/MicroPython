# MicroPython Projects

A collection of MicroPython code for ESP32-based sensor modules.

## BreakSense Project

**BreakSense** is a MicroPython script for ESP32 that monitors a digital brake sensor connected to a GPIO pin and reports state changes to an MQTT server.

---

## Features

- Connects to Wi-Fi and MQTT broker with automatic reconnection
- Monitors GPIO pin with software debounce to avoid false triggers
- Publishes pin state changes (`HIGH`/`LOW`) to a configurable MQTT topic
- Uses a unique MQTT client ID based on the ESP32 MAC address
- Sets MQTT Last Will and Testament (LWT) to indicate device offline status
- Externalizes sensitive credentials in a separate secrets file
- Configurable debounce time, loop delay, and debug output

---

## Directory Structure

```
BreakSense/
  BreakSense.py        # Main MicroPython script
Secrets/
  secrets.py           # Wi-Fi and MQTT credentials (excluded from git)
README.md              # This documentation
.gitignore             # Ignores Secrets/ folder
```

---

## Setup Instructions

### 1. Create `Secrets/secrets.py`

Create a file named `Secrets/secrets.py` containing your **private Wi-Fi and MQTT credentials**. This file is **excluded from version control** for security.

Your `secrets.py` **must define** the following variables:

- `WIFI_SSID`: your Wi-Fi network name (string)
- `WIFI_PASSWORD`: your Wi-Fi password (string)
- `MQTT_BROKER`: the IP address or hostname of your MQTT broker (string)
- `MQTT_PORT`: the port number of your MQTT broker (integer)

**Example structure (do NOT share your real credentials):**

```python
WIFI_SSID = 'your_wifi_ssid'
WIFI_PASSWORD = 'your_wifi_password'

MQTT_BROKER = 'your_mqtt_broker_ip_or_hostname'
MQTT_PORT = 1883  # or your broker's port
```

### 2. Configure the script

In `BreakSense/BreakSense.py`, you can adjust:

- `MQTT_TOPIC` (default: `'esp32/breaksense'`)
- `GPIO_PIN` (default: `2`)
- `DEBOUNCE_TIME` (default: `0.02` seconds)
- `LOOP_DELAY` (default: `0.1` seconds)
- `DEBUG` flag (set to `False` to reduce console output)

### 3. Upload files to ESP32

Upload **both**:
- `BreakSense/BreakSense.py`
- `Secrets/secrets.py`

to your ESP32 using tools like `ampy`, `rshell`, or your IDE.

---

## How it works

- Connects to Wi-Fi using credentials from `secrets.py`.
- Connects to the MQTT broker with a unique client ID.
- Sets an LWT message (`OFFLINE`) to notify if the device disconnects unexpectedly.
- Monitors the GPIO pin for state changes with debounce filtering.
- Publishes `"HIGH"` or `"LOW"` to the MQTT topic on each stable change.
- If Wi-Fi or MQTT disconnects, the script automatically attempts to reconnect without blocking operation.

---

## Version Control Notes

The `Secrets/` directory is **excluded** from git via `.gitignore` to protect sensitive credentials.

---

## License

MIT License
