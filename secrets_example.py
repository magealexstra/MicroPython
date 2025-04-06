"""
MicroPython ESP32 Projects - Secrets Configuration Example

This file serves as a template for creating your own 'secrets.py' file.
To use it:
1. Copy this file to 'secrets.py' in the same directory
2. Replace the example values with your actual credentials
3. Upload the file to your ESP32 device

⚠️ SECURITY WARNING:
- Never commit your 'secrets.py' file to version control
- Keep your credentials private
- This file is excluded from git via .gitignore
"""

# Wi-Fi credentials
# Replace with your actual Wi-Fi network name (case-sensitive)
WIFI_SSID = "your_wifi_network_name"

# Replace with your actual Wi-Fi password
WIFI_PASSWORD = "your_wifi_password"

# MQTT broker details
# Replace with your MQTT broker's IP address or hostname
# Examples:
# - Local Mosquitto: "192.168.1.100" or "raspberrypi.local"
# - Public broker: "broker.hivemq.com" or "test.mosquitto.org"
MQTT_BROKER = "mqtt.example.com"

# MQTT port (standard port is 1883 for unencrypted connections)
# Change if your broker uses a different port
# Common ports: 1883 (standard), 8883 (TLS/SSL), 8884, etc.
MQTT_PORT = 1883

# Optional: MQTT authentication (if your broker requires it)
# Uncomment and fill these in if needed
# MQTT_USERNAME = "mqtt_user"
# MQTT_PASSWORD = "mqtt_password"

# Optional: SSL/TLS settings (for secure connections)
# Uncomment and set to True if your broker requires SSL/TLS
# MQTT_SSL = False

"""
Advanced settings - Uncomment and modify as needed
"""

# Device naming (optional)
# This will be used as a prefix for MQTT client IDs and topics
# DEVICE_NAME = "esp32_device1"  

# Device location (optional)
# Used for more specific topic naming
# DEVICE_LOCATION = "living_room"
