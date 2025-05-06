# Living Room IoT Controller - Status LED Guide

This document summarizes the color and pattern combinations for the RGB LED status indicator on the ESP32-S3, providing visual feedback on the device's operational state.

## Status Indicators

Here are the suggested states and their corresponding visual indicators:

*   **Booting / Initializing:**
    *   **Color:** White
    *   **Pattern:** Solid
    *   **Meaning:** The device is starting up and performing initial setup.

*   **Connecting to Wi-Fi:**
    *   **Color:** Blue
    *   **Pattern:** Solid
    *   **Meaning:** Actively attempting to connect to the configured Wi-Fi network.

*   **Wi-Fi Connected, Network Services Not Ready (e.g., MQTT disconnected):**
    *   **Color:** Cyan
    *   **Pattern:** Solid
    *   **Meaning:** Device is on the local network but cannot reach essential external services.

*   **Connecting to MQTT:**
    *   **Color:** Purple
    *   **Pattern:** Solid
    *   **Meaning:** Wi-Fi is connected, attempting to connect to the MQTT broker.

*   **Operational / Working Properly:**
    *   **Color:** Green
    *   **Pattern:** Solid
    *   **Meaning:** Wi-Fi and MQTT are connected, main loop is running, device is ready and functioning normally.

*   **General Error / Fault:**
    *   **Color:** Red
    *   **Pattern:** Solid
    *   **Meaning:** Indicates a critical error, such as reaching maximum Wi-Fi or MQTT reconnection retries, or an unexpected error in the main loop.

*   **Specific Error (Example: Sensor Read Error):**
    *   **Color:** Orange
    *   **Pattern:** Solid
    *   **Meaning:** Indicates a specific, potentially non-critical error.

*   **MQTT Communication (Sending/Receiving):**
    *   **Color:** Yellow
    *   **Pattern:** Single 1-second flash
    *   **Meaning:** Indicates successful transmission of data to or reception of data from the MQTT server. This occurs for sensor updates, relay state changes, and command receptions.

