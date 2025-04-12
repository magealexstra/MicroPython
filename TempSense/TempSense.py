###############################################################################
# TempSense - ESP32 MicroPython Temperature & Humidity Sensor Monitor
#
# Description: Reads temperature and humidity data from an I2C sensor (e.g., SHT31),
#              applies Exponential Moving Average (EMA) smoothing, and publishes
#              the data to an MQTT broker when changes exceed defined thresholds.
#              Includes Wi-Fi/MQTT connection management with LWT.
#
# Features:
# - Connects to Wi-Fi using credentials from secrets.py.
# - Connects to an MQTT broker with a unique client ID and LWT.
# - Initializes and reads data from a specified I2C temperature/humidity sensor.
#   (Requires sensor-specific code in init_sensor and read_sensor).
# - Applies EMA smoothing to temperature and humidity readings.
# - Publishes data in JSON format (e.g., {"temperature": 25.50, "humidity": 45.80}).
# - Only publishes data when smoothed values change by more than configured thresholds.
# - Publishes 'ONLINE'/'OFFLINE' status messages.
# - Includes optional debug printing.
# - Attempts automatic reconnection for Wi-Fi and MQTT.
###############################################################################

#------------------------------------------------------------------------------
# IMPORTS
#------------------------------------------------------------------------------
import network     # Handles WiFi connectivity
import time        # Time functions (delays, ticks)
import ubinascii   # Binary/ASCII conversions (for MAC address)
import sys         # System-specific parameters and functions (exception printing)
from machine import Pin, I2C # Hardware control for GPIO pins and I2C communication
from umqtt.simple import MQTTClient # MQTT protocol client

# Import local secrets
try:
    # Network & MQTT credentials
    from secrets import WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT
except ImportError:
    print("CRITICAL ERROR: Failed to import secrets.")
    print("Please ensure 'secrets.py' exists in the root directory with WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, and MQTT_PORT defined.")
    raise # Re-raise the exception

#------------------------------------------------------------------------------
# CONFIGURATION SETTINGS - ADJUST THESE FOR YOUR SETUP
#------------------------------------------------------------------------------
# MQTT Settings
MQTT_TOPIC = 'esp32/tempsense'  # [CONFIG] Base MQTT topic for publishing sensor data and status.

# Debugging
DEBUG = False                   # [CONFIG] Set to True for detailed log messages.

# Timing and Smoothing
LOOP_DELAY_S = 5                # [CONFIG] Delay in seconds between sensor readings and checks.
EMA_ALPHA = 0.1                 # [CONFIG] Smoothing factor (0.0 to 1.0). Lower = smoother, more delayed response. Higher = less smooth, faster response.

# Publishing Thresholds
TEMP_THRESHOLD = 0.3            # [CONFIG] Minimum change in smoothed temperature (degrees C) to trigger MQTT publish.
HUMIDITY_THRESHOLD = 1.0        # [CONFIG] Minimum change in smoothed humidity (percent RH) to trigger MQTT publish.

# I2C Sensor Configuration
I2C_SCL_PIN = 22                # [CONFIG] GPIO pin number for I2C Serial Clock (SCL).
I2C_SDA_PIN = 21                # [CONFIG] GPIO pin number for I2C Serial Data (SDA).
I2C_FREQ = 100000               # [CONFIG] I2C bus frequency in Hz (e.g., 100000 for 100kHz, 400000 for 400kHz).
SENSOR_I2C_ADDR = 0x44          # [CONFIG] I2C address of the temperature/humidity sensor (e.g., 0x44 for SHT31). Verify with datasheet or I2C scan.

#------------------------------------------------------------------------------
# GLOBAL VARIABLES - INTERNAL STATE TRACKING (DO NOT MODIFY)
#------------------------------------------------------------------------------
# State variables like EMA values and last sent values are managed within main() scope.

#------------------------------------------------------------------------------
# UTILITY FUNCTIONS
#------------------------------------------------------------------------------
def debug_print(*args, **kwargs):
    """Prints messages only if the global DEBUG flag is True.

    Args:
        *args: Positional arguments to pass to print().
        **kwargs: Keyword arguments to pass to print().

    Returns:
        None
    """
    if DEBUG:
        print("DEBUG:", *args, **kwargs)

def get_unique_client_id():
    """Generates a unique MQTT client ID based on the device's MAC address.

    Returns:
        bytes: A unique client ID string (e.g., b'tempsense_a1b2c3d4e5').
    """
    wlan = network.WLAN(network.STA_IF)
    mac = wlan.config('mac') # Get the MAC address
    # Use last 6 hex digits of MAC for uniqueness, prefixed with script name
    client_id = b'tempsense_' + ubinascii.hexlify(mac)[-6:]
    debug_print("Generated MQTT Client ID:", client_id)
    return client_id

#------------------------------------------------------------------------------
# CONNECTION FUNCTIONS
#------------------------------------------------------------------------------
def connect_wifi():
    """Connects the device to the Wi-Fi network specified in secrets.py.

    Handles activation and connection attempts. Blocks until connected or timeout.

    Returns:
        network.WLAN: The active WLAN interface object, or None if connection fails.
    """
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('Connecting to Wi-Fi network:', WIFI_SSID)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        # Wait for connection with a timeout
        max_wait = 15 # seconds
        start_time = time.time()
        while not wlan.isconnected() and time.time() - start_time < max_wait:
            print('.', end='')
            time.sleep(0.5)
        print() # Newline after dots

    if wlan.isconnected():
        print('Connected to Wi-Fi. IP Address:', wlan.ifconfig()[0])
        return wlan
    else:
        print('ERROR: Failed to connect to Wi-Fi after {} seconds.'.format(max_wait))
        wlan.active(False) # Deactivate if connection failed
        return None

def connect_mqtt():
    """Connects to the MQTT broker specified in secrets.py.

    Sets LWT and publishes 'ONLINE' status upon successful connection.

    Returns:
        umqtt.simple.MQTTClient: The connected MQTT client object, or None if connection fails.
    """
    try:
        client_id = get_unique_client_id()
        client = MQTTClient(client_id, MQTT_BROKER, port=MQTT_PORT)
        # Set Last Will and Testament (LWT)
        client.set_last_will(MQTT_TOPIC.encode(), b'OFFLINE', retain=True, qos=0)
        print('Connecting to MQTT broker at {}:{}...'.format(MQTT_BROKER, MQTT_PORT))
        client.connect()
        debug_print('Successfully connected to MQTT broker.')
        # Publish ONLINE status message (retained)
        client.publish(MQTT_TOPIC.encode(), b'ONLINE', retain=True, qos=0)
        debug_print("Published 'ONLINE' status to topic:", MQTT_TOPIC)
        return client
    except Exception as e:
        print('ERROR: Failed to connect to MQTT broker.')
        sys.print_exception(e)
        return None

#------------------------------------------------------------------------------
# SENSOR FUNCTIONS - *** MODIFY FOR YOUR SPECIFIC SENSOR ***
#------------------------------------------------------------------------------
def init_sensor():
    """Initializes the I2C bus and checks for the specified sensor.

    *** This function contains placeholder logic for an SHT31 sensor. ***
    *** You MUST adapt it for the actual sensor you are using. ***

    Returns:
        tuple: (I2C object, sensor_address) if successful, otherwise None.
    """
    try:
        print(f"Initializing I2C on SCL={I2C_SCL_PIN}, SDA={I2C_SDA_PIN}, Freq={I2C_FREQ}Hz")
        i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)

        debug_print("Scanning I2C bus...")
        devices = i2c.scan()
        debug_print('I2C devices found:', [hex(d) for d in devices])

        if SENSOR_I2C_ADDR not in devices:
            print(f"ERROR: Sensor not found at address {hex(SENSOR_I2C_ADDR)}.")
            print("Please check wiring and SENSOR_I2C_ADDR configuration.")
            return None

        print(f"Sensor found at address {hex(SENSOR_I2C_ADDR)}.")
        # Add any sensor-specific initialization commands here if needed
        # e.g., i2c.writeto(SENSOR_I2C_ADDR, b'\xsome_init_command')
        return i2c, SENSOR_I2C_ADDR

    except Exception as e:
        print("ERROR: Failed to initialize I2C sensor.")
        sys.print_exception(e)
        return None

def read_sensor(i2c, addr):
    """Reads temperature (Celsius) and humidity (%) from the I2C sensor.

    *** This function contains placeholder logic for an SHT31 sensor. ***
    *** You MUST adapt it for the actual sensor and communication protocol. ***

    Args:
        i2c (I2C): The initialized I2C object.
        addr (int): The I2C address of the sensor.

    Returns:
        tuple: (temperature_celsius, humidity_percent) if successful, otherwise (None, None).
    """
    try:
        # --- SHT31 Example ---
        # Send measurement command (High repeatability)
        i2c.writeto(addr, b'\x24\x00')
        # Wait for measurement to complete (refer to sensor datasheet)
        time.sleep(0.02) # SHT31 typical measurement time ~15ms
        # Read 6 bytes: Temp MSB, Temp LSB, Temp CRC, Hum MSB, Hum LSB, Hum CRC
        data = i2c.readfrom(addr, 6)
        # --- End SHT31 Example ---

        # --- Data Processing (SHT31 Example) ---
        # Combine bytes and perform calculations according to datasheet
        temp_raw = data[0] << 8 | data[1]
        humidity_raw = data[3] << 8 | data[4]
        # Formula from SHT31 datasheet
        temperature = -45 + (175 * (temp_raw / 65535.0))
        humidity = 100 * (humidity_raw / 65535.0)
        # --- End Data Processing ---

        debug_print(f"Raw Sensor: Temp={temperature:.2f}C, Humidity={humidity:.1f}%")
        return temperature, humidity

    except Exception as e:
        print("ERROR: Failed to read sensor data.")
        # Avoid printing stack trace every time for common read errors
        debug_print("Underlying exception:", e) # Show details only in debug mode
        return None, None

#------------------------------------------------------------------------------
# MAIN FUNCTION
#------------------------------------------------------------------------------
def main():
    """Main execution function.

    Connects to Wi-Fi/MQTT, initializes the sensor, and enters a loop to
    read data, apply smoothing, and publish changes via MQTT.
    """
    print("--- TempSense Starting ---")
    wlan = connect_wifi()
    if not wlan:
        print("CRITICAL: Wi-Fi connection failed on startup. Cannot proceed.")
        return # Exit if initial Wi-Fi fails

    client = connect_mqtt()
    # Allow proceeding without initial MQTT connection, will retry in loop

    sensor_init_result = init_sensor()
    if sensor_init_result is None:
        print("CRITICAL: Sensor initialization failed. Cannot proceed.")
        return # Exit if sensor fails
    i2c, sensor_addr = sensor_init_result

    # Initialize state variables
    ema_temp = None
    ema_humidity = None
    last_sent_temp = None
    last_sent_humidity = None
    last_successful_read_time = time.time()

    print("Starting main loop...")
    while True:
        try:
            # --- Connection Management ---
            if not wlan.isconnected():
                print('Wi-Fi disconnected. Attempting to reconnect...')
                wlan = connect_wifi()
                if not wlan or not wlan.isconnected():
                    print("Reconnect failed. Will retry later.")
                    time.sleep(5)
                    continue # Skip sensor reading if no WiFi
                else:
                    print("Wi-Fi reconnected.")
                    client = None # Force MQTT reconnect attempt

            if client is None and wlan and wlan.isconnected():
                print("MQTT client not connected. Attempting to reconnect...")
                client = connect_mqtt()
                if client:
                    print("MQTT reconnected.")
                    # Optionally re-publish last known state after reconnecting
                    if last_sent_temp is not None and last_sent_humidity is not None:
                         payload = '{{"temperature": {:.2f}, "humidity": {:.2f}}}'.format(last_sent_temp, last_sent_humidity)
                         try:
                             client.publish(MQTT_TOPIC.encode(), payload.encode(), retain=True) # Retain last good state
                             debug_print('Re-published last state:', payload)
                         except Exception as e:
                             print("ERROR: Failed to re-publish state after MQTT reconnect.")
                             sys.print_exception(e)
                             client = None # Mark for reconnect again
                else:
                    print("MQTT reconnect failed. Will retry later.")
                    time.sleep(5) # Wait before next cycle if MQTT fails


            # --- Sensor Reading ---
            temperature, humidity = read_sensor(i2c, sensor_addr)

            if temperature is not None and humidity is not None:
                last_successful_read_time = time.time() # Track successful reads

                # --- EMA Smoothing ---
                # Initialize EMA on the first valid reading
                if ema_temp is None:
                    ema_temp = temperature
                else:
                    ema_temp = EMA_ALPHA * temperature + (1 - EMA_ALPHA) * ema_temp

                if ema_humidity is None:
                    ema_humidity = humidity
                else:
                    ema_humidity = EMA_ALPHA * humidity + (1 - EMA_ALPHA) * ema_humidity

                debug_print(f"Smoothed: Temp={ema_temp:.2f}C, Humidity={ema_humidity:.1f}%")

                # --- Check Thresholds and Publish ---
                # Calculate change since last *sent* value, or first value if never sent
                temp_change = abs(ema_temp - (last_sent_temp if last_sent_temp is not None else ema_temp))
                humidity_change = abs(ema_humidity - (last_sent_humidity if last_sent_humidity is not None else ema_humidity))

                # Publish if either threshold is met OR if it's the very first valid reading
                should_publish = (
                    (last_sent_temp is None) or # Publish first reading
                    (temp_change >= TEMP_THRESHOLD) or
                    (humidity_change >= HUMIDITY_THRESHOLD)
                )

                if should_publish:
                    payload = '{{"temperature": {:.2f}, "humidity": {:.2f}}}'.format(ema_temp, ema_humidity)
                    debug_print(f"Change detected (T:{temp_change:.2f}, H:{humidity_change:.1f}). Publishing: {payload}")

                    if client:
                        try:
                            client.publish(MQTT_TOPIC.encode(), payload.encode(), retain=True) # Retain last published state
                            # Update last sent values ONLY on successful publish
                            last_sent_temp = ema_temp
                            last_sent_humidity = ema_humidity
                        except Exception as e:
                            print("ERROR: Failed to publish MQTT message.")
                            sys.print_exception(e)
                            client = None # Assume MQTT connection is broken
                    else:
                        debug_print('MQTT client not connected, skipping publish.')
                else:
                    debug_print(f"Change below threshold (T:{temp_change:.2f}, H:{humidity_change:.1f}). Skipping publish.")

            else:
                # Handle sensor read failure
                print("WARNING: Failed to read sensor data.")
                # Optional: Implement logic if sensor fails repeatedly (e.g., reset I2C, notify via MQTT)
                if time.time() - last_successful_read_time > 60: # Example: If no read for 60s
                    print("ERROR: Sensor has not responded for 60 seconds. Check hardware.")
                    # Could try re-initializing I2C or sensor here
                    # sensor_init_result = init_sensor() ... etc.


            # --- Loop Delay ---
            time.sleep(LOOP_DELAY_S)

        except KeyboardInterrupt:
            print("Interrupted by user.")
            break
        except Exception as e:
            print("An unexpected error occurred in the main loop:")
            sys.print_exception(e)
            print("Restarting loop after 5 seconds...")
            time.sleep(5)

    # --- Cleanup ---
    print("--- TempSense Stopping ---")
    if client:
        try:
            client.publish(MQTT_TOPIC.encode(), b'OFFLINE', retain=True, qos=0)
            client.disconnect()
            print("Disconnected from MQTT.")
        except Exception as e:
            print("Error during MQTT disconnect:", e)
    if wlan and wlan.isconnected():
        wlan.disconnect()
        wlan.active(False)
        print("Disconnected from Wi-Fi.")

#------------------------------------------------------------------------------
# MAIN EXECUTION
#------------------------------------------------------------------------------
if __name__ == '__main__':
    main()
