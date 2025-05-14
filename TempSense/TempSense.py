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
import json        # JSON for MQTT payloads
from machine import Pin, I2C # Hardware control for GPIO pins and I2C communication
from umqtt.simple import MQTTClient # MQTT protocol client
import aht         # Driver for AHT2x sensor

# Import local secrets
try:
    # Network & MQTT credentials
    from secrets import WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD, MQTT_TOPIC
except ImportError:
    print("CRITICAL ERROR: Failed to import secrets.")
    print("Please ensure 'secrets.py' exists in the root directory with WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD, and MQTT_TOPIC defined.")
    raise # Re-raise the exception

#------------------------------------------------------------------------------
# CONFIGURATION SETTINGS - ADJUST THESE FOR YOUR SETUP
#------------------------------------------------------------------------------

# Debugging
DEBUG = False                   # [CONFIG] Set to True for detailed log messages.

# Timing and Smoothing
LOOP_DELAY_S = 5                # [CONFIG] Delay in seconds between sensor readings and checks.
EMA_ALPHA = 0.1                 # [CONFIG] Smoothing factor (0.0 to 1.0). Lower = smoother, more delayed response. Higher = less smooth, faster response.
PERIODIC_PUBLISH_INTERVAL_S = 600 # [CONFIG] Interval in seconds for periodic publishing (e.g., 600 for 10 minutes).

# Publishing Thresholds
TEMP_THRESHOLD = 0.3            # [CONFIG] Minimum change in smoothed temperature (degrees C) to trigger MQTT publish.
HUMIDITY_THRESHOLD = 1.0        # [CONFIG] Minimum change in smoothed humidity (percent RH) to trigger MQTT publish.

# I2C Sensor Configuration (for AHT2x)
I2C_SCL_PIN = 22                # [CONFIG] GPIO pin number for I2C Serial Clock (SCL).
I2C_SDA_PIN = 21                # [CONFIG] GPIO pin number for I2C Serial Data (SDA).
I2C_FREQ = 100000               # [CONFIG] I2C bus frequency in Hz (e.g., 100000 for 100kHz, 400000 for 400kHz).
SENSOR_I2C_ADDR = 0x38          # [CONFIG] I2C address of the AHT2x temperature/humidity sensor (0x38).

# Break Sensor Configuration
BREAK_SENSOR_PINS = []          # [CONFIG] List of GPIO pins connected to break sensors (e.g., [19, 23]). Configure this list.
BREAK_SENSOR_DEBOUNCE_MS = 300  # [CONFIG] Debounce time in milliseconds for break sensors.

#------------------------------------------------------------------------------
# GLOBAL VARIABLES - INTERNAL STATE TRACKING (DO NOT MODIFY)
#------------------------------------------------------------------------------
# State variables like EMA values and last sent values are managed within main() scope.
break_sensor_states = {} # {pin_num: state} - 1=open, 0=closed (assuming pull-up)
last_break_sensor_times = {} # {pin_num: timestamp} - for debouncing in ticks_ms
aht_sensor = None # AHT2x sensor object when initialized

#------------------------------------------------------------------------------
# UTILITY FUNCTIONS
#------------------------------------------------------------------------------
def sync_time():
    """Synchronizes the device's internal clock using NTP."""
    print("Attempting to sync time with NTP...")
    try:
        ntptime.settime() # Blocks until time is set or error occurs
        print(f"Time synchronized successfully (UTC): {time.localtime()}")
        return True
    except Exception as e:
        debug_print(f"Failed to sync time with NTP: {type(e).__name__} - {str(e)}")
        return False

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

#------------------------------------------------------------------------------
# MQTT FUNCTIONS
#------------------------------------------------------------------------------
def publish_mqtt_message(topic, payload, retain=False):
    """Publishes a message to the MQTT broker."""
    global mqtt_client
    if mqtt_client:
        try:
            mqtt_client.publish(topic.encode(), str(payload).encode(), retain=retain, qos=0)
            debug_print(f"Published to topic '{topic}': {payload}")
        except Exception as e:
            debug_print(f"ERROR: Failed to publish MQTT message to topic '{topic}'.")
            sys.print_exception(e)
            mqtt_client = None # Assume MQTT connection is broken
    else:
        debug_print(f"MQTT client not connected, skipping publish to topic '{topic}'.")

def connect_mqtt():
    """Connects to the MQTT broker specified in secrets.py.

    Sets LWT and publishes 'ONLINE' status upon successful connection.

    Returns:
        umqtt.simple.MQTTClient: The connected MQTT client object, or None if connection fails.
    """
    global mqtt_client # Use global mqtt_client variable
    try:
        client_id = get_unique_client_id()
        # Include username and password for authentication
        client = MQTTClient(client_id, MQTT_BROKER, port=MQTT_PORT, user=MQTT_USERNAME, password=MQTT_PASSWORD)
        # Set Last Will and Testament (LWT)
        client.set_last_will(MQTT_TOPIC.encode(), b'OFFLINE', retain=True, qos=0)
        print('Connecting to MQTT broker at {}:{}...'.format(MQTT_BROKER, MQTT_PORT))
        client.connect()
        debug_print('Successfully connected to MQTT broker.')
        # Publish ONLINE status message (retained)
        publish_mqtt_message(MQTT_TOPIC, 'ONLINE', retain=True) # Use unified publish function
        debug_print("Published 'ONLINE' status to topic:", MQTT_TOPIC)
        mqtt_client = client # Store the connected client

        # Publish initial state of break sensors
        for pin_num in BREAK_SENSOR_PINS:
            try:
                # Get the current state from the initialized break_sensor_states dictionary
                current_state = break_sensor_states.get(pin_num)
                if current_state is not None: # Ensure the pin was successfully initialized
                    state_str = 'OPEN' if current_state else 'CLOSED'
                    topic = f'{MQTT_TOPIC}/break_sensors/{pin_num}'
                    publish_mqtt_message(topic, state_str, retain=True)
                else:
                    debug_print(f"Warning: Break sensor on pin {pin_num} not found in break_sensor_states. Skipping initial publish.")

            except Exception as e:
                debug_print(f"Error publishing initial state for break sensor on pin {pin_num}: {e}")

        return client
    except Exception as e:
        debug_print('ERROR: Failed to connect to MQTT broker.')
        sys.print_exception(e)
        mqtt_client = None # Ensure global client is None on failure
        return None

#------------------------------------------------------------------------------
# SENSOR FUNCTIONS - *** MODIFY FOR YOUR SPECIFIC SENSOR ***
#------------------------------------------------------------------------------
def init_sensor():
    """Initializes the I2C bus and the AHT2x sensor.

    Returns:
        aht.AHT2x: The initialized AHT2x sensor object if successful, otherwise None.
    """
    global aht_sensor
    try:
        print(f"Initializing I2C on SCL={I2C_SCL_PIN}, SDA={I2C_SDA_PIN}, Freq={I2C_FREQ}Hz")
        i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)

        debug_print("Scanning I2C bus...")
        devices = i2c.scan()
        debug_print('I2C devices found:', [hex(d) for d in devices])

        if SENSOR_I2C_ADDR not in devices:
            print(f"ERROR: AHT2x sensor not found at address {hex(SENSOR_I2C_ADDR)}.")
            print("Please check wiring and SENSOR_I2C_ADDR configuration.")
            return None

        print(f"AHT2x sensor found at address {hex(SENSOR_I2C_ADDR)}.")
        aht_sensor = aht.AHT2x(i2c, address=SENSOR_I2C_ADDR)
        print("AHT2x sensor initialized.")
        return aht_sensor

    except Exception as e:
        debug_print("ERROR: Failed to initialize AHT2x sensor.")
        sys.print_exception(e)
        return None

def read_sensor(sensor):
    """Reads temperature (Celsius) and humidity (%) from the AHT2x sensor.

    Args:
        sensor (aht.AHT2x): The initialized AHT2x sensor object.

    Returns:
        tuple: (temperature_celsius, humidity_percent) if successful, otherwise (None, None).
    """
    if sensor is None:
        debug_print("AHT2x sensor not initialized.")
        return None, None

    try:
        # The AHT2x driver handles the measurement trigger and reading
        temperature = sensor.temperature
        humidity = sensor.humidity

        if temperature is not None and humidity is not None:
             debug_print(f"Raw Sensor: Temp={temperature:.2f}C, Humidity={humidity:.1f}%")
             return temperature, humidity
        else:
             debug_print("AHT2x sensor returned None for temperature or humidity.")
             return None, None

    except Exception as e:
        print("ERROR: Failed to read AHT2x sensor data.")
        debug_print("Underlying exception:", e)
        return None, None

#------------------------------------------------------------------------------
# BREAK SENSOR FUNCTIONS
#------------------------------------------------------------------------------
def init_break_sensors():
    """Initializes the configured break sensor pins."""
    global break_sensor_states, last_break_sensor_times
    print("Initializing break sensors...")
    for pin_num in BREAK_SENSOR_PINS:
        try:
            sensor_pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)
            break_sensor_states[pin_num] = sensor_pin.value() # Initial state
            last_break_sensor_times[pin_num] = time.ticks_ms()
            debug_print(f"Initialized break sensor on pin {pin_num}. Initial state: {'OPEN' if break_sensor_states[pin_num] else 'CLOSED'}")
        except Exception as e:
            print(f"ERROR: Failed to initialize break sensor on pin {pin_num}.")
            sys.print_exception(e)

def check_break_sensors():
    """Checks the state of break sensors, debounces, and publishes changes."""
    global break_sensor_states, last_break_sensor_times, mqtt_client
    current_time = time.ticks_ms()

    for pin_num in BREAK_SENSOR_PINS:
        try:
            sensor_pin = Pin(pin_num) # Get the initialized pin object
            current_state = sensor_pin.value()

            # Debounce check
            if current_state != break_sensor_states.get(pin_num) and time.ticks_diff(current_time, last_break_sensor_times.get(pin_num, 0)) > BREAK_SENSOR_DEBOUNCE_MS:
                break_sensor_states[pin_num] = current_state
                last_break_sensor_times[pin_num] = current_time
                state_str = 'OPEN' if current_state else 'CLOSED'
                debug_print(f"Break sensor on pin {pin_num} changed state to {state_str}")

                # Publish state change via MQTT
                if mqtt_client:
                    topic = f'{MQTT_TOPIC}/break_sensors/{pin_num}'
                    publish_mqtt_message(topic, state_str, retain=True)
                else:
                    debug_print(f"MQTT client not connected, skipping publish for break sensor {pin_num}.")

        except Exception as e:
            debug_print(f"Error checking break sensor on pin {pin_num}: {e}")


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

    sync_time() # Synchronize time after initial Wi-Fi connection

    client = connect_mqtt()
    # Allow proceeding without initial MQTT connection, will retry in loop

    sensor_init_result = init_sensor()
    if sensor_init_result is None:
        print("CRITICAL: Sensor initialization failed. Cannot proceed.")
        return # Exit if sensor fails
    # The init_sensor function now returns the aht_sensor object directly

    # Initialize break sensors
    init_break_sensors()

    # Initialize state variables
    ema_temp = None
    ema_humidity = None
    last_sent_temp = None
    last_sent_humidity = None
    last_successful_read_time = time.time()
    last_periodic_publish_time = 0 # Track time of last periodic publish

    print("Starting main loop...")
    while True:
        try:
            current_time = time.time() # Get current time at the start of the loop

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
                    sync_time() # Synchronize time after Wi-Fi reconnect
                    client = None # Force MQTT reconnect attempt

            if client is None and wlan and wlan.isconnected():
                print("MQTT client not connected. Attempting to reconnect...")
                client = connect_mqtt()
                if client:
                    print("MQTT reconnected.")
                    # Optionally re-publish last known state after reconnecting
                    if last_sent_temp is not None and last_sent_humidity is not None:
                         payload = '{{"temperature": {:.2f}, "humidity": {:.2f}}}'.format(last_sent_temp, last_sent_humidity)
                         publish_mqtt_message(MQTT_TOPIC, payload, retain=True) # Use unified publish function
                else:
                    print("MQTT reconnect failed. Will retry later.")
                    time.sleep(5) # Wait before next cycle if MQTT fails


            # --- Sensor Reading ---
            # Pass the aht_sensor object instead of i2c and addr
            temperature, humidity = read_sensor(aht_sensor)

            if temperature is not None and humidity is not None:
                last_successful_read_time = current_time # Track successful reads
                failed_read_count = 0 # Reset failure count on successful read
                failure_start_time = 0 # Reset failure timer

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

                # Check if periodic publish interval has passed
                should_periodic_publish = (current_time - last_periodic_publish_time) >= PERIODIC_PUBLISH_INTERVAL_S

                # Publish if either threshold is met OR if it's the very first valid reading OR if periodic interval passed
                should_publish = (
                    (last_sent_temp is None) or # Publish first reading
                    (temp_change >= TEMP_THRESHOLD) or
                    (humidity_change >= HUMIDITY_THRESHOLD) or
                    should_periodic_publish
                )

                if should_publish:
                    payload = '{{"temperature": {:.2f}, "humidity": {:.2f}}}'.format(ema_temp, ema_humidity)
                    debug_print(f"Change detected (T:{temp_change:.2f}, H:{humidity_change:.1f}) or periodic publish. Publishing: {payload}")

                    # Use unified publish function
                    publish_mqtt_message(MQTT_TOPIC, payload, retain=True)

                    # Update last sent values and periodic publish time ONLY on successful publish
                    last_sent_temp = ema_temp
                    last_sent_humidity = ema_humidity
                    last_periodic_publish_time = current_time # Update periodic publish time

                else:
                    debug_print(f"Change below threshold (T:{temp_change:.2f}, H:{humidity_change:.1f}). Skipping publish.")

            else:
                # Handle sensor read failure
                print("WARNING: Failed to read sensor data.")
                failed_read_count += 1
                if failure_start_time == 0: # Start the failure timer on the first failure
                    failure_start_time = current_time

                # Attempt sensor re-initialization after a certain number of failures or a timeout
                # Configure thresholds as needed (e.g., 10 consecutive failures or 300 seconds of failure)
                if failed_read_count >= 10 or (current_time - failure_start_time) >= 300:
                    print("Attempting to re-initialize sensor...")
                    sensor_init_result = init_sensor()
                    if sensor_init_result is not None:
                        print("Sensor re-initialization successful.")
                        aht_sensor = sensor_init_result # Update the global sensor object
                        # Reset failure tracking on successful re-initialization
                        failed_read_count = 0
                        failure_start_time = 0
                    else:
                        print("Sensor re-initialization failed.")


            # --- Check Break Sensors ---
            check_break_sensors()

            # --- Loop Delay ---
            time.sleep(LOOP_DELAY_S)

        except KeyboardInterrupt:
            print("Interrupted by user.")
            break
        except Exception as e:
            debug_print("An unexpected error occurred in the main loop:")
            sys.print_exception(e)
            debug_print("Restarting loop after 5 seconds...")
            time.sleep(5)

    # --- Cleanup ---
    print("--- TempSense Stopping ---")
    if client:
        try:
            # Use unified publish function for OFFLINE status
            publish_mqtt_message(MQTT_TOPIC, 'OFFLINE', retain=True)
            client.disconnect()
            print("Disconnected from MQTT.")
        except Exception as e:
            debug_print("Error during MQTT disconnect:", e)
    if wlan and wlan.isconnected():
        wlan.disconnect()
        wlan.active(False)
        print("Disconnected from Wi-Fi.")

#------------------------------------------------------------------------------
# MAIN EXECUTION
#------------------------------------------------------------------------------
if __name__ == '__main__':
    main()
