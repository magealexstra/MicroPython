###############################################################################
# TempSense - ESP32 MicroPython Temperature & Humidity Sensor Monitor
#
# Description: Reads temperature and humidity data from an I2C sensor (AHT2x),
#              applies Exponential Moving Average (EMA) smoothing, and publishes
#              the data to an MQTT broker when changes exceed defined thresholds.
#              Includes Wi-Fi/MQTT connection management with LWT.
###############################################################################

#------------------------------------------------------------------------------
# IMPORTS
#------------------------------------------------------------------------------
import ntptime      # Network Time Protocol client
import network      # Handles WiFi connectivity
import time
import ubinascii    # Binary/ASCII conversions (for MAC address)
import sys          # System-specific parameters and functions (exception printing)
import json         # JSON for MQTT payloads
from machine import Pin, I2C, reset # Hardware control for GPIO pins, I2C, and reset
from umqtt.simple import MQTTClient # MQTT protocol client
import gc           # Garbage collector

import aht          # Alternative Driver for AHT10/AHT20 sensors (ahtx0.py renamed to aht.py)

# Import local secrets
try:
    # Network and MQTT credentials, plus optional static IP config
    from secrets import WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD, STATIC_IP, SUBNET_MASK, GATEWAY, DNS_SERVER
except ImportError:
    print("CRITICAL ERROR: Failed to import secrets.")
    print("Please ensure 'secrets.py' exists in the root directory with WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD, and optionally STATIC_IP, SUBNET_MASK, GATEWAY, DNS_SERVER defined.")
    raise

#------------------------------------------------------------------------------
# CONFIGURATION SETTINGS
#------------------------------------------------------------------------------

DEBUG = False # Set to True for debug prints
I2C_SCL_PIN = 22
I2C_SDA_PIN = 21
I2C_FREQ = 10000
SENSOR_I2C_ADDR = 0x38

EMA_ALPHA = 0.1
PERIODIC_PUBLISH_INTERVAL_S = 600

TEMP_THRESHOLD = 0.3
HUMIDITY_THRESHOLD = 1.0

BREAK_SENSOR_PINS = [32]
BREAK_SENSOR_DEBOUNCE_MS = 300

MAX_CONNECTION_RETRIES = 10
CONNECTION_RETRY_DELAY_S = 5
CONNECTION_BACKOFF_FACTOR = 2
MAX_RETRY_DELAY_S = 25

LOOP_DELAY_S = 5
GC_COLLECT_LOOP_INTERVAL = 1000 # Perform gc.collect() every this many main loop iterations

#------------------------------------------------------------------------------
# GLOBAL VARIABLES
#------------------------------------------------------------------------------
wlan = None
mqtt_client = None
base_topic_prefix = None
sensor = None
DEVICE_MAC_SUFFIX = None # Initialized once at startup

ema_temp = None
ema_humidity = None
last_sent_temp = None
last_sent_humidity = None
last_periodic_publish_time = 0

failed_read_count = 0
failure_start_time = None

break_sensor_states = {}
last_break_sensor_times = {}
break_sensor_pin_objs = {} # Stores initialized Pin objects for break sensors

wifi_retry_count = 0
mqtt_retry_count = 0
wifi_backoff_time = CONNECTION_RETRY_DELAY_S
mqtt_backoff_time = CONNECTION_RETRY_DELAY_S

#------------------------------------------------------------------------------
# UTILITY FUNCTIONS
#------------------------------------------------------------------------------
def debug_print(*args, **kwargs):
    if DEBUG:
        sys.stdout.write("DEBUG: ")
        for arg in args:
            sys.stdout.write(str(arg) + " ")
        sys.stdout.write("\n")

def initialize_device_identifiers():
    global DEVICE_MAC_SUFFIX
    wlan_temp_if = None
    try:
        wlan_temp_if = network.WLAN(network.STA_IF)
        wlan_temp_if.active(True)
        # A short delay might be needed if activating interface takes time before MAC is readable.
        # Test without it first, add time.sleep(0.1) or time.sleep(0.5) if MAC read is unstable.
        mac = wlan_temp_if.config('mac')
        DEVICE_MAC_SUFFIX = ubinascii.hexlify(mac)[-6:].decode('utf-8')
        debug_print(f"Device MAC Suffix initialized: {DEVICE_MAC_SUFFIX}")
        # If WLAN was activated *only* for MAC and isn't connected, can deactivate.
        # However, connect_wifi will manage active state later anyway.
        # if not wlan_temp_if.isconnected():
        #     wlan_temp_if.active(False)
        #     debug_print("Temporarily activated WLAN for MAC, then deactivated.")
    except Exception as e:
        print(f"CRITICAL ERROR: Failed to initialize device MAC suffix: {e}")
        # DEVICE_MAC_SUFFIX will remain None.
        # This is a critical failure for unique ID generation. Consider device reset or error state.

def sync_time():
    debug_print("Attempting to sync time with NTP...")
    try:
        ntptime.settime()
        debug_print(f"Time synchronized successfully (UTC): {time.localtime()}")
        return True
    except Exception as e:
        debug_print(f"Failed to sync time with NTP: {type(e).__name__} - {str(e)}")
        return False

def get_unique_client_id():
    if DEVICE_MAC_SUFFIX:
        client_id = b'tempsense_' + DEVICE_MAC_SUFFIX.encode('utf-8')
        debug_print("Generated MQTT Client ID using stored MAC suffix:", client_id)
        return client_id
    else:
        # Fallback: This should ideally not happen if initialize_device_identifiers succeeds.
        debug_print("ERROR: DEVICE_MAC_SUFFIX not available. Attempting dynamic MAC fetch for client ID (fallback).")
        try:
            wlan_if_fallback = network.WLAN(network.STA_IF)
            wlan_if_fallback.active(True) # Ensure active for MAC reading
            mac_fallback = wlan_if_fallback.config('mac')
            client_id_fallback = b'tempsense_' + ubinascii.hexlify(mac_fallback)[-6:]
            debug_print("Generated MQTT Client ID using fallback MAC fetch:", client_id_fallback)
            return client_id_fallback
        except Exception as e:
            debug_print(f"CRITICAL ERROR: Fallback MAC fetch failed for client ID: {e}")
            # Generate a pseudo-random ID if MAC fetch completely fails, not ideal
            random_suffix = ubinascii.hexlify(ubinascii. পরবর্তী(6)) # Requires import os for os.urandom
            # For simplicity, if truly stuck, could use a fixed default or raise error.
            # This part depends on how critical a unique ID is vs. partial operation.
            # Let's assume for now it might raise an error or return a non-unique ID.
            # A truly robust solution might involve storing a generated UUID in flash.
            return b'tempsense_fallback' + ubinascii.hexlify(time.ticks_us().to_bytes(4, 'little'))[-4:]


#------------------------------------------------------------------------------
# CONNECTION FUNCTIONS
#------------------------------------------------------------------------------
def connect_wifi():
    global wlan
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if 'STATIC_IP' in globals() and STATIC_IP and \
       'SUBNET_MASK' in globals() and SUBNET_MASK and \
       'GATEWAY' in globals() and GATEWAY and \
       'DNS_SERVER' in globals() and DNS_SERVER:
        try:
            wlan.ifconfig((STATIC_IP, SUBNET_MASK, GATEWAY, DNS_SERVER))
            debug_print(f'Configured static IP: {STATIC_IP}')
        except Exception as e:
            debug_print(f"Error configuring static IP: {e}")
            pass

    if not wlan.isconnected():
        debug_print(f"Attempting to connect to Wi-Fi network: {WIFI_SSID}")
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        max_wait = 15
        start_time = time.time()
        while not wlan.isconnected() and time.time() - start_time < max_wait:
            if DEBUG:
                sys.stdout.write('.')
            time.sleep(0.5)
        if DEBUG and not wlan.isconnected(): # Add a newline if dots were printed and failed
             sys.stdout.write("\n")


    if wlan.isconnected():
        debug_print('Connected to Wi-Fi. IP Address:', wlan.ifconfig()[0])
        return wlan
    else:
        debug_print('ERROR: Failed to connect to Wi-Fi after {} seconds.'.format(max_wait))
        wlan.active(False)
        wlan = None
        return None

def publish_mqtt_message(topic, payload, retain=False):
    global mqtt_client
    if mqtt_client:
        try:
            topic_bytes = topic.encode() if isinstance(topic, str) else topic
            payload_bytes = str(payload).encode() if not isinstance(payload, bytes) else payload
            mqtt_client.publish(topic_bytes, payload_bytes, retain=retain, qos=0)
            debug_print(f"Published to topic '{topic}': {payload}")
        except Exception as e:
            debug_print(f"ERROR: Failed to publish MQTT message to topic '{topic}'.")
            sys.print_exception(e)
            mqtt_client = None
    else:
        debug_print(f"MQTT client not connected, skipping publish to topic '{topic}'.")

def connect_mqtt():
    global mqtt_client
    global base_topic_prefix

    if base_topic_prefix is None:
        debug_print("ERROR: base_topic_prefix is None when attempting MQTT connect. Cannot proceed.")
        return None

    debug_print(f"connect_mqtt: base_topic_prefix = {base_topic_prefix}")
    try:
        client_id = get_unique_client_id()
        if client_id is None : # get_unique_client_id could fail in extreme fallback
            debug_print("CRITICAL: Could not generate MQTT client_id. MQTT connect aborted.")
            return None

        client = MQTTClient(client_id, MQTT_BROKER, port=MQTT_PORT, user=MQTT_USERNAME, password=MQTT_PASSWORD)
        client.set_last_will(f"{base_topic_prefix}/status".encode(), b'OFFLINE', retain=True, qos=0)
        debug_print(f"Attempting to connect to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
        client.connect()
        debug_print('Successfully connected to MQTT broker.')
        mqtt_client = client
        publish_mqtt_message(f"{base_topic_prefix}/status", 'ONLINE', retain=True)
        debug_print("Published 'ONLINE' status to topic:", f"{base_topic_prefix}/status")

        for pin_num in BREAK_SENSOR_PINS:
            try:
                current_state = break_sensor_states.get(pin_num)
                if current_state is not None:
                    state_str = 'OPEN' if current_state else 'CLOSED'
                    topic = f'{base_topic_prefix}/Break/GPIO{pin_num}'
                    publish_mqtt_message(topic, state_str, retain=True)
            except Exception as e:
                debug_print(f"Error publishing initial state for break sensor on pin {pin_num}: {e}")
        return client
    except Exception as e:
        debug_print('ERROR: Failed to connect to MQTT broker.')
        sys.print_exception(e)
        mqtt_client = None
        return None

#------------------------------------------------------------------------------
# SENSOR FUNCTIONS
#------------------------------------------------------------------------------
def init_sensor(scl_pin, sda_pin, freq, sensor_addr):
    # No 'global i2c_bus' needed here anymore
    debug_print("DEBUG: Entered init_sensor function with arguments.")
    debug_print(f"DEBUG: init_sensor args - scl_pin:{scl_pin}, sda_pin:{sda_pin}, freq:{freq}, sensor_addr:{hex(sensor_addr)}")

    try:
        debug_print(f"Initializing I2C on SCL={scl_pin}, SDA={sda_pin}, Freq={freq}Hz")
        # Create I2C object locally; it will be held by the sensor driver instance
        temp_i2c = I2C(0, scl=Pin(scl_pin), sda=Pin(sda_pin), freq=freq)
        
        debug_print("Scanning I2C bus...")
        devices = temp_i2c.scan()
        debug_print('I2C devices found:', [hex(d) for d in devices])

        if sensor_addr not in devices:
            debug_print(f"ERROR: AHT2x sensor not found at address {hex(sensor_addr)}.")
            debug_print("Please check wiring and SENSOR_I2C_ADDR configuration.")
            return None

        debug_print(f"AHT2x sensor found at address {hex(sensor_addr)}.")
        aht_sensor_obj = None
        try:
            aht_sensor_obj = aht.AHT20(temp_i2c, address=sensor_addr)
            debug_print("Using AHT20 class from new driver.")
        except AttributeError:
            try:
                aht_sensor_obj = aht.AHT10(temp_i2c, address=sensor_addr)
                debug_print("Using AHT10 class from new driver.")
            except AttributeError:
                debug_print("ERROR: Could not find AHT20 or AHT10 class in the new aht.py driver.")
                return None
        
        if aht_sensor_obj:
            debug_print("AHT2x sensor object created with new driver.")
            return aht_sensor_obj
        else:
            debug_print("ERROR: AHT sensor object was not created by the driver.")
            return None
    except Exception as e:
        debug_print("ERROR: Failed to initialize I2C bus or AHT2x sensor with new driver.")
        sys.print_exception(e)
        return None

def check_aht_read_and_publish():
    global sensor, mqtt_client # Removed i2c_bus from here
    global ema_temp, ema_humidity, last_sent_temp, last_sent_humidity, last_periodic_publish_time
    global failed_read_count, failure_start_time
    global base_topic_prefix

    if sensor is None:
        debug_print("Skipping AHT2x read/publish: Sensor not initialized.")
        return
    if mqtt_client is None or base_topic_prefix is None:
        debug_print("Skipping AHT2x read/publish: MQTT or base topic not ready.")
        return

    current_time = time.time()
    try:
        debug_print("DEBUG: Attempting sensor read with new driver...")
        temperature = sensor.temperature
        humidity = sensor.relative_humidity
        debug_print(f"DEBUG: Properties read - Temp: {temperature}, Hum: {humidity}")

        if temperature is not None and humidity is not None:
            failed_read_count = 0
            failure_start_time = None
            # EMA and publishing logic... ( 그대로 유지 )
            if ema_temp is None: ema_temp = temperature
            else: ema_temp = EMA_ALPHA * temperature + (1 - EMA_ALPHA) * ema_temp
            if ema_humidity is None: ema_humidity = humidity
            else: ema_humidity = EMA_ALPHA * humidity + (1 - EMA_ALPHA) * ema_humidity
            debug_print(f"Smoothed (AHT2x): Temp={ema_temp:.2f}C, Humidity={ema_humidity:.1f}% (EMA_Temp: {ema_temp}, EMA_Hum: {ema_humidity})")

            temp_change = abs(ema_temp - (last_sent_temp if last_sent_temp is not None else ema_temp))
            humidity_change = abs(ema_humidity - (last_sent_humidity if last_sent_humidity is not None else ema_humidity))
            should_periodic_publish = (current_time - last_periodic_publish_time) >= PERIODIC_PUBLISH_INTERVAL_S
            should_publish = ((last_sent_temp is None) or (temp_change >= TEMP_THRESHOLD) or \
                              (humidity_change >= HUMIDITY_THRESHOLD) or should_periodic_publish)

            if should_publish:
                debug_print(f"Change detected (AHT2x T:{temp_change:.2f}, H:{humidity_change:.1f}) or periodic publish. Publishing...")
                temp_payload = '{{"temperature": {:.2f}}}'.format(ema_temp)
                publish_mqtt_message(f"{base_topic_prefix}/Temp", temp_payload, retain=True)
                humidity_payload = '{{"humidity": {:.2f}}}'.format(ema_humidity)
                publish_mqtt_message(f"{base_topic_prefix}/Humidity", humidity_payload, retain=True)
                last_sent_temp = ema_temp
                last_sent_humidity = ema_humidity
                last_periodic_publish_time = current_time
            else:
                debug_print(f"Change below threshold (AHT2x T:{temp_change:.2f}, H:{humidity_change:.1f}). Skipping publish.")

        else:
            debug_print("WARNING: AHT2x sensor properties returned None (New Driver).")
            failed_read_count += 1
            if failure_start_time is None: failure_start_time = current_time
    except Exception as e:
        debug_print("ERROR: Unhandled exception during AHT2x sensor read, processing, or publishing.")
        sys.print_exception(e)
        failed_read_count += 1
        if failure_start_time is None: failure_start_time = current_time
        # Removed experimental I2C bus recovery scan here

#------------------------------------------------------------------------------
# BREAK SENSOR FUNCTIONS
#------------------------------------------------------------------------------
def init_break_sensors():
    global break_sensor_states, last_break_sensor_times, break_sensor_pin_objs
    debug_print("Initializing break sensors...")
    for pin_num in BREAK_SENSOR_PINS:
        try:
            pin_obj = Pin(pin_num, Pin.IN, Pin.PULL_UP)
            break_sensor_pin_objs[pin_num] = pin_obj # Store the Pin object
            break_sensor_states[pin_num] = pin_obj.value() # Read initial state from stored object
            last_break_sensor_times[pin_num] = time.ticks_ms()
            debug_print(f"Initialized break sensor on pin {pin_num}. Initial state: {'OPEN' if break_sensor_states[pin_num] else 'CLOSED'}")
        except Exception as e:
            debug_print(f"ERROR: Failed to initialize break sensor on pin {pin_num}.")
            sys.print_exception(e)

def check_break_sensors():
    global break_sensor_states, last_break_sensor_times, mqtt_client, base_topic_prefix, break_sensor_pin_objs

    if mqtt_client is None or base_topic_prefix is None:
        debug_print("Skipping break sensor check: MQTT or base topic not ready.")
        return

    current_ticks = time.ticks_ms()
    for pin_num in BREAK_SENSOR_PINS:
        sensor_pin_obj = break_sensor_pin_objs.get(pin_num)
        if sensor_pin_obj is None: # Check if Pin object was stored
            debug_print(f"Break sensor on pin {pin_num} was not properly initialized (no Pin object). Skipping check.")
            continue
        # Ensure state tracking dictionaries are also initialized for this pin
        if pin_num not in break_sensor_states or pin_num not in last_break_sensor_times:
            debug_print(f"Break sensor state for pin {pin_num} not fully initialized. Skipping check.")
            continue
            
        try:
            current_state = sensor_pin_obj.value() # Use stored Pin object
            last_known_state = break_sensor_states.get(pin_num)
            last_event_time = last_break_sensor_times.get(pin_num, 0)

            if current_state != last_known_state:
                if time.ticks_diff(current_ticks, last_event_time) > BREAK_SENSOR_DEBOUNCE_MS:
                    break_sensor_states[pin_num] = current_state
                    last_break_sensor_times[pin_num] = current_ticks
                    state_str = 'OPEN' if current_state else 'CLOSED'
                    debug_print(f"Break sensor on pin {pin_num} changed state to {state_str}")
                    topic = f'{base_topic_prefix}/Break/GPIO{pin_num}'
                    publish_mqtt_message(topic, state_str, retain=True)
        except Exception as e:
            debug_print(f"Error checking break sensor on pin {pin_num}:")
            sys.print_exception(e)

#------------------------------------------------------------------------------
# MAIN FUNCTION
#------------------------------------------------------------------------------
def main():
    """Main execution function."""
    global base_topic_prefix, failed_read_count, failure_start_time
    global wifi_retry_count, mqtt_retry_count, wifi_backoff_time, mqtt_backoff_time
    global wlan, mqtt_client, sensor
    # Module-level constants like LOOP_DELAY_S are accessed directly

    # Use standard print for major lifecycle events that should always be visible
    print("--- TempSense Starting ---")

    initialize_device_identifiers() # Fetch MAC suffix early; critical errors printed by function

    init_break_sensors()

    debug_print("Attempting initial sensor initialization...")
    sensor = init_sensor(I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ, SENSOR_I2C_ADDR)
    if sensor is None:
        debug_print("WARNING: AHT2x sensor initialization failed at startup. Will retry in loop.")
    else:
        debug_print("AHT2x sensor initialized successfully at startup.")

    # Reset failure tracking and retry counters before starting the main loop
    failed_read_count = 0
    failure_start_time = None
    wifi_retry_count = 0
    mqtt_retry_count = 0
    wifi_backoff_time = CONNECTION_RETRY_DELAY_S
    mqtt_backoff_time = CONNECTION_RETRY_DELAY_S

    main_loop_counter = 0 # Counter for periodic gc.collect()

    debug_print("Initialization complete. Starting main loop...")

    while True:
        main_loop_counter += 1

        # --- Connection Management ---
        if wlan is None or not wlan.isconnected():
            wifi_retry_count += 1
            debug_print(f"Wi-Fi disconnected. Attempting reconnect {wifi_retry_count}/{MAX_CONNECTION_RETRIES}...")
            wlan = connect_wifi() # connect_wifi updates global wlan

            if wlan is None or not wlan.isconnected(): # Check after attempt
                wifi_backoff_time = min(MAX_RETRY_DELAY_S, CONNECTION_RETRY_DELAY_S * (CONNECTION_BACKOFF_FACTOR ** wifi_retry_count))
                debug_print(f"Wi-Fi reconnection failed. Waiting {wifi_backoff_time:.0f}s...")
                time.sleep(wifi_backoff_time)
                if wifi_retry_count >= MAX_CONNECTION_RETRIES:
                    print("Max Wi-Fi retries reached. Restarting device.") # Critical print
                    time.sleep(5)
                    reset()
                continue # Skip rest of loop if no WiFi
            else: # Wi-Fi reconnected successfully
                debug_print("Wi-Fi reconnected.")
                wifi_retry_count = 0
                wifi_backoff_time = CONNECTION_RETRY_DELAY_S
                mqtt_client = None # Force MQTT reconnect
                base_topic_prefix = None # Force base topic recalculation
                debug_print("Wi-Fi restored, forcing MQTT reconnect attempt and time sync.")
                sync_time()

        if mqtt_client is None and wlan is not None and wlan.isconnected():
            mqtt_retry_count += 1
            debug_print(f"MQTT client not connected. Attempting reconnect {mqtt_retry_count}/{MAX_CONNECTION_RETRIES}...")

            if base_topic_prefix is None:
                if DEVICE_MAC_SUFFIX:
                    base_topic_prefix = f"/esp32/TempHum/{DEVICE_MAC_SUFFIX}"
                    debug_print(f"Generated base MQTT topic prefix: {base_topic_prefix}")
                else:
                    # This is a critical issue if MAC suffix wasn't obtained.
                    debug_print("ERROR: Cannot generate base MQTT topic prefix, MAC suffix unknown. MQTT connection will be retried.")
                    # Wait before retrying the loop which will re-attempt MAC suffix dependent logic
                    time.sleep(CONNECTION_RETRY_DELAY_S)
                    continue

            if base_topic_prefix is not None: # Ensure base_topic_prefix was successfully set
                mqtt_client = connect_mqtt() # connect_mqtt updates global mqtt_client

            if mqtt_client is None: # Check after attempt
                mqtt_backoff_time = min(MAX_RETRY_DELAY_S, CONNECTION_RETRY_DELAY_S * (CONNECTION_BACKOFF_FACTOR ** mqtt_retry_count))
                debug_print(f"MQTT reconnection failed. Waiting {mqtt_backoff_time:.0f}s...")
                time.sleep(mqtt_backoff_time)
                if mqtt_retry_count >= MAX_CONNECTION_RETRIES:
                    print("Max MQTT retries reached. Restarting device.") # Critical print
                    time.sleep(5)
                    reset()
                continue # Skip sensor checks if MQTT failed
            else: # MQTT reconnected successfully
                debug_print("MQTT reconnected.")
                mqtt_retry_count = 0
                mqtt_backoff_time = CONNECTION_RETRY_DELAY_S
                # Initial break sensor states are published within connect_mqtt now

        # --- Main Operational Logic (only if both Wi-Fi and MQTT are connected) ---
        if wlan is not None and wlan.isconnected() and mqtt_client is not None:
            try:
                mqtt_client.check_msg() # Check for incoming MQTT messages
            except Exception as e:
                debug_print("ERROR checking MQTT messages:")
                sys.print_exception(e)
                mqtt_client = None # Assume connection is broken
                continue # Skip rest of this iteration, connection logic will handle next

            # Attempt to initialize sensor if it's None (e.g., initial failure or after persistent read errors)
            if sensor is None:
                debug_print("AHT2x sensor object is None in main loop. Attempting initialization...")
                sensor = init_sensor(I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ, SENSOR_I2C_ADDR)
                if sensor is not None:
                    debug_print("AHT2x sensor re-initialization successful in loop.")
                    failed_read_count = 0 # Reset failure tracking
                    failure_start_time = None
                    # Reset EMA and last_sent states for the re-initialized sensor
                    global ema_temp, ema_humidity, last_sent_temp, last_sent_humidity, last_periodic_publish_time
                    ema_temp, ema_humidity, last_sent_temp, last_sent_humidity = None, None, None, None
                    last_periodic_publish_time = 0 # Reset to force publish on next valid read
                else:
                    debug_print("AHT2x sensor re-initialization failed in loop.")
                    # Optional: add a short delay here if re-init fails often, to prevent rapid retries
                    # time.sleep(CONNECTION_RETRY_DELAY_S)

            # Process AHT2x sensor data if sensor is initialized
            if sensor is not None:
                check_aht_read_and_publish()

                # Handle sensor re-initialization if reads consistently fail
                # This logic is for when sensor object exists but reads are failing.
                if failure_start_time is not None and failed_read_count >= 10: # Threshold for re-init
                    current_time_for_failure_check = time.time()
                    # Example: 10 failures over at least 5 minutes (300s)
                    if (current_time_for_failure_check - failure_start_time) >= 300:
                        debug_print(f"AHT2x sensor read failed {failed_read_count} times over ~{current_time_for_failure_check - failure_start_time:.0f}s. Attempting to re-initialize...")
                        sensor = init_sensor(I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ, SENSOR_I2C_ADDR)
                        if sensor is not None:
                            debug_print("AHT2x sensor re-initialization successful after persistent failures.")
                            failed_read_count = 0
                            failure_start_time = None
                            # Re-declare global for assignment if needed by Python scoping rules in this block
                            global ema_temp, ema_humidity, last_sent_temp, last_sent_humidity, last_periodic_publish_time
                            ema_temp, ema_humidity, last_sent_temp, last_sent_humidity = None, None, None, None
                            last_periodic_publish_time = 0
                        else:
                            debug_print("AHT2x sensor re-initialization FAILED after persistent failures.")
                            # To prevent rapid re-init cycles if init_sensor itself fails quickly:
                            # Reset failure_start_time to now, so it waits another period before trying this block again.
                            failure_start_time = time.time()
                            # Alternatively, could set sensor to None here to trigger the other re-init block,
                            # or increase failed_read_count threshold, or trigger a device reset.

            # Check break sensors (runs even if AHT sensor is having issues, as they are independent)
            check_break_sensors()
        else:
            # If not fully connected (Wi-Fi or MQTT), print status and wait before next check
            debug_print("Network or MQTT not fully connected, skipping operational tasks.")
            # A small sleep here prevents rapid looping when disconnected,
            # but the main backoff logic is handled by the time.sleep() calls
            # within the connection retry blocks.
            time.sleep(1)

        # --- Periodic Garbage Collection ---
        if main_loop_counter >= GC_COLLECT_LOOP_INTERVAL:
            if DEBUG: # Only print memory info if debugging
                debug_print(f"Loop count {main_loop_counter}, performing gc.collect(). Mem before: {gc.mem_free()}")
            gc.collect()
            if DEBUG:
                debug_print(f"Memory free after gc.collect(): {gc.mem_free()} bytes")
            main_loop_counter = 0 # Reset counter

        # --- Loop Delay ---
        time.sleep(LOOP_DELAY_S)
        
#------------------------------------------------------------------------------
# MAIN EXECUTION (SCRIPT ENTRY POINT)
#------------------------------------------------------------------------------
if __name__ == '__main__':
    time.sleep(1) # Short delay for power stabilization after reset
    try:
        main()
    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Exiting gracefully.") # Critical print
    except Exception as e:
        # Catch any critical unhandled exception that escapes the main loop
        print("CRITICAL ERROR: Unhandled exception during main execution.") # Critical print
        sys.print_exception(e)
        print("Device will restart in 10 seconds...") # Critical print
        time.sleep(10)
        reset() # machine.reset
    finally:
        # This block executes on normal exit from try (if main() could end),
        # or after a caught KeyboardInterrupt.
        # It will NOT execute if reset() is called in the general Exception block above.
        print("--- TempSense Finalizing ---") # Critical print

        # Attempt to publish OFFLINE status if MQTT client was connected
        if mqtt_client and base_topic_prefix: # Check if objects exist
            try:
                # Use direct publish call as publish_mqtt_message might also be debug-heavy
                # and mqtt_client might be None if publish_mqtt_message itself failed.
                # However, if mqtt_client is not None here, it should be usable.
                mqtt_client.publish(f"{base_topic_prefix}/status".encode(), b'OFFLINE', retain=True, qos=0)
                debug_print("Final OFFLINE status published.")
                mqtt_client.disconnect()
                debug_print("Disconnected from MQTT broker (finally).")
            except Exception as final_e:
                debug_print(f"Error during final MQTT cleanup: {final_e}")

        # Disconnect Wi-Fi if it was connected
        if wlan is not None and wlan.isconnected(): # wlan is a global
            try:
                wlan.disconnect()
                wlan.active(False)
                debug_print("Disconnected from Wi-Fi (finally).")
            except Exception as final_e:
                debug_print(f"Error during final Wi-Fi cleanup: {final_e}")
