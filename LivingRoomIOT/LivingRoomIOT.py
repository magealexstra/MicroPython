###############################################################################
# Living Room IoT Controller - ESP32 MicroPython
#
# Description:
#     Manages sensors and relays in a living room environment.
#     Connects to Wi-Fi, communicates via MQTT, and handles physical inputs.
#     Designed for resource-constrained ESP32 running MicroPython.
#
# Features:
# - Wi-Fi connectivity with automatic reconnection
# - MQTT communication with QoS 0 for efficiency
# - SHT31 temperature/humidity readings with EMA smoothing
# - Generic open/close sensor support with input debouncing
# - Analog light sensor with automatic relay control
# - Multiple relay control via MQTT and physical inputs
# - Momentary button and static switch support
# - Explicit mapping of physical inputs to relays
# - Unique device ID based on MAC address
# - Connection retry with exponential backoff
# - Configurable debug logging
###############################################################################

import network     # WiFi connectivity (STA_IF = station mode)
import time        # Time functions (ticks_ms for timing, sleep for delays)
import ubinascii   # Binary/hex conversion (smaller than binascii)
import sys         # System functions and error handling
import ntptime     # NTP time sync (no dependencies)
import json        # JSON for MQTT payloads (memory efficient)
from machine import Pin, I2C, ADC, reset  # ESP32 hardware control
from umqtt.simple import MQTTClient  # Lightweight MQTT client
import aht
import webrepl
import neopixel


# Import secrets (Wi-Fi credentials, MQTT broker info)
try:
    from secrets import WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD, STATIC_IP, SUBNET_MASK, GATEWAY, DNS_SERVER
except ImportError:
    print("Failed to import secrets. Please ensure 'secrets.py' is in the project root directory.")
    raise

#------------------------------------------------------------------------------
# Configuration Settings - adjust these for your setup
# ------------------------------------------------------------------------------

# Debug settings
DEBUG = False  # [CONFIG] Set to True to see detailed logs (helpful for troubleshooting)

# Time Synchronization Settings
NTP_SYNC_INTERVAL = 3600  # [CONFIG] Seconds between NTP sync attempts if initial sync fails

# MQTT Configuration
MQTT_TOPIC_PREFIX = 'livingroom'        # [CONFIG] Base prefix for all MQTT topics for this device type
MQTT_USE_DEVICE_ID = True               # [CONFIG] Set True to include unique device ID (last 5 MAC digits) in topics
                                        #          e.g., livingroom/abc12/sensor vs livingroom/sensor
MQTT_TOPIC_TEMPERATURE = 'temperature'  # [CONFIG] Sub-topic for publishing temperature data (appended to prefix/device_id)
MQTT_TOPIC_HUMIDITY = 'humidity'        # [CONFIG] Sub-topic for publishing humidity data (appended to prefix/device_id)
MQTT_TOPIC_LIGHT_SENSOR = 'livingroomlight' # [CONFIG] Sub-topic for publishing light sensor readings (appended to prefix/device_id) - NOTE: This is no longer published by default
MQTT_TOPIC_DAYLIGHT_EVENT = 'daylight_event' # [CONFIG] Sub-topic for publishing DAY/NIGHT transition events
# Relay topics are generated dynamically using prefix, device_id (optional), pin number, and command/state
# Generic Open/Close Sensor topics are defined in SENSOR_CONFIG below

# I2C Configuration for AHT2x Temperature/Humidity Sensor
I2C_SCL_PIN = 13    # [CONFIG] GPIO pin for I2C Clock (SCL)
I2C_SDA_PIN = 12    # [CONFIG] GPIO pin for I2C Data (SDA)
I2C_FREQ = 100000 # [CONFIG] I2C communication frequency in Hz
SENSOR_ADDR = 0x38  # [CONFIG] I2C address for AHT2x sensor (common is 0x38 or 0x3F)
        
# Sensor Reading & Smoothing Configuration (AHT2x)
EMA_ALPHA = 0.1           # [CONFIG] Exponential Moving Average smoothing factor (0.0 to 1.0). Lower values = smoother but slower response.
TEMP_THRESHOLD = 0.3      # [CONFIG] Minimum temperature change (degrees C) to trigger MQTT publish
HUMIDITY_THRESHOLD = 1.0  # [CONFIG] Minimum humidity change (%) to trigger MQTT publish

# Light Sensor Configuration (Analog)
LIGHT_SENSOR_PIN = None     # [CONFIG] GPIO pin for analog light sensor (e.g., 36 for ADC1_CH0). None = disabled.
LIGHT_THRESHOLD = 500       # [CONFIG] Light level cutoff (lower = darker when lights turn ON). Depends on sensor/circuit.
LIGHT_CHECK_INTERVAL = 10   # [CONFIG] Seconds between light sensor checks for automatic control.
LIGHT_CONTROLLED_RELAYS = []# [CONFIG] List of relay pins (e.g., [5, 18]) to be controlled by the light sensor. Empty = no auto control.

# GPIO Pin Configuration
RELAY_PINS = [20]  # [CONFIG] List of GPIO pins connected to relays. MUST match relays controlled by buttons/light sensor.
BUTTON_PINS = [4] # [CONFIG] List of GPIO pins connected to physical buttons/switches. MUST match keys in BUTTON_RELAY_MAP and BUTTON_TYPES.

# Button/Switch Configuration
# [CONFIG] Defines which button/switch controls which relay. Format: {button_pin: relay_pin}
BUTTON_RELAY_MAP = {
    4: 20,    # Button on pin 4 controls relay on pin 15
   # 5: 16,  # Button on pin 5 controls relay on pin 16
   # 6: 17,  # Button on pin 6 controls relay on pin 17
}
BUTTON_TYPES = {
    4: "momentary",  # Pin 4 is a push button (toggles on press)
    5: "momentary", # Pin 5 is a push button
    6: "momentary",    # Pin 6 is a momentary button
        # use "momentary" and "static" as the button types
}
 
# Debounce settings (prevents spurious triggers)
BUTTON_DEBOUNCE_MS = 300           # Ignores changes within this window
 
# Generic open/close sensor configuration (extensible)
SENSOR_CONFIG = {
    "frontdoor": { # Renamed from "door"
        "pin": 2,      # [CONFIG] GPIO pin for front door sensor. None = disabled.
        "topic": "frontdoor", # [CONFIG] MQTT sub-topic for this sensor.
        "debounce_ms": 300 # [CONFIG] Debounce time in milliseconds.
    },
    "livingroomwindow1": { # Renamed from "window"
        "pin": None,   # [CONFIG] GPIO pin for first living room window sensor. None = disabled (placeholder).
        "topic": "livingroomwindow1", # [CONFIG] MQTT sub-topic for this sensor.
        "debounce_ms": 300
    },
    "livingroomwindow2": { # Added second window sensor
        "pin": None,   # [CONFIG] GPIO pin for second living room window sensor. None = disabled (placeholder).
        "topic": "livingroomwindow2", # [CONFIG] MQTT sub-topic for this sensor.
        "debounce_ms": 300
    },
}



# RGB LED Status Indicator
RGB_LED_PIN = 48
STATUS_COLOR_WHITE = (255, 255, 255)
STATUS_COLOR_BLUE = (0, 0, 255)
STATUS_COLOR_CYAN = (0, 255, 255)
STATUS_COLOR_PURPLE = (128, 0, 128)
STATUS_COLOR_GREEN = (0, 255, 0)
STATUS_COLOR_RED = (255, 0, 0)
STATUS_COLOR_ORANGE = (255, 165, 0)
STATUS_COLOR_YELLOW = (255, 255, 0)

# Main loop and connection settings
LOOP_DELAY = 1                     # Controls CPU usage and responsiveness
MAX_CONNECTION_RETRIES = 10        # Prevents boot loops, triggers reset after failures
 
#------------------------------------------------------------------------------
# GLOBAL VARIABLES - INTERNAL STATE TRACKING (DO NOT MODIFY)
#------------------------------------------------------------------------------
device_id = None            # Unique device identifier (last 5 MAC digits), set during init if MQTT_USE_DEVICE_ID is True.
mqtt_client = None          # Holds the MQTT client object after successful connection.
relays = {}                 # Dictionary to store relay Pin objects, keyed by GPIO pin number. e.g., {5: Pin(5, Pin.OUT)}
buttons = {}                # Dictionary to store button Pin objects, keyed by GPIO pin number. e.g., {4: Pin(4, Pin.IN, Pin.PULL_UP)}
light_sensor = None         # ADC object when enabled, None otherwise
aht_sensor = None           # AHT2x sensor object when initialized, None otherwise
status_led = None           # NeoPixel object for status indicator
status_led_current_color = None # Global variable to track the current LED color

last_button_states = {}     # {pin_num: state} - 1=released, 0=pressed (pull-up)
last_button_times = {}      # {pin_num: timestamp} - for debouncing in ticks_ms
sensors = {}                # {name: Pin} - generic sensor pin objects
last_sensor_states = {}     # {name: state} - sensor state (1=open, 0=closed)
last_sensor_times = {}      # {name: timestamp} - sensor state change time

# Sensor data processing
ema_temp = None             # Smoothed temperature using EMA algorithm
ema_humidity = None         # Smoothed humidity using EMA algorithm
last_sent_temp = None       # Last published temperature for threshold comparison
last_sent_humidity = None   # Last published humidity for threshold comparison
was_previously_night = None # Previous light state for transition detection
last_light_check = 0        # Timestamp for rate-limiting light sensor checks

#------------------------------------------------------------------------------
# UTILITY FUNCTIONS
#------------------------------------------------------------------------------

def debug_print(*args, **kwargs):
    """Prints debug messages only if DEBUG is enabled.
    
    Optimized for memory usage by joining strings before printing.
    """
    if DEBUG:
        # Format messages efficiently using f-strings
        message = " ".join(f"{arg}" for arg in args)
        # Limit message length to reduce memory pressure
        if len(message) > 100:
            message = message[:97] + "..."
        print(message)

def get_device_id_str():
    """Gets a unique device identifier from the MAC address.

    Uses the network module to access the MAC address without
    requiring additional storage. Extracts only the last 5 characters
    to keep topic strings short while maintaining uniqueness.

    Returns:
        str: Last 5 characters of the MAC address (10 hex digits would be overkill)
    """
    wlan = network.WLAN(network.STA_IF)  # Station interface
    mac = wlan.config('mac')  # Raw byte representation of MAC
    mac_hex = ubinascii.hexlify(mac).decode()  # Convert to hex string
    return mac_hex[-5:]  # Last 5 chars (unique enough for home use)

def sync_time():
    """Synchronizes the device's internal clock using NTP.

    Returns:
        bool: True if synchronization succeeded, False otherwise.
    """
    print("Attempting to sync time with NTP...")
    try:
        ntptime.settime() # Blocks until time is set or error occurs
        print(f"Time synchronized successfully (UTC): {time.localtime()}")
        return True
    except Exception as e:
        debug_print(f"Failed to sync time with NTP: {type(e).__name__} - {str(e)}")
        return False

def get_unique_client_id():
    """Creates a unique MQTT client ID string (bytes) for this device.

    Combines the MQTT_TOPIC_PREFIX and the unique device ID.

    Returns:
        bytes: A unique client ID string encoded in bytes.
    """
    global device_id
    if device_id is None: # Ensure device_id is fetched if needed
         device_id = get_device_id_str()
    return f"{MQTT_TOPIC_PREFIX}_{device_id}".encode()

def connect_wifi():
    """Connects to the WiFi network using credentials from secrets.py.

    Retries connection until successful. Required for MQTT communication.

    Returns:
        network.WLAN: The connected WLAN interface object.
    """
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    # Configure static IP if provided in secrets.py
    if STATIC_IP and SUBNET_MASK and GATEWAY and DNS_SERVER:
        wlan.ifconfig((STATIC_IP, SUBNET_MASK, GATEWAY, DNS_SERVER))
        print(f'Configured static IP: {STATIC_IP}')
    
    if not wlan.isconnected():
        print('Connecting to Wi-Fi...')
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        # Wait for connection to establish with timeout
        connect_timeout = 20  # seconds
        start_time = time.time()
        while not wlan.isconnected() and time.time() - start_time < connect_timeout:
            time.sleep(0.5)
        
        if not wlan.isconnected():
            print('Wi-Fi connection failed after timeout')
            return wlan  # Return the WLAN object even if not connected, for consistent handling
        
        print(f'Connected to Wi-Fi: {wlan.ifconfig()}')
    return wlan

#------------------------------------------------------------------------------
# MQTT TOPIC HELPER FUNCTIONS
#------------------------------------------------------------------------------

def get_base_topic():
    """Gets the base MQTT topic string (prefix + optional device ID)."""
    if MQTT_USE_DEVICE_ID:
        return f'{MQTT_TOPIC_PREFIX}/{device_id}'
    else:
        return MQTT_TOPIC_PREFIX

def get_main_command_topic():
    """Gets the main command topic string (for controlling all relays or device actions)."""
    return f'{get_base_topic()}/command'

def get_main_state_topic():
    """Gets the main state topic string (for device ONLINE/OFFLINE status)."""
    return f'{get_base_topic()}/state'

def get_relay_command_topic(pin_num):
    """Gets the command topic string for a specific relay pin."""
    return f'{get_base_topic()}/relay/{pin_num}/command'

def get_relay_state_topic(pin_num):
    """Gets the state topic string for a specific relay pin."""
    return f'{get_base_topic()}/relay/{pin_num}/state'

def get_temperature_topic():
    """Gets the topic string for publishing SHT31 temperature data."""
    return f'{get_base_topic()}/{MQTT_TOPIC_TEMPERATURE}'

def get_humidity_topic():
    """Gets the topic string for publishing SHT31 humidity data."""
    return f'{get_base_topic()}/{MQTT_TOPIC_HUMIDITY}'

def get_specific_sensor_topic(sensor_name):
    """Gets the MQTT topic string for a specific generic sensor."""
    if sensor_name in SENSOR_CONFIG and "topic" in SENSOR_CONFIG[sensor_name]:
        return f'{get_base_topic()}/{SENSOR_CONFIG[sensor_name]["topic"]}'
    else:
        debug_print(f"Warning: Topic configuration not found for sensor '{sensor_name}'.")
        return None # Or return a default/error topic

def get_light_sensor_topic():
    """Gets the topic string for publishing analog light sensor readings."""
    return f'{get_base_topic()}/{MQTT_TOPIC_LIGHT_SENSOR}'

def get_daylight_event_topic():
    """Gets the topic string for publishing DAY/NIGHT transition events."""
    return f'{get_base_topic()}/{MQTT_TOPIC_DAYLIGHT_EVENT}'

#------------------------------------------------------------------------------
# MQTT FUNCTIONS
#------------------------------------------------------------------------------

def get_button_event_topic():
    """Gets the MQTT topic string for publishing button press events."""
    return f'{get_base_topic()}/button_event'


def mqtt_callback(topic, msg):
    """Handles incoming MQTT messages.

    Called by the MQTT client. Parses topics for main commands or specific relay commands.

    Args:
        topic (bytes): The topic the message was received on.
        msg (bytes): The message payload.
    """
    try:
        topic_str = topic.decode()
        command = msg.decode().strip().upper()
        debug_print(f"Received MQTT: Topic='{topic_str}', Message='{command}'")

        # Check for main command topic
        if topic_str == get_main_command_topic():
            debug_print(f"Processing main command: {command}")
            if command == 'ON' or command == 'OFF':
                for pin_num in relays:
                    set_relay(pin_num, command) # Apply to all initialized relays
            else:
                debug_print(f"Unknown main command: {command}")
            return

        # Check for individual relay command topics
        for pin_num in relays:
            if topic_str == get_relay_command_topic(pin_num):
                debug_print(f"Processing command for relay {pin_num}: {command}")
                if command == 'ON' or command == 'OFF':
                    set_relay(pin_num, command)
                else:
                    debug_print(f"Unknown command '{command}' for relay {pin_num}")
                return # Handled this specific relay

        # If loop completes, no matching topic was found
        debug_print(f"Received message on unhandled topic: {topic_str}")

    except Exception as e:
        debug_print(f"Error in MQTT callback: {type(e).__name__} - {str(e)}")

def connect_mqtt():
    """Connects to the MQTT broker and subscribes to relevant topics.

    Sets up the MQTT client with a unique ID, defines the LWT, connects,
    subscribes to main and relay command topics, and publishes initial states.

    Returns:
        MQTTClient: The connected MQTT client object, or None if connection failed.
    """
    global mqtt_client
    try:
        client_id = get_unique_client_id() # Ensure device_id is set here
        client = MQTTClient(client_id, MQTT_BROKER, port=MQTT_PORT, user=MQTT_USERNAME, password=MQTT_PASSWORD)

        # Set Last Will and Testament (LWT) on the main state topic
        main_state_topic = get_main_state_topic()
        client.set_last_will(main_state_topic.encode(), b'OFFLINE', retain=True, qos=0)
        debug_print(f"Set LWT on topic '{main_state_topic}' to 'OFFLINE'")

        client.set_callback(mqtt_callback)
        client.connect()
        debug_print(f"Connected to MQTT broker '{MQTT_BROKER}:{MQTT_PORT}' with client ID '{client_id.decode()}'")

        # Publish 'ONLINE' status to the main state topic (retained)
        client.publish(main_state_topic.encode(), b'ONLINE', retain=True, qos=0)
        debug_print(f"Published 'ONLINE' to topic '{main_state_topic}'")

        # Subscribe to main command topic
        main_cmd_topic = get_main_command_topic()
        client.subscribe(main_cmd_topic.encode())
        debug_print(f"Subscribed to main command topic: {main_cmd_topic}")

        # Subscribe to command topics for each relay
        for relay_pin in RELAY_PINS: # Subscribe based on configured relays
             # Check if relay was actually initialized (might not be if config mismatch)
            if relay_pin in relays:
                topic = get_relay_command_topic(relay_pin)
                client.subscribe(topic.encode())
                debug_print(f"Subscribed to relay command topic: {topic}")

        mqtt_client = client

        # Publish the initial state of all initialized relays
        for relay_pin in relays:
            publish_relay_state(relay_pin)
        # Publish initial light sensor reading (if enabled)
        if light_sensor:
             publish_light_sensor_reading()

        return client

    except Exception as e:
        debug_print(f"Failed to connect to MQTT broker: {type(e).__name__} - {str(e)}")
        return None

def publish_relay_state(relay_pin):
    """Publishes the current state (ON/OFF) of a specific relay to its MQTT state topic.
    
    Uses retained messages (retain=True) to ensure subscribers receive the last known
    state immediately upon connection. QoS 0 is used for minimal overhead on the ESP32.
    """
    global mqtt_client
    if mqtt_client is None or relay_pin not in relays:
        debug_print(f"MQTT client not available or relay {relay_pin} not initialized, skipping state publish.")
        return
    try:
        state = 'ON' if relays[relay_pin].value() else 'OFF'
        topic = get_relay_state_topic(relay_pin)
        mqtt_client.publish(topic.encode(), state.encode(), retain=True, qos=0)
        debug_print(f"Published relay {relay_pin} state '{state}' to topic '{topic}'")
        flash_status_led(STATUS_COLOR_YELLOW, 1000)
    except Exception as e:
        debug_print(f"Failed to publish relay state for pin {relay_pin}: {type(e).__name__} - {str(e)}")
        mqtt_client = None # Signal to reconnect

def publish_sensor_state(sensor_name):
    """Publishes the current state (OPEN/CLOSED) of a generic sensor.
    
    Uses retained messages (retain=True) to ensure Home Assistant or other subscribers
    receive the current state immediately on connection. Handles pull-up resistor
    logic (1=OPEN, 0=CLOSED) for standardized MQTT messaging.
    """
    global mqtt_client # Uses global mqtt_client
    if mqtt_client is None or sensor_name not in sensors:
        debug_print(f"MQTT client or sensor '{sensor_name}' not available, skipping state publish.")
        return
    try:
        # Use last known debounced state
        current_state = last_sensor_states.get(sensor_name)
        if current_state is None: # Should not happen if initialized correctly, but safety check
             current_state = sensors[sensor_name].value()
             debug_print(f"Warning: last_sensor_state for '{sensor_name}' was None, reading current value.")

        # Assuming PULL_UP: 1 = OPEN (switch open), 0 = CLOSED (switch closed)
        state_str = 'OPEN' if current_state else 'CLOSED'
        topic = get_specific_sensor_topic(sensor_name)
        if topic:
            mqtt_client.publish(topic.encode(), state_str.encode(), retain=True, qos=0)
            debug_print(f"Published {sensor_name} state '{state_str}' to topic '{topic}'")
            flash_status_led(STATUS_COLOR_YELLOW, 1000)
    except Exception as e:
        debug_print(f"Failed to publish {sensor_name} sensor state: {type(e).__name__} - {str(e)}")
        mqtt_client = None # Signal to reconnect

def publish_light_sensor_reading():
    """Reads and publishes the current analog light sensor reading.
    
    Provides raw ADC values rather than interpreted values to allow
    for flexible threshold configuration on the receiving end. Uses
    non-retained messages since light readings are frequently updated.
    """
    global mqtt_client
    if mqtt_client is None or light_sensor is None:
        debug_print("MQTT client or light sensor not available, skipping light reading publish.")
        return
    try:
        light_level = read_light_sensor()
        if light_level is not None:
            topic = get_light_sensor_topic()
            mqtt_client.publish(topic.encode(), f"{light_level}".encode(), retain=False, qos=0)
            debug_print(f"Published light level '{light_level}' to topic '{topic}'")
            flash_status_led(STATUS_COLOR_YELLOW, 1000)
    except Exception as e:
        debug_print(f"Failed to publish light sensor reading: {type(e).__name__} - {str(e)}")
        mqtt_client = None # Signal to reconnect

def publish_daylight_event(event_type, utc_timestamp):
    """Publishes a DAY or NIGHT event with the UTC timestamp.
    
    Uses retained messages (retain=True) to ensure new subscribers can 
    immediately determine the current daylight state. JSON format includes 
    UTC timestamp to allow for time-based automations and analysis.
    """
    global mqtt_client
    if mqtt_client is None:
        debug_print("MQTT client not available, skipping daylight event publish.")
        return
    try:
        payload = json.dumps({
            "event": event_type,        # "DAY" or "NIGHT"
            "utc_timestamp": utc_timestamp # Integer epoch seconds
        })
        topic = get_daylight_event_topic()
        mqtt_client.publish(topic.encode(), payload.encode(), retain=True, qos=0) # Retain the last event
        debug_print(f"Published daylight event '{payload}' to topic '{topic}'")
        flash_status_led(STATUS_COLOR_YELLOW, 1000)
    except Exception as e:
        debug_print(f"Failed to publish daylight event: {type(e).__name__} - {str(e)}")
        mqtt_client = None # Signal to reconnect

#------------------------------------------------------------------------------
# HARDWARE CONTROL FUNCTIONS
#------------------------------------------------------------------------------

def set_relay(pin_num, state):
    """Sets the state of a specific relay and publishes the change via MQTT.
    
    Handles error cases gracefully to prevent crashing when hardware issues occur.
    Only publishes state changes to MQTT when the hardware state actually changes,
    reducing network traffic. GPIO pins are completely abstracted through the Pin 
    class for cross-compatibility.

    Args:
        pin_num (int): The GPIO pin number of the relay.
        state (str): The desired state ('ON' or 'OFF').
    """
    if pin_num not in relays:
        debug_print(f"Attempted to set unknown relay pin: {pin_num}")
        return
    
    target_value = 1 if state == 'ON' else 0
    current_value = relays[pin_num].value()

    if current_value != target_value:
        try:
            relays[pin_num].value(target_value)
            debug_print(f"Relay {pin_num} set to {state}")
            publish_relay_state(pin_num) # Publish the change
        except Exception as e:
            debug_print(f"Failed to set relay state for pin {pin_num}: {type(e).__name__} - {str(e)}")
    else:
        debug_print(f"Relay {pin_num} already in state {state}")


#------------------------------------------------------------------------------
# LED CONTROL FUNCTIONS
#------------------------------------------------------------------------------

def init_status_led():
    global status_led
    try:
        status_led = neopixel.NeoPixel(Pin(RGB_LED_PIN), 1)
        print(f"Status LED initialized on GPIO {RGB_LED_PIN}.")
    except Exception as e:
        debug_print(f"Failed to initialize status LED on GPIO {RGB_LED_PIN}: {type(e).__name__} - {str(e)}")
        status_led = None

def set_status_led(color):
    if status_led:
        try:
            status_led[0] = color
            status_led.write()
            global status_led_current_color
            status_led_current_color = color
        except Exception as e:
            debug_print(f"Failed to set status LED color: {type(e).__name__} - {str(e)}")


def flash_status_led(color, duration_ms):
    """Flashes the status LED a specific color for a duration and restores the previous color."""
    global status_led_current_color
    if status_led and status_led_current_color is not None:
        try:
            # Save current color
            previous_color = status_led_current_color

            # Set flash color
            status_led[0] = color
            status_led.write()

            # Wait
            time.sleep_ms(duration_ms)

            # Restore previous color
            status_led[0] = previous_color
            status_led.write()

        except Exception as e:
            debug_print(f"Failed to flash status LED: {type(e).__name__} - {str(e)}")


#------------------------------------------------------------------------------
# INITIALIZATION FUNCTIONS
#------------------------------------------------------------------------------

def init_relays():
    """Initializes all configured relay pins.
    
    Creates Pin objects for each defined relay pin from RELAY_PINS list. 
    Sets all relays to OFF state initially for safety. Handles individual 
    initialization failures gracefully to allow the system to continue 
    running even if some relays aren't properly connected.
    """
    global relays
    print("Initializing relays...")
    count = 0
    for pin_num in RELAY_PINS:
        try:
            pin = Pin(pin_num, Pin.OUT)
            pin.value(0)  # Ensure relay starts OFF
            relays[pin_num] = pin
            debug_print(f"Relay initialized on GPIO {pin_num}")
            count += 1
        except Exception as e:
             debug_print(f"Failed to initialize relay on GPIO {pin_num}: {type(e).__name__} - {str(e)}")
    print(f"Initialized {count}/{len(RELAY_PINS)} relays.")

def init_buttons():
    """Initializes configured button/switch pins based on BUTTON_RELAY_MAP.
    
    Creates Pin objects for input pins with internal pull-up resistors activated.
    Sets up initial state tracking for debouncing. Validates button-to-relay 
    mapping to detect configuration issues early during initialization rather 
    than during runtime operation when issues would be harder to diagnose.
    """
    global buttons, last_button_states, last_button_times
    print("Initializing buttons/switches...")
    count = 0
    # Iterate through the mapping to initialize only mapped buttons
    for button_pin in BUTTON_RELAY_MAP.keys():
         # Check if the button pin is also in the main BUTTON_PINS list (optional sanity check)
        if button_pin not in BUTTON_PINS:
            print(f"Warning: Button pin {button_pin} in BUTTON_RELAY_MAP but not in BUTTON_PINS list. Skipping.")
            continue
        try:
            pin = Pin(button_pin, Pin.IN, Pin.PULL_UP) # Input with internal pull-up
            buttons[button_pin] = pin
            last_button_states[button_pin] = pin.value() # Store initial state
            last_button_times[button_pin] = 0 # Initialize debounce timer
            button_type = BUTTON_TYPES.get(button_pin, "momentary") # Default to momentary
            debug_print(f"Button/Switch ({button_type}) initialized on GPIO {button_pin}, controlling relay {BUTTON_RELAY_MAP[button_pin]}")
            count += 1
        except Exception as e:
            debug_print(f"Failed to initialize button on GPIO {button_pin}: {type(e).__name__} - {str(e)}")

    # Verify all pins in BUTTON_PINS were considered (optional check)
    for pin_num in BUTTON_PINS:
        if pin_num not in BUTTON_RELAY_MAP:
             print(f"Warning: Button pin {pin_num} in BUTTON_PINS list but not in BUTTON_RELAY_MAP. It will not be functional.")

    print(f"Initialized {count} buttons/switches.")

def init_sensor(sensor_name, config):
    """Initializes a generic open/close sensor based on configuration.

    Creates Pin objects with pull-up resistors for door/window sensors where
    CLOSED=0 (circuit complete) and OPEN=1 (circuit broken). Safely handles
    disabled sensors (pin=None) and initialization failures. Records initial
    state and debounce timing information for state change detection.

    Returns:
        bool: True if initialization successful, False otherwise.
    """
    # Uses global dictionaries: sensors, last_sensor_states, last_sensor_times
    global sensors, last_sensor_states, last_sensor_times

    pin_num = config.get("pin")
    if pin_num is None:
        debug_print(f"Sensor '{sensor_name}' disabled (pin not set).")
        return False # Indicate not initialized

    print(f"Initializing sensor '{sensor_name}' on GPIO {pin_num}...")
    try:
        pin_obj = Pin(pin_num, Pin.IN, Pin.PULL_UP) # Input with internal pull-up
        sensors[sensor_name] = pin_obj
        last_sensor_states[sensor_name] = pin_obj.value() # Store initial state
        last_sensor_times[sensor_name] = time.ticks_ms() # Initialize debounce timer
        debug_print(f"Sensor '{sensor_name}' initialized on GPIO {pin_num}")
        print(f"Sensor '{sensor_name}' initialized.")
        return True # Indicate success
    except Exception as e:
        debug_print(f"Failed to initialize sensor '{sensor_name}' on GPIO {pin_num}: {type(e).__name__} - {str(e)}")
        # Ensure sensor entry is removed or marked as invalid if init fails partially
        if sensor_name in sensors: del sensors[sensor_name]
        if sensor_name in last_sensor_states: del last_sensor_states[sensor_name]
        if sensor_name in last_sensor_times: del last_sensor_times[sensor_name]
        return False # Indicate failure


def init_light_sensor():
    """Initializes the analog light sensor if configured.

    Creates an ADC object on the specified pin. Configures 11dB attenuation
    for full 0-3.3V input range on ESP32. The light sensor is essential for
    daylight detection and automatic relay control features. Safely handles
    both disabled configuration (LIGHT_SENSOR_PIN=None) and hardware failures.
    """
    global light_sensor
    if LIGHT_SENSOR_PIN is None:
        debug_print("Analog light sensor disabled (LIGHT_SENSOR_PIN not set).")
        light_sensor = None
        return

    print(f"Initializing analog light sensor on GPIO {LIGHT_SENSOR_PIN}...")
    try:
        # Create ADC (Analog to Digital Converter) object on the specified pin
        pin = Pin(LIGHT_SENSOR_PIN)
        adc = ADC(pin)

        # Configure ADC attenuation for full voltage range (0-3.3V)
        try:
            adc.atten(ADC.ATTN_11DB)
            debug_print("Set ADC attenuation to 11dB (0-3.3V range)")
        except AttributeError:
            debug_print("Warning: Could not set ADC attenuation.")
            pass # Continue with default range

        light_sensor = adc
        print(f"Analog light sensor initialized on GPIO {LIGHT_SENSOR_PIN}.")
        if LIGHT_CONTROLLED_RELAYS:
             print(f"Light sensor will control relays: {LIGHT_CONTROLLED_RELAYS}")
        else:
             print("Light sensor automatic control disabled (LIGHT_CONTROLLED_RELAYS is empty).")

    except Exception as e:
        debug_print(f"Failed to initialize analog light sensor on GPIO {LIGHT_SENSOR_PIN}: {type(e).__name__} - {str(e)}")
        light_sensor = None

#------------------------------------------------------------------------------
# SENSOR READING FUNCTIONS
#------------------------------------------------------------------------------


def read_light_sensor():
    """Reads the current value from the analog light sensor.

    ESP32 ADC provides 12-bit resolution (0-4095 values). Error handling
    prevents crashes if the sensor becomes disconnected. The raw ADC value
    is returned without conversion to allow flexible interpretation by
    different parts of the code.

    Returns:
        int: The raw ADC reading (typically 0-4095), or None if sensor disabled/error.
    """
    if light_sensor is None:
        return None
    try:
        raw_value = light_sensor.read()
        debug_print(f"Analog light sensor reading: {raw_value}")
        return raw_value
    except Exception as e:
        debug_print(f"Error reading analog light sensor: {type(e).__name__} - {str(e)}")
        return None

#------------------------------------------------------------------------------
# MAIN LOOP CHECK FUNCTIONS
#------------------------------------------------------------------------------

def check_buttons():
    """Checks button/switch states, debounces, and controls relays via BUTTON_RELAY_MAP.

    Implements time-based debouncing using time.ticks_ms() and ticks_diff() for
    efficient timing comparisons. Uses different logic paths for:
    - Momentary buttons: Toggles relays only on press (not release)
    - Static switches: Directly follows switch position

    Handles various error cases such as unmapped buttons and missing relays
    to prevent crashes during reconfiguration.
    """
    current_time = time.ticks_ms()
    for button_pin, button in buttons.items(): # Iterate through initialized buttons
        current_state = button.value()
        last_state = last_button_states.get(button_pin) # Use .get for safety

        if current_state != last_state:
            if time.ticks_diff(current_time, last_button_times.get(button_pin, 0)) > BUTTON_DEBOUNCE_MS:
                # Debounce time passed, process the change

                # Get the type and mapped relay for this button
                button_type = BUTTON_TYPES.get(button_pin, "momentary") # Default to momentary
                relay_pin = BUTTON_RELAY_MAP.get(button_pin)
                global mqtt_client
                if mqtt_client: # Check if MQTT client is connected
                    try:
                        # Create a payload with button information
                        payload = json.dumps({
                            "button_pin": button_pin,
                            "button_type": button_type,
                            "state": current_state # 0 for pressed (pull-up), 1 for released
                        })
                        topic = get_button_event_topic()
                        mqtt_client.publish(topic.encode(), payload.encode(), retain=False, qos=0) # Don't retain button events
                        debug_print(f"Published button event for pin {button_pin}: {payload}")
                        flash_status_led(STATUS_COLOR_YELLOW, 1000)
                    except Exception as e:
                        sys.print_exception(e)
                        debug_print(f"Failed to publish button event for pin {button_pin}: {e}")
                        # Optionally, set mqtt_client = None here to trigger a reconnect if publish fails
                # --- End of added section ---




                if relay_pin is None:
                     debug_print(f"Button {button_pin} changed state but is not in BUTTON_RELAY_MAP.")
                     continue # Skip if button isn't mapped

                if relay_pin not in relays:
                     debug_print(f"Button {button_pin} mapped to uninitialized relay {relay_pin}.")
                     continue # Skip if mapped relay wasn't initialized

                # --- Logic based on button type ---
                if button_type == "momentary":
                    # Toggle only on press (1 -> 0 transition with pull-up)
                    if current_state == 0:
                        current_relay_state = relays[relay_pin].value()
                        new_state_str = 'OFF' if current_relay_state else 'ON'
                        debug_print(f"Momentary button {button_pin} pressed, toggling relay {relay_pin} to {new_state_str}")
                        set_relay(relay_pin, new_state_str) # Use central function
                elif button_type == "static":
                    # Follow the switch state (0 = ON, 1 = OFF with pull-up)
                    new_state_str = 'ON' if current_state == 0 else 'OFF'
                    debug_print(f"Static switch {button_pin} changed, setting relay {relay_pin} to {new_state_str}")
                    set_relay(relay_pin, new_state_str) # Use central function
                else:
                     debug_print(f"Unknown button type '{button_type}' for pin {button_pin}")

                # Update last change time
                last_button_times[button_pin] = current_time
            # Update last known state
            last_button_states[button_pin] = current_state
            
def check_sensor(sensor_name):
    """Checks the state of a generic sensor, debounces, and publishes changes.
    
    Uses time-based debouncing with configurable periods per sensor type.
    Only publishes when state changes occur after the debounce period,
    which reduces MQTT traffic. Safely handles hardware read errors by
    skipping the current check cycle rather than crashing.
    """
    global last_sensor_states, last_sensor_times

    if sensor_name not in sensors:
        return # Skip if this sensor wasn't initialized

    sensor_pin = sensors[sensor_name]
    last_state = last_sensor_states.get(sensor_name)
    last_time = last_sensor_times.get(sensor_name, 0)
    debounce_ms = SENSOR_CONFIG.get(sensor_name, {}).get("debounce_ms", 300) # Default debounce

    current_time = time.ticks_ms()
    current_state = None
    try:
        current_state = sensor_pin.value()
    except Exception as e:
        debug_print(f"Error reading sensor '{sensor_name}' pin value: {type(e).__name__} - {str(e)}")
        return # Skip this sensor for this cycle

    # Proceed only if reading was successful and state is different
    if current_state is not None and current_state != last_state:
        # Check debounce timer
        if time.ticks_diff(current_time, last_time) > debounce_ms:
            state_str = 'OPEN' if current_state else 'CLOSED'
            debug_print(f"Sensor '{sensor_name}' state changed to: {state_str}")
            last_sensor_states[sensor_name] = current_state # Update state before publishing
            last_sensor_times[sensor_name] = current_time   # Update time
            publish_sensor_state(sensor_name) # Publish the new debounced state
def check_aht2x_sensor():
    """Reads AHT2x data, applies EMA, and publishes if thresholds are met."""
    global aht_sensor, mqtt_client, ema_temp, ema_humidity, last_sent_temp, last_sent_humidity

    if aht_sensor is None or mqtt_client is None:
        debug_print("Skipping AHT2x check: Sensor or MQTT not ready.")
        return # Skip if sensor not initialized or MQTT not connected

    try:
        # Trigger measurement and check if successful
        if aht_sensor._measure():
            # Read temperature and humidity from AHT2x sensor
            temperature = aht_sensor.temperature
            humidity = aht_sensor.humidity

            if temperature is not None and humidity is not None:
                # Apply Exponential Moving Average (EMA)
                if ema_temp is None:
                    ema_temp = temperature
                    ema_humidity = humidity
                    debug_print(f"EMA initialized (AHT2x): Temp={ema_temp:.2f}, Hum={ema_humidity:.2f}")
                else:
                    ema_temp = (EMA_ALPHA * temperature) + ((1 - EMA_ALPHA) * ema_temp)
                    ema_humidity = (EMA_ALPHA * humidity) + ((1 - EMA_ALPHA) * ema_humidity)
                    debug_print(f"EMA updated (AHT2x): Temp={ema_temp:.2f}, Hum={ema_humidity:.2f}")

                # Check if smoothed values exceed thresholds for publishing
            temp_changed = last_sent_temp is None or abs(ema_temp - last_sent_temp) >= TEMP_THRESHOLD
            humidity_changed = last_sent_humidity is None or abs(ema_humidity - last_sent_humidity) >= HUMIDITY_THRESHOLD

            if mqtt_client and (temp_changed or humidity_changed):
                debug_print("Threshold exceeded (AHT2x), publishing data.")
                # Publish temperature if changed
                if temp_changed:
                    try:
                        topic = get_temperature_topic()
                        mqtt_client.publish(topic.encode(), f"{ema_temp:.2f}".encode(), retain=False, qos=0)
                        last_sent_temp = ema_temp
                        debug_print(f"Published temperature (AHT2x): {last_sent_temp:.2f} C to {topic}")
                        flash_status_led(STATUS_COLOR_YELLOW, 1000)
                    except Exception as e:
                        debug_print(f"Failed to publish temperature (AHT2x): {type(e).__name__} - {str(e)}")
                        # Don't update last_sent_temp if publish fails
                        mqtt_client = None # Signal to reconnect

                # Publish humidity if changed
                if humidity_changed:
                    try:
                        topic = get_humidity_topic()
                        mqtt_client.publish(topic.encode(), f"{ema_humidity:.2f}".encode(), retain=False, qos=0)
                        last_sent_humidity = ema_humidity
                        debug_print(f"Published humidity (AHT2x): {last_sent_humidity:.2f} % to {topic}")
                        flash_status_led(STATUS_COLOR_YELLOW, 1000)
                    except Exception as e:
                        debug_print(f"Failed to publish humidity (AHT2x): {type(e).__name__} - {str(e)}")
                        # Don't update last_sent_humidity if publish fails
                        mqtt_client = None # Signal to reconnect

        else:
            debug_print('Invalid AHT2x sensor data received, skipping processing.')
            set_status_led(STATUS_COLOR_ORANGE) # Indicate sensor error

    except Exception as e:
        debug_print(f"Error reading or processing AHT2x data: {type(e).__name__} - {str(e)}")
        set_status_led(STATUS_COLOR_ORANGE) # Indicate sensor error
        # Consider setting aht_sensor = None here if a permanent error is detected
        # For now, just log and continue




def check_light_sensor():
    """Reads analog light sensor, detects DAY/NIGHT transitions, publishes events, and optionally controls relays.
    Rate-limited to LIGHT_CHECK_INTERVAL seconds to reduce CPU and network usage.
    
    Requires NTP time synchronization for accurate event timestamps. Automatically
    controls configured relays based on light levels, enabling automated lighting
    without requiring an external home automation controller.
    
    The threshold-based approach with hysteresis prevents rapid switching
    during edge conditions (dawn/dusk or fluctuating light).
    """
    global last_light_check, was_previously_night
    
    if light_sensor is None: return # Skip if disabled

    current_time_ms = time.ticks_ms() # Use a different name to avoid conflict with time module

    # Check light level and transitions only at specified intervals
    if time.ticks_diff(current_time_ms, last_light_check) >= (LIGHT_CHECK_INTERVAL * 1000):
        last_light_check = current_time_ms
        light_level = read_light_sensor()

        if light_level is not None:
            is_currently_night = light_level < LIGHT_THRESHOLD

            # Check for state transition (DAY -> NIGHT or NIGHT -> DAY)
            if was_previously_night is not None and is_currently_night != was_previously_night:
                try:
                    # Get current UTC epoch time
                    current_utc_epoch = time.time()
                    event_type = "NIGHT" if is_currently_night else "DAY"
                    debug_print(f"Light transition detected: {event_type} (Level: {light_level})")
                    publish_daylight_event(event_type, current_utc_epoch)
                except Exception as e:
                    # time.time() might fail if RTC is not set (e.g., NTP failed)
                    debug_print(f"Failed to get UTC time or publish daylight event: {type(e).__name__} - {str(e)}")

            # Update the state for the next check
            was_previously_night = is_currently_night

            # --- Automatic Relay Control (if enabled) ---
            if LIGHT_CONTROLLED_RELAYS:
                debug_print(f"Checking light control: Level={light_level}, Threshold={LIGHT_THRESHOLD}, Currently Night={is_currently_night}")
                for pin_num in LIGHT_CONTROLLED_RELAYS:
                    if pin_num not in relays:
                        debug_print(f"Relay {pin_num} in LIGHT_CONTROLLED_RELAYS but not initialized, skipping.")
                        continue

                    current_relay_state_val = relays[pin_num].value()

                    # If it's dark (night) and light is off, turn it on
                    if is_currently_night and current_relay_state_val == 0:
                        debug_print(f"Night detected (Level {light_level} < {LIGHT_THRESHOLD}), turning ON relay {pin_num}")
                        set_relay(pin_num, 'ON')

                    # If it's bright (day) and light is on, turn it off
                    elif not is_currently_night and current_relay_state_val == 1:
                        debug_print(f"Day detected (Level {light_level} >= {LIGHT_THRESHOLD}), turning OFF relay {pin_num}")
                        set_relay(pin_num, 'OFF')

#------------------------------------------------------------------------------
# MAIN EXECUTION
#------------------------------------------------------------------------------

def main():
    """Main program execution function.

    Orchestrates the entire application lifecycle:
    1. Hardware initialization (relays, buttons, sensors)
    2. Network setup (WiFi, MQTT, NTP)
    3. Initial state publishing
    4. Main operational loop with:
       - Connection monitoring and recovery with exponential backoff
       - Sensor data collection and threshold-based publishing
       - Physical input processing (buttons/switches)
       - Automatic relay control

    Implements restart strategy when reconnection fails, providing
    resilience against network outages. Uses memory-efficient techniques
    like dictionary-based state tracking rather than classes, and
    avoiding frequent string creation in tight loops.
    """
    global mqtt_client, device_id

    print("Starting Living Room IoT Controller...")
    set_status_led(STATUS_COLOR_WHITE) # Indicate booting/initializing

    # Set device ID early if needed for topics
    if MQTT_USE_DEVICE_ID:
        device_id = get_device_id_str()
        print(f"Device ID: {device_id}")
    else:
        print("Using non-unique MQTT topics (MQTT_USE_DEVICE_ID is False).")


    # Initialize hardware first
    init_relays()
    init_buttons()
    
    # Initialize generic sensors based on SENSOR_CONFIG
    print("Initializing generic sensors...")
    initialized_sensors = []
    for name, config in SENSOR_CONFIG.items():
        if init_sensor(name, config):
             initialized_sensors.append(name)
    print(f"Initialized {len(initialized_sensors)} generic sensors: {initialized_sensors}")
    init_light_sensor()
    init_status_led()

    # Connect to network services
    wlan = connect_wifi()

    # Synchronize time via NTP (requires Wi-Fi)
    time_synced = sync_time()
    # Continue regardless of time sync success

    # Connect to MQTT (requires Wi-Fi)
    mqtt_client = connect_mqtt() # Handles initial relay state publishing

    # Publish initial state for generic sensors
    if mqtt_client:
        for name in sensors: # Loop through successfully initialized sensors
             publish_sensor_state(name)

    # Publish initial DAY/NIGHT state if MQTT connected, light sensor enabled, and time synced
    if mqtt_client and light_sensor and time_synced:
        initial_light_level = read_light_sensor()
        if initial_light_level is not None:
            global was_previously_night # Need to set the global state
            was_previously_night = initial_light_level < LIGHT_THRESHOLD
            initial_event = "NIGHT" if was_previously_night else "DAY"
            try:
                initial_utc_epoch = time.time()
                publish_daylight_event(initial_event, initial_utc_epoch)
                print(f"Published initial daylight state: {initial_event}")
            except Exception as e:
                debug_print(f"Failed to get time or publish initial daylight state: {type(e).__name__} - {str(e)}")
        else:
             debug_print("Could not read initial light level for daylight state.")
    elif not time_synced:
         debug_print("Skipping initial daylight state publish because time is not synchronized.")


    print("Initialization complete. Entering main loop...")

    # Connection retry counters and separate backoff times
    wifi_retry_count = 0
    mqtt_retry_count = 0
    wifi_backoff_time = LOOP_DELAY
    mqtt_backoff_time = LOOP_DELAY

    # Initialize I2C bus for AHT2x sensor (using ID 0 for ESP32)
    print("Initializing I2C bus for AHT2x sensor...")
    try:
        i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
        devices = i2c.scan()
        debug_print('I2C devices found:', [hex(device) for device in devices])
        if SENSOR_ADDR not in devices:
            print(f"AHT2x sensor not found at address {hex(SENSOR_ADDR)}. Check wiring and address.")
            # Continue even if sensor not found, but aht_sensor will remain None
        else:
            print(f"AHT2x sensor found at address {hex(SENSOR_ADDR)}.")
            # Initialize AHT2x sensor
            global aht_sensor
            aht_sensor = aht.AHT2x(i2c, address=SENSOR_ADDR)
            print("AHT2x sensor initialized.")
    except Exception as e:
        debug_print(f"Failed to initialize I2C bus or AHT2x sensor: {type(e).__name__} - {str(e)}")
        # aht_sensor remains None
    webrepl.start(password='zigy5857')

    while True:

        try:
            # --- Connection Management ---
            # Reconnect Wi-Fi if disconnected
            if not wlan.isconnected():
                wifi_retry_count += 1
                print(f"Wi-Fi disconnected. Attempting reconnect {wifi_retry_count}/{MAX_CONNECTION_RETRIES}...")
                set_status_led(STATUS_COLOR_BLUE) # Indicate connecting to Wi-Fi
                status_code = wlan.status()
                print(f"Wi-Fi status code: {status_code}")
                time.sleep(5)
                wlan = connect_wifi() # Attempt reconnection
                print(f"WLAN status after connect_wifi: {wlan.status()}")

            if not wlan.isconnected():
                wifi_backoff_time = min(60, LOOP_DELAY * (2 ** wifi_retry_count)) # Increase max backoff
                print(f"Wi-Fi reconnection failed. Waiting {wifi_backoff_time}s...")
                time.sleep(wifi_backoff_time)
                if wifi_retry_count >= MAX_CONNECTION_RETRIES:
                    print("Max Wi-Fi retries reached. Restarting device.")
                    set_status_led(STATUS_COLOR_RED) # Indicate major error
                    time.sleep(1)
                    reset()

            # Check if Wi-Fi is connected but MQTT is not
            if wlan.isconnected() and mqtt_client is None:
                set_status_led(STATUS_COLOR_CYAN) # Indicate Wi-Fi connected, MQTT disconnected

            # Reconnect MQTT immediately after Wi-Fi comes back up
            if mqtt_client is None and wlan.isconnected():
                print("Attempting MQTT reconnect after Wi-Fi restored...")
                set_status_led(STATUS_COLOR_PURPLE) # Indicate connecting to MQTT
                mqtt_client = connect_mqtt()
                mqtt_retry_count = 0 # Reset MQTT counter too


            # Reconnect MQTT if disconnected (and Wi-Fi is connected)
            if mqtt_client is None and wlan.isconnected():
                mqtt_retry_count += 1
                print(f"MQTT disconnected. Attempting reconnect {mqtt_retry_count}/{MAX_CONNECTION_RETRIES}...")
                mqtt_client = connect_mqtt() # Attempt reconnection

                if mqtt_client is None:
                    mqtt_backoff_time = min(60, LOOP_DELAY * (2 ** mqtt_retry_count)) # Increase max backoff
                    print(f"MQTT reconnection failed. Waiting {mqtt_backoff_time}s...")
                    time.sleep(mqtt_backoff_time)
                    if mqtt_retry_count >= MAX_CONNECTION_RETRIES:
                        print("Max MQTT retries reached. Restarting device.")
                        set_status_led(STATUS_COLOR_RED) # Indicate major error
                        time.sleep(1)
                        reset()
                else:
                    print("MQTT reconnected.")
                    mqtt_retry_count = 0
                    mqtt_backoff_time = LOOP_DELAY

            # --- Sensor Checks (only if connected and time potentially synced) ---
            if wlan.isconnected() and mqtt_client is not None:
                set_status_led(STATUS_COLOR_GREEN) # Indicate working properly
                # Attempt NTP time sync periodically if initial sync failed
                current_epoch = time.time()
                if not time_synced and (current_epoch % NTP_SYNC_INTERVAL < LOOP_DELAY):
                    print(f"Attempting scheduled NTP sync...")
                    time_synced = sync_time()
                    if time_synced:
                        print("Time successfully synchronized with NTP.")
                        # Re-publish initial daylight state with correct timestamp if light sensor enabled
                        if light_sensor:
                            initial_light_level = read_light_sensor()
                            if initial_light_level is not None:
                                was_previously_night = initial_light_level < LIGHT_THRESHOLD
                                initial_event = "NIGHT" if was_previously_night else "DAY"
                                try:
                                    initial_utc_epoch = time.time()
                                    publish_daylight_event(initial_event, initial_utc_epoch)
                                    print(f"Published initial daylight state: {initial_event}")
                                except Exception as e:
                                    debug_print(f"Failed to publish initial daylight state after NTP sync: {type(e).__name__} - {str(e)}")

                check_aht2x_sensor()
                # Only check light sensor transitions if time has been synced at least once
                if time_synced:
                     check_light_sensor()
                else:
                     debug_print("Skipping light sensor check because time is not synchronized.")
                # Check generic sensors
                for name in sensors:
                     check_sensor(name)
                check_buttons()

                # Check for incoming MQTT messages
                mqtt_client.check_msg()

            else:
                 debug_print("Network or MQTT disconnected, skipping sensor checks and MQTT message check.")


            # --- Loop Delay ---
            time.sleep(LOOP_DELAY)

        except KeyboardInterrupt:
            print("Ctrl+C detected. Exiting main loop.")
            break
        except Exception as e:
            debug_print(f"An unexpected error occurred in the main loop: {type(e).__name__} - {str(e)}. Attempting to continue...")
            set_status_led(STATUS_COLOR_RED) # Indicate major error
            # Consider adding a short delay or specific error handling here
        time.sleep(5) # Wait a bit before potentially retrying

if __name__ == '__main__':
    try:
        main()
    finally:
        # Cleanup actions if needed (e.g., turn off relays)
        if mqtt_client:
             try:
                 # Publish OFFLINE status cleanly if possible
                 main_state_topic = get_main_state_topic()
                 mqtt_client.publish(main_state_topic.encode(), b'OFFLINE', retain=True, qos=0)
                 mqtt_client.disconnect()
                 print("MQTT client disconnected cleanly.")
             except Exception as e:
                 debug_print(f"Error during MQTT cleanup: {type(e).__name__} - {str(e)}")
        print("Program terminated.")


