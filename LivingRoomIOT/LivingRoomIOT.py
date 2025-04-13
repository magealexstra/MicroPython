###############################################################################
# Living Room IoT Controller - ESP32 MicroPython
#
# Description: Manages various sensors and relays in a living room environment.
#              Connects to WiFi, communicates via MQTT, and handles local inputs.
#
# Features:
# - WiFi connectivity
# - MQTT communication for sensor data and relay control
# - SHT31 Temperature & Humidity sensor reading with EMA smoothing
# - Door open/close detection using a magnetic reed switch
# - Analog Light Sensor reading and optional relay control
# - Control of multiple relays via MQTT commands and physical buttons/switches
# - Support for both momentary buttons and static toggle switches
# - Explicit Button-to-Relay mapping
# - Unique device ID based on MAC address for MQTT topics (optional)
# - Debouncing for button and door sensor inputs
# - Automatic reconnection for WiFi and MQTT
# - Exponential backoff for connection retries
# - Optional debug logging
###############################################################################

import network     # Handles WiFi connectivity
import time        # Time functions (delays, timers)
import ubinascii   # Converts binary data to hex and vice versa
import sys         # System functions
import ntptime     # Network Time Protocol synchronization
import json        # For creating JSON payloads
from machine import Pin, I2C, ADC, reset  # Hardware control: Pins, I2C, ADC, Reset
from umqtt.simple import MQTTClient  # MQTT protocol for IoT messaging

# Import secrets (Wi-Fi credentials, MQTT broker info)
try:
    from secrets import WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT
except ImportError:
    print("Failed to import secrets. Please ensure 'secrets.py' is in the project root directory.")
    raise

#------------------------------------------------------------------------------
# CONFIGURATION SETTINGS - ADJUST THESE FOR YOUR SETUP
#------------------------------------------------------------------------------

# Debug settings
DEBUG = False  # [CONFIG] Set to True to see detailed logs (helpful for troubleshooting)

# Time Synchronization Settings
NTP_SYNC_INTERVAL = 3600  # [CONFIG] Seconds between NTP sync attempts if initial sync fails
FALLBACK_TO_LOCAL_TIME = True  # [CONFIG] Use local timekeeping if NTP unavailable

# MQTT Configuration
MQTT_TOPIC_PREFIX = 'livingroom'        # [CONFIG] Base prefix for all MQTT topics for this device type
MQTT_USE_DEVICE_ID = True               # [CONFIG] Set True to include unique device ID (last 5 MAC digits) in topics
                                        #          e.g., livingroom/abc12/sensor vs livingroom/sensor
MQTT_TOPIC_TEMPERATURE = 'temperature'  # [CONFIG] Sub-topic for publishing temperature data (appended to prefix/device_id)
MQTT_TOPIC_HUMIDITY = 'humidity'        # [CONFIG] Sub-topic for publishing humidity data (appended to prefix/device_id)
# MQTT_TOPIC_DOOR = 'door'                # [CONFIG] Sub-topic for publishing door open/close state (appended to prefix/device_id) - Replaced by SENSOR_CONFIG
MQTT_TOPIC_LIGHT_SENSOR = 'livingroomlight' # [CONFIG] Sub-topic for publishing light sensor readings (appended to prefix/device_id) - NOTE: This is no longer published by default
MQTT_TOPIC_DAYLIGHT_EVENT = 'daylight_event' # [CONFIG] Sub-topic for publishing DAY/NIGHT transition events
# Relay topics are generated dynamically using prefix, device_id (optional), pin number, and command/state
# Generic Open/Close Sensor topics are defined in SENSOR_CONFIG below

# I2C Configuration for SHT31 Temperature/Humidity Sensor
I2C_SCL_PIN = 22    # [CONFIG] GPIO pin for I2C Clock (SCL)
I2C_SDA_PIN = 21    # [CONFIG] GPIO pin for I2C Data (SDA)
I2C_FREQ = 100000 # [CONFIG] I2C communication frequency in Hz
SENSOR_ADDR = 0x44  # [CONFIG] I2C address of the SHT31 sensor

# Sensor Reading & Smoothing Configuration (SHT31)
EMA_ALPHA = 0.1           # [CONFIG] Exponential Moving Average smoothing factor (0.0 to 1.0). Lower values = smoother but slower response.
TEMP_THRESHOLD = 0.3      # [CONFIG] Minimum temperature change (degrees C) to trigger MQTT publish
HUMIDITY_THRESHOLD = 1.0  # [CONFIG] Minimum humidity change (%) to trigger MQTT publish

# Light Sensor Configuration (Analog)
LIGHT_SENSOR_PIN = None     # [CONFIG] GPIO pin for analog light sensor (e.g., 36 for ADC1_CH0). None = disabled.
LIGHT_THRESHOLD = 500       # [CONFIG] Light level cutoff (lower = darker when lights turn ON). Depends on sensor/circuit.
LIGHT_CHECK_INTERVAL = 10   # [CONFIG] Seconds between light sensor checks for automatic control.
LIGHT_CONTROLLED_RELAYS = []# [CONFIG] List of relay pins (e.g., [5, 18]) to be controlled by the light sensor. Empty = no auto control.

# GPIO Pin Configuration
RELAY_PINS = [5, 18, 19, 23]  # [CONFIG] List of GPIO pins connected to relays. MUST match relays controlled by buttons/light sensor.
BUTTON_PINS = [4, 15, 16, 17] # [CONFIG] List of GPIO pins connected to physical buttons/switches. MUST match keys in BUTTON_RELAY_MAP and BUTTON_TYPES.
# DOOR_SENSOR_PIN = 2         # [CONFIG] GPIO pin connected to the door reed switch. - Replaced by SENSOR_CONFIG

# Button/Switch Configuration
# [CONFIG] Defines which button/switch controls which relay. Format: {button_pin: relay_pin}
BUTTON_RELAY_MAP = {
    4: 5,    # Button on pin 4 controls relay on pin 5
    15: 18,  # Button on pin 15 controls relay on pin 18
    16: 19,  # Button on pin 16 controls relay on pin 19
    17: 23,  # Button on pin 17 controls relay on pin 23
    # Add or modify mappings as needed
}
# [CONFIG] Defines the type of input connected to each button pin. "momentary" or "static".
BUTTON_TYPES = {
    4: "momentary",  # Pin 4 is a push button (toggles on press)
    15: "momentary", # Pin 15 is a push button
    16: "static",    # Pin 16 is a toggle switch (follows position)
    17: "momentary", # Pin 17 is a push button
    # Add or modify types as needed, ensure keys match BUTTON_PINS and BUTTON_RELAY_MAP
}

# Debounce Settings (Milliseconds) - Prevents false triggers from noisy signals
BUTTON_DEBOUNCE_MS = 300  # [CONFIG] Minimum time between valid button/switch changes
# DOOR_DEBOUNCE_MS = 300    # [CONFIG] Minimum time between valid door state changes - Replaced by SENSOR_CONFIG

# Generic Open/Close Sensor Configuration
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
    # Add more sensors here if needed, e.g., "garage_door": {"pin": 15, "topic": "garage", "debounce_ms": 500}
}

# Timing and Connection Settings
LOOP_DELAY = 1  # [CONFIG] Main loop delay in seconds (affects responsiveness and sensor read frequency)
MAX_CONNECTION_RETRIES = 10  # [CONFIG] Max attempts to reconnect WiFi/MQTT before restarting device

#------------------------------------------------------------------------------
# GLOBAL VARIABLES - INTERNAL STATE TRACKING (DO NOT MODIFY)
#------------------------------------------------------------------------------
device_id = None            # Unique device identifier (last 5 MAC digits), set during init if MQTT_USE_DEVICE_ID is True.
mqtt_client = None          # Holds the MQTT client object after successful connection.
relays = {}                 # Dictionary to store relay Pin objects, keyed by GPIO pin number. e.g., {5: Pin(5, Pin.OUT)}
buttons = {}                # Dictionary to store button Pin objects, keyed by GPIO pin number. e.g., {4: Pin(4, Pin.IN, Pin.PULL_UP)}
last_button_states = {}     # Dictionary to track the last known state (0 or 1) of each button for debouncing. e.g., {4: 1}
last_button_times = {}      # Dictionary to track the timestamp (milliseconds) of the last valid button state change for debouncing. e.g., {4: 12345678}
last_light_check = 0        # Tracks timestamp (milliseconds) of the last light sensor check for automatic control interval.

# Generic Sensor State (Internal)
sensors = {}                # Stores Pin objects, keyed by sensor name ("door", "window", etc.)
last_sensor_states = {}     # Stores last state (0 or 1), keyed by sensor name
last_sensor_times = {}      # Stores last timestamp (ms), keyed by sensor name

# Variables for EMA (Exponential Moving Average) smoothing and threshold-based publishing (SHT31)
ema_temp = None             # Stores the calculated smoothed temperature value.
ema_humidity = None         # Stores the calculated smoothed humidity value.
last_sent_temp = None       # Stores the last temperature value published via MQTT to check against the threshold.
last_sent_humidity = None   # Stores the last humidity value published via MQTT to check against the threshold.
was_previously_night = None # Tracks the light state (True=Night, False=Day) from the previous check for event detection.

#------------------------------------------------------------------------------
# UTILITY FUNCTIONS
#------------------------------------------------------------------------------

def debug_print(*args, **kwargs):
    """Prints debug messages only if DEBUG is enabled.
    
    Optimized for memory usage by joining strings before printing.
    """
    if DEBUG:
        # Join messages with a space to reduce memory impact
        message = " ".join(str(arg) for arg in args)
        # Limit message length to reduce memory pressure
        if len(message) > 100:
            message = message[:97] + "..."
        print(message)

def get_device_id_str():
    """Gets the unique identifier for this device from its MAC address.

    Used for unique MQTT topics if MQTT_USE_DEVICE_ID is True.

    Returns:
        str: Last 5 characters of the device's MAC address.
    """
    wlan = network.WLAN(network.STA_IF)
    mac = wlan.config('mac')  # Get MAC address as raw bytes
    mac_hex = ubinascii.hexlify(mac).decode()  # Convert to hexadecimal string
    return mac_hex[-5:]  # Extract the last 5 characters

def sync_time():
    """Synchronizes the device's internal clock using NTP. Requires Wi-Fi."""
    print("Attempting to sync time with NTP...")
    # Add retry logic here if desired, especially if NTP fails occasionally
    try:
        # Set the NTP server if needed (optional, defaults usually work)
        # ntptime.host = 'pool.ntp.org'
        ntptime.settime() # Blocks until time is set or error occurs
        # time.localtime() uses the RTC which is now set to UTC
        print("Time synchronized successfully (UTC):", time.localtime()) # Keep this print as it's useful info
        return True
    except Exception as e:
        # Non-critical: Log details only if DEBUG is True
        if DEBUG:
            sys.print_exception(e)
        debug_print("Failed to sync time with NTP:", e)
        # Consider how to handle time sync failure - maybe retry later?
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
    return (MQTT_TOPIC_PREFIX + '_' + device_id).encode()

def connect_wifi():
    """Connects to the WiFi network using credentials from secrets.py.

    Retries connection until successful. Required for MQTT communication.

    Returns:
        network.WLAN: The connected WLAN interface object.
    """
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('Connecting to Wi-Fi...')
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        # Wait until connection is established
        while not wlan.isconnected():
            time.sleep(0.5)
        print('Connected to Wi-Fi:', wlan.ifconfig())
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

# def get_door_topic(): # Replaced by get_sensor_topic(sensor_name)
#     """Gets the topic string for publishing door sensor state."""
#     return f'{get_base_topic()}/{MQTT_TOPIC_DOOR}'

def get_sensor_topic(sensor_name):
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
        sys.print_exception(e)
        debug_print('Error in MQTT callback:', e)

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
        client = MQTTClient(client_id, MQTT_BROKER, port=MQTT_PORT)

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
        # Publish initial door state - Removed erroneous call, handled by publish_sensor_state loop later
        # publish_door_state()
        # Publish initial light sensor reading (if enabled)
        if light_sensor:
             publish_light_sensor_reading()


        return client

    except Exception as e:
        sys.print_exception(e)
        debug_print('Failed to connect to MQTT broker:', e)
        return None

def publish_relay_state(relay_pin):
    """Publishes the current state (ON/OFF) of a specific relay to its MQTT state topic."""
    global mqtt_client
    if mqtt_client is None or relay_pin not in relays:
        debug_print(f"MQTT client not available or relay {relay_pin} not initialized, skipping state publish.")
        return
    try:
        state = 'ON' if relays[relay_pin].value() else 'OFF'
        topic = get_relay_state_topic(relay_pin)
        mqtt_client.publish(topic.encode(), state.encode(), retain=True, qos=0)
        debug_print(f"Published relay {relay_pin} state '{state}' to topic '{topic}'")
    except Exception as e:
        sys.print_exception(e)
        debug_print(f"Failed to publish relay state for pin {relay_pin}: {e}")
        mqtt_client = None # Signal to reconnect

# def publish_door_state(): # Replaced by publish_sensor_state(sensor_name)
#     """Publishes the current state (OPEN/CLOSED) of the door sensor."""
#     ... (old code removed) ...

def publish_sensor_state(sensor_name):
    """Publishes the current state (OPEN/CLOSED) of a generic sensor."""
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
        topic = get_sensor_topic(sensor_name)
        if topic:
            mqtt_client.publish(topic.encode(), state_str.encode(), retain=True, qos=0)
            debug_print(f"Published {sensor_name} state '{state_str}' to topic '{topic}'")
    except Exception as e:
        sys.print_exception(e)
        debug_print(f"Failed to publish {sensor_name} sensor state: {e}")
        mqtt_client = None # Signal to reconnect

# Removed publish_sht31_data function

def publish_light_sensor_reading():
    """Reads and publishes the current analog light sensor reading."""
    global mqtt_client
    if mqtt_client is None or light_sensor is None:
        debug_print("MQTT client or light sensor not available, skipping light reading publish.")
        return
    try:
        light_level = read_light_sensor()
        if light_level is not None:
            topic = get_light_sensor_topic()
            mqtt_client.publish(topic.encode(), str(light_level).encode(), retain=False, qos=0)
            debug_print(f"Published light level '{light_level}' to topic '{topic}'")
    except Exception as e:
        sys.print_exception(e)
        debug_print(f"Failed to publish light sensor reading: {e}")
        mqtt_client = None # Signal to reconnect

def publish_daylight_event(event_type, utc_timestamp):
    """Publishes a DAY or NIGHT event with the UTC timestamp."""
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
    except Exception as e:
        sys.print_exception(e)
        debug_print(f"Failed to publish daylight event: {e}")
        mqtt_client = None # Signal to reconnect

#------------------------------------------------------------------------------
# HARDWARE CONTROL FUNCTIONS
#------------------------------------------------------------------------------

def set_relay(pin_num, state):
    """Sets the state of a specific relay and publishes the change via MQTT.

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
            sys.print_exception(e)
            debug_print(f"Failed to set relay state for pin {pin_num}: {e}")
    else:
        debug_print(f"Relay {pin_num} already in state {state}")


#------------------------------------------------------------------------------
# INITIALIZATION FUNCTIONS
#------------------------------------------------------------------------------

def init_relays():
    """Initializes all configured relay pins."""
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
             sys.print_exception(e)
             print(f"Failed to initialize relay on GPIO {pin_num}: {e}")
    print(f"Initialized {count}/{len(RELAY_PINS)} relays.")

def init_buttons():
    """Initializes configured button/switch pins based on BUTTON_RELAY_MAP."""
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
            sys.print_exception(e)
            print(f"Failed to initialize button on GPIO {button_pin}: {e}")

    # Verify all pins in BUTTON_PINS were considered (optional check)
    for pin_num in BUTTON_PINS:
        if pin_num not in BUTTON_RELAY_MAP:
             print(f"Warning: Button pin {pin_num} in BUTTON_PINS list but not in BUTTON_RELAY_MAP. It will not be functional.")

    print(f"Initialized {count} buttons/switches.")

# def init_door_sensor(): # Replaced by init_sensor(sensor_name, config)
#     """Initializes the door sensor pin."""
#     ... (old code removed) ...

def init_sensor(sensor_name, config):
    """Initializes a generic open/close sensor based on configuration."""
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
        sys.print_exception(e)
        print(f"Failed to initialize sensor '{sensor_name}' on GPIO {pin_num}: {e}")
        # Ensure sensor entry is removed or marked as invalid if init fails partially
        if sensor_name in sensors: del sensors[sensor_name]
        if sensor_name in last_sensor_states: del last_sensor_states[sensor_name]
        if sensor_name in last_sensor_times: del last_sensor_times[sensor_name]
        return False # Indicate failure

def init_sht31_sensor():
    """Initializes the I2C communication and checks for the SHT31 sensor."""
    print("Initializing I2C SHT31 sensor...")
    try:
        # Initialize I2C bus (using ID 0 for ESP32)
        i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
        # Scan for connected I2C devices
        devices = i2c.scan()
        debug_print('I2C devices found:', [hex(device) for device in devices])
        # Check if the expected sensor address is present
        if SENSOR_ADDR not in devices:
            print(f"SHT31 sensor not found at address {hex(SENSOR_ADDR)}. Check wiring and address.")
            return None, None
        print(f"SHT31 sensor found at address {hex(SENSOR_ADDR)}.")
        return i2c, SENSOR_ADDR
    except Exception as e:
        sys.print_exception(e)
        print("Failed to initialize I2C SHT31 sensor:", e)
        return None, None

def init_light_sensor():
    """Initializes the analog light sensor if configured."""
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

        # Configure ADC attenuation for full voltage range (0-3.3V on ESP32)
        # ATTN_11DB gives full range
        try:
            adc.atten(ADC.ATTN_11DB)
            debug_print("Set ADC attenuation to 11dB (0-3.3V range)")
        except AttributeError:
            debug_print("Warning: Could not set ADC attenuation (might not be supported on this board/firmware).")
            pass # Continue anyway, might work with default range

        light_sensor = adc
        print(f"Analog light sensor initialized on GPIO {LIGHT_SENSOR_PIN}.")
        if LIGHT_CONTROLLED_RELAYS:
             print(f"Light sensor will control relays: {LIGHT_CONTROLLED_RELAYS}")
        else:
             print("Light sensor automatic control disabled (LIGHT_CONTROLLED_RELAYS is empty).")

    except Exception as e:
        sys.print_exception(e)
        print(f"Failed to initialize analog light sensor on GPIO {LIGHT_SENSOR_PIN}: {e}")
        light_sensor = None

#------------------------------------------------------------------------------
# SENSOR READING FUNCTIONS
#------------------------------------------------------------------------------

def read_sht31_sensor(i2c, addr):
    """Reads temperature and humidity from the SHT31 sensor."""
    if i2c is None:
        return None, None
    try:
        # Send measurement command (0x2400 = Clock Stretching, High Repeatability)
        i2c.writeto(addr, b'\x24\x00')
        # Wait for measurement to complete (datasheet suggests max 15ms for high repeatability)
        time.sleep(0.02) # Use slightly longer delay
        # Read 6 bytes of data: Temp MSB, Temp LSB, Temp CRC, Hum MSB, Hum LSB, Hum CRC
        data = i2c.readfrom(addr, 6)

        # Combine MSB and LSB for temperature and humidity
        temp_raw = (data[0] << 8) | data[1]
        humidity_raw = (data[3] << 8) | data[4]

        # Calculate temperature and humidity using datasheet formulas
        temperature = -45.0 + (175.0 * temp_raw / 65535.0)
        humidity = 100.0 * (humidity_raw / 65535.0)

        debug_print(f"Raw Temp: {temp_raw}, Calc Temp: {temperature:.2f} C")
        debug_print(f"Raw Hum: {humidity_raw}, Calc Hum: {humidity:.2f} %")

        # Basic check for invalid readings (e.g., all FF or 00)
        if temp_raw == 0xFFFF or humidity_raw == 0xFFFF or (temp_raw == 0 and humidity_raw == 0):
             debug_print("Warning: SHT31 read potentially invalid data (0xFF or 0x00).")
             # Return None if data looks invalid
             return None, None

        return temperature, humidity
    except Exception as e:
        sys.print_exception(e)
        debug_print("Failed to read SHT31 sensor data:", e)
        return None, None

def read_light_sensor():
    """Reads the current value from the analog light sensor.

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
        sys.print_exception(e)
        debug_print(f"Error reading analog light sensor: {e}")
        return None

#------------------------------------------------------------------------------
# MAIN LOOP CHECK FUNCTIONS
#------------------------------------------------------------------------------

def check_buttons():
    """Checks button/switch states, debounces, and controls relays via BUTTON_RELAY_MAP."""
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

# def check_door_sensor(): # Replaced by check_sensor(sensor_name)
#     """Checks the state of the door sensor, debounces, and publishes changes via MQTT."""
#     ... (old code removed) ...

def check_sensor(sensor_name):
    """Checks the state of a generic sensor, debounces, and publishes changes."""
    # Uses global dictionaries: sensors, last_sensor_states, last_sensor_times
    # Uses SENSOR_CONFIG for debounce value
    global last_sensor_states, last_sensor_times

    if sensor_name not in sensors:
        return # Skip if this sensor wasn't initialized

    sensor_pin = sensors[sensor_name]
    last_state = last_sensor_states.get(sensor_name)
    last_time = last_sensor_times.get(sensor_name, 0)
    debounce_ms = SENSOR_CONFIG.get(sensor_name, {}).get("debounce_ms", 300) # Default debounce

    current_time = time.ticks_ms()
    current_state = None # Initialize to None
    try:
        current_state = sensor_pin.value()
    except Exception as e:
        # Non-critical: Log details only if DEBUG is True
        if DEBUG:
            sys.print_exception(e)
        debug_print(f"Error reading sensor '{sensor_name}' pin value: {e}")
        # Optionally, handle this error further, e.g., by trying to re-init the pin
        # For now, we'll just skip processing this sensor for this cycle
        return # Exit the check for this sensor this cycle

    # Proceed only if reading was successful and state is different
    if current_state is not None and current_state != last_state:
        # Check debounce timer
        if time.ticks_diff(current_time, last_time) > debounce_ms:
            state_str = 'OPEN' if current_state else 'CLOSED'
            debug_print(f"Sensor '{sensor_name}' state changed to: {state_str}")
            last_sensor_states[sensor_name] = current_state # Update state *before* publishing
            last_sensor_times[sensor_name] = current_time   # Update time
            publish_sensor_state(sensor_name) # Publish the new debounced state
        # else:
            # debug_print(f"Sensor '{sensor_name}' change ignored due to debounce.")

def check_sht31_sensor(i2c, addr):
    """Reads SHT31, applies EMA smoothing, and publishes temp/humidity independently if thresholds are met."""
    global mqtt_client, ema_temp, ema_humidity, last_sent_temp, last_sent_humidity # Need to modify globals
    if i2c is None or mqtt_client is None:
        debug_print("Skipping SHT31 check: I2C or MQTT not ready.")
        return # Skip if I2C not initialized or MQTT not connected

    temperature, humidity = read_sht31_sensor(i2c, addr)
    if temperature is not None and humidity is not None:
        # Apply EMA smoothing
        prev_ema_temp = ema_temp
        prev_ema_humidity = ema_humidity

        if ema_temp is None: ema_temp = temperature
        else: ema_temp = EMA_ALPHA * temperature + (1 - EMA_ALPHA) * ema_temp

        if ema_humidity is None: ema_humidity = humidity
        else: ema_humidity = EMA_ALPHA * humidity + (1 - EMA_ALPHA) * ema_humidity

        # --- Temperature Check & Publish (Option A) ---
        temp_change = abs((last_sent_temp if last_sent_temp is not None else ema_temp) - ema_temp)
        if (last_sent_temp is None) or (temp_change >= TEMP_THRESHOLD):
            debug_print(f"SHT31 Temperature threshold met (T_diff={temp_change:.2f}). Publishing.")
            try:
                temp_topic = get_temperature_topic()
                mqtt_client.publish(temp_topic.encode(), f"{ema_temp:.2f}".encode(), retain=False, qos=0)
                debug_print(f"Published Temperature '{ema_temp:.2f}' to topic '{temp_topic}'")
                last_sent_temp = ema_temp # Update last sent value only on success
            except Exception as e:
                sys.print_exception(e)
                debug_print(f"Failed to publish Temperature data: {e}")
                mqtt_client = None # Signal to reconnect
        else:
             debug_print(f"SHT31 Temperature threshold not met (T_diff={temp_change:.2f}). Skipping publish.")

        # --- Humidity Check & Publish (Option A) ---
        # Ensure MQTT client is still valid after potential temp publish error
        if mqtt_client is not None:
            humidity_change = abs((last_sent_humidity if last_sent_humidity is not None else ema_humidity) - ema_humidity)
            if (last_sent_humidity is None) or (humidity_change >= HUMIDITY_THRESHOLD):
                debug_print(f"SHT31 Humidity threshold met (H_diff={humidity_change:.2f}). Publishing.")
                try:
                    hum_topic = get_humidity_topic()
                    mqtt_client.publish(hum_topic.encode(), f"{ema_humidity:.2f}".encode(), retain=False, qos=0)
                    debug_print(f"Published Humidity '{ema_humidity:.2f}' to topic '{hum_topic}'")
                    last_sent_humidity = ema_humidity # Update last sent value only on success
                except Exception as e:
                    sys.print_exception(e)
                    debug_print(f"Failed to publish Humidity data: {e}")
                    mqtt_client = None # Signal to reconnect
            else:
                 debug_print(f"SHT31 Humidity threshold not met (H_diff={humidity_change:.2f}). Skipping publish.")

    else:
        debug_print('Invalid SHT31 sensor data received, skipping processing.')

def check_light_sensor():
    """Reads analog light sensor, detects DAY/NIGHT transitions, publishes events, and optionally controls relays."""
    global last_light_check, was_previously_night # Add was_previously_night
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
                    sys.print_exception(e)
                    debug_print("Failed to get UTC time or publish daylight event:", e)

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
    """Main program execution function."""
    global mqtt_client, device_id # Need device_id for connect_mqtt

    print("Starting Living Room IoT Controller...")

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
    # init_door_sensor() # Replaced by loop above
    i2c, sht31_addr = init_sht31_sensor() # Specific name
    init_light_sensor()

    # Connect to network services
    wlan = connect_wifi()

    # Synchronize time via NTP (requires Wi-Fi)
    time_synced = sync_time()
    # We might want to handle the case where time sync fails, but for now proceed.

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
                 sys.print_exception(e)
                 print("Failed to get time or publish initial daylight state:", e)
        else:
             print("Could not read initial light level for daylight state.")
    elif not time_synced:
         print("Skipping initial daylight state publish because time is not synchronized.")


    print("Initialization complete. Entering main loop...")

    # Connection retry counters and separate backoff times
    wifi_retry_count = 0
    mqtt_retry_count = 0
    wifi_backoff_time = LOOP_DELAY
    mqtt_backoff_time = LOOP_DELAY

    while True:
        try:
            # --- Connection Management ---
            # Reconnect Wi-Fi if disconnected
            if not wlan.isconnected():
                wifi_retry_count += 1
                print(f"Wi-Fi disconnected. Attempting reconnect {wifi_retry_count}/{MAX_CONNECTION_RETRIES}...")
                wlan = connect_wifi() # Attempt reconnection

                if not wlan.isconnected():
                    wifi_backoff_time = min(60, LOOP_DELAY * (2 ** wifi_retry_count)) # Increase max backoff
                    print(f"Wi-Fi reconnection failed. Waiting {wifi_backoff_time}s...")
                    time.sleep(wifi_backoff_time)
                    if wifi_retry_count >= MAX_CONNECTION_RETRIES:
                        print("Max Wi-Fi retries reached. Restarting device.")
                        time.sleep(1)
                        reset()
                else:
                    print("Wi-Fi reconnected.")
                    wifi_retry_count = 0
                    wifi_backoff_time = LOOP_DELAY
                    # Reconnect MQTT immediately after Wi-Fi comes back up
                    if mqtt_client is None:
                         print("Attempting MQTT reconnect after Wi-Fi restored...")
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
                        time.sleep(1)
                        reset()
                else:
                    print("MQTT reconnected.")
                    mqtt_retry_count = 0
                    mqtt_backoff_time = LOOP_DELAY

            # --- Sensor Checks (only if connected and time potentially synced) ---
            if wlan.isconnected() and mqtt_client is not None:
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
                                global was_previously_night
                                was_previously_night = initial_light_level < LIGHT_THRESHOLD
                                initial_event = "NIGHT" if was_previously_night else "DAY"
                                try:
                                    initial_utc_epoch = time.time()
                                    publish_daylight_event(initial_event, initial_utc_epoch)
                                    print(f"Published initial daylight state: {initial_event}")
                                except Exception as e:
                                    sys.print_exception(e)
                                    print("Failed to publish initial daylight state after NTP sync:", e)

                check_sht31_sensor(i2c, sht31_addr)
                # Only check light sensor transitions if time has been synced at least once
                if time_synced:
                     check_light_sensor()
                else:
                     debug_print("Skipping light sensor check because time is not synchronized.")
                # Check generic sensors
                for name in sensors:
                     check_sensor(name)
                # check_door_sensor() # Replaced by loop above
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
             sys.print_exception(e)
             print("An unexpected error occurred in the main loop. Attempting to continue...")
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
                 print("Error during MQTT cleanup:", e)
        print("Program terminated.")
