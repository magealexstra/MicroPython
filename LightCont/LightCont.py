###############################################################################
# Light Controller - ESP32 MicroPython
#
# Description: Controls lights/relays via buttons, MQTT commands, and optional
#              light sensors. Connects to WiFi and an MQTT broker to allow
#              remote control and status reporting. Supports momentary buttons
#              and static switches.
#
# Features:
# - WiFi connectivity using credentials from secrets.py
# - MQTT control with unique device identifiers (based on MAC address)
# - Support for both momentary push buttons and static toggle switches
# - Optional automatic light control based on analog light sensor readings
# - Configurable GPIO pins for relays, buttons, and sensor
# - Debouncing for button inputs
# - MQTT Last Will and Testament for offline status reporting
# - Connection monitoring and automatic reconnection for WiFi and MQTT
#
# Author: MageAlexstra
# Date: 4/12/2025 (Update date)
###############################################################################

#------------------------------------------------------------------------------
# IMPORTS
#------------------------------------------------------------------------------
import network     # Handles WiFi network interfaces (STA_IF, AP_IF)
import time        # Provides time-related functions (sleep, ticks_ms, ticks_diff)
import sys         # System-specific parameters and functions (print_exception)
import ubinascii   # Conversions between binary and ASCII (hexlify for MAC)
from machine import Pin, ADC  # Hardware pin control and Analog-to-Digital Converter
import machine     # Access to specific hardware functions (reset)
from umqtt.simple import MQTTClient  # Basic MQTT client implementation

# Import secrets (Wi-Fi credentials, MQTT broker info)
try:
    # [CONFIG] Requires secrets.py in the root directory with:
    # WIFI_SSID = "YourWiFiNetworkName"
    # WIFI_PASSWORD = "YourWiFiPassword"
    # MQTT_BROKER = "your.mqtt.broker.address"
    # MQTT_PORT = 1883  # Or your MQTT port
    from secrets import WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT
except ImportError:
    print("ERROR: Failed to import secrets.py. Please ensure 'secrets.py' exists in the project root directory with WiFi and MQTT credentials.")
    # Stop execution if secrets are missing, as they are essential
    raise

#------------------------------------------------------------------------------
# CONFIGURATION SETTINGS - ADJUST THESE FOR YOUR SETUP
#------------------------------------------------------------------------------

# --- General Settings ---
DEBUG = False  # [CONFIG] Set to True to enable detailed print statements for troubleshooting

# --- MQTT Configuration ---
MQTT_TOPIC_PREFIX = 'esp32/lightcontrol'  # [CONFIG] Base prefix for all MQTT topics published/subscribed by this device
MQTT_USE_DEVICE_ID = True  # [CONFIG] Set True to include unique device ID (last 5 MAC digits) in topics (e.g., prefix/devid/5/state). Set False for legacy non-unique topics (e.g., prefix/5/state).

# --- Timing Settings ---
LOOP_DELAY = 1  # [CONFIG] Main loop delay in seconds. Affects responsiveness vs power consumption.
MAX_CONNECTION_RETRIES = 10  # [CONFIG] Max consecutive failed connection attempts (WiFi or MQTT) before restarting the device.
DEBOUNCE_MS = 300      # [CONFIG] Minimum time in milliseconds between recognizing button presses/changes to prevent noise/bouncing.

# --- Relay Configuration ---
RELAY_PINS = [5,]  # [CONFIG] List of GPIO pins connected to relays. Add more using commas: [5, 6, 7]
                   # These pins control your lights/devices. They are configured as outputs.

# --- Button Configuration ---
BUTTON_PINS = [4,]  # [CONFIG] List of GPIO pins connected to physical buttons or switches. Add more using commas: [4, 15]
                   # These pins are configured as inputs with pull-up resistors.

# [CONFIG] Map which button/switch controls which relay.
# Format: {button_gpio_pin: relay_gpio_pin}
BUTTON_RELAY_MAP = {
    4: 5,  # Example: Button on pin 4 controls relay on pin 5
    # 15: 6, # Example: Button/Switch on pin 15 controls relay on pin 6
}

# [CONFIG] Define the type for each button/switch pin.
# "momentary": A push button that toggles the relay state on each press (usually press = 1->0 transition).
# "static": A toggle switch where the relay state follows the switch position (0 = ON, 1 = OFF with PULL_UP).
BUTTON_TYPES = {
    4: "momentary",  # Example: Button on pin 4 is a momentary push button
    # 15: "static",  # Example: Switch on pin 15 is a static toggle switch
}

# --- Light Sensor Configuration ---
LIGHT_SENSOR_PIN = None  # [CONFIG] GPIO pin connected to the analog light sensor (e.g., 36 for ESP32 ADC1_CH0). Set to None to disable automatic light control.
LIGHT_THRESHOLD = 500    # [CONFIG] ADC reading threshold. If sensor reads below this value, lights turn ON. If above or equal, lights turn OFF. Adjust based on your sensor and desired darkness level. Lower value = darker trigger point.
LIGHT_CHECK_INTERVAL = 10  # [CONFIG] How often (in seconds) to check the light sensor value.
LIGHT_CONTROLLED_RELAYS = []  # [CONFIG] List of relay pins (from RELAY_PINS) that should be automatically controlled by the light sensor (e.g., [5]). Empty list disables auto-control for all relays.

#------------------------------------------------------------------------------
# GLOBAL VARIABLES - INTERNAL STATE TRACKING (DO NOT MODIFY DIRECTLY)
#------------------------------------------------------------------------------
mqtt_client = None          # Holds the MQTTClient object instance after connection
device_id = None            # Unique device identifier derived from MAC address (last 5 hex digits)
relays = {}                 # Dictionary to store initialized relay Pin objects {pin_num: Pin(...)}
buttons = {}                # Dictionary to store initialized button Pin objects {pin_num: Pin(...)}
last_button_states = {}     # Tracks the last known state (0 or 1) for each button pin for debouncing/edge detection {button_pin: state}
last_button_times = {}      # Tracks the time (in ms) of the last detected state change for each button pin for debouncing {button_pin: time_ms}
light_sensor = None         # Holds the ADC object for the light sensor if initialized
last_light_check = 0        # Timestamp (in ms) of the last light sensor check

# Legacy topic names (derived from prefix, used if MQTT_USE_DEVICE_ID is False)
MQTT_COMMAND_TOPIC = f'{MQTT_TOPIC_PREFIX}/command'
MQTT_STATE_TOPIC = f'{MQTT_TOPIC_PREFIX}/state'

#------------------------------------------------------------------------------
# UTILITY FUNCTIONS
#------------------------------------------------------------------------------

def debug_print(*args, **kwargs):
    """Prints messages only if the global DEBUG flag is True.

    Args:
        *args: Positional arguments to pass to the print function.
        **kwargs: Keyword arguments to pass to the print function.
    """
    if DEBUG:
        print(*args, **kwargs)

def get_device_id():
    """Retrieves the last 5 characters of the device's MAC address as a unique ID.

    Returns:
        str: The last 5 hexadecimal characters of the WiFi MAC address.
    """
    wlan = network.WLAN(network.STA_IF)
    mac = wlan.config('mac')  # Get MAC address as bytes
    mac_hex = ubinascii.hexlify(mac).decode()  # Convert bytes to hexadecimal string
    return mac_hex[-5:]  # Return the last 5 characters

def get_unique_client_id():
    """Generates a unique MQTT client ID based on the script name and device ID.

    Ensures that each device connecting to the MQTT broker has a distinct client ID.
    Sets the global `device_id` if it hasn't been set yet.

    Returns:
        bytes: A unique client ID string encoded in bytes, suitable for MQTT.
    """
    global device_id
    if device_id is None:
        device_id = get_device_id()
    # MQTT client ID must be bytes
    return ('lightcont_' + device_id).encode()

#------------------------------------------------------------------------------
# NETWORK FUNCTIONS (WiFi & MQTT)
#------------------------------------------------------------------------------

def connect_wifi():
    """Connects the device to the WiFi network specified in secrets.py.

    Activates the station interface and attempts to connect. Blocks until
    a connection is established.

    Returns:
        network.WLAN: The WLAN station interface object.

    Raises:
        Exception: Propagates exceptions if WiFi connection fails persistently
                   (handled in the main loop's retry logic).
    """
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('Connecting to Wi-Fi network:', WIFI_SSID)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        # Wait for connection
        while not wlan.isconnected():
            time.sleep(0.5)
            # Add a timeout check here if needed, though main loop handles retries
    print('Connected to Wi-Fi. IP Address:', wlan.ifconfig()[0])
    return wlan

def connect_mqtt(client_id):
    """Connects to the MQTT broker specified in secrets.py.

    Initializes the MQTT client, sets the Last Will and Testament (LWT),
    sets the message callback, connects, subscribes to topics, and publishes
    initial states.

    Args:
        client_id (bytes): The unique client ID for this device.

    Returns:
        MQTTClient: The connected MQTT client object, or None if connection fails.
    """
    try:
        client = MQTTClient(client_id, MQTT_BROKER, port=MQTT_PORT)

        # Set Last Will and Testament (LWT)
        # If the device disconnects unexpectedly, the broker will publish 'OFFLINE'
        # to the main state topic.
        lwt_topic = get_main_state_topic()
        client.set_last_will(lwt_topic.encode(), b'OFFLINE', retain=True, qos=0)

        # Register callback for incoming messages
        client.set_callback(mqtt_callback)

        # Connect to the broker
        client.connect()
        debug_print(f'Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}')

        # Publish 'ONLINE' status to the main state topic
        client.publish(lwt_topic.encode(), b'ONLINE', retain=True, qos=0)

        # Subscribe to command topics
        subscribe_all(client)

        # Publish initial state for all configured relays
        for pin_num in RELAY_PINS:
            # Assume initial state is OFF after hardware init
            initial_state = 'OFF'
            state_topic = get_state_topic(pin_num)
            client.publish(state_topic.encode(), initial_state.encode(), retain=True, qos=0)
            debug_print(f'Published initial state {initial_state} to {state_topic}')

        return client
    except Exception as e:
        sys.print_exception(e)
        debug_print(f'ERROR: Failed to connect to MQTT broker: {e}')
        return None

def subscribe_all(client):
    """Subscribes the MQTT client to all relevant command topics.

    Subscribes to the main command topic (either legacy or device-specific)
    and individual command topics for each configured relay.

    Args:
        client (MQTTClient): The MQTT client instance.
    """
    # Subscribe to the main command topic (device-specific or legacy)
    main_cmd_topic = get_main_command_topic()
    client.subscribe(main_cmd_topic.encode())
    debug_print(f'Subscribed to main command topic: {main_cmd_topic}')

    # Subscribe to legacy main topic only if not using device ID (for backward compatibility)
    if not MQTT_USE_DEVICE_ID and main_cmd_topic != MQTT_COMMAND_TOPIC:
        client.subscribe(MQTT_COMMAND_TOPIC.encode())
        debug_print(f'Subscribed to legacy main command topic: {MQTT_COMMAND_TOPIC}')

    # Subscribe to pin-specific command topics
    for pin_num in RELAY_PINS:
        topic = get_command_topic(pin_num)
        client.subscribe(topic.encode())
        debug_print(f'Subscribed to pin-specific topic: {topic}')

def publish_state(pin_num):
    """Publishes the current state ('ON' or 'OFF') of a specific relay to its MQTT state topic.

    Reads the relay's current value and sends it to the broker. Sets retain flag
    so new subscribers get the current state immediately.

    Args:
        pin_num (int): The GPIO pin number of the relay whose state is being published.
    """
    global mqtt_client # Use the global client object

    if mqtt_client is None:
        debug_print(f'MQTT client not connected. Skipping state publish for pin {pin_num}.')
        return

    if pin_num not in relays:
        debug_print(f'Attempted to publish state for uninitialized relay pin: {pin_num}')
        return

    try:
        # Read current physical state of the relay
        current_value = relays[pin_num].value()
        state = 'ON' if current_value == 1 else 'OFF'
        topic = get_state_topic(pin_num)
        mqtt_client.publish(topic.encode(), state.encode(), retain=True, qos=0)
        debug_print(f'Published state {state} to topic {topic}')
    except Exception as e:
        sys.print_exception(e)
        debug_print(f'ERROR: Failed to publish state for pin {pin_num}: {e}')
        # Assume connection is lost, force reconnect on next loop
        mqtt_client = None

def mqtt_callback(topic, msg):
    """Callback function executed when an MQTT message is received.

    Parses the topic to determine if it's a main command or a pin-specific
    command. Decodes the message payload (usually 'ON' or 'OFF') and calls
    `set_relay` accordingly.

    Args:
        topic (bytes): The topic the message was received on.
        msg (bytes): The message payload.
    """
    try:
        topic_str = topic.decode()
        command_str = msg.decode().strip().upper()
        debug_print(f'Received MQTT message - Topic: "{topic_str}", Message: "{command_str}"')

        # --- Check for Main Command Topic ---
        is_main_command = False
        if topic_str == get_main_command_topic():
            is_main_command = True
        # Check legacy topic only if not using device ID
        elif not MQTT_USE_DEVICE_ID and topic_str == MQTT_COMMAND_TOPIC:
             is_main_command = True

        if is_main_command:
            debug_print(f'Processing main command: {command_str}')
            # Apply command to all configured relays
            for pin_num in relays:
                if command_str == 'ON' or command_str == 'OFF':
                    set_relay(pin_num, command_str)
                else:
                    debug_print(f'Ignoring invalid main command: {command_str}')
            return # Finished processing main command

        # --- Check for Pin-Specific Command Topic ---
        # Expected formats:
        # - prefix/device_id/pin/command (if MQTT_USE_DEVICE_ID is True)
        # - prefix/pin/command (if MQTT_USE_DEVICE_ID is False)
        try:
            parts = topic_str.split('/')
            if len(parts) > 1 and parts[-1] == 'command':
                target_pin_num = -1
                if MQTT_USE_DEVICE_ID:
                    # Check format: prefix/device_id/pin/command
                    if len(parts) >= 4 and parts[-3] == device_id:
                        target_pin_num = int(parts[-2])
                else:
                    # Check format: prefix/pin/command
                    if len(parts) >= 3:
                         target_pin_num = int(parts[-2])

                # Check if the extracted pin number is valid and configured
                if target_pin_num in relays:
                    debug_print(f'Processing command for pin {target_pin_num}: {command_str}')
                    if command_str == 'ON' or command_str == 'OFF':
                        set_relay(target_pin_num, command_str)
                    else:
                        debug_print(f'Ignoring invalid pin command: {command_str}')
                else:
                    debug_print(f'Ignoring command for unconfigured/invalid pin number {target_pin_num} in topic: {topic_str}')

        except (ValueError, IndexError):
            # Error parsing pin number from topic
            debug_print(f'Could not parse pin number from topic: {topic_str}')

    except Exception as e:
        sys.print_exception(e)
        debug_print(f'ERROR in MQTT callback: {e}')

#------------------------------------------------------------------------------
# MQTT TOPIC GENERATION FUNCTIONS
#------------------------------------------------------------------------------

def get_main_command_topic():
    """Generates the main command topic string based on configuration.

    Returns:
        str: The MQTT topic for sending commands to all relays on this device.
    """
    if MQTT_USE_DEVICE_ID:
        return f'{MQTT_TOPIC_PREFIX}/{device_id}/command'
    else:
        return MQTT_COMMAND_TOPIC # Use legacy topic

def get_main_state_topic():
    """Generates the main state topic string for device status (ONLINE/OFFLINE).

    Returns:
        str: The MQTT topic for publishing the overall device status.
    """
    if MQTT_USE_DEVICE_ID:
        return f'{MQTT_TOPIC_PREFIX}/{device_id}/state'
    else:
        return MQTT_STATE_TOPIC # Use legacy topic

def get_command_topic(pin_num):
    """Generates the command topic string for a specific relay pin.

    Args:
        pin_num (int): The GPIO pin number of the relay.

    Returns:
        str: The MQTT topic for sending commands to the specified relay.
    """
    if MQTT_USE_DEVICE_ID:
        return f'{MQTT_TOPIC_PREFIX}/{device_id}/{pin_num}/command'
    else:
        return f'{MQTT_TOPIC_PREFIX}/{pin_num}/command' # Use legacy topic format

def get_state_topic(pin_num):
    """Generates the state topic string for a specific relay pin.

    Args:
        pin_num (int): The GPIO pin number of the relay.

    Returns:
        str: The MQTT topic for publishing the state of the specified relay.
    """
    if MQTT_USE_DEVICE_ID:
        return f'{MQTT_TOPIC_PREFIX}/{device_id}/{pin_num}/state'
    else:
        return f'{MQTT_TOPIC_PREFIX}/{pin_num}/state' # Use legacy topic format

#------------------------------------------------------------------------------
# HARDWARE CONTROL FUNCTIONS
#------------------------------------------------------------------------------

def set_relay(pin_num, state):
    """Sets the physical state of a relay and publishes the change to MQTT.

    Args:
        pin_num (int): The GPIO pin number of the relay to control.
        state (str): The desired state, either 'ON' or 'OFF'.

    Returns:
        None
    """
    if pin_num not in relays:
        debug_print(f'WARNING: Attempted to set state for unknown relay pin: {pin_num}')
        return

    target_value = 1 if state == 'ON' else 0
    try:
        # Only change state if it's different from current state
        if relays[pin_num].value() != target_value:
            relays[pin_num].value(target_value)
            debug_print(f'Relay {pin_num} set to {state}')
            publish_state(pin_num) # Publish the new state after changing it
        else:
            debug_print(f'Relay {pin_num} already in state {state}, no change.')
    except Exception as e:
        sys.print_exception(e)
        debug_print(f'ERROR: Failed to set relay state for pin {pin_num}: {e}')

def check_buttons():
    """Checks the state of configured buttons/switches and triggers relay actions.

    Handles debouncing and differentiates between momentary and static button types
    based on the `BUTTON_TYPES` configuration.
    """
    current_time = time.ticks_ms() # Get current time for debouncing

    for button_pin, button in buttons.items():
        # Ensure this button is mapped to a relay
        if button_pin not in BUTTON_RELAY_MAP:
            continue

        relay_pin = BUTTON_RELAY_MAP[button_pin]
        current_state = button.value() # Read the button's digital input state (0 or 1)
        last_state = last_button_states[button_pin]
        button_type = BUTTON_TYPES.get(button_pin, "momentary") # Default to momentary

        # Check if state has changed since last check
        if current_state != last_state:
            # Check if debounce time has passed since the last registered change
            if time.ticks_diff(current_time, last_button_times[button_pin]) > DEBOUNCE_MS:

                if button_type == "momentary":
                    # Momentary button: Toggle relay only on press (1 -> 0 transition with PULL_UP)
                    if current_state == 0: # Button pressed
                        # Determine the state to toggle to
                        current_relay_state = relays[relay_pin].value()
                        new_state_str = 'ON' if current_relay_state == 0 else 'OFF'
                        debug_print(f"Momentary button {button_pin} pressed, toggling relay {relay_pin} to {new_state_str}")
                        set_relay(relay_pin, new_state_str)

                elif button_type == "static":
                    # Static switch: Relay state follows switch position
                    # With PULL_UP: 0 = switch closed (ON), 1 = switch open (OFF)
                    new_state_str = 'ON' if current_state == 0 else 'OFF'
                    debug_print(f"Static switch {button_pin} changed state ({current_state}), setting relay {relay_pin} to {new_state_str}")
                    set_relay(relay_pin, new_state_str)

                else:
                     debug_print(f"WARNING: Unknown button type '{button_type}' for pin {button_pin}")

                # Update the time of the last valid state change
                last_button_times[button_pin] = current_time

            # Always update the last known state, even if debouncing ignored the change
            last_button_states[button_pin] = current_state

def read_light_sensor():
    """Reads the analog value from the configured light sensor.

    Returns:
        int: The raw ADC reading (typically 0-4095 for ESP32 12-bit ADC),
             or None if the sensor is not initialized or an error occurs.
    """
    if light_sensor is None:
        return None # Sensor not enabled or failed to initialize

    try:
        # Read the raw analog value
        raw_value = light_sensor.read()
        debug_print(f'Light sensor ADC reading: {raw_value}')
        return raw_value
    except Exception as e:
        sys.print_exception(e)
        debug_print(f'ERROR reading light sensor: {e}')
        return None

def check_light_sensor():
    """Checks light level and controls designated relays based on the threshold.

    Reads the sensor value at intervals defined by `LIGHT_CHECK_INTERVAL`.
    Compares the reading to `LIGHT_THRESHOLD` and turns relays in
    `LIGHT_CONTROLLED_RELAYS` ON if below threshold (dark) and OFF if above (bright).
    """
    global last_light_check # Need to modify the global timestamp

    # Skip if sensor is not enabled or no relays are configured for light control
    if light_sensor is None or not LIGHT_CONTROLLED_RELAYS:
        return

    current_time = time.ticks_ms()

    # Check if it's time for the next sensor reading
    if time.ticks_diff(current_time, last_light_check) < (LIGHT_CHECK_INTERVAL * 1000):
        return # Not time yet

    last_light_check = current_time # Update timestamp for this check

    # Read the current light level
    light_level = read_light_sensor()
    if light_level is None:
        return # Sensor read error

    debug_print(f"Light level check: Reading={light_level}, Threshold={LIGHT_THRESHOLD}")

    # Determine desired state based on threshold
    desired_state = 'ON' if light_level < LIGHT_THRESHOLD else 'OFF'

    # Apply desired state to all light-controlled relays
    for pin_num in LIGHT_CONTROLLED_RELAYS:
        if pin_num in relays:
            # Check current state to avoid unnecessary set_relay calls/MQTT publishes
            current_relay_state_val = relays[pin_num].value()
            current_relay_state_str = 'ON' if current_relay_state_val == 1 else 'OFF'

            if current_relay_state_str != desired_state:
                 debug_print(f"Light level ({light_level}) triggered change for relay {pin_num}. Setting to {desired_state}.")
                 set_relay(pin_num, desired_state)
            # else:
            #    debug_print(f"Light level ({light_level}), relay {pin_num} already in desired state {desired_state}.")
        else:
            debug_print(f"WARNING: Relay pin {pin_num} in LIGHT_CONTROLLED_RELAYS but not configured in RELAY_PINS.")


#------------------------------------------------------------------------------
# INITIALIZATION FUNCTIONS
#------------------------------------------------------------------------------

def init_relays():
    """Initializes all GPIO pins specified in RELAY_PINS as outputs.

    Sets the initial state of each relay to OFF and stores the Pin objects
    in the global `relays` dictionary.
    """
    global relays # Modifying the global dictionary
    relays = {} # Clear any previous state
    print("Initializing relays...")
    for pin_num in RELAY_PINS:
        try:
            pin = Pin(pin_num, Pin.OUT)
            pin.value(0)  # Ensure relay starts in OFF state
            relays[pin_num] = pin
            debug_print(f'Relay initialized on GPIO {pin_num}')
        except Exception as e:
            sys.print_exception(e)
            print(f'ERROR: Failed to initialize relay on GPIO {pin_num}: {e}')
    print(f'Relays initialized: {len(relays)} configured.')


def init_buttons():
    """Initializes all GPIO pins specified in BUTTON_PINS as inputs with pull-ups.

    Only initializes buttons that are mapped to a relay in `BUTTON_RELAY_MAP`.
    Stores the Pin objects in the global `buttons` dictionary and initializes
    state tracking variables.
    """
    global buttons, last_button_states, last_button_times # Modifying globals
    buttons = {} # Clear previous state
    last_button_states = {}
    last_button_times = {}
    print("Initializing buttons...")
    initialized_count = 0
    for button_pin in BUTTON_PINS:
        # Only initialize if this button is actually used to control a relay
        if button_pin in BUTTON_RELAY_MAP:
            try:
                pin = Pin(button_pin, Pin.IN, Pin.PULL_UP)
                buttons[button_pin] = pin
                # Read initial state immediately after setup
                initial_state = pin.value()
                last_button_states[button_pin] = initial_state
                last_button_times[button_pin] = time.ticks_ms() # Initialize time
                debug_print(f'Button initialized on GPIO {button_pin} (Type: {BUTTON_TYPES.get(button_pin, "momentary")}), controlling relay {BUTTON_RELAY_MAP[button_pin]}. Initial state: {initial_state}')
                initialized_count += 1
            except Exception as e:
                 sys.print_exception(e)
                 print(f'ERROR: Failed to initialize button on GPIO {button_pin}: {e}')
        else:
            debug_print(f'Skipping initialization for button pin {button_pin} as it is not mapped in BUTTON_RELAY_MAP.')
    print(f'Buttons initialized: {initialized_count} configured and mapped.')


def init_light_sensor():
    """Initializes the ADC for the light sensor if LIGHT_SENSOR_PIN is configured.

    Sets up the ADC on the specified pin, configures attenuation for full
    voltage range (on ESP32), and stores the ADC object in the global
    `light_sensor` variable.
    """
    global light_sensor # Modifying global
    light_sensor = None # Reset state

    if LIGHT_SENSOR_PIN is None:
        debug_print('Light sensor is disabled (LIGHT_SENSOR_PIN is None).')
        return # Exit if not configured

    print(f"Initializing light sensor on GPIO {LIGHT_SENSOR_PIN}...")
    try:
        # Ensure the pin is valid for ADC use (specific checks might be needed per board)
        pin_obj = Pin(LIGHT_SENSOR_PIN)
        adc = ADC(pin_obj)

        # Configure ADC attenuation for full range (0-3.3V typical for ESP32)
        # ATTN_11DB gives the full range. Other options: ATTN_0DB, ATTN_2_5DB, ATTN_6DB
        try:
            adc.atten(ADC.ATTN_11DB)
            debug_print('Set ADC attenuation to 11dB (full range)')
        except AttributeError:
            debug_print('ADC attenuation setting not available on this board, using default.')
            # Board might not support/require explicit attenuation setting

        light_sensor = adc
        print(f'Light sensor initialized successfully on GPIO {LIGHT_SENSOR_PIN}.')
        if not LIGHT_CONTROLLED_RELAYS:
             print('WARNING: Light sensor initialized, but LIGHT_CONTROLLED_RELAYS list is empty. No relays will be automatically controlled.')

    except Exception as e:
        sys.print_exception(e)
        print(f'ERROR: Failed to initialize light sensor on GPIO {LIGHT_SENSOR_PIN}: {e}')
        light_sensor = None # Ensure it's None if init failed


#------------------------------------------------------------------------------
# MAIN EXECUTION
#------------------------------------------------------------------------------

def main():
    """Main program execution function.

    Initializes hardware, connects to network services (WiFi, MQTT), and enters
    an infinite loop to handle button presses, MQTT messages, light sensor checks,
    and connection monitoring/reconnection.
    """
    global mqtt_client, device_id # Allow modification of global client object and device ID

    print("--- ESP32 Light Controller Starting ---")

    # --- Initialization ---
    init_relays()
    init_buttons()
    init_light_sensor()

    # --- Network Connection ---
    wlan = connect_wifi() # Blocks until connected

    # Get unique client ID (also sets global device_id)
    client_id_bytes = get_unique_client_id()
    print(f'Device ID set to: {device_id}')

    # Connect to MQTT broker (retries handled in main loop)
    mqtt_client = connect_mqtt(client_id_bytes)

    print('--- Initialization Complete - Entering Main Loop ---')

    # --- Connection Retry Logic Variables ---
    wifi_retry_count = 0
    mqtt_retry_count = 0
    # Start with base delay, increases exponentially on failure
    backoff_time = LOOP_DELAY

    # --- Main Loop ---
    while True:
        try:
            # --- WiFi Connection Check & Reconnect ---
            if not wlan.isconnected():
                wifi_retry_count += 1
                print(f'WiFi disconnected. Attempting reconnect {wifi_retry_count}/{MAX_CONNECTION_RETRIES}...')
                try:
                    wlan = connect_wifi() # Attempt reconnect
                except Exception as e:
                    sys.print_exception(e)
                    print(f"WiFi reconnection attempt failed: {e}")
                    # No immediate sleep here, handled below

                if not wlan.isconnected():
                    # Apply exponential backoff
                    backoff_time = min(60, LOOP_DELAY * (2 ** wifi_retry_count)) # Cap backoff at 60s
                    print(f'WiFi reconnection failed. Waiting {backoff_time} seconds before next attempt.')
                    time.sleep(backoff_time)

                    # Reset device if max retries exceeded
                    if wifi_retry_count >= MAX_CONNECTION_RETRIES:
                        print(f'FATAL: Failed to reconnect to Wi-Fi after {MAX_CONNECTION_RETRIES} attempts. Restarting device.')
                        time.sleep(1) # Short delay before reset
                        machine.reset()
                else:
                    # WiFi reconnected successfully
                    print("WiFi reconnected successfully.")
                    wifi_retry_count = 0 # Reset counter
                    backoff_time = LOOP_DELAY # Reset backoff
                    # Force MQTT reconnect as well, as connection might have been lost
                    if mqtt_client:
                        try:
                            mqtt_client.disconnect()
                        except Exception:
                            pass # Ignore errors during disconnect
                    mqtt_client = None
                    mqtt_retry_count = 0 # Reset MQTT counter too

            # --- MQTT Connection Check & Reconnect ---
            # Requires WiFi to be connected
            if wlan.isconnected() and mqtt_client is None:
                mqtt_retry_count += 1
                print(f'MQTT disconnected. Attempting reconnect {mqtt_retry_count}/{MAX_CONNECTION_RETRIES}...')
                mqtt_client = connect_mqtt(client_id_bytes) # Attempt reconnect

                if mqtt_client is None:
                    # Apply exponential backoff
                    backoff_time = min(60, LOOP_DELAY * (2 ** mqtt_retry_count)) # Cap backoff at 60s
                    print(f'MQTT reconnection failed. Waiting {backoff_time} seconds before next attempt.')
                    time.sleep(backoff_time)

                    # Reset device if max retries exceeded
                    if mqtt_retry_count >= MAX_CONNECTION_RETRIES:
                        print(f'FATAL: Failed to reconnect to MQTT after {MAX_CONNECTION_RETRIES} attempts. Restarting device.')
                        time.sleep(1)
                        machine.reset()
                else:
                    # MQTT reconnected successfully
                    print("MQTT reconnected successfully.")
                    mqtt_retry_count = 0 # Reset counter
                    backoff_time = LOOP_DELAY # Reset backoff

            # --- Process MQTT Messages ---
            # Only if connected to MQTT
            if mqtt_client:
                try:
                    # Check for incoming messages (non-blocking)
                    mqtt_client.check_msg()
                except Exception as e:
                    sys.print_exception(e)
                    print(f'ERROR during MQTT message check: {e}. Disconnecting.')
                    # Assume connection is lost
                    if mqtt_client:
                        try:
                            mqtt_client.disconnect()
                        except Exception:
                            pass # Ignore errors during disconnect
                    mqtt_client = None
                    mqtt_retry_count = 0 # Reset counter to allow immediate reconnect attempt

            # --- Check Hardware Inputs ---
            if buttons: # Only check if buttons are configured
                check_buttons()

            # --- Check Light Sensor ---
            # Handles its own timing internally
            check_light_sensor()

            # --- Loop Delay ---
            time.sleep(LOOP_DELAY)

        except KeyboardInterrupt:
            print("Keyboard interrupt detected. Exiting main loop.")
            break # Exit the while loop
        except Exception as e:
            sys.print_exception(e)
            print(f"ERROR in main loop: {e}. Continuing...")
            # Add a small delay to prevent rapid error loops
            time.sleep(5)

    # --- Cleanup (optional) ---
    print("Cleaning up...")
    if mqtt_client:
        try:
            # Publish OFFLINE status before disconnecting cleanly
            lwt_topic = get_main_state_topic()
            mqtt_client.publish(lwt_topic.encode(), b'OFFLINE', retain=True, qos=0)
            mqtt_client.disconnect()
            print("Disconnected from MQTT.")
        except Exception as e:
            sys.print_exception(e)
            print(f"Error during MQTT cleanup: {e}")

    print("--- Light Controller Stopped ---")


# Standard boilerplate to run the main function
if __name__ == '__main__':
    main()
