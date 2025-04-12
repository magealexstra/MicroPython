###############################################################################
# Light Controller - ESP32 MicroPython
# 
# Description: Controls lights/relays via buttons, MQTT commands, and light sensors
# 
# Features:
# - WiFi connectivity
# - MQTT control with unique device identifiers (based on MAC address)
# - Support for both momentary buttons and static toggle switches
# - Automatic light control based on light sensor readings
# - Multi-device deployment with no topic conflicts
###############################################################################

import network     # Handles WiFi connectivity
import time        # Time functions (delays, timers)
import sys         # System functions
import ubinascii   # Converts binary data to hex and vice versa
from machine import Pin, ADC  # Controls hardware pins and analog-to-digital conversion
import machine     # Provides access to hardware-specific functions
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
DEBUG = False  # Set to True to see detailed logs (helpful for troubleshooting)

# Light sensor configuration
LIGHT_SENSOR_PIN = None  # [CONFIG] GPIO pin for light sensor (e.g., 36) - None = disabled
LIGHT_THRESHOLD = 500    # [CONFIG] Light level cutoff point (lower number = darker when lights turn on)
LIGHT_CHECK_INTERVAL = 10  # [CONFIG] Seconds between light sensor checks
LIGHT_CONTROLLED_RELAYS = []  # [CONFIG] List of relay pins controlled by light (e.g., [5, 6])
                              # - Empty list means no automatic light control

# Internal variables (don't change these)
light_sensor = None      # Holds the light sensor object when initialized
last_light_check = 0     # Tracks when we last checked the light sensor

# MQTT client - handles communication with MQTT server
mqtt_client = None
device_id = None  # Will store the last 5 digits of MAC address for unique topics

# MQTT topics - These define the messaging channels
MQTT_TOPIC_PREFIX = 'esp32/lightcontrol'  # [CONFIG] Base topic for all messages
MQTT_USE_DEVICE_ID = True  # [CONFIG] Set to False to use non-unique topics (for backward compatibility)

# Legacy topic names (for backward compatibility)
MQTT_COMMAND_TOPIC = f'{MQTT_TOPIC_PREFIX}/command'
MQTT_STATE_TOPIC = f'{MQTT_TOPIC_PREFIX}/state'

# Timing settings
LOOP_DELAY = 1  # [CONFIG] Main loop delay in seconds (affects responsiveness)
MAX_CONNECTION_RETRIES = 10  # [CONFIG] Max attempts before restarting device

# Relay configuration
RELAY_PINS = [5,]  # [CONFIG] GPIO pins connected to relays, add more using comma: [5, 6, 7]
                   # These are the pins that control your lights/devices
relays = {}  # Internal storage of relay objects

# Button configuration
BUTTON_PINS = [4,]  # [CONFIG] GPIO pins connected to physical buttons/switches

# Button-to-Relay mapping - which button controls which light
# [CONFIG] Format: {button_pin: relay_pin}
BUTTON_RELAY_MAP = {
    4: 5,  # Button on pin 4 controls light on relay pin 5
    # Add more mappings as needed, e.g.: 15: 6,  # Button on pin 15 controls relay on pin 6
}

# Button type configuration
# [CONFIG] "momentary" = push button (toggle on press), "static" = toggle switch (follow position)
BUTTON_TYPES = {
    4: "momentary",  # Button on pin 4 is a momentary push button
    # Add more as needed, e.g.: 15: "static",  # Switch on pin 15 is a static toggle switch
}

# Button state tracking - prevents false triggers
buttons = {}           # Internal storage of button objects
last_button_states = {}  # Tracks previous button state (pressed/not pressed)
last_button_times = {}   # Tracks timing of button presses for debounce
DEBOUNCE_MS = 300      # [CONFIG] Minimum milliseconds between button presses (prevents "bounce")

#------------------------------------------------------------------------------
# UTILITY FUNCTIONS
#------------------------------------------------------------------------------

def debug_print(*args, **kwargs):
    """Prints debug messages only if DEBUG is enabled"""
    if DEBUG:
        print(*args, **kwargs)

def get_device_id():
    """Gets the unique identifier for this device from its MAC address
    
    This is used to create unique MQTT topics for each device
    """
    wlan = network.WLAN(network.STA_IF)
    mac = wlan.config('mac')  # raw bytes
    mac_hex = ubinascii.hexlify(mac).decode()  # convert to hex string
    return mac_hex[-5:]  # get last 5 characters

def get_unique_client_id():
    """Creates a unique identifier for this device from its MAC address
    
    This ensures each device has its own identity on the MQTT network
    """
    global device_id
    
    # Use the global device_id if already set, otherwise get it
    if device_id is None:
        device_id = get_device_id()
        
    return ('lightcont_' + device_id).encode()  # MQTT client ID must be bytes

def connect_wifi():
    """Connects to your WiFi network using credentials from secrets.py
    
    The system will wait until a connection is established before continuing.
    WiFi is required for all MQTT functionality to work.
    """
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('Connecting to Wi-Fi...')
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        while not wlan.isconnected():
            time.sleep(0.5)
        print('Connected to Wi-Fi:', wlan.ifconfig())
    return wlan

def connect_mqtt(client_id):
    """Connects to the MQTT server for sending/receiving commands
    
    This sets up the communication channel that allows:
    - Remote control of lights from phones, computers, etc.
    - Status reporting so other devices know if lights are on/off
    - Last Will and Testament so the system shows as offline if it loses power
    """
    try:
        client = MQTTClient(client_id, MQTT_BROKER, port=MQTT_PORT)
        
        # Set Last Will and Testament for the main topic
        state_topic = get_main_state_topic()
        client.set_last_will(state_topic.encode(), b'OFFLINE', retain=True, qos=0)
        client.set_callback(mqtt_callback)
        client.connect()
        debug_print('Connected to MQTT broker at {}:{}'.format(MQTT_BROKER, MQTT_PORT))
        
        # Publish ONLINE status to main topic
        client.publish(state_topic.encode(), b'ONLINE', retain=True, qos=0)
        
        # Subscribe to all relevant topics and publish initial states
        subscribe_all(client)
        
        # Publish initial states for all relays
        for pin_num in RELAY_PINS:
            state = 'OFF'  # Default state after initialization
            client.publish(get_state_topic(pin_num).encode(), state.encode(), retain=True, qos=0)
            
        return client
    except Exception as e:
        sys.print_exception(e)
        debug_print('Failed to connect to MQTT broker:', e)
        return None

def init_relays():
    """Sets up the relay outputs that control your lights/devices
    
    Each relay pin is set as an output and turned OFF initially.
    Relays are what actually switch your lights or devices on and off.
    """
    global relays
    for pin_num in RELAY_PINS:
        pin = Pin(pin_num, Pin.OUT)
        pin.value(0)  # Start OFF
        relays[pin_num] = pin
        debug_print('Relay initialized on GPIO', pin_num)

def init_light_sensor():
    """Sets up the light sensor hardware if configured
    
    This only runs if you've set LIGHT_SENSOR_PIN to a valid GPIO number.
    The light sensor needs to be an analog sensor (like a photoresistor 
    or photodiode in a voltage divider circuit).
    """
    global light_sensor
    
    if LIGHT_SENSOR_PIN is None:
        debug_print('Light sensor disabled (LIGHT_SENSOR_PIN not set)')
        return
    
    try:
        # Create ADC (Analog to Digital Converter) object on the specified pin
        pin = Pin(LIGHT_SENSOR_PIN)
        adc = ADC(pin)
        
        # Configure ADC for full voltage range (0-3.3V on ESP32)
        try:
            adc.atten(ADC.ATTN_11DB)  # Full range: 3.3V
        except AttributeError:
            # Skip if not available on this board
            pass
            
        light_sensor = adc
        debug_print(f'Light sensor initialized on GPIO {LIGHT_SENSOR_PIN}')
    except Exception as e:
        sys.print_exception(e)
        debug_print(f'Failed to initialize light sensor: {e}')
        light_sensor = None

def init_buttons():
    """Sets up the physical buttons/switches connected to GPIO pins
    
    Supports both momentary buttons and static toggle switches:
    - Momentary buttons: Toggle light state on each press (defined as "momentary" in BUTTON_TYPES)
    - Static switches: Light state follows switch position (defined as "static" in BUTTON_TYPES)
    
    Only buttons/switches that are mapped to relays in BUTTON_RELAY_MAP will be used.
    """
    global buttons, last_button_states, last_button_times
    for button_pin in BUTTON_PINS:
        # Only initialize if this button is mapped to a relay
        if button_pin in BUTTON_RELAY_MAP:
            pin = Pin(button_pin, Pin.IN, Pin.PULL_UP)
            buttons[button_pin] = pin
            last_button_states[button_pin] = pin.value()
            last_button_times[button_pin] = 0
            debug_print(f'Button initialized on GPIO {button_pin}, controlling relay on GPIO {BUTTON_RELAY_MAP[button_pin]}')

def check_buttons():
    """Monitors buttons and switches to control lights
    
    Supports two types of controls:
    - Momentary buttons: Toggle light state on each press (push buttons)
    - Static switches: Light state directly follows switch position (toggle/flip switches)
    """
    current_time = time.ticks_ms()
    
    for button_pin, button in buttons.items():
        # Get the associated relay pin from the mapping
        if button_pin not in BUTTON_RELAY_MAP:
            continue  # Skip if no mapping exists
            
        relay_pin = BUTTON_RELAY_MAP[button_pin]
        
        # Read current button/switch state
        current_state = button.value()
        
        # Determine the button type (default to momentary if not specified)
        button_type = BUTTON_TYPES.get(button_pin, "momentary")
        
        # State has changed (for both types we need debouncing)
        if current_state != last_button_states[button_pin]:
            # Debounce check
            if time.ticks_diff(current_time, last_button_times[button_pin]) > DEBOUNCE_MS:
                
                if button_type == "momentary":
                    # For momentary buttons: toggle only on press (1->0 transition with pull-up)
                    if current_state == 0:
                        # Toggle relay state
                        new_state = 'ON' if relays[relay_pin].value() == 0 else 'OFF'
                        set_relay(relay_pin, new_state)
                        debug_print(f"Button {button_pin} pressed, toggled relay {relay_pin} to {new_state}")
                else:
                    # For static switches: follow the switch position
                    # With pull-up resistors: 0 = switch closed (ON), 1 = switch open (OFF)
                    new_state = 'ON' if current_state == 0 else 'OFF'
                    set_relay(relay_pin, new_state)
                    debug_print(f"Switch {button_pin} changed to {'closed' if current_state == 0 else 'open'}, set relay {relay_pin} to {new_state}")
                
                # Update timer
                last_button_times[button_pin] = current_time
            
            # Update previous state
            last_button_states[button_pin] = current_state

def get_main_command_topic():
    """Creates the main MQTT topic for sending commands to all relays"""
    if MQTT_USE_DEVICE_ID:
        return f'{MQTT_TOPIC_PREFIX}/{device_id}/command'
    else:
        return f'{MQTT_TOPIC_PREFIX}/command'

def get_main_state_topic():
    """Creates the main MQTT topic for reporting device status"""
    if MQTT_USE_DEVICE_ID:
        return f'{MQTT_TOPIC_PREFIX}/{device_id}/state'
    else:
        return f'{MQTT_TOPIC_PREFIX}/state'

def get_command_topic(pin_num):
    """Creates the MQTT topic name for sending commands to a specific relay"""
    if MQTT_USE_DEVICE_ID:
        return f'{MQTT_TOPIC_PREFIX}/{device_id}/{pin_num}/command'
    else:
        return f'{MQTT_TOPIC_PREFIX}/{pin_num}/command'

def get_state_topic(pin_num):
    """Creates the MQTT topic name for reporting the state of a specific relay"""
    if MQTT_USE_DEVICE_ID:
        return f'{MQTT_TOPIC_PREFIX}/{device_id}/{pin_num}/state'
    else:
        return f'{MQTT_TOPIC_PREFIX}/{pin_num}/state'

def subscribe_all(client):
    """Sets up MQTT subscriptions to listen for remote commands
    
    This allows the device to receive commands like "ON" or "OFF"
    through MQTT from other devices, apps, or automation systems.
    """
    # Subscribe to the legacy main topic for backward compatibility if not using device ID
    if not MQTT_USE_DEVICE_ID:
        client.subscribe(MQTT_COMMAND_TOPIC.encode())
        debug_print('Subscribed to legacy main command topic:', MQTT_COMMAND_TOPIC)
    
    # Subscribe to the device-specific main command topic
    main_topic = get_main_command_topic()
    client.subscribe(main_topic.encode())
    debug_print('Subscribed to main command topic:', main_topic)
    
    # Subscribe to pin-specific topics
    for pin_num in RELAY_PINS:
        topic = get_command_topic(pin_num)
        client.subscribe(topic.encode())
        debug_print('Subscribed to pin-specific topic:', topic)

def set_relay(pin_num, state):
    """Turns a light/device ON or OFF
    
    This is the main function that actually controls the hardware:
    - Changes the physical state of the relay
    - Updates the MQTT status so other devices know the current state
    
    Args:
        pin_num: The GPIO pin number connected to the relay
        state: Either 'ON' or 'OFF'
    """
    if pin_num not in relays:
        debug_print('Unknown relay pin:', pin_num)
        return
        
    try:
        if state == 'ON':
            relays[pin_num].value(1)  # Turn on the relay
        else:
            relays[pin_num].value(0)  # Turn off the relay
        debug_print('Relay {} set to {}'.format(pin_num, state))
        publish_state(pin_num)  # Announce the new state via MQTT
    except Exception as e:
        sys.print_exception(e)
        debug_print('Failed to set relay state for pin {}: {}'.format(pin_num, e))

def publish_state(pin_num):
    """Broadcasts the current light state to MQTT
    
    This lets other devices and systems know if a light is currently
    ON or OFF, enabling synchronized displays and automation.
    """
    global mqtt_client

    if mqtt_client is None:
        debug_print('MQTT not connected, skipping state publish')   
        return

    try:
        state = 'ON' if relays[pin_num].value() else 'OFF'
        mqtt_client.publish(get_state_topic(pin_num).encode(), state.encode(), retain=True, qos=0)
        debug_print('Published state:', state)
    except Exception as e:
        sys.print_exception(e)
        debug_print('Failed to publish state for pin {}: {}'.format(pin_num, e))
        mqtt_client = None  # Force reconnect on next loop

def mqtt_callback(topic, msg):
    """Processes incoming commands from MQTT
    
    This is called automatically when remote commands are received.
    It handles two types of commands:
    1. Main topic commands - apply to all lights at once
    2. Pin-specific commands - control individual lights
    
    Commands are typically "ON" or "OFF" messages.
    """
    try:
        topic_str = topic.decode()
        debug_print('Received message on topic {}: {}'.format(topic_str, msg))
        
        # Handle legacy main topic for backward compatibility
        if topic_str == MQTT_COMMAND_TOPIC or topic_str == get_main_command_topic():
            command = msg.decode().strip().upper()
            debug_print('Received command on main topic:', command)
            
            # Apply to all relays
            for pin_num in relays:
                if command == 'ON':
                    relays[pin_num].value(1)
                elif command == 'OFF':
                    relays[pin_num].value(0)
                publish_state(pin_num)
            return
            
        # Handle pin-specific topics
        try:
            parts = topic_str.split('/')
            # Check if this is a command topic (ends with /command)
            if parts[-1] == 'command':
                # Try to extract pin number from different topic formats
                pin_num = None
                
                if MQTT_USE_DEVICE_ID:
                    # Format: esp32/lightcontrol/abc12/5/command
                    if len(parts) >= 5 and parts[-3] == device_id:
                        pin_num = int(parts[-2])
                else:
                    # Format: esp32/lightcontrol/5/command
                    if len(parts) >= 4:
                        pin_num = int(parts[-2])
                
                if pin_num is not None and pin_num in relays:
                    command = msg.decode().strip().upper()
                    if command == 'ON':
                        relays[pin_num].value(1)
                    elif command == 'OFF':
                        relays[pin_num].value(0)
                    publish_state(pin_num)
                else:
                    debug_print('Unknown relay pin or invalid topic format:', topic_str)
        except (ValueError, IndexError):
            debug_print('Could not parse pin number from topic:', topic_str)
            
    except Exception as e:
        sys.print_exception(e)
        debug_print('Error in MQTT callback:', e)

def read_light_sensor():
    """Reads the current light level from the sensor
    
    Returns:
        A number representing light intensity (higher = brighter)
        Or None if the sensor isn't configured/available
        
    The specific range depends on your sensor, but typical values are:
    - Low numbers (0-500): Dark conditions
    - High numbers (1000+): Bright conditions
    """
    if light_sensor is None:
        return None
        
    try:
        # Read raw ADC value
        raw_value = light_sensor.read()
        debug_print(f'Light sensor reading: {raw_value}')
        return raw_value
    except Exception as e:
        sys.print_exception(e)
        debug_print(f'Error reading light sensor: {e}')
        return None

def check_light_sensor():
    """Checks light level and automatically controls lights
    
    This function:
    1. Reads the current light level from the sensor
    2. If it's dark (below LIGHT_THRESHOLD), turns on lights
    3. If it's bright (above LIGHT_THRESHOLD), turns off lights
    
    Only relays listed in LIGHT_CONTROLLED_RELAYS will be affected.
    """
    global last_light_check
    
    # Skip if light sensor not configured
    if light_sensor is None:
        return
        
    current_time = time.ticks_ms()
    
    # Only check at specified intervals to avoid constant checking
    if time.ticks_diff(current_time, last_light_check) < (LIGHT_CHECK_INTERVAL * 1000):
        return
        
    last_light_check = current_time
    
    # Read sensor
    light_level = read_light_sensor()
    if light_level is None:
        return
        
    # Debug output
    debug_print(f"Light level: {light_level}, threshold: {LIGHT_THRESHOLD}")
    
    # Control only specified relays based on light level
    for pin_num in LIGHT_CONTROLLED_RELAYS:
        # Skip if this relay is not initialized
        if pin_num not in relays:
            debug_print(f"Relay {pin_num} specified in LIGHT_CONTROLLED_RELAYS but not in RELAY_PINS, skipping")
            continue
            
        current_state = 'ON' if relays[pin_num].value() else 'OFF'
        
        # If it's dark (reading below threshold) and light is off, turn it on
        if light_level < LIGHT_THRESHOLD and current_state == 'OFF':
            debug_print(f"Dark detected, turning ON light on relay {pin_num}")
            set_relay(pin_num, 'ON')
            
        # If it's bright (reading above threshold) and light is on, turn it off
        elif light_level >= LIGHT_THRESHOLD and current_state == 'ON':
            debug_print(f"Bright detected, turning OFF light on relay {pin_num}")
            set_relay(pin_num, 'OFF')

def main():
    """Main program that runs continuously
    
    This function:
    1. Initializes all hardware (relays, buttons, light sensor)
    2. Connects to WiFi and MQTT
    3. Enters a continuous loop to:
       - Check for button presses
       - Check light sensor (if enabled)
       - Listen for MQTT commands
       - Monitor connections and reconnect if needed
    """
    global mqtt_client, device_id
    
    # Initialize hardware first to ensure it's ready before any MQTT commands arrive
    init_relays()
    print('Relays initialized')
    
    # Initialize buttons if any are configured
    init_buttons()
    if buttons:
        print('Buttons initialized')
        
    # Initialize light sensor if configured
    init_light_sensor()
    if light_sensor:
        if LIGHT_CONTROLLED_RELAYS:
            print(f'Light sensor initialized, controlling relays: {LIGHT_CONTROLLED_RELAYS}')
        else:
            print('Light sensor initialized but no relays set for light control (LIGHT_CONTROLLED_RELAYS is empty)')
    
    # Connect to network services
    wlan = connect_wifi()
    
    # Set device ID from MAC address (get_unique_client_id will handle this now)
    client_id = get_unique_client_id()
    print(f'Device ID: {device_id}')
    mqtt_client = connect_mqtt(client_id)
    
    print('Light controller initialized')

    # Connection retry counters and limits
    wifi_retry_count = 0
    mqtt_retry_count = 0
    backoff_time = LOOP_DELAY

    while True:
        # Reconnect Wi-Fi if disconnected
        if not wlan.isconnected():
            wifi_retry_count += 1
            debug_print('Wi-Fi disconnected, attempting to reconnect... (Attempt {}/{})'.format(
                wifi_retry_count, MAX_CONNECTION_RETRIES))
            
            wlan = connect_wifi()
            
            if not wlan.isconnected():
                # Apply exponential backoff for reconnection attempts
                backoff_time = min(30, LOOP_DELAY * (2 ** wifi_retry_count))
                debug_print('Wi-Fi reconnection failed. Waiting {} seconds before retry'.format(backoff_time))
                time.sleep(backoff_time)
                
                # Reset device after too many failed attempts
                if wifi_retry_count >= MAX_CONNECTION_RETRIES:
                    print('Failed to reconnect to Wi-Fi after {} attempts. Restarting device...'.format(MAX_CONNECTION_RETRIES))
                    time.sleep(1)
                    machine.reset()
            else:
                # Reset counter on successful connection
                wifi_retry_count = 0
                backoff_time = LOOP_DELAY
            
        # Reconnect MQTT if disconnected
        if mqtt_client is None:
            mqtt_retry_count += 1
            debug_print('MQTT disconnected, attempting to reconnect... (Attempt {}/{})'.format(
                mqtt_retry_count, MAX_CONNECTION_RETRIES))
            
            mqtt_client = connect_mqtt(client_id)
            
            if mqtt_client is None:
                # Apply exponential backoff for reconnection attempts
                backoff_time = min(30, LOOP_DELAY * (2 ** mqtt_retry_count))
                debug_print('MQTT reconnection failed. Waiting {} seconds before retry'.format(backoff_time))
                time.sleep(backoff_time)
                
                # Reset device after too many failed attempts
                if mqtt_retry_count >= MAX_CONNECTION_RETRIES:
                    print('Failed to reconnect to MQTT after {} attempts. Restarting device...'.format(MAX_CONNECTION_RETRIES))
                    time.sleep(1)
                    machine.reset()
            else:
                # Reset counter on successful connection
                mqtt_retry_count = 0
                backoff_time = LOOP_DELAY
            
        # Check for any pending MQTT messages
        if mqtt_client:
            try:
                mqtt_client.check_msg()  # Non-blocking check for new messages
            except Exception as e:
                sys.print_exception(e)
                debug_print('MQTT error:', e)
                mqtt_client = None  # Force reconnect on next loop
        
        # Check for button presses
        if buttons:
            check_buttons()
            
        # Check light sensor
        check_light_sensor()
        
        time.sleep(LOOP_DELAY)

if __name__ == '__main__':
    main()
