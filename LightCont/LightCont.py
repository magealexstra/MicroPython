import network
import time
import sys
import ubinascii
from machine import Pin
import machine  # Needed for machine.reset()
from umqtt.simple import MQTTClient

# Import secrets (Wi-Fi credentials, MQTT broker info)
try:
    from secrets import WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT
except ImportError:
    print("Failed to import secrets. Please ensure 'secrets.py' is in the project root directory.")
    raise

# Debug flag
DEBUG = False

# Global variable
mqtt_client = None

# MQTT topics
MQTT_COMMAND_TOPIC = 'esp32/lightcontrol/command'  # Topic to receive ON/OFF commands edit per your needs
MQTT_STATE_TOPIC = 'esp32/lightcontrol/state'      # Topic to publish current light state edit per your needs

# Main loop delay in seconds
LOOP_DELAY = 1

# Maximum number of connection retry attempts before device reset
MAX_CONNECTION_RETRIES = 10

# GPIO pin configuration for relay control
RELAY_PINS = [5,]  # Example GPIO pins for relay control, add more if needed using comma
relays = {}  # Dictionary to hold Pin objects

# GPIO pin configuration for momentary switches
BUTTON_PINS = [4,]  # Example GPIO pins connected to momentary switches

# Button-to-Relay mapping - each button controls a specific relay
# Format: {button_pin: relay_pin}
BUTTON_RELAY_MAP = {
    4: 5,  # Button on pin 4 controls relay on pin 5
    # Add more mappings as needed
}

# Button state tracking
buttons = {}           # Dictionary to hold button Pin objects
last_button_states = {}  # Store previous button states
last_button_times = {}   # Store time of last button press for debounce
DEBOUNCE_MS = 300      # Debounce time in milliseconds

def debug_print(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)

def get_unique_client_id():
    wlan = network.WLAN(network.STA_IF)
    mac = wlan.config('mac')  # raw bytes
    mac_hex = ubinascii.hexlify(mac).decode()  # convert to hex string
    short_mac = mac_hex[-5:]  # get last 5 characters
    return ('lightcont_' + short_mac).encode()  # MQTT client ID must be bytes

def connect_wifi():
    """Connect to Wi-Fi using credentials from secrets.py"""
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
    """Connect to MQTT broker and return client"""
    try:
        client = MQTTClient(client_id, MQTT_BROKER, port=MQTT_PORT)
        # Set Last Will and Testament for the main topic
        client.set_last_will(MQTT_STATE_TOPIC.encode(), b'OFFLINE', retain=True, qos=0)
        client.set_callback(mqtt_callback)
        client.connect()
        debug_print('Connected to MQTT broker at {}:{}'.format(MQTT_BROKER, MQTT_PORT))
        
        # Publish ONLINE status to main topic
        client.publish(MQTT_STATE_TOPIC.encode(), b'ONLINE', retain=True, qos=0)
        
        # Subscribe to all relevant topics and publish initial states
        subscribe_all(client)
        
        # Publish initial states for all relays
        for pin_num in RELAY_PINS:
            state = 'OFF'  # Default state after initialization
            client.publish(get_state_topic(pin_num).encode(), state.encode(), retain=True, qos=0)
            
        return client
    except Exception as e:
        debug_print('Failed to connect to MQTT broker:', e)
        return None

def init_relays():
    global relays
    for pin_num in RELAY_PINS:
        pin = Pin(pin_num, Pin.OUT)
        pin.value(0)  # Start OFF
        relays[pin_num] = pin
        debug_print('Relay initialized on GPIO', pin_num)

def init_buttons():
    """Initialize the button pins with pull-up resistors"""
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
    """Check the state of buttons and toggle relays if pressed"""
    current_time = time.ticks_ms()
    
    for button_pin, button in buttons.items():
        # Get the associated relay pin from the mapping
        if button_pin not in BUTTON_RELAY_MAP:
            continue  # Skip if no mapping exists
            
        relay_pin = BUTTON_RELAY_MAP[button_pin]
        
        # Read current button state
        current_state = button.value()
        
        # Check for button press (transition from HIGH to LOW for pull-up)
        if current_state != last_button_states[button_pin]:
            # Debounce check
            if time.ticks_diff(current_time, last_button_times[button_pin]) > DEBOUNCE_MS:
                # For pull-up: button is pressed when goes from 1 to 0
                if current_state == 0:  # Adjust based on your configuration
                    # Toggle relay state
                    new_state = 'ON' if relays[relay_pin].value() == 0 else 'OFF'
                    set_relay(relay_pin, new_state)
                    debug_print(f'Button {button_pin} pressed, toggled relay {relay_pin} to {new_state}')
                
                # Update timer
                last_button_times[button_pin] = current_time
            
            # Update previous state
            last_button_states[button_pin] = current_state

def get_command_topic(pin_num):
    return f'esp32/lightcontrol/{pin_num}/command'

def get_state_topic(pin_num):
    return f'esp32/lightcontrol/{pin_num}/state'

def subscribe_all(client):
    # Subscribe to the main topic for backward compatibility
    client.subscribe(MQTT_COMMAND_TOPIC.encode())
    debug_print('Subscribed to main command topic:', MQTT_COMMAND_TOPIC)
    
    # Subscribe to pin-specific topics
    for pin_num in RELAY_PINS:
        topic = get_command_topic(pin_num)
        client.subscribe(topic.encode())
        debug_print('Subscribed to pin-specific topic:', topic)

def set_relay(pin_num, state):
    """Set relay state and publish new state"""
    if pin_num not in relays:
        debug_print('Unknown relay pin:', pin_num)
        return
        
    try:
        if state == 'ON':
            relays[pin_num].value(1)
        else:
            relays[pin_num].value(0)
        debug_print('Relay {} set to {}'.format(pin_num, state))
        publish_state(pin_num)
    except Exception as e:
        debug_print('Failed to set relay state for pin {}: {}'.format(pin_num, e))

def publish_state(pin_num):
    """Publish current relay state to MQTT"""
    global mqtt_client

    if mqtt_client is None:
        debug_print('MQTT not connected, skipping state publish')   
        return

    try:
        state = 'ON' if relays[pin_num].value() else 'OFF'
        mqtt_client.publish(get_state_topic(pin_num).encode(), state.encode(), retain=True, qos=0)
        debug_print('Published state:', state)
    except Exception as e:
        debug_print('Failed to publish state for pin {}: {}'.format(pin_num, e))
        mqtt_client = None  # Force reconnect on next loop

def mqtt_callback(topic, msg):
    try:
        topic_str = topic.decode()
        debug_print('Received message on topic {}: {}'.format(topic_str, msg))
        
        # Handle legacy main topic first (if used)
        if topic_str == MQTT_COMMAND_TOPIC:
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
            if len(parts) >= 4 and parts[-1] == 'command':
                pin_num = int(parts[-2])  # e.g., 'esp32/lightcontrol/18/command'
                command = msg.decode().strip().upper()
                
                if pin_num in relays:
                    if command == 'ON':
                        relays[pin_num].value(1)
                    elif command == 'OFF':
                        relays[pin_num].value(0)
                    publish_state(pin_num)
                else:
                    debug_print('Unknown relay pin:', pin_num)
        except (ValueError, IndexError):
            debug_print('Could not parse pin number from topic:', topic_str)
            
    except Exception as e:
        debug_print('Error in MQTT callback:', e)

def main():
    global mqtt_client
    
    # Initialize hardware first to ensure it's ready before any MQTT commands arrive
    init_relays()
    print('Relays initialized')
    
    # Initialize buttons if any are configured
    init_buttons()
    if buttons:
        print('Buttons initialized')
    
    # Connect to network services
    wlan = connect_wifi()
    client_id = get_unique_client_id()
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
                debug_print('MQTT error:', e)
                mqtt_client = None  # Force reconnect on next loop
        
        # Check for button presses
        if buttons:
            check_buttons()
        
        time.sleep(LOOP_DELAY)

if __name__ == '__main__':
    main()
