###############################################################################
# BreakSense - ESP32 MicroPython Break Beam Sensor Monitor
#
# Description: Monitors a digital input pin (connected to a break beam sensor)
#              and publishes state changes (HIGH/LOW) to an MQTT broker.
#              Includes Wi-Fi connection management, MQTT connection with
#              Last Will and Testament (LWT), and input debouncing.
#
# Features:
# - Connects to Wi-Fi using credentials from secrets.py.
# - Connects to an MQTT broker with a unique client ID and LWT.
# - Monitors a specified GPIO pin with an internal pull-up resistor.
# - Debounces the input signal to prevent spurious readings.
# - Publishes 'HIGH' or 'LOW' to a configured MQTT topic on state change.
# - Publishes 'ONLINE'/'OFFLINE' status messages.
# - Includes optional debug printing.
# - Attempts to automatically reconnect Wi-Fi and MQTT if connections drop.
###############################################################################

#------------------------------------------------------------------------------
# IMPORTS
#------------------------------------------------------------------------------
import network     # Handles WiFi connectivity
import time        # Time functions (delays, ticks)
import ubinascii   # Binary/ASCII conversions (for MAC address)
import sys         # System-specific parameters and functions (exception printing)
from machine import Pin # Hardware control for GPIO pins
from umqtt.simple import MQTTClient # MQTT protocol client

# Import local secrets
try:
    from secrets import WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT # Network & MQTT credentials
except ImportError:
    print("CRITICAL ERROR: Failed to import secrets.")
    print("Please ensure 'secrets.py' exists in the root directory with WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, and MQTT_PORT defined.")
    # In a real scenario, might halt or enter a safe mode
    # For now, we'll let it raise the ImportError later if needed functions are called
    raise # Re-raise the exception to make the failure clear

#------------------------------------------------------------------------------
# CONFIGURATION SETTINGS - ADJUST THESE FOR YOUR SETUP
#------------------------------------------------------------------------------
MQTT_TOPIC = 'esp32/breaksense' # [CONFIG] Base MQTT topic for publishing sensor state and status.
GPIO_PIN = 2                    # [CONFIG] GPIO pin number connected to the break beam sensor's output.
DEBUG = False                   # [CONFIG] Set to True to enable detailed log messages for troubleshooting.
DEBOUNCE_TIME_MS = 200          # [CONFIG] Time in milliseconds the pin state must be stable before registering a change.
LOOP_DELAY_S = 0.1              # [CONFIG] Delay in seconds between checks in the main loop.

#------------------------------------------------------------------------------
# GLOBAL VARIABLES - INTERNAL STATE TRACKING (DO NOT MODIFY)
#------------------------------------------------------------------------------
# No global variables required for this script's logic beyond configuration.
# State is managed within the main function's scope.

#------------------------------------------------------------------------------
# UTILITY FUNCTIONS
#------------------------------------------------------------------------------
def debug_print(*args, **kwargs):
    """Prints messages only if the global DEBUG flag is True.

    Acts as a wrapper around the built-in print function.

    Args:
        *args: Positional arguments to pass to print().
        **kwargs: Keyword arguments to pass to print().

    Returns:
        None
    """
    if DEBUG:
        print(*args, **kwargs)

def get_unique_client_id():
    """Generates a unique MQTT client ID based on the device's MAC address.

    Ensures that this device has a distinct ID when connecting to the MQTT broker.

    Returns:
        bytes: A unique client ID string (e.g., b'breaksense_a1b2c3d4e5').
    """
    wlan = network.WLAN(network.STA_IF)
    mac = wlan.config('mac') # Get the MAC address
    client_id = b'breaksense_' + ubinascii.hexlify(mac)[-6:] # Use last 6 hex digits
    debug_print("Generated MQTT Client ID:", client_id)
    return client_id

#------------------------------------------------------------------------------
# CONNECTION FUNCTIONS
#------------------------------------------------------------------------------
def connect_wifi():
    """Connects the device to the Wi-Fi network specified in secrets.py.

    Activates the WLAN station interface and attempts connection. Blocks until
    connection is successful. Handles reconnection attempts implicitly if called
    when already disconnected.

    Returns:
        network.WLAN: The active and connected WLAN interface object.

    Raises:
        Exception: Can raise exceptions if Wi-Fi connection fails persistently
                   (though the internal loop retries indefinitely).
    """
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('Connecting to Wi-Fi network:', WIFI_SSID)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        # Wait for connection
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
        # Depending on requirements, could raise an error or return None
        # For robustness, we might want to allow main loop to retry
        return None # Indicate failure

def connect_mqtt():
    """Connects to the MQTT broker specified in secrets.py.

    Generates a unique client ID, sets the Last Will and Testament (LWT)
    to publish 'OFFLINE' on unexpected disconnect, and attempts connection.
    Publishes 'ONLINE' upon successful connection.

    Returns:
        umqtt.simple.MQTTClient: The connected MQTT client object, or None if connection fails.
    """
    try:
        client_id = get_unique_client_id()
        client = MQTTClient(client_id, MQTT_BROKER, port=MQTT_PORT)

        # Set Last Will and Testament (LWT)
        # If the client disconnects uncleanly, the broker will publish 'OFFLINE'
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
        sys.print_exception(e) # Print detailed exception traceback
        return None # Indicate failure

#------------------------------------------------------------------------------
# MAIN FUNCTION
#------------------------------------------------------------------------------
def main():
    """Main execution function.

    Connects to Wi-Fi and MQTT, then enters an infinite loop to monitor the
    break beam sensor pin, handle debouncing, and publish state changes.
    Includes logic for attempting reconnections if Wi-Fi or MQTT links drop.

    Returns:
        None. Runs indefinitely until interrupted or reset.
    """
    print("--- BreakSense Starting ---")
    wlan = connect_wifi()
    if not wlan:
        print("CRITICAL: Wi-Fi connection failed on startup. Halting or retrying might be needed.")
        # Depending on requirements, could loop here trying to connect, or reset.
        return # Exit main if initial Wi-Fi fails

    client = connect_mqtt()
    # Note: MQTT connection failure on startup might be acceptable if the device
    # can function locally or retry connection later.

    # Initialize the GPIO pin for the break beam sensor
    # Pin.IN: Input mode
    # Pin.PULL_UP: Enable internal pull-up resistor (assuming sensor pulls LOW when beam is broken)
    pin = Pin(GPIO_PIN, Pin.IN, Pin.PULL_UP)
    last_stable_state = pin.value() # Get initial state
    last_state_change_time = time.ticks_ms() # Initialize debounce timer

    print('Monitoring GPIO Pin:', GPIO_PIN)
    print('Initial State:', 'HIGH (Beam Intact)' if last_stable_state else 'LOW (Beam Broken)')

    while True:
        try:
            # --- Connection Management ---
            # Check Wi-Fi connection
            if not wlan.isconnected():
                print('Wi-Fi disconnected. Attempting to reconnect...')
                # Try to reconnect WiFi - might block for a bit
                wlan = connect_wifi()
                if not wlan or not wlan.isconnected():
                    print("Reconnect failed. Will retry later.")
                    time.sleep(5) # Wait before next check cycle if WiFi fails
                    continue # Skip sensor reading if no WiFi
                else:
                    # WiFi reconnected, force MQTT reconnect attempt next
                    print("Wi-Fi reconnected.")
                    client = None # Set client to None to trigger MQTT reconnect

            # Check MQTT connection (and attempt reconnect if needed and WiFi is up)
            if client is None and wlan and wlan.isconnected():
                print("MQTT client not connected. Attempting to reconnect...")
                client = connect_mqtt()
                if client:
                    print("MQTT reconnected.")
                    # Re-publish current state after reconnecting
                    current_state_msg = b'HIGH' if last_stable_state else b'LOW'
                    try:
                        client.publish(MQTT_TOPIC.encode(), current_state_msg, retain=True, qos=0)
                        debug_print('Re-published current state:', current_state_msg)
                    except Exception as e:
                        print("ERROR: Failed to re-publish state after MQTT reconnect.")
                        sys.print_exception(e)
                        client = None # Mark for reconnect again if publish fails
                else:
                    print("MQTT reconnect failed. Will retry later.")
                    # No client, wait before next cycle
                    time.sleep(5)


            # --- Sensor Reading and Debouncing ---
            current_state = pin.value()
            now = time.ticks_ms()

            # If state changed, reset the debounce timer
            if current_state != last_stable_state:
                 # Check if enough time has passed since the last *registered* change
                 # This prevents rapid toggling from flooding MQTT if DEBOUNCE_TIME is short
                 if time.ticks_diff(now, last_state_change_time) > DEBOUNCE_TIME_MS:
                    # State has been stable for long enough, register the change
                    last_stable_state = current_state
                    last_state_change_time = now # Record time of this stable change

                    state_str = 'HIGH (Beam Intact)' if last_stable_state else 'LOW (Beam Broken)'
                    print(f"State changed to: {state_str}")

                    # --- MQTT Publishing ---
                    if client:
                        msg = b'HIGH' if last_stable_state else b'LOW'
                        try:
                            client.publish(MQTT_TOPIC.encode(), msg, retain=True, qos=0) # Retain the latest state
                            debug_print('Published state change:', msg)
                        except Exception as e:
                            print("ERROR: Failed to publish MQTT message.")
                            sys.print_exception(e)
                            # Assume MQTT connection is broken, trigger reconnect on next loop
                            client = None
                    else:
                        debug_print('MQTT client not connected, skipping publish.')

            # --- Loop Delay ---
            time.sleep(LOOP_DELAY_S)

        except KeyboardInterrupt:
            print("Interrupted by user.")
            break
        except Exception as e:
            print("An unexpected error occurred in the main loop:")
            sys.print_exception(e)
            # Potentially add a longer delay or reset mechanism here
            print("Restarting loop after 5 seconds...")
            time.sleep(5)

    # --- Cleanup (optional, may not run if reset) ---
    print("--- BreakSense Stopping ---")
    if client:
        try:
            # Publish OFFLINE status cleanly if possible
            client.publish(MQTT_TOPIC.encode(), b'OFFLINE', retain=True, qos=0)
            client.disconnect()
            print("Disconnected from MQTT.")
        except Exception as e:
            print("Error during MQTT disconnect:")
            sys.print_exception(e)
    if wlan and wlan.isconnected():
        wlan.disconnect()
        wlan.active(False)
        print("Disconnected from Wi-Fi.")

#------------------------------------------------------------------------------
# MAIN EXECUTION
#------------------------------------------------------------------------------
if __name__ == '__main__':
    main()
