import network
import time
import ubinascii
import sys
from machine import Pin
from umqtt.simple import MQTTClient

# Import secrets
try:
    from secrets import WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT
except ImportError:
    print("Failed to import secrets. Please ensure 'Secrets/secrets.py' is uploaded to the device.")
    raise

# MQTT topic
MQTT_TOPIC = 'esp32/breaksense'

# GPIO pin number
GPIO_PIN = 2

# Debug flag
DEBUG = False

# Debounce time in seconds
DEBOUNCE_TIME = 0.2  # 2 milliseconds

# Main loop delay in seconds
LOOP_DELAY = 0.1

def debug_print(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)

def get_unique_client_id():
    wlan = network.WLAN(network.STA_IF)
    mac = wlan.config('mac')
    return b'breaksense_' + ubinascii.hexlify(mac)

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('Connecting to Wi-Fi...')
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        while not wlan.isconnected():
            time.sleep(0.5)
    print('Connected to Wi-Fi:', wlan.ifconfig())
    return wlan

def connect_mqtt():
    try:
        client_id = get_unique_client_id()
        client = MQTTClient(client_id, MQTT_BROKER, port=MQTT_PORT)
        # Set Last Will and Testament
        client.set_last_will(MQTT_TOPIC.encode(), b'OFFLINE', retain=True, qos=0)
        client.connect()
        debug_print('Connected to MQTT broker at {}:{}'.format(MQTT_BROKER, MQTT_PORT))
        # Publish ONLINE status
        client.publish(MQTT_TOPIC.encode(), b'ONLINE', retain=True, qos=0)
        return client
    except Exception as e:
        debug_print('Failed to connect to MQTT broker:', e)
        return None

def main():
    wlan = connect_wifi()
    client = connect_mqtt()

    pin = Pin(GPIO_PIN, Pin.IN, Pin.PULL_UP)
    last_state = pin.value()
    last_debounce_time = time.ticks_ms()
    debug_print('Initial GPIO{} state: {}'.format(GPIO_PIN, last_state))

    while True:
        # Attempt to reconnect Wi-Fi if disconnected
        if not wlan.isconnected():
            debug_print('Wi-Fi disconnected, attempting to reconnect...')
            wlan = connect_wifi()

        # Attempt to reconnect MQTT if not connected
        if client is None:
            client = connect_mqtt()

        current_state = pin.value()
        now = time.ticks_ms()

        if current_state != last_state:
            # Debounce: wait for the state to be stable for DEBOUNCE_TIME
            last_debounce_time = now

        if time.ticks_diff(now, last_debounce_time) > int(DEBOUNCE_TIME * 100):
            stable_state = pin.value()
            if stable_state != last_state:
                msg = b'HIGH' if stable_state else b'LOW'
                if client:
                    try:
                        client.publish(MQTT_TOPIC.encode(), msg)
                        debug_print('Published:', msg)
                    except Exception as e:
                        debug_print('Failed to publish MQTT message:', e)
                        client = None  # Force reconnect on next loop
                else:
                    debug_print('MQTT not connected, skipping publish')
                last_state = stable_state

        time.sleep(LOOP_DELAY)

if __name__ == '__main__':
    main()