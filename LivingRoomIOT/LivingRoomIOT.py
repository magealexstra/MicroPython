import network
import time
import ubinascii
import sys
from machine import Pin, I2C, reset
from umqtt.simple import MQTTClient

# Import secrets (Wi-Fi credentials, MQTT broker info)
try:
    from secrets import WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT
except ImportError:
    print("Failed to import secrets. Please ensure 'secrets.py' is in the project directory.")
    raise

# Debug flag
DEBUG = False

# MQTT topics
MQTT_TOPIC_SENSOR = 'livingroom/temperature_humidity'
MQTT_TOPIC_DOOR = 'livingroom/door'
RELAY_COMMAND_TOPIC_TEMPLATE = 'livingroom/relay/{}/command'
RELAY_STATE_TOPIC_TEMPLATE = 'livingroom/relay/{}/state'

# I2C configuration for SHT31 sensor
I2C_SCL_PIN = 22
I2C_SDA_PIN = 21
I2C_FREQ = 100000
SENSOR_ADDR = 0x44  # SHT31 default address

# EMA smoothing factor
EMA_ALPHA = 0.1
TEMP_THRESHOLD = 0.3
HUMIDITY_THRESHOLD = 1.0

# GPIO pin configuration
RELAY_PINS = [5, 18, 19, 23]  # Example relay pins
BUTTON_PINS = [4, 15, 16, 17]  # Example button pins
DOOR_SENSOR_PIN = 2  # Example door sensor pin

# Debounce settings
BUTTON_DEBOUNCE_MS = 300
DOOR_DEBOUNCE_MS = 300

# Loop delay
LOOP_DELAY = 1

# Maximum connection retries before reset
MAX_CONNECTION_RETRIES = 10

# Globals
mqtt_client = None
relays = {}
buttons = {}
last_button_states = {}
last_button_times = {}
door_sensor = None
last_door_state = None
last_door_time = 0

ema_temp = None
ema_humidity = None
last_sent_temp = None
last_sent_humidity = None

def debug_print(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)

def get_unique_client_id():
    wlan = network.WLAN(network.STA_IF)
    mac = wlan.config('mac')  # raw bytes
    mac_hex = ubinascii.hexlify(mac).decode()  # convert to hex string
    short_mac = mac_hex[-5:]  # get last 5 characters
    return ('livingroom_' + short_mac).encode()  # MQTT client ID must be bytes

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

def mqtt_callback(topic, msg):
    try:
        topic_str = topic.decode()
        debug_print('Received message on topic {}: {}'.format(topic_str, msg))
        # Check if it's a relay command topic
        for relay_pin in RELAY_PINS:
            expected_topic = RELAY_COMMAND_TOPIC_TEMPLATE.format(relay_pin)
            if topic_str == expected_topic:
                command = msg.decode().strip().upper()
                if command == 'ON':
                    relays[relay_pin].value(1)
                elif command == 'OFF':
                    relays[relay_pin].value(0)
                publish_relay_state(relay_pin)
    except Exception as e:
        sys.print_exception(e)
        debug_print('Error in MQTT callback:', e)

def connect_mqtt():
    global mqtt_client
    try:
        client_id = get_unique_client_id()
        client = MQTTClient(client_id, MQTT_BROKER, port=MQTT_PORT)
        client.set_last_will(MQTT_TOPIC_SENSOR.encode(), b'OFFLINE', retain=True, qos=0)
        client.set_callback(mqtt_callback)
        client.connect()
        debug_print('Connected to MQTT broker at {}:{}'.format(MQTT_BROKER, MQTT_PORT))
        # Publish ONLINE status
        client.publish(MQTT_TOPIC_SENSOR.encode(), b'ONLINE', retain=True, qos=0)
        # Subscribe to relay command topics
        for relay_pin in RELAY_PINS:
            topic = RELAY_COMMAND_TOPIC_TEMPLATE.format(relay_pin)
            client.subscribe(topic.encode())
            debug_print('Subscribed to topic:', topic)
        mqtt_client = client
        # Publish initial relay states
        for relay_pin in RELAY_PINS:
            publish_relay_state(relay_pin)
        return client
    except Exception as e:
        sys.print_exception(e)
        debug_print('Failed to connect to MQTT broker:', e)
        return None

def publish_relay_state(relay_pin):
    global mqtt_client
    if mqtt_client is None:
        return
    try:
        state = 'ON' if relays[relay_pin].value() else 'OFF'
        topic = RELAY_STATE_TOPIC_TEMPLATE.format(relay_pin)
        mqtt_client.publish(topic.encode(), state.encode(), retain=True, qos=0)
        debug_print('Published relay {} state: {}'.format(relay_pin, state))
    except Exception as e:
        sys.print_exception(e)
        debug_print('Failed to publish relay state:', e)
        mqtt_client = None

def init_relays():
    global relays
    for pin_num in RELAY_PINS:
        pin = Pin(pin_num, Pin.OUT)
        pin.value(0)  # Start OFF
        relays[pin_num] = pin
        debug_print('Relay initialized on GPIO', pin_num)

def init_buttons():
    global buttons, last_button_states, last_button_times
    for pin_num in BUTTON_PINS:
        pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)
        buttons[pin_num] = pin
        last_button_states[pin_num] = pin.value()
        last_button_times[pin_num] = 0
        debug_print('Button initialized on GPIO', pin_num)

def init_door_sensor():
    global door_sensor, last_door_state, last_door_time
    door_sensor = Pin(DOOR_SENSOR_PIN, Pin.IN, Pin.PULL_UP)
    last_door_state = door_sensor.value()
    last_door_time = time.ticks_ms()
    debug_print('Door sensor initialized on GPIO', DOOR_SENSOR_PIN)

def init_sensor():
    try:
        i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
        devices = i2c.scan()
        debug_print('I2C devices found:', devices)
        if SENSOR_ADDR not in devices:
            print("SHT31 sensor not found. Check wiring.")
            return None, None
        return i2c, SENSOR_ADDR
    except Exception as e:
        sys.print_exception(e)
        print("Failed to initialize I2C sensor:", e)
        return None, None

def read_sensor(i2c, addr):
    try:
        i2c.writeto(addr, b'\x24\x00')
        time.sleep(0.015)
        data = i2c.readfrom(addr, 6)
        temp_raw = data[0] << 8 | data[1]
        humidity_raw = data[3] << 8 | data[4]
        temperature = -45 + (175 * (temp_raw / 65535))
        humidity = 100 * (humidity_raw / 65535)
        return temperature, humidity
    except Exception as e:
        sys.print_exception(e)
        debug_print("Failed to read sensor data:", e)
        return None, None

def check_buttons():
    current_time = time.ticks_ms()
    for pin_num, button in buttons.items():
        current_state = button.value()
        if current_state != last_button_states[pin_num]:
            if time.ticks_diff(current_time, last_button_times[pin_num]) > BUTTON_DEBOUNCE_MS:
                if current_state == 0:  # Button pressed (active low)
                    relay_pin = RELAY_PINS[BUTTON_PINS.index(pin_num)]
                    new_state = 0 if relays[relay_pin].value() else 1
                    relays[relay_pin].value(new_state)
                    publish_relay_state(relay_pin)
                    debug_print('Button {} pressed, toggled relay {}'.format(pin_num, relay_pin))
                last_button_times[pin_num] = current_time
            last_button_states[pin_num] = current_state

def check_door_sensor():
    global last_door_state, last_door_time
    current_time = time.ticks_ms()
    current_state = door_sensor.value()
    if current_state != last_door_state:
        if time.ticks_diff(current_time, last_door_time) > DOOR_DEBOUNCE_MS:
            state_str = 'OPEN' if current_state else 'CLOSED'
            try:
                if mqtt_client:
                    mqtt_client.publish(MQTT_TOPIC_DOOR.encode(), state_str.encode(), retain=True, qos=0)
                    debug_print('Door sensor state published:', state_str)
            except Exception as e:
                sys.print_exception(e)
                debug_print('Failed to publish door sensor state:', e)
            last_door_time = current_time
        last_door_state = current_state

def main():
    global mqtt_client, ema_temp, ema_humidity, last_sent_temp, last_sent_humidity

    wlan = connect_wifi()
    mqtt_client = connect_mqtt()

    init_relays()
    init_buttons()
    init_door_sensor()

    i2c, sensor_addr = init_sensor()
    if i2c is None:
        print("Sensor initialization failed. Continuing without sensor data.")

    # Connection retry counters and backoff time
    wifi_retry_count = 0
    mqtt_retry_count = 0
    backoff_time = LOOP_DELAY

    while True:
        # Reconnect Wi-Fi if disconnected
        if not wlan.isconnected():
            wifi_retry_count += 1
            debug_print('Wi-Fi disconnected, attempting to reconnect... (Attempt {}/{})'.format(
                wifi_retry_count, MAX_CONNECTION_RETRIES))
            
            wlan = connect_wifi() # Attempt reconnection

            if not wlan.isconnected():
                # Apply exponential backoff
                backoff_time = min(30, LOOP_DELAY * (2 ** wifi_retry_count))
                debug_print('Wi-Fi reconnection failed. Waiting {} seconds before retry'.format(backoff_time))
                time.sleep(backoff_time)

                # Reset device after too many failed attempts
                if wifi_retry_count >= MAX_CONNECTION_RETRIES:
                    print('Failed to reconnect to Wi-Fi after {} attempts. Restarting device...'.format(MAX_CONNECTION_RETRIES))
                    time.sleep(1)
                    reset() # Call reset directly
            else:
                # Reset counter and backoff on successful connection
                wifi_retry_count = 0
                backoff_time = LOOP_DELAY

        # Reconnect MQTT if disconnected
        if mqtt_client is None:
            mqtt_retry_count += 1
            debug_print('MQTT disconnected, attempting to reconnect... (Attempt {}/{})'.format(
                mqtt_retry_count, MAX_CONNECTION_RETRIES))
            
            mqtt_client = connect_mqtt() # Attempt reconnection

            if mqtt_client is None:
                # Apply exponential backoff
                backoff_time = min(30, LOOP_DELAY * (2 ** mqtt_retry_count))
                debug_print('MQTT reconnection failed. Waiting {} seconds before retry'.format(backoff_time))
                time.sleep(backoff_time)

                # Reset device after too many failed attempts
                if mqtt_retry_count >= MAX_CONNECTION_RETRIES:
                    print('Failed to reconnect to MQTT after {} attempts. Restarting device...'.format(MAX_CONNECTION_RETRIES))
                    time.sleep(1)
                    reset() # Call reset directly
            else:
                # Reset counter and backoff on successful connection
                mqtt_retry_count = 0
                backoff_time = LOOP_DELAY

        # Read and publish sensor data only if connected
        if i2c and wlan.isconnected() and mqtt_client is not None:
            temperature, humidity = read_sensor(i2c, sensor_addr)
            if temperature is not None and humidity is not None:
                if ema_temp is None:
                    ema_temp = temperature
                else:
                    ema_temp = EMA_ALPHA * temperature + (1 - EMA_ALPHA) * ema_temp

                if ema_humidity is None:
                    ema_humidity = humidity
                else:
                    ema_humidity = EMA_ALPHA * humidity + (1 - EMA_ALPHA) * ema_humidity

                temp_change = abs((last_sent_temp if last_sent_temp is not None else ema_temp) - ema_temp)
                humidity_change = abs((last_sent_humidity if last_sent_humidity is not None else ema_humidity) - ema_humidity)

                if (temp_change >= TEMP_THRESHOLD) or (humidity_change >= HUMIDITY_THRESHOLD):
                    payload = '{{"temperature": {:.2f}, "humidity": {:.2f}}}'.format(ema_temp, ema_humidity)
                    debug_print('Publishing sensor data:', payload)
                    if mqtt_client:
                        try:
                            mqtt_client.publish(MQTT_TOPIC_SENSOR.encode(), payload.encode())
                            last_sent_temp = ema_temp
                            last_sent_humidity = ema_humidity
                        except Exception as e:
                            sys.print_exception(e)
                            debug_print('Failed to publish sensor data:', e)
                            mqtt_client = None
            else:
                debug_print('Invalid sensor data, skipping publish')

        # Check buttons
        check_buttons()

        # Check door sensor
        check_door_sensor()

        # Check for MQTT messages
        if mqtt_client:
            try:
                mqtt_client.check_msg()
            except Exception as e:
                sys.print_exception(e)
                debug_print('MQTT error:', e)
                mqtt_client = None

        time.sleep(LOOP_DELAY)

if __name__ == '__main__':
    main()
