import network
import time
import ubinascii
import sys
from machine import Pin, I2C
from umqtt.simple import MQTTClient

# Import secrets (Wi-Fi credentials, MQTT broker info)
try:
    from secrets import WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT
except ImportError:
    print("Failed to import secrets. Please ensure 'secrets.py' is in the project root directory.")
    raise

# MQTT topic for temperature and humidity data
MQTT_TOPIC = 'esp32/tempsense'  # Change to your desired topic

# Debug flag
DEBUG = False

# Main loop delay in seconds
LOOP_DELAY = 5  # Adjust data publishing interval as needed
# EMA smoothing factor (0 < EMA_ALPHA <= 1)
EMA_ALPHA = 0.1

# Minimum change thresholds to trigger MQTT publish
TEMP_THRESHOLD = 0.3  # degrees Celsius
HUMIDITY_THRESHOLD = 1.0  # percent RH

# I2C configuration (adjust pins and frequency as needed)
I2C_SCL_PIN = 22  # Example GPIO pin for SCL
I2C_SDA_PIN = 21  # Example GPIO pin for SDA
I2C_FREQ = 100000  # 100kHz standard frequency

def debug_print(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)

def get_unique_client_id():
    wlan = network.WLAN(network.STA_IF)
    mac = wlan.config('mac')
    return b'tempsense_' + ubinascii.hexlify(mac)

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


def init_sensor():
    """
    Initialize the I2C temperature and humidity sensor.
    Replace with your specific sensor initialization code.
    """
    try:
        i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
        # Scan for devices
        devices = i2c.scan()
        debug_print('I2C devices found:', devices)
        # Replace with your sensor's I2C address
        SENSOR_ADDR = 0x44  # Example address for SHT31 sensor
        if SENSOR_ADDR not in devices:
            print("Sensor not found on I2C bus. Check wiring.")
            return None
        # Additional sensor setup can be added here
        return i2c, SENSOR_ADDR
    except Exception as e:
        print("Failed to initialize I2C sensor:", e)
        return None

def read_sensor(i2c, addr):
    """
    Read temperature and humidity from the sensor.
    Replace with your specific sensor reading code.
    Returns (temperature_celsius, humidity_percent)
    """
    try:
        # Example for SHT31 sensor: send measurement command and read data
        # This is placeholder code; replace with your sensor's protocol
        i2c.writeto(addr, b'\x24\x00')  # High repeatability measurement command
        time.sleep(0.015)  # Wait for measurement
        data = i2c.readfrom(addr, 6)
        temp_raw = data[0] << 8 | data[1]
        humidity_raw = data[3] << 8 | data[4]
        temperature = -45 + (175 * (temp_raw / 65535))
        humidity = 100 * (humidity_raw / 65535)
        return temperature, humidity
    except Exception as e:
        debug_print("Failed to read sensor data:", e)
        return None, None

def main():
    wlan = connect_wifi()
    client = connect_mqtt()
    sensor = init_sensor()
    if sensor is None:
        print("Sensor initialization failed. Exiting.")
        return
    i2c, sensor_addr = sensor

    while True:
           # Initialize EMA and last sent values
        ema_temp = None
        ema_humidity = None
        last_sent_temp = None
        last_sent_humidity = None
        # Reconnect Wi-Fi if disconnected
        if not wlan.isconnected():
            debug_print('Wi-Fi disconnected, attempting to reconnect...')
 
        # Reconnect Wi-Fi if disconnected
        if not wlan.isconnected():
            debug_print('Wi-Fi disconnected, attempting to reconnect...')
            wlan = connect_wifi()
        # Reconnect MQTT if disconnected
        if client is None:
            client = connect_mqtt()

        temperature, humidity = read_sensor(i2c, sensor_addr)
        if temperature is not None and humidity is not None:
            # Initialize EMA on first valid reading
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
                debug_print('Publishing:', payload)
                if client:
                    try:
                        client.publish(MQTT_TOPIC.encode(), payload.encode())
                        last_sent_temp = ema_temp
                        last_sent_humidity = ema_humidity
                    except Exception as e:
                        debug_print('Failed to publish MQTT message:', e)
                        client = None  # Force reconnect on next loop
                else:
                    debug_print('MQTT not connected, skipping publish')
            else:
                debug_print('Change below threshold, skipping publish')
        else:
            debug_print('Invalid sensor data, skipping publish')

        time.sleep(LOOP_DELAY)

if __name__ == '__main__':
    main()