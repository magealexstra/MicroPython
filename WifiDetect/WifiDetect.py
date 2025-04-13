"""
WiFi Scanner for ESP8266 with OLED Display
Displays network names and signal strength (2.4GHz only due to ESP8266 hardware limitation)
"""
import network
import time
from machine import Pin, I2C
import ssd1306
import gc

# Enable aggressive garbage collection
gc.enable()
gc.collect()

# Pin definitions for OLED
SDA_PIN = 5  # D1 on ESP8266
SCL_PIN = 4  # D2 on ESP8266

# Scan interval 
SCAN_INTERVAL = 10  # Increased to reduce memory pressure

def main():
    try:
        # Initialize display
        i2c = I2C(scl=Pin(SCL_PIN), sda=Pin(SDA_PIN))
        display = ssd1306.SSD1306_I2C(128, 64, i2c)
        
        # Initialize WiFi
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        
        # Quick startup message
        display.fill(0)
        display.text("WiFi Scanner", 20, 25, 1)
        display.show()
        time.sleep(1)
        
        # Main loop
        last_scan_time = 0
        current_networks = []  # Store networks to keep display between scans
        
        while True:
            # Force memory cleanup
            gc.collect()
            
            # Check if scan is needed
            current_time = time.time()
            if current_time - last_scan_time >= SCAN_INTERVAL:
                try:
                    # Scan networks silently
                    gc.collect()
                    
                    # Do the scan (ESP8266 only supports 2.4GHz)
                    scan_results = wlan.scan()
                    
                    # Process results one by one to avoid memory pressure
                    networks = []
                    for net in scan_results:
                        ssid = net[0].decode('utf-8') if net[0] else "[Hidden]"
                        rssi = net[3]
                        # Signal percentage calculation
                        signal = min(99, max(0, 2 * (rssi + 100)))
                        networks.append((ssid, signal))
                    
                    # Sort by signal strength
                    networks.sort(key=lambda x: x[1], reverse=True)
                    
                    # Update current networks
                    current_networks = networks
                    last_scan_time = current_time
                    
                except Exception as e:
                    # Simple error display
                    display.fill(0)
                    display.text("Scan Error:", 0, 0, 1)
                    display.text(str(e)[:16], 0, 15, 1)
                    display.show()
            
            # Always update display with current networks
            try:
                # Clear display
                display.fill(0)
                
                # Get at most 4 networks to display
                display_count = min(4, len(current_networks))
                
                # Display each network
                for i in range(display_count):
                    net = current_networks[i]
                    name = net[0][:10]  # Limit name to fit
                    signal = net[1]
                    
                    # Calculate Y position - leave more space between lines
                    y_pos = i * 15
                    
                    # Display network name and signal
                    display.text(f"{name}", 0, y_pos, 1)
                    display.text(f"{signal}%", 90, y_pos, 1)
                
                # Update display
                display.show()
                
            except Exception as e:
                # Display error without clearing screen completely
                display.text("Err:" + str(e)[:8], 0, 55, 1)
                display.show()
            
            # Sleep between checks
            time.sleep(0.2)
    
    except Exception as e:
        print("Fatal error:", e)

if __name__ == "__main__":
    main()
