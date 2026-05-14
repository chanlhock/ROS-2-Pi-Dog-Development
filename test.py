from robot_hat import ADC
import time

# Initialize ADC for battery (usually on a high-number channel)
battery_adc = ADC("A4") # Some V4/V5 boards use "A4" for battery check

while True:
    try:
        voltage = battery_adc.read_voltage()
        print(f"Battery Voltage: {voltage:.2f}V")
        time.sleep(1)
    except Exception as e:
        print(f"✗ Failed to initialize battery ADC: {e}")
