# test_pickup_rate.py
from pidog import Pidog
import time
import math

dog = Pidog()
time.sleep(1)
dog.do_action('stand', speed=50)
dog.wait_all_done()

print("Pickup Test - Lift the dog and put down")
print("=" * 60)

last_az = None
last_time = None

try:
    while True:
        ax_raw, ay_raw, az_raw = dog.accData
        ax = ax_raw / 16384.0
        ay = ay_raw / 16384.0
        az = az_raw / 16384.0
        
        vertical_g = -ax
        forward_g = az
        right_g = ay
        
        magnitude = math.sqrt(forward_g*forward_g + right_g*right_g + vertical_g*vertical_g)
        
        # Calculate rate of change
        current_time = time.time()
        if last_az is not None and last_time is not None:
            dt = current_time - last_time
            if dt > 0:
                rate = (vertical_g - last_az) / dt
                print(f"up={vertical_g:6.3f}g, rate={rate:6.2f}g/s, mag={magnitude:6.3f}g")
        
        last_az = vertical_g
        last_time = current_time
        time.sleep(0.05)
        
except KeyboardInterrupt:
    print("\nTest complete")
finally:
    dog.close()