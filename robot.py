#from servo import *
import time
time.sleep(3)
from camera import *
from machine import I2C, Pin
import gc
#from colorsensor import *
#from hcsr04 import *

color_list = ["GREEN", "RED", "BLUE"] # the three colors to cycle through in order
i = 0 # start at index 0 = "RED"

def run_robot():
    # 1. HUGE SLEEP AT THE START
    # This gives Windows/Thonny 10 full seconds to claim the COM port 
    # and settle down before we touch the SPI bus.
    print("USB Connecting... Wait 3 seconds.")
    time.sleep(3) 
    
    color_list = ["GREEN", "RED", "BLUE"]
    i = 0 
    
    # Create buffers inside the function
    frame_buffer = bytearray(80000)
    mask_buffer = bytearray(40000)
    
    # 2. Slow down I2C slightly for stability on the new RP2350 chip
    i2c = I2C(1, sda=Pin(14, Pin.IN, Pin.PULL_UP), scl=Pin(15, Pin.IN, Pin.PULL_UP), freq=100000)
    print("I2C Scan:", i2c.scan())
    
    # 3. Wake up the camera LAST
    time.sleep(1)
    boot_camera()
    
    print("Starting Main Loop. If it freezes here, the SPI bus is clashing with USB.")
    
    while True:
            try:
                target = color_list[i]
                findblobs(target, frame_buffer, mask_buffer) 
                i = (i + 1) % 3
                
                gc.collect()
                # Give the USB driver 'breathing room' between frames
                time.sleep(0.5) 
                
            except KeyboardInterrupt:
                print("Stopping robot...")
                # servo.stop() 
                break
            except Exception as e:
                print("Error:", e)
                break
            except MemoryError:
                print("memory low...")
                gc.collect()
                time.sleep(.5)
                break

run_robot()
'''
while True:
    try:
        
        
        detected = detect_color(sensor)
        if detected is not None:
            print("detected:", detected)
            sort(detected)
        
        distance = distsensor.distance_mm()
        if distance is not -1:
            if distance < 50:
                print('STOP')
            print('Distance: {} mm'.format(distance))
        
            
        gc.collect()
        time.sleep(0.5)
        
        
    except KeyboardInterrupt:
        print("Stopping servo...")
        servo.stop()
        break
        
    except MemoryError:
        print("memory low...")
        gc.collect()
        time.sleep(.5)
        break
'''