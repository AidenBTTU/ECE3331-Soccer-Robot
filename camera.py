import machine
import time
import gc
import cv2 as cv
from ulab import numpy as np
import micropython
spi = None
i2c = None
cs = None

@micropython.viper
def hsv_mask(src: ptr8, dst: ptr8, start: int, end: int, h_min: int, h_max: int):
    for i in range(start, end):
        idx = i * 2
        # Fast bit-shifting extraction
        c16 = (src[idx] << 8) | src[idx + 1]
        r = int((c16 >> 11) & 0x1F) << 3
        g = int((c16 >> 5) & 0x3F) << 2
        b = int(c16 & 0x1F) << 3

        mx = r
        if g > mx: mx = g
        if b > mx: mx = b

        mn = r
        if g < mn: mn = g
        if b < mn: mn = b

        df = mx - mn 
        h = 0

        if df > 0:
            if mx == r:
                h = (60 * (g - b)) // df
            elif mx == g:
                h = 120 + ((60 * (b - r)) // df)
            else:
                h = 240 + ((60 * (r - g)) // df)

            if h < 0:
                h += 360

        if mx > 50 and df > 20:
            if h_min < h_max:
                if h >= h_min and h <= h_max:
                    dst[i] = 255
                else:
                    dst[i] = 0
            else:
                if h >= h_min or h <= h_max:
                    dst[i] = 255
                else:
                    dst[i] = 0
        else:
            dst[i] = 0

def setup():
    global spi, i2c, cs
    spi = machine.SPI(0,baudrate=4000000,sck=machine.Pin(2),mosi=machine.Pin(3),miso=machine.Pin(4))
    i2c = machine.I2C(0,sda=machine.Pin(8),scl=machine.Pin(9),freq=50000)

    time.sleep_ms(100) # wait 100ms for the buses to stabilize

    cs = machine.Pin(5, machine.Pin.OUT)
    cs.value(1)

def w_reg(reg, val):
    cs.value(0)
    spi.write(bytes([reg | 0x80, val])) # set write bit
    cs.value(1)

def r_reg(reg):
    cs.value(0)
    spi.write(bytes([reg & 0x7F])) # set read bit
    val = spi.read(1)[0] # read 1 byte back, grab the value from the returned list
    cs.value(1)
    return val

def init_cam():
    w_reg(0x07, 0x80) # set hardware reset bit
    time.sleep_ms(100)
    w_reg(0x07, 0x00) # clear  bit
    time.sleep_ms(100)

    regs = [
        (0xff, 0x01), (0x12, 0x80), (0xff, 0x00), (0x2c, 0xff), (0x2e, 0xdf),
        (0xff, 0x01), (0x11, 0x01), (0x12, 0x00), (0x3c, 0x32), (0xff, 0x00),
        (0x44, 0x00), (0x12, 0x40), (0x13, 0x00), (0x11, 0x03), (0x14, 0x00),
        (0x0c, 0x00), (0x3e, 0x00), (0x0d, 0x00), (0xff, 0x01), (0x12, 0x40),
        (0x47, 0x01), (0x4b, 0x09), (0x10, 0x00), (0xff, 0x00), (0xda, 0x08),
        (0xd7, 0x03), (0xdf, 0x02), (0x33, 0x40), (0x3c, 0x00), (0xba, 0x01),
        (0xbb, 0x20), (0xff, 0x00), (0xe0, 0x04), (0x12, 0x00), (0x5a, 0x50), (0x3c, 0x3c)]

    for r, v in regs:
        i2c.writeto_mem(0x30, r, bytes([v]))
        if r == 0x12 and v == 0x80:
            time.sleep_ms(50)
        else:
            time.sleep_ms(2)

def findblobs(targetcolor, frame_buffer, mask_buffer):
    w_reg(0x04, 0x01) # clear flag
    w_reg(0x04, 0x02) # capture new frame

    start_time = time.ticks_ms()
    while not (r_reg(0x41) & 0x08): # capture is done when bit 3 is set
        if time.ticks_diff(time.ticks_ms(), start_time) > 1000: # if we've waited more than 1 second...
            print("[DEBUG] Camera capture timed out!")
            return # ...camera is stuck, give up on this frame

    size = r_reg(0x42)|(r_reg(0x43) << 8)|((r_reg(0x44) & 0x7f) << 16)

    if size < 5000: # if the image is too small, skip
        w_reg(0x04, 0x01)
        return

    width = 320
    height = 240

    crop_height = height // 2 # only get middle slit of frame
    start_row = (height - crop_height) // 2

    ptot = width * crop_height

    # --- 1. CHUNKED SPI READ ---
    cs.value(0) 
    spi.write(bytes([0x3C])) 
    
    skip_bytes = start_row * width * 2
    chunk_size = 10000
    tmp_buf = bytearray(chunk_size)
    
    # Skip the top rows
    for _ in range(0, skip_bytes, chunk_size):
        spi.readinto(tmp_buf)
        time.sleep_us(100) # Give USB breathing room!
        
    # Read the middle rows we actually want
    target_bytes = crop_height * width * 2
    for offset in range(0, target_bytes, chunk_size):
        spi.readinto(memoryview(frame_buffer)[offset : offset + chunk_size])
        time.sleep_us(100) # Give USB breathing room!
        
    cs.value(1) 

    # --- 2. SET TARGET HUE ---
    if targetcolor == "RED":
        h_min, h_max = 340, 20
    elif targetcolor == "GREEN":
        h_min, h_max = 90, 150
    elif targetcolor == "BLUE":
        h_min, h_max = 200, 260
    else:
        h_min, h_max = 0, 0

    # --- 3. CHUNKED VIPER MATH ---
    chunk_pixels = 5000
    for offset in range(0, ptot, chunk_pixels):
        end_idx = offset + chunk_pixels
        if end_idx > ptot:
            end_idx = ptot # Catch the final uneven chunk
            
        hsv_mask(frame_buffer, mask_buffer, offset, end_idx, h_min, h_max)
        time.sleep_us(100) # Give USB breathing room!

    # --- 4. FIND CONTOURS ---
    maskedimg = np.frombuffer(memoryview(mask_buffer)[0:ptot], dtype=np.uint8).reshape((crop_height, width))
    contours = cv.findContours(maskedimg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE )[0]

    best = None
    max_area = 0

    for i in contours:
        area = cv.contourArea(i)
        
        # Only print details for blobs that aren't tiny specs of noise

        if area > 500:
            perimeter = cv.arcLength(i, True) # treat as closed shape
            if perimeter == 0: continue # avoiding divide by zero

            radius = perimeter / (2 * 3.14159) # estimate radius as if the blob were a perfect circle
            if radius > 0:
                roundness = area / (3.14159 *(radius**2))

                if 0.79 < roundness < 1.5 and area > max_area:
                    max_area = area
                    M = cv.moments(i)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"]) + start_row # shift
                        best = (targetcolor, cx, cy, max_area, roundness)
                elif area > max_area:
                    print("  -> Blob REJECTED: Not round enough (needs to be between 0.79 and 1.5).")


    if best != None: #if there is a blob
        color, cx, cy, final_size, roundness = best
        mem = gc.mem_alloc() // 1000
        memtot = (gc.mem_free() // 1000) + mem
        print(f"SUCCESS: target: {color} | x:{cx} y:{cy} | size:{final_size} | roundness:{roundness:.2f} | mem:{mem}kb / {memtot}kb")
    
    del maskedimg #FREE RAM
    del contours
    gc.collect()
    


def boot_camera():
    while True:
        setup() # set up pins
        init_cam() # config registers
        print("camera working...")
        time.sleep(.5)
        return