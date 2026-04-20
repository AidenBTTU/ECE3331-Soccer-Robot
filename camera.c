#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "pico/bootrom.h"
#include <pico/bootrom.h>

// ===== PIN CONFIGURATION =====
#define SPI_PORT spi0           // SPI channel 0 (camera data)
#define I2C_PORT i2c0           // I2C channel 0 (camera config)

#define PIN_SCK  2              // SPI clock
#define PIN_MOSI 3              // SPI data out (Pico → Camera)
#define PIN_MISO 4              // SPI data in (Camera → Pico)
#define PIN_CS   5              // SPI chip select (enable/disable camera)

#define PIN_SDA  8              // I2C data
#define PIN_SCL  9              // I2C clock

#define CAM_I2C_ADDR 0x30       // Camera I2C address
#define WIDTH 320               // image width in pixels
#define HEIGHT 240              // image height in pixels
#define CROP_HEIGHT (HEIGHT / 2) // only use middle half of frame (120 pixels high)
#define START_ROW ((HEIGHT - CROP_HEIGHT) / 2) // start at row 60
#define MIN_BLOB_AREA 500       // minimum connected pixels to treat as a blob

#define REBOOT_BUTTON 16        // GPIO pin for reboot button

// ===== MEMORY BUFFERS =====
uint8_t frame_buffer[WIDTH * CROP_HEIGHT * 2];  // raw frame data from camera (RGB565 = 2 bytes/pixel)
uint8_t mask_buffer[WIDTH * CROP_HEIGHT];       // mask: 255=color match, 0=no match
uint16_t queue_x[WIDTH * CROP_HEIGHT];          // BFS queue: x coordinates
uint16_t queue_y[WIDTH * CROP_HEIGHT];          // BFS queue: y coordinates


// ===== CAMERA COMMUNICATION =====
// Pull CS LOW (chip select active)
void cs_select() {
    asm volatile("nop \n nop \n nop"); // tiny delay for timing
    gpio_put(PIN_CS, 0);
    asm volatile("nop \n nop \n nop");
}

// Pull CS HIGH (chip select inactive)
void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

// Write to camera register via SPI: send (register | 0x80) and value
void w_reg(uint8_t reg, uint8_t val) {
    cs_select();
    uint8_t buf[2] = {(uint8_t)(reg | 0x80), val};  // 0x80 = write flag
    spi_write_blocking(SPI_PORT, buf, 2);
    cs_deselect();
}

// Read camera register via SPI: send register address (7 bits), read 1 byte back
uint8_t r_reg(uint8_t reg) {
    cs_select();
    uint8_t reg_addr = reg & 0x7F;  // 0x7F = read flag (no high bit)
    spi_write_blocking(SPI_PORT, &reg_addr, 1);
    uint8_t val;
    spi_read_blocking(SPI_PORT, 0, &val, 1);
    cs_deselect();
    return val;
}

// Convert RGB565 pixel to HSV and check if it matches the color range
// c16 = 16-bit RGB565 color, index = pixel index in mask_buffer
// h_min/h_max = hue range (0-360), wraparound for red (e.g., 340-20 includes 350-360 AND 0-20)
static inline void process_pixel_hsv(uint16_t c16, int index, int h_min, int h_max) {
    // Extract R, G, B from RGB565 format (5 bits each, scaled to 0-255)
    uint8_t r = ((c16 >> 11) & 0x1F) << 3;  // bits 15-11 → scale 0-31 to 0-255
    uint8_t g = ((c16 >> 5)  & 0x3F) << 2;  // bits 10-5 → scale 0-63 to 0-255
    uint8_t b = (c16 & 0x1F) << 3;          // bits 4-0 → scale 0-31 to 0-255

    // Find max and min for HSV calculation
    uint8_t mx = r;
    if (g > mx) mx = g;
    if (b > mx) mx = b;

    uint8_t mn = r;
    if (g < mn) mn = g;
    if (b < mn) mn = b;

    // Calculate hue (0-360 degrees)
    uint8_t df = mx - mn;  // saturation depends on this
    int h = 0;

    if (df > 0) {
        // Which color component is largest? Use that to compute hue angle
        if (mx == r) {
            h = (60 * (g - b)) / df;  // red dominant
        } else if (mx == g) {
            h = 120 + ((60 * (b - r)) / df);  // green dominant
        } else {
            h = 240 + ((60 * (r - g)) / df);  // blue dominant
        }
        if (h < 0) h += 360;  // wrap negative angles to 0-360
    }

    // Only accept bright, saturated colors (ignore dull grays)
    if (mx > 50 && df > 20) {  // brightness > 50, saturation > 20
        bool match = false;
        if (h_min < h_max) {
            match = (h >= h_min && h <= h_max);  // normal range
        } else {
            match = (h >= h_min || h <= h_max);  // wraparound range (e.g., red)
        }
        mask_buffer[index] = match ? 255 : 0;  // white if match, black if not
    } else {
        mask_buffer[index] = 0;  // too dull/desaturated → not a color match
    }
}

// ===== CAMERA INITIALIZATION =====
// Configure camera registers for  mode and color settings
void init_cam() {
    printf("[DEBUG] Starting camera init...\n");
   
    // Perform hardware reset
    w_reg(0x07, 0x80);  // pull reset bit
    sleep_ms(100);
    w_reg(0x07, 0x00);  // release reset bit
    sleep_ms(100);

    // Camera register init table: pairs of (register_address, register_value)
    // These configure resolution, color format, autofocus, etc.
    uint8_t regs[][2] = {
        {0xff, 0x01}, {0x12, 0x80}, {0xff, 0x00}, {0x2c, 0xff}, {0x2e, 0xdf},
        {0xff, 0x01}, {0x11, 0x01}, {0x12, 0x00}, {0x3c, 0x32}, {0xff, 0x00},
        {0x44, 0x00}, {0x12, 0x40}, {0x13, 0x00}, {0x11, 0x03}, {0x14, 0x00},
        {0x0c, 0x00}, {0x3e, 0x00}, {0x0d, 0x00}, {0xff, 0x01}, {0x12, 0x40},
        {0x47, 0x01}, {0x4b, 0x09}, {0x10, 0x00}, {0xff, 0x00}, {0xda, 0x08},
        {0xd7, 0x03}, {0xdf, 0x02}, {0x33, 0x40}, {0x3c, 0x00}, {0xba, 0x01},
        {0xbb, 0x20}, {0xff, 0x00}, {0xe0, 0x04}, {0x12, 0x00}, {0x5a, 0x50}, {0x3c, 0x3c}
    };

    // Write each register, with longer waits for reset (0x12, 0x80)
    for (int i = 0; i < sizeof(regs)/sizeof(regs[0]); i++) {
        i2c_write_blocking(I2C_PORT, CAM_I2C_ADDR, regs[i], 2, false);
        if (regs[i][0] == 0x12 && regs[i][1] == 0x80) {
            sleep_ms(50);  // reset takes longer
        } else {
            sleep_ms(2);   // normal delay between regs
        }
    }
   
    // Verify camera is responding: read camera ID (should be 0x26)
    uint8_t test_reg = r_reg(0x0A);
    printf("[DEBUG] Camera ID register (0x0A): 0x%02X (should be 0x26)\n", test_reg);
}

// --- BLOB FINDING AND ANALYSIS ---
void findblobs(const char* targetcolor) {
    w_reg(0x04, 0x01); // clear flag
    sleep_ms(2);
    w_reg(0x04, 0x02); // capture new frame

    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    while (!(r_reg(0x41) & 0x08)) {
        if (to_ms_since_boot(get_absolute_time()) - start_time > 1000) {
            // Retry once after clearing stale FIFO state.
            w_reg(0x04, 0x01);
            sleep_ms(10);
            w_reg(0x04, 0x02);
            start_time = to_ms_since_boot(get_absolute_time());

            while (!(r_reg(0x41) & 0x08)) {
                if (to_ms_since_boot(get_absolute_time()) - start_time > 1000) {
                    printf("[DEBUG] Camera capture timed out!\n");
                    w_reg(0x04, 0x01);
                    return;
                }
            }

            break;
        }
    }

    uint32_t size = r_reg(0x42) | (r_reg(0x43) << 8) | ((r_reg(0x44) & 0x7f) << 16);
    if (size < 5000) {
        printf("[DEBUG] Frame too small, skipping\n");
        w_reg(0x04, 0x01);
        return;
    }

    // --- 1. SPI READ AND MASK GENERATION ---
    // LOGIC CHANGE: Read and compute HSV mask concurrently to save memory and loops.
    cs_select();
    uint8_t cmd = 0x3C;
    spi_write_blocking(SPI_PORT, &cmd, 1);

    uint8_t dummy[100];
    uint32_t skip_bytes = START_ROW * WIDTH * 2;
    for (uint32_t i = 0; i < skip_bytes; i += 100) {
        spi_read_blocking(SPI_PORT, 0, dummy, (skip_bytes - i < 100) ? skip_bytes - i : 100);
    }

    int h_min = 0, h_max = 0;
    if (strcmp(targetcolor, "RED") == 0)   { h_min = 340; h_max = 20;  }
    else if (strcmp(targetcolor, "GREEN") == 0) { h_min = 90;  h_max = 150; }
    else if (strcmp(targetcolor, "BLUE") == 0)  { h_min = 200; h_max = 260; }

    uint8_t pixel_buf[2];
    int masked_count = 0;
    for (int i = 0; i < WIDTH * CROP_HEIGHT; i++) {
        spi_read_blocking(SPI_PORT, 0, pixel_buf, 2);
        uint16_t c16 = (pixel_buf[0] << 8) | pixel_buf[1];
        process_pixel_hsv(c16, i, h_min, h_max);
        if (mask_buffer[i] == 255) masked_count++;
    }
    cs_deselect();
   
    // --- 2. CONNECTED COMPONENT LABELING (Blob Detection) ---
    // LOGIC CHANGE: Replaced cv2 with a BFS algorithm. Extremely fast and lightweight.
    int best_area = 0;
    int best_cx = 0, best_cy = 0;
    float best_roundness = 0.0f;
    int blob_count = 0;
    int largest_area = 0;

    for (int y = 0; y < CROP_HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            int idx = y * WIDTH + x;
           
            if (mask_buffer[idx] == 255) { // Unvisited valid pixel
                // Track bounding box
                int min_x = x, max_x = x, min_y = y, max_y = y;
               
                int head = 0, tail = 0;
               
                queue_x[tail] = x;
                queue_y[tail] = y;
                tail++;
                mask_buffer[idx] = 127; // Mark as visited

                int area = 0;
                int perimeter = 0;
                long sum_x = 0, sum_y = 0;

                while (head < tail) {
                    int cx = queue_x[head];
                    int cy = queue_y[head];
                    head++;

                    area++;
                    sum_x += cx;
                    sum_y += cy;
                   
                    // Update bounding box
                    if (cx < min_x) min_x = cx;
                    if (cx > max_x) max_x = cx;
                    if (cy < min_y) min_y = cy;
                    if (cy > max_y) max_y = cy;

                    // Check 4 neighbors
                    int dx[] = {0, 0, -1, 1};
                    int dy[] = {-1, 1, 0, 0};
                   
                    for (int i = 0; i < 4; i++) {
                        int nx = cx + dx[i];
                        int ny = cy + dy[i];

                        if (nx >= 0 && nx < WIDTH && ny >= 0 && ny < CROP_HEIGHT) {
                            int n_idx = ny * WIDTH + nx;
                            if (mask_buffer[n_idx] == 255) { // Found connected pixel
                                mask_buffer[n_idx] = 127; // Mark visited
                                queue_x[tail] = nx;
                                queue_y[tail] = ny;
                                tail++;
                            } else if (mask_buffer[n_idx] == 0) {
                                perimeter++; // Touches empty space = edge
                            }
                        } else {
                            perimeter++; // Touches boundary of crop region = edge
                        }
                    }
                }

                if (area > largest_area) {
                    largest_area = area;
                }

                if (area > MIN_BLOB_AREA) {
                    int bbox_width = max_x - min_x + 1;
                    int bbox_height = max_y - min_y + 1;
                    int bbox_area = bbox_width * bbox_height;
                    float aspect_ratio = (float)bbox_width / bbox_height;
                    if (aspect_ratio < 1.0f) aspect_ratio = 1.0f / aspect_ratio;
                    float solidity = (float)area / bbox_area;
                   
                    //printf("[DEBUG] Blob: area=%d, bbox=%dx%d, aspect=%.2f, solidity=%.2f\r\n",area, bbox_width, bbox_height, aspect_ratio, solidity);
                    if (aspect_ratio < 1.5f && solidity > 0.17f && solidity < 0.85f && area > best_area) {
                        best_area = area;
                        best_cx = sum_x / area;
                        best_cy = (sum_y / area) + START_ROW;
                        best_roundness = solidity;
                    } else if (aspect_ratio >= 1.5f) {
                        printf(" -> too elongated\n");
                    } else if (solidity < 0.17f) {
                        printf(" -> too hollow\n");
                    } else if (solidity > 0.85f) {
                        printf(" -> too solid (probably square)\n");
                    }
                    blob_count++;
                }
            }
        }
    }
    if (best_area > 0) {
        printf("SUCCESS: target: %s | x:%d y:%d | size:%d | roundness:%.2f\n",
               targetcolor, best_cx, best_cy, best_area, best_roundness);
    }
}

void setup() {
    stdio_init_all();  // enable serial output (USB)

    // ===== REBOOT BUTTON =====
    gpio_init(REBOOT_BUTTON);           // initialize GPIO 15
    gpio_set_dir(REBOOT_BUTTON, GPIO_IN); // set as input
    gpio_pull_up(REBOOT_BUTTON);        // pull-up so normal state is HIGH, button press = LOW
   
    // ===== SPI INIT (Camera data) =====
    spi_init(SPI_PORT, 4000000);        // SPI at 4MHz
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);   // clock
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);  // data out
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);  // data in
   
    gpio_init(PIN_CS);                  // chip select (manual control)
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);                // start HIGH (chip deselected)

    // ===== I2C INIT (Camera config) =====
    i2c_init(I2C_PORT, 50000);          // I2C at 50kHz
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);              // enable pull-ups
    gpio_pull_up(PIN_SCL);

    sleep_ms(100);                      // let buses stabilize
}

int main() {
    setup();  // initialize all hardware (SPI, I2C, GPIO)
   
    // Give user time to open serial monitor before spamming output
    printf("Starting...\n");
    sleep_ms(5000);
   
    init_cam();  // configure camera registers
    printf("Cam init done\n");
    printf("Camera initialized...\n");
    sleep_ms(500);

    // ===== MAIN LOOP =====
    while (true) {
        // Check reboot button: if pressed (GPIO low), reboot into bootloader
        if (!gpio_get(REBOOT_BUTTON)) {
            printf("Rebooting to bootloader...\n");
            sleep_ms(500);              // let USB flush before reboot
            reset_usb_boot(0, 0);       // reboot into UF2 bootloader mode (drive appears)
        }
   
        printf("working..\r\n");
     
        // Capture one frame and search for each color in sequence
        findblobs("RED");
        findblobs("GREEN");
        findblobs("BLUE");
        
        sleep_ms(1000);  // wait 1 second before next frame
    }
    
    return 0;
}