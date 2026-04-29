#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "camera.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "pico/bootrom.h"
#include <pico/bootrom.h>

// ===== PIN CONFIGURATION =====
#define SPI_PORT spi0           // SPI channel 0 (camera data)
#define I2C_PORT i2c0           // I2C channel 0 (camera config)

#define PIN_CS   5              // SPI chip select (enable/disable camera)
#define PIN_MOSI 3              // SPI data out (Pico → Camera)
#define PIN_MISO 4              // SPI data in (Camera → Pico)
#define PIN_SCK  2              // SPI clock

#define PIN_SDA  8              // I2C data
#define PIN_SCL  9              // I2C clock

#define CAM_I2C_ADDR 0x30       // Camera I2C address
#define WIDTH 320               // image width in pixels
#define HEIGHT 240              // image height in pixels
#define PIXEL_COUNT (WIDTH * HEIGHT)
#define DIAG_I2C_TIMEOUT_US 50000
#define MIN_BLOB_AREA 500       // minimum connected pixels to treat as a blob
#define MIN_BALL_BLOB_AREA 900  // stricter minimum for RGB balls

// ===== MEMORY BUFFERS =====
uint8_t mask_buffer[PIXEL_COUNT];  // mask: 255=color match, 0=no match
uint8_t hue_buffer[PIXEL_COUNT];   // quantized hue for matched pixels (h/2)
uint16_t queue_x[PIXEL_COUNT];     // BFS queue: x coordinates
uint16_t queue_y[PIXEL_COUNT];     // BFS queue: y coordinates

static bool sensor_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_write_blocking(I2C_PORT, CAM_I2C_ADDR, buf, 2, false) == 2;
}

static bool sensor_read_reg(uint8_t reg, uint8_t *value) {
    if (i2c_write_blocking(I2C_PORT, CAM_I2C_ADDR, &reg, 1, true) != 1) {
        return false;
    }
    return i2c_read_blocking(I2C_PORT, CAM_I2C_ADDR, value, 1, false) == 1;
}

static bool get_color_hue_range(const char *targetcolor, int *h_min, int *h_max) {
    if (strcmp(targetcolor, "RED") == 0) {
        *h_min = 315;
        *h_max = 20;
        return true;
    }
    if (strcmp(targetcolor, "GREEN") == 0) {
        *h_min = 110;
        *h_max = 155;
        return true;
    }
    if (strcmp(targetcolor, "BLUE") == 0) {
        *h_min = 190;
        *h_max = 215;
        return true;
    }
    if (strcmp(targetcolor, "YELLOW") == 0) {
        *h_min = 40;
        *h_max = 47;
        return true;
    }
    if (strcmp(targetcolor, "PURPLE") == 0) {
        *h_min = 255;
        *h_max = 290;
        return true;
    }

    return false;
}

static bool target_requires_circularity(const char *targetcolor) {
    return strcmp(targetcolor, "RED") == 0 ||
           strcmp(targetcolor, "GREEN") == 0 ||
           strcmp(targetcolor, "BLUE") == 0;
}

void w_reg(uint8_t reg, uint8_t val);
uint8_t r_reg(uint8_t reg);

static bool sensor_read_reg_timeout(uint8_t reg, uint8_t *value) {
    int wrote = i2c_write_timeout_us(I2C_PORT, CAM_I2C_ADDR, &reg, 1, true, DIAG_I2C_TIMEOUT_US);
    if (wrote != 1) {
        return false;
    }

    int read = i2c_read_timeout_us(I2C_PORT, CAM_I2C_ADDR, value, 1, false, DIAG_I2C_TIMEOUT_US);
    return read == 1;
}

void camera_bus_diagnostic(void) {
    printf("[DIAG] Camera bus test start\n");
    printf("[DIAG] I2C step: reading camera ID regs...\n");

    uint8_t pid = 0;
    uint8_t ver = 0;
    bool i2c_ok = sensor_read_reg_timeout(0x0A, &pid) && sensor_read_reg_timeout(0x0B, &ver);

    if (i2c_ok) {
        printf("[DIAG] I2C PASS: PID=0x%02X VER=0x%02X\n", pid, ver);
    } else {
        printf("[DIAG] I2C FAIL/TIMEOUT: cannot read camera ID regs\n");
    }

    printf("[DIAG] SPI step: register readback test...\n");
    uint8_t patterns[] = {0x55, 0xAA};
    bool spi_ok = true;
    for (int i = 0; i < 2; i++) {
        w_reg(0x00, patterns[i]);
        sleep_ms(1);
        uint8_t got = r_reg(0x00);
        if (got != patterns[i]) {
            spi_ok = false;
            printf("[DIAG] SPI mismatch: wrote 0x%02X read 0x%02X\n", patterns[i], got);
            break;
        }
    }

    if (spi_ok) {
        printf("[DIAG] SPI PASS: register readback OK\n");
    } else {
        printf("[DIAG] SPI FAIL\n");
    }
}


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

static bool hue_in_range(int hue, int h_min, int h_max) {
    if (h_min < h_max) {
        return hue >= h_min && hue <= h_max;
    }
    return hue >= h_min || hue <= h_max;
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
        bool match = hue_in_range(h, h_min, h_max);
        if (match) {
            mask_buffer[index] = 255;                 // white if match
            hue_buffer[index] = (uint8_t)(h / 2);    // store hue compactly (0..180)
        } else {
            mask_buffer[index] = 0;
            hue_buffer[index] = 0;
        }
    } else {
        mask_buffer[index] = 0;  // too dull/desaturated → not a color match
        hue_buffer[index] = 0;
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
        if (!sensor_write_reg(regs[i][0], regs[i][1])) {
            return;
        }
        if (regs[i][0] == 0x12 && regs[i][1] == 0x80) {
            sleep_ms(50);  // reset takes longer
        } else {
            sleep_ms(2);   // normal delay between regs
        }
    }

    w_reg(0x00, 0x55);
    uint8_t spi_test = r_reg(0x00);

    uint8_t pid = 0;
    uint8_t ver = 0;
    sensor_read_reg(0x0A, &pid) && sensor_read_reg(0x0B, &ver);
}

// --- BLOB FINDING AND ANALYSIS ---
static bool find_blob_for_target(const char *targetcolor, int *out_x, int *out_y) {
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
                    printf("[DEBUG] Camera capture timed out! trig=0x%02X size=%lu\n",
                           r_reg(0x41),
                           (unsigned long)(r_reg(0x42) | (r_reg(0x43) << 8) | ((r_reg(0x44) & 0x7f) << 16)));
                    w_reg(0x04, 0x01);
                    return false;
                }
            }

            break;
        }
    }

    uint32_t size = r_reg(0x42) | (r_reg(0x43) << 8) | ((r_reg(0x44) & 0x7f) << 16);
    if (size < 5000) {
        printf("[DEBUG] Frame too small, skipping\n");
        w_reg(0x04, 0x01);
        return false;
    }

    // --- 1. SPI READ AND MASK GENERATION ---
    // LOGIC CHANGE: Read and compute HSV mask concurrently to save memory and loops.
    cs_select();
    uint8_t cmd = 0x3C;
    spi_write_blocking(SPI_PORT, &cmd, 1);

    int h_min = 0;
    int h_max = 0;
    if (!get_color_hue_range(targetcolor, &h_min, &h_max)) {
        cs_deselect();
        printf("[DEBUG] Unsupported target color: %s\n", targetcolor);
        w_reg(0x04, 0x01);
        return false;
    }

    uint8_t pixel_buf[2];
    int masked_count = 0;
    for (int i = 0; i < PIXEL_COUNT; i++) {
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
    int best_hue = 0;
    float best_roundness = 0.0f;
    int blob_count = 0;
    int largest_area = 0;

    for (int y = 0; y < HEIGHT; y++) {
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
                long sum_x = 0, sum_y = 0;
                long sum_hue = 0;
                uint64_t sum_x2 = 0;
                uint64_t sum_y2 = 0;
                uint64_t sum_xy = 0;

                while (head < tail) {
                    int cx = queue_x[head];
                    int cy = queue_y[head];
                    head++;

                    area++;
                    sum_x += cx;
                    sum_y += cy;
                    sum_hue += (long)hue_buffer[cy * WIDTH + cx] * 2;
                    sum_x2 += (uint64_t)cx * (uint64_t)cx;
                    sum_y2 += (uint64_t)cy * (uint64_t)cy;
                    sum_xy += (uint64_t)cx * (uint64_t)cy;
                   
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

                        if (nx >= 0 && nx < WIDTH && ny >= 0 && ny < HEIGHT) {
                            int n_idx = ny * WIDTH + nx;
                            if (mask_buffer[n_idx] == 255) { // Found connected pixel
                                mask_buffer[n_idx] = 127; // Mark visited
                                queue_x[tail] = nx;
                                queue_y[tail] = ny;
                                tail++;
                            }
                        }
                    }
                }

                if (area > largest_area) {
                    largest_area = area;
                }

                bool requires_circularity = target_requires_circularity(targetcolor);
                int min_area = requires_circularity ? MIN_BALL_BLOB_AREA : MIN_BLOB_AREA;

                if (area > min_area) {
                    int bbox_width = max_x - min_x + 1;
                    int bbox_height = max_y - min_y + 1;
                    int bbox_area = bbox_width * bbox_height;
                    int blob_hue = (int)(sum_hue / area);

                    // Use second-moment (PCA) axis ratio instead of bbox ratio.
                    // This is much more reliable for near-circular blobs.
                    float mean_x = (float)sum_x / (float)area;
                    float mean_y = (float)sum_y / (float)area;
                    float ex2 = (float)sum_x2 / (float)area;
                    float ey2 = (float)sum_y2 / (float)area;
                    float exy = (float)sum_xy / (float)area;

                    float cov_xx = ex2 - mean_x * mean_x;
                    float cov_yy = ey2 - mean_y * mean_y;
                    float cov_xy = exy - mean_x * mean_y;

                    if (cov_xx < 0.0f) cov_xx = 0.0f;
                    if (cov_yy < 0.0f) cov_yy = 0.0f;

                    float trace = cov_xx + cov_yy;
                    float delta = cov_xx - cov_yy;
                    float rad = sqrtf(delta * delta + 4.0f * cov_xy * cov_xy);
                    float lambda_max = 0.5f * (trace + rad);
                    float lambda_min = 0.5f * (trace - rad);

                    if (lambda_max < 1e-6f) lambda_max = 1e-6f;
                    if (lambda_min < 1e-6f) lambda_min = 1e-6f;

                    float aspect_ratio = sqrtf(lambda_max / lambda_min);
                    float solidity = (float)area / bbox_area;
                    bool shape_match = false;

                    if (requires_circularity) {
                        shape_match = aspect_ratio < 2.3f && solidity > 0.52f;
                    } else {
                        shape_match = aspect_ratio < 4.0f && solidity > 0.35f;
                    }
                   
                    // printf("[DEBUG] Blob: %d area=%d, bbox=%dx%d, aspect=%.2f, solidity=%.2f\r\n",blob_hue, area, bbox_width, bbox_height, aspect_ratio, solidity);
                    if (shape_match && area > best_area) {
                        best_area = area;
                        best_cx = sum_x / area;
                        best_cy = sum_y / area;
                        best_hue = blob_hue;
                        best_roundness = solidity;
                    } else if (requires_circularity && aspect_ratio >= 2.3f) {
                        printf("%s -> not circular enough %.2f\n", targetcolor, aspect_ratio);
                    } else if (!requires_circularity && aspect_ratio >= 4.0f) {
                        printf("%s -> too elongated %.2f\n", targetcolor, aspect_ratio);
                    } else if (requires_circularity && solidity <= 0.70f) {
                        printf("%s -> too hollow | hue:%d | solidity:%.2f\n", targetcolor, blob_hue, solidity);
                    } else if (!requires_circularity && solidity <= 0.35f) {
                        printf("%s -> too hollow\n", targetcolor);
                    }
                    blob_count++;
                }
            }
        }
    }
    if (best_area > 0 && best_roundness > 0.52f) {
        if (out_x != NULL) {
            *out_x = best_cx;
        }
        if (out_y != NULL) {
            *out_y = best_cy;
        }
        return true;
    }

    return false;
}

bool findblobs(BlobDetection *result) {
    static const char *targets[] = {"RED", "BLUE", "GREEN", "YELLOW", "PURPLE"};
    const int target_count = (int)(sizeof(targets) / sizeof(targets[0]));

    if (result == NULL) {
        return false;
    }

    result->label = NULL;
    result->x = -1;
    result->y = -1;

    for (int i = 0; i < target_count; i++) {
        int detected_x = -1;
        int detected_y = -1;
        if (find_blob_for_target(targets[i], &detected_x, &detected_y)) {
            result->label = targets[i];
            result->x = detected_x;
            result->y = detected_y;
            return true;
        }
    }

    return false;
}

void setup() {
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

    // Explicitly configure weak internal pull-ups on SDA/SCL.
    gpio_init(PIN_SDA);
    gpio_init(PIN_SCL);
    gpio_set_pulls(PIN_SDA, true, false);
    gpio_set_pulls(PIN_SCL, true, false);

    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);              // enable pull-ups
    gpio_pull_up(PIN_SCL);
}