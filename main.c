#include "global.h"


#define COLOR_I2C_PORT i2c0
#define COLOR_I2C_SDA 8
#define COLOR_I2C_SCL 9
#define COLOR_I2C_FREQ 50000
#define COLOR_SENSOR_ADDR 0x29


// #define PIN_CS   5              // SPI chip select (enable/disable camera)
// #define PIN_MOSI 3              // SPI data out (Pico → Camera)
// #define PIN_MISO 4              // SPI data in (Camera → Pico)
// #define PIN_SCK  2  


#define DIST_TRIGGER_PIN 27
#define DIST_ECHO_PIN 28
#define DIST_ECHO_PIN_RIGHT 26


#define REBOOT_BUTTON 12
#define LFWD_MOTOR     6
#define LREV_MOTOR     7
#define RFWD_MOTOR     10
#define RREV_MOTOR     11
#define LChannelA    18
#define LChannelB    19
#define RChannelA    20
#define RChannelB    21




#define scoop_PIN 1
#define sorter_PIN 0
#define BEAM_SENSOR_PIN 13
#define BEAM_BROKEN_STATE 0
#define GREENFLYWHEEL 17
#define REDFLYWHEEL 16








//        ┌──────────────────────────────┐
//        │        USB PORT              │
//        └──────────────────────────────┘
//
//LEFT SIDE                         RIGHT SIDE
//────────                         ──────────
//GP0   scoop_PIN              VBUS
//GP1   sorter_PIN             VSYS
//GND                          GND
//GP2   MOTOR_LFWD_MOTOR         3V3_EN
//GP3   MOTOR_LREV_MOTOR         3V3(OUT)
//GP4   MOTOR_RFWD_MOTOR         ADC_VREF
//GP5   MOTOR_RREV_MOTOR         GP28  DIST_ECHO_PIN
//GND                          GND
//GP6   FREE                   GP27  DIST_TRIGGER_PIN
//GP7   FREE                   GP26  FREE (ADC)
//GP8   I2C SDA (color)        RUN (reset)
//GP9   I2C SCL (color)        GP22  FREE
//GND                          GND
//GP10  FREE                   GP21  RIGHTENCODERB
//GP11  FREE                   GP20  RIGHTENCODERA
//GP12  REBOOT_BUTTON          GP19  LEFTENCODERB
//GP13  BEAM_SENSOR            GP18  LEFTENCODERA
//GND                          GND
//GP14  OVERCURRENT1           GP17  REDFLYWHEEL
//GP15  OVERCURRENT2           GP16  GREENFLYWHEEL  






// // ===== GLOBAL STATE =====
volatile int x_pos = 105; // starting position: middle of field
volatile int y_pos = 0;
int heading = 0;
int max_x = 205;    //we can move roughly 205cm to the right from 0,0
int min_x = 10;
int max_y = 245;    //we can move roughly 245cm towards the enemy
int min_y = 0;


Encoder left_enc;
Encoder right_enc;


// ===== GLOBAL VARIABLES =====
MotorNoEncoder motor;
TCS34725 sensor;
Servo scoop;
Servo sorter;
Compass compass;
float enemy_goal_heading = -1.0f; // heading (degrees, 0-360) recorded at startup when facing enemy goal
bool beam_was_broken = false;
bool searching = true;
struct repeating_timer timer;
int gb_in_chamber = 0;
int red_in_chamber = 0;


// Distance sensor pulse tracking for interrupt-driven collision detection
volatile uint64_t pulse_start_left = 0;
volatile uint64_t pulse_start_right = 0;
volatile bool COLLISION = false;
volatile int COLLISION_PIN = 0; //aka none
volatile int left_distance_cm = 0;
volatile int right_distance_cm = 0;
volatile uint8_t collision_hits_left = 0;
volatile uint8_t collision_hits_right = 0;
volatile int L_collision_count = 0;
volatile int R_collision_count = 0;


void collisionDetected();


void sleepcheck(int x){
    double reps = x/100;
    for (int i = 0; i < reps; i++) {
        if (COLLISION == true){ collisionDetected();}
        sleepcheck(100);
    }
}


#define DISTANCE_THRESHOLD_MM 100
bool timer_callback(struct repeating_timer *t) {
    // This callback runs every 70ms to check if a collision has been detected via distance sensors
    gpio_put(DIST_TRIGGER_PIN, true);
    sleep_us(10);
    gpio_put(DIST_TRIGGER_PIN, false);
    return true; // keep repeating
}
// GPIO interrupt handler for rear distance sensor (echo pin 26)
void dist_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        if (gpio == DIST_ECHO_PIN) {
            pulse_start_left = time_us_64();
        } else if (gpio == DIST_ECHO_PIN_RIGHT) {
            pulse_start_right = time_us_64();
        }
        return;
    }


    if (events & GPIO_IRQ_EDGE_FALL) {
        uint64_t start = (gpio == DIST_ECHO_PIN) ? pulse_start_left : pulse_start_right;
        if (start == 0) return;


        uint64_t duration = time_us_64() - start;
        int distance = (int)(((float)duration / 2.0f) / 29.1f);
        if (gpio == DIST_ECHO_PIN) {
            left_distance_cm = distance;
            pulse_start_left = 0;
            //printf("Left distance: %d cm\n", left_distance_cm);
            if (distance < 15) {
                L_collision_count++;
                if(L_collision_count >= 3) {
                    COLLISION = true;
                    COLLISION_PIN = DIST_ECHO_PIN;
                } else {
                    L_collision_count = 0;
                }
            }
        } else {
            right_distance_cm = distance;
            pulse_start_right = 0;
            //printf("Right distance: %d cm\n", right_distance_cm);
            if (distance < 15) {
                if(R_collision_count >= 3) {
                    COLLISION = true;
                    COLLISION_PIN = DIST_ECHO_PIN_RIGHT;
                } else {
                    R_collision_count = 0;
                }
            }
        }
    }
}


// // ===== HELPER FUNCTIONS =====
// void init_pins() { //Initialize GPIO pins for encoders and motors
//     //Initialize Encoders
//     gpio_set_function(LREV_MOTOR, GPIO_FUNC_PWM);
//     gpio_set_function(LFWD_MOTOR, GPIO_FUNC_PWM);
//     gpio_set_function(RFWD_MOTOR, GPIO_FUNC_PWM);
//     gpio_set_function(RREV_MOTOR, GPIO_FUNC_PWM);
// }


void turn(bool direction, short angleChange, int speed) { // 0 == LEFT, 1 == RIGHT, angleChange in DEGREES, speed [0 - 100]
    printf("ENTERING TURN");
    int duty_cycle = (speed * 65535) / 100;
    int pastHeading = heading;
    if(direction == 0) {
        int futureHeading = pastHeading - angleChange;
        if (futureHeading < -180) {
            futureHeading += 360;
        }
        while(heading < futureHeading + 2 || heading > futureHeading - 2) {
            printf("heading: %d", heading);
            //UPDATE HEADING HERE IF NOT AUTOMATIC
            pwm_set_gpio_level(LREV_MOTOR, duty_cycle);
            pwm_set_gpio_level(LFWD_MOTOR, 0);
            pwm_set_gpio_level(RFWD_MOTOR, duty_cycle);
            pwm_set_gpio_level(RREV_MOTOR, 0);
            sleepcheck(5);
        }
        pwm_set_gpio_level(LREV_MOTOR, 0);
        pwm_set_gpio_level(LFWD_MOTOR, 0);
        pwm_set_gpio_level(RFWD_MOTOR, 0);
        pwm_set_gpio_level(RREV_MOTOR, 0);
        printf("EXITED TURN");
        return;
    }
    else {
        int futureHeading = pastHeading + angleChange;
        if (futureHeading > 180) {
            futureHeading -= 360;
        }
        while(heading < futureHeading + 2 || heading > futureHeading - 2) {
            //UPDATE HEADING HERE IF NOT AUTOMATIC
            pwm_set_gpio_level(LREV_MOTOR, 0);
            pwm_set_gpio_level(LFWD_MOTOR, duty_cycle);
            pwm_set_gpio_level(RFWD_MOTOR, 0);
            pwm_set_gpio_level(RREV_MOTOR, duty_cycle);
            sleepcheck(5);
        }
        pwm_set_gpio_level(LREV_MOTOR, 0);
        pwm_set_gpio_level(LFWD_MOTOR, 0);
        pwm_set_gpio_level(RFWD_MOTOR, 0);
        pwm_set_gpio_level(RREV_MOTOR, 0);
        printf("EXITED TURN");
        return;
    }
    printf("EXITED TURN");
    return;
}


// void move(bool DIR, int distance, int speed) { //DIR 1 = FWD, 0 = REV, distance in cm, speed [0 - 100]
//     int start_pos_x = x_pos;
//     int start_pos_y = y_pos;
//     int leftCount = 0;
//     int rightCount = 0;
//     int heading = 0; //IMU HEADING NUMBER?
//     int x_distance = distance * cos(heading * M_PI / 180);
//     int y_distance = distance * sin(heading * M_PI / 180);


//     // Set direction pins
//     while((x_pos < start_pos_x + x_distance) && (x_pos < max_x) && (y_pos < max_y) && (y_pos > min_y) && (x_pos > min_x)) {
//         //UPDATE ENCODER COUNTS AND HEADING
//         leftCount = 0; //encoder_delta(&left_enc);
//         rightCount = 0; //encoder_delta(&lright_enc);
//         x_pos += ((leftCount + rightCount) / 2) * cos(heading * M_PI / 180) * (8*M_PI/1920); // 8cm wheel diameter, 1920 counts per revolution
//         y_pos += ((leftCount + rightCount) / 2) * sin(heading * M_PI / 180) * (8*M_PI/1920);


//         // Set PWM duty cycle based on speed
//         int duty_cycle = (speed * 65535) / 100; // Convert percentage to 16-bit value
//         if (DIR) { // Forward PWM
//             pwm_set_gpio_level(LFWD_MOTOR, duty_cycle);
//             pwm_set_gpio_level(LREV_MOTOR, 0); }
//         else {
//             pwm_set_gpio_level(LFWD_MOTOR, 0);
//             pwm_set_gpio_level(LREV_MOTOR, duty_cycle); }
//         if (DIR) {
//             pwm_set_gpio_level(RFWD_MOTOR, duty_cycle);
//             pwm_set_gpio_level(RREV_MOTOR, 0); }
//         else {
//             pwm_set_gpio_level(RFWD_MOTOR, 0);
//             pwm_set_gpio_level(RREV_MOTOR, duty_cycle); }
//         //go direction for at least 10ms.
//         sleepcheck(10);
//     }
//     //When we have reached the target position,
//     //or hit a boundary, stop the motors
//     pwm_set_gpio_level(LFWD_MOTOR, 0);
//     pwm_set_gpio_level(LREV_MOTOR, 0);
//     pwm_set_gpio_level(RFWD_MOTOR, 0);
//     pwm_set_gpio_level(RREV_MOTOR, 0);
//     return;
// }


// //----------------STATE MACHINES----------------------






typedef void (*StateFunc)(void);
StateFunc current_state;


static const char *captured_ball_label = NULL;
static int chamber_ball_count = 0;


#define TARGET 3
#define EXTRA_SEARCH_TIMEOUT_MS 12000


// Field geometry (cm): 7ft wide field, centered 3ft-wide goals.
#define FIELD_WIDTH_CM 213.36f
#define GOAL_WIDTH_CM 91.44f
#define FIELD_CENTER_X_CM (FIELD_WIDTH_CM * 0.5f)
#define GOAL_HALF_WIDTH_CM (GOAL_WIDTH_CM * 0.5f)
#define GOAL_MIN_X_CM (FIELD_CENTER_X_CM - GOAL_HALF_WIDTH_CM)
#define GOAL_MAX_X_CM (FIELD_CENTER_X_CM + GOAL_HALF_WIDTH_CM)


// Forward declarations
void state_initial(void);
void state_goalie(void);
void state_capture(void);
void state_deposit(void);


// // States in State Machine
// void state_initial(void) {// Rush towards middle line
//     move(1, 91, 80);
//     // transfer to captu
//     current_state = state_capture;
// }
// void state_goalie(void) {// Move back and forth in front of goal
//     move(0, 107, 90);
//     turn(1, 90, 90);
//     while(current_state == state_goalie){
//         move(1,48, 90);
//         sleepcheck(500);
//         move(0, 91, 90);
//         sleepcheck(500);
//     }
// //    if (/* trigger */) current_state = state_search;
// }


static bool is_ball(const char *label) {
    return label != NULL &&
           (strcmp(label, "RED") == 0 ||
            strcmp(label, "BLUE") == 0 ||
            strcmp(label, "GREEN") == 0);
}


// static bool is_enemy_goal_ball(const char *label) {
//     return label != NULL && strcmp(label, "RED") == 0;
// }


// static bool is_own_goal_ball(const char *label) {
//     return label != NULL &&
//            (strcmp(label, "GREEN") == 0 || strcmp(label, "BLUE") == 0);
// }


// static bool early_deposit(uint32_t search_start_ms) {
//     if (chamber_ball_count <= 0) {
//         return false;
//     }


//     uint32_t now_ms = to_ms_since_boot(get_absolute_time());
//     return (now_ms - search_start_ms) >= EXTRA_SEARCH_TIMEOUT_MS;
// }
float normalize_heading(float h) {
    while (h > 180.0f) h -= 360.0f;
    while (h < -180.0f) h += 360.0f;
    return h;
}

float heading0 = 0.0f;
void compass_init_reference(Compass *compass) {
        float initial_heading;

        compass_read_heading(compass, &initial_heading, NULL);
        heading0 = initial_heading;
    }
float compass_get_relative_heading(Compass *compass) {
    float heading;

    compass_read_heading(compass, &heading, NULL);

    float relative = heading - heading0;

    return normalize_heading(relative);
}


void initalizeEverything() {
    // INIT
    stdio_init_all();


    // MOTOR INIT OLD
    motor_noencoder_init(
        &motor,
        LREV_MOTOR,
        RFWD_MOTOR,
        LFWD_MOTOR,
        RREV_MOTOR,
        MOTOR_NOENCODER_NO_FAULT_PIN,
        MOTOR_NOENCODER_NO_FAULT_PIN,
        1000,
        300,
        150
    );
    motor_noencoder_stop_all(&motor);
    sleep_ms(3000);


    // ENCODER INIT
    encoder_init(&left_enc, LChannelA, LChannelB);
    encoder_init(&right_enc, RChannelA, RChannelB);


    // GPIO INIT
    gpio_init(REBOOT_BUTTON);
    gpio_set_dir(REBOOT_BUTTON, GPIO_IN);
    gpio_pull_up(REBOOT_BUTTON);


    // Initialize shared camera/color-sensor bus and SPI camera interface
    setup();
    camera_bus_diagnostic();
    // BEAM SENSOR CODE
    gpio_init(BEAM_SENSOR_PIN);
    gpio_set_dir(BEAM_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(BEAM_SENSOR_PIN);


    //TRIGGER PIN FOR DISTANCE SENSOR INTERRUPT SETUP
    gpio_init(DIST_TRIGGER_PIN);
    gpio_set_dir(DIST_TRIGGER_PIN, GPIO_OUT);
    gpio_put(DIST_TRIGGER_PIN, false);
   
    // DISTANCE SENSOR INTERRUPT SETUP - collision detection
    gpio_init(DIST_ECHO_PIN);
    gpio_set_dir(DIST_ECHO_PIN, GPIO_IN);
    gpio_pull_down(DIST_ECHO_PIN);
    gpio_set_irq_enabled_with_callback(DIST_ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &dist_callback);
   
    gpio_init(DIST_ECHO_PIN_RIGHT);
    gpio_set_dir(DIST_ECHO_PIN_RIGHT, GPIO_IN);
    gpio_pull_down(DIST_ECHO_PIN_RIGHT);
    gpio_set_irq_enabled_with_callback(DIST_ECHO_PIN_RIGHT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &dist_callback);
   
    printf("Distance collision detection active - threshold: %d mm\n", DISTANCE_THRESHOLD_MM);


    // COLOR SENSOR INIT
    if (!tcs34725_init(&sensor, COLOR_I2C_PORT, COLOR_SENSOR_ADDR)) {
        printf("Color sensor init failed\n");
        while (true) {
            sleepcheck(1000);
        }
    }


    uint8_t sensor_id = 0;
    if (tcs34725_get_sensor_id(&sensor, &sensor_id)) {
        printf("Color sensor ID: 0x%02X\n", sensor_id);
    }


    // SERVOS INIT
    servo_init(&scoop, scoop_PIN);
    servo_init(&sorter, sorter_PIN);
    servo_move(&scoop, 0.0f);
    servo_move(&sorter, 0.0f);
    sleep_ms(5000);
    // COMPASS / MAGNETOMETER INIT (commented out for now)

    ak8963_vector_t COMPASS_OFFSET = {
        .x = 30.335f,
        .y = 1.235f,
        .z = -18.330f
    };

    ak8963_vector_t COMPASS_SCALE = {
        .x = 1.215f,
        .y = 0.895f,
        .z = 0.944f
    };
    const float lubbock_declination_deg = -5.44f;

    bool compass_ok = compass_init(&compass, COLOR_I2C_PORT, lubbock_declination_deg);
    if (compass_apply_calibration(&compass, COMPASS_OFFSET, COMPASS_SCALE)) {
        printf("Calibration applied");
        sleep_ms(200);
    }
    
    if(!compass_ok) {
        printf("Compass init failed\n");
        sleep_ms(10000);
    }
    compass_init_reference(&compass);
    

    
    
    // NOTE: Distance sensor triggers need regular pulses to work
    // Ensure hcsr04_send_pulse_and_wait() is called periodically in main loop
   
    // CAMERA INIT
    printf("Starting...\n");
    sleep_ms(1000);


    init_cam();  // configure camera registers
    printf("Cam init done\n");
    printf("Camera initialized...\n");
    sleep_ms(5000);

    // FLYWHEEL INIT
    gpio_init(GREENFLYWHEEL);
    gpio_set_dir(GREENFLYWHEEL, GPIO_OUT);
    gpio_init(REDFLYWHEEL);
    gpio_set_dir(REDFLYWHEEL, GPIO_OUT);
    gpio_put(GREENFLYWHEEL, 1);
    gpio_put(REDFLYWHEEL, 1);
    printf("made it out of Flywheel setting");


    add_repeating_timer_ms(-100, timer_callback, NULL, &timer);
    printf("Timer initialized\n");
}
void collisionDetected() {
    if(COLLISION_PIN == DIST_ECHO_PIN) {
        printf("Collision detected on LEFT sensor: %d cm\n", left_distance_cm);
    } else {
        printf("Collision detected on RIGHT sensor: %d cm\n", right_distance_cm);
    }
    motor_noencoder_move(&motor, .5, false, false); // Back up a little
    sleepcheck(500);
    motor_noencoder_stop_all(&motor);
    COLLISION = false;


}
void rightPivot() {
    motor_noencoder_move(&motor, .5, true, false); // R1
    sleepcheck(350);
    motor_noencoder_stop_all(&motor);
    sleepcheck(50);
}
// State machine functions
void state_capture(void) { //Search and Capture
    uint32_t search_start_ms = to_ms_since_boot(get_absolute_time());
    BlobDetection blob;
    bool found = findblobs(&blob);
    bool beam_broken = gpio_get(BEAM_SENSOR_PIN) == 0;
    //SEARCHING
    while (!found || !is_ball(blob.label)) {


        gpio_put(GREENFLYWHEEL, 1);
        gpio_put(REDFLYWHEEL, 1);
        sleepcheck(120000);
        gpio_put(GREENFLYWHEEL, 0);
        gpio_put(REDFLYWHEEL, 0);


        found = findblobs(&blob);
        // Forward burst
        printf("forward burst");
        motor_noencoder_move(&motor, .5, true, true);
        sleepcheck(1000);
        motor_noencoder_stop_all(&motor);
        if (findblobs(&blob) && is_ball(blob.label)) {
            printf("ball found: %s\n", blob.label);
            break;
        }
        if (gpio_get(BEAM_SENSOR_PIN) == 0) {
            printf("Beam BROKEN");
            break; }
       
        // TURN LEFT
       
        printf("turn LEFT");
        motor_noencoder_move(&motor, .5, false, true);
        sleepcheck(750);
        motor_noencoder_stop_all(&motor);
        if (findblobs(&blob) && is_ball(blob.label)) {
            printf("ball found: %s\n", blob.label);
            break;
        }
        if (gpio_get(BEAM_SENSOR_PIN) == 0) {
            printf("beam broken");
            break; }
       
        // Right pivot (8 in a row)
       
        printf("start RIGHT pivots");
        rightPivot();
        if (findblobs(&blob) && is_ball(blob.label)) {
            printf("ball found: %s\n", blob.label);
            break;
        }
        if (gpio_get(BEAM_SENSOR_PIN) == 0) { break; }
       
        rightPivot();
        if (findblobs(&blob) && is_ball(blob.label)) {
            printf("ball found: %s\n", blob.label);
            break;
        }
        if (gpio_get(BEAM_SENSOR_PIN) == 0) { break; }


        rightPivot();
        if (findblobs(&blob) && is_ball(blob.label)) {
            printf("ball found: %s\n", blob.label);
            break;
        }
        if (gpio_get(BEAM_SENSOR_PIN) == 0) { break; }


        rightPivot();
        if (findblobs(&blob) && is_ball(blob.label)) {
            printf("ball found: %s\n", blob.label);
            break;
        }
        if (gpio_get(BEAM_SENSOR_PIN) == 0) { break; }


        // Search motor run stuff
    }
   
    motor_noencoder_stop_all(&motor); // STOP ALL
    sleepcheck(500); //Prepare to attack


    //Move towards the ball until the beam is broken
    tcs34725_raw_data_t data;
    bool moving = true;
    time_t seconds;
    int retc = tcs34725_read_raw(&sensor, &data);
    const char *detected = tcs34725_detect_color_from_raw(&data);
    printf("get that thing fr ");
    while(COLLISION) {
        collisionDetected();
    }
    motor_noencoder_move(&motor, .4, true, true);
   
    // CAPTURE THE BALL
    while(moving){
        if(COLLISION) {
            collisionDetected();
            break;
        }
        beam_broken = gpio_get(BEAM_SENSOR_PIN) == 0;
        tcs34725_raw_data_t data;
        int retc = tcs34725_read_raw(&sensor, &data);
        const char *detected = tcs34725_detect_color_from_raw(&data);
        printf("moving towardws balls\n");
        if (beam_broken && !beam_was_broken) { //STOP MOVING, READ
            sleepcheck(250);
            printf("Beam broken\n");
            scoop_up(&scoop);
            seconds = time(NULL);
            sleepcheck(1000);
            motor_noencoder_move(&motor, 0, false, false);
            moving = false;
            printf("the search is over.... Yurah");


            while (detected == NULL && time(NULL) - seconds < 5) {
                retc = tcs34725_read_raw(&sensor, &data);
                detected = tcs34725_detect_color_from_raw(&data);
                if (detected != NULL) {
                    printf("color sensor %s\r\n", detected);


                    if (strcmp(detected, "RED") == 0) { // Just want to track ball count for deposit state
                        red_in_chamber++; }
                    else if ((strcmp(detected, "GREEN") == 0) || (strcmp(detected, "BLUE") == 0)) {
                        gb_in_chamber++; }
                   
                    servo_sort_with(&sorter, detected);
                    break;
                }
            }
            scoop_down(&scoop);
           
            printf("while loop over");
        }
        beam_was_broken = beam_broken;
        // Insert check for transition to deposit state
    } motor_noencoder_move(&motor, 0, false, false);


}
// Returns the smallest angular difference between two headings (0-360), range [0, 180]
static float heading_diff(float a, float b) {
    float d = fabsf(a - b);
    if (d > 180.0f) d = 360.0f - d;
    return d;
}


void state_deposit(void) { // SCORE red or green
    bool oriented_towards_enemy_goal = false;
    bool oriented_towards_own_goal = false;


    // Determine orientation using compass vs startup (enemy-goal) heading.
    // Goal is 3 ft wide (91.44 cm) on a 7 ft field (213.36 cm), centered.
    // Use 45 deg tolerance to cover the goal arc from scoring range.
    if (enemy_goal_heading >= 0.0f) {
        float current_heading = 0.0f;
        if (compass_read_heading(&compass, &current_heading, NULL)) {
            float own_goal_heading = fmodf(enemy_goal_heading + 180.0f, 360.0f);
            if (heading_diff(current_heading, enemy_goal_heading) <= 45.0f) {
                oriented_towards_enemy_goal = true;
            } else if (heading_diff(current_heading, own_goal_heading) <= 45.0f) {
                oriented_towards_own_goal = true;
            }
            printf("Deposit: current=%.1f enemy_ref=%.1f -> enemy=%d own=%d\n",
                   current_heading, enemy_goal_heading,
                   oriented_towards_enemy_goal, oriented_towards_own_goal);
        }
    }


    // Score either the blue/green or red balls in storage
   




    // Kick on correct flywheel using if
    if(oriented_towards_enemy_goal) {
        gpio_put(GREENFLYWHEEL, 0);
        gpio_put(REDFLYWHEEL, 1);
        sleepcheck(5000);
        gpio_put(REDFLYWHEEL, 0); // Shut flywheel back off
        red_in_chamber = 0;
    } else if(oriented_towards_own_goal) {
        gpio_put(REDFLYWHEEL, 0);
        gpio_put(GREENFLYWHEEL, 1);
        sleepcheck(5000);
        gpio_put(GREENFLYWHEEL, 0);
        gb_in_chamber = 0;
    }


    current_state = state_capture; // Back to search & capture state


    // void state_deposit(void) { // SCORE
//     if (chamber_ball_count <= 0) {
//         current_state = state_capture;
//         return;
//     }


//     if (!is_ball(captured_ball_label)) {
//         current_state = state_capture;
//         return;
//     }


//     if (is_enemy_goal_ball(captured_ball_label)) {
//         // RED: shoot toward the opponent goal (forward from starting orientation).
//     } else if (is_own_goal_ball(captured_ball_label)) {
//         // GREEN/BLUE: shoot back toward our own goal.
//         turn(1, 180, 90);
//     }
// }  
}


// FUTURE STORAGE IMPLEMENTATION
// if (chamber_ball_count < TARGET) {
//     chamber_ball_count++;
// }
// captured_ball_label = blob.label;


// if (chamber_ball_count >= TARGET) {
//     current_state = state_deposit;
// } else {
//     current_state = state_capture;
// }

float raw_heading = 0.0f;
int main() {
    // Initialize hardware
    initalizeEverything();
    printf("Hardware initialized\n");
    
    // Initialize state machine
    // current_state = state_capture;
    while (true) {
        
        raw_heading = compass_get_relative_heading(&compass);
        printf("Raw heading: %.1f\n", raw_heading);
        motor_noencoder_move(&motor, .5, true, true);
        sleep_ms(500);
        motor_noencoder_stop_all(&motor);
        sleep_ms(500);
        encoder_delta_cm(&left_enc);
        encoder_delta_cm(&right_enc);
        printf("Encoder delta: %d cm\n", encoder_delta_cm(&left_enc));
        printf("Encoder delta: %d cm\n", encoder_delta_cm(&right_enc));
    }

    // Main loop
    while (true) {
        //run the current state
        if (!gpio_get(REBOOT_BUTTON)) {
            printf("Rebooting to bootloader...\n");
            sleep_ms(500);
            reset_usb_boot(0, 0);
        }
        current_state();
        // printf("yeah we looped");
        // gpio_put(GREENFLYWHEEL, 0);
        // sleep_ms(3000);
        // gpio_put(GREENFLYWHEEL, 1);
        // sleep_ms(3000);
        // printf("made it through ");
        // motor_noencoder_move(&motor, .3, true, true);
        // sleep_ms(500);
        // motor_noencoder_stop_all(&motor);
        // int change = encoder_delta(&left_enc);
        // printf("Encoder change: %d\n", change);
        // // Check for reboot button
        


        // sleep_ms(100);
        // printf("okay we are going to the next loop now please");
    }


    return 0;
}