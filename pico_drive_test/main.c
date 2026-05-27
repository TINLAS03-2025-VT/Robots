#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// --- Hardware Pins ---
#define PWM_LM 6        // Left motor
#define PWM_RM 7        // Right motor
#define EN_L   11       // Left IR encoder
#define EN_R   14       // Right IR encoder

// --- Global Variables for Speed Tracking & Debouncing ---
volatile uint64_t last_ticks[2] = {0, 0}; // 0 = Left, 1 = Right
volatile uint64_t cum_deltas[2] = {0, 0};
volatile uint32_t num_deltas[2] = {0, 0};
const uint64_t REFRACTORY_TIME_US = 10000; 

// Calibration control variables
volatile int32_t I_count = 0;
volatile int32_t Target_I_count = 0;
volatile uint64_t cal_last_tick = 0;
volatile uint64_t cal_current_tick = 0;
volatile bool calibration_mode = false;

// --- Cubic Polynomial Coefficients (Fallbacks) ---
float coeff_LP[4] = {-0.1777802,  0.6523898, -0.1553434,  0.08150997};
float coeff_RP[4] = { 0.0814823, -0.4187451,  0.08430676, -0.1871835};
float coeff_LN[4] = { 0.2038952, -0.6254874,  0.03229254, -0.1854067};
float coeff_RN[4] = { 0.04240555, 0.0646009,  0.2027688, -0.006709341};

// --- Helper Functions ---

// Evaluates a 3rd degree polynomial: ax^3 + bx^2 + cx + d
float eval_poly3(float x, float coeff[4]) {
    return (coeff[0] * powf(x, 3)) + (coeff[1] * powf(x, 2)) + (coeff[2] * x) + coeff[3];
}

// --- Unified Interrupt Service Routine (ISR) ---
void encoder_callback(uint gpio, uint32_t events) {
    uint64_t current_tick = time_us_64();
    
    if (calibration_mode) {
        if (I_count == 0) {
            cal_last_tick = current_tick;
        }
        if (I_count == Target_I_count) {
            cal_current_tick = current_tick;
        }
        I_count++;
        return;
    }

    // Normal Driving Encoder Mode
    int idx = (gpio == EN_L) ? 0 : 1;
    uint64_t delta = current_tick - last_ticks[idx];
    
    if (delta < REFRACTORY_TIME_US) {
        return; // Debounce filter
    }
    
    last_ticks[idx] = current_tick;
    cum_deltas[idx] += delta;
    num_deltas[idx]++;
}

// --- Servo Configuration Layer ---
void init_servo_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    
    // Pico clock runs at 125MHz. 
    // To get a 50Hz frequency (20ms period) with a 16-bit top wrap value of 65535:
    // Clock divider = 125,000,000 / (50 * 65536) = 38.1469
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 38.147f);
    pwm_config_set_wrap(&config, 65535);
    
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(gpio, 5000); // 5000 / 65535 is approx ~1.5ms pulse (Stop state)
}

// --- Navigation Engine ---
void calculate_PWM(float RPS_L, float RPS_R, int *pwm_L, int *pwm_R) {
    if (RPS_L > 0) {
        *pwm_L = (int)(eval_poly3(RPS_L, coeff_LP) * 2000.0f + 5000.0f);
    } else {
        *pwm_L = (int)(eval_poly3(-RPS_L, coeff_LN) * 2000.0f + 5000.0f);
    }

    if (RPS_R > 0) {
        *pwm_R = (int)(eval_poly3(RPS_R, coeff_RP) * 2000.0f + 5000.0f);
    } else {
        *pwm_R = (int)(eval_poly3(-RPS_R, coeff_RN) * 2000.0f + 5000.0f);
    }
}

void set_motor_speed(float RPS_L, float RPS_R, uint32_t ms_sleep) {
    int pwm_L, pwm_R;
    calculate_PWM(RPS_L, RPS_R, &pwm_L, &pwm_R);
    
    // Flush interrupt tracking counters
    cum_deltas[0] = cum_deltas[1] = 0;
    num_deltas[0] = num_deltas[1] = 0;
    uint64_t now = time_us_64();
    last_ticks[0] = last_ticks[1] = now;
    
    pwm_L = (RPS_L == 0.0f) ? 5000 : pwm_L;
    pwm_R = (RPS_R == 0.0f) ? 5000 : pwm_R;
    
    calibration_mode = false;
    
    // Turn on Interrupts
    gpio_set_irq_enabled(EN_L, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(EN_R, GPIO_IRQ_EDGE_RISE, true);
    
    pwm_set_gpio_level(PWM_LM, pwm_L);
    pwm_set_gpio_level(PWM_RM, pwm_R);
    
    sleep_ms(ms_sleep);
    
    // Turn off Peripherals and stop motors
    gpio_set_irq_enabled(EN_L, GPIO_IRQ_EDGE_RISE, false);
    gpio_set_irq_enabled(EN_R, GPIO_IRQ_EDGE_RISE, false);
    pwm_set_gpio_level(PWM_LM, 5000);
    pwm_set_gpio_level(PWM_RM, 5000);
}

// --- Calibration System ---
float measure_motor_speed(int pwm_val, char motor, int steps) {
    I_count = 0;
    Target_I_count = 2 * steps;
    calibration_mode = true;
    
    uint motor_pwm_pin = (motor == 'L' || motor == 'l') ? PWM_LM : PWM_RM;
    uint sensor_pin = (motor == 'L' || motor == 'l') ? EN_L : EN_R;
    uint other_sensor = (motor == 'L' || motor == 'l') ? EN_R : EN_L;
    
    gpio_set_irq_enabled(other_sensor, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    gpio_set_irq_enabled(sensor_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    
    pwm_set_gpio_level(motor_pwm_pin, pwm_val);
    
    while (I_count <= Target_I_count) {
        sleep_ms(10);
    }
    sleep_ms(500);
    
    gpio_set_irq_enabled(sensor_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    pwm_set_gpio_level(motor_pwm_pin, 5000);
    
    uint64_t delta = cal_current_tick - cal_last_tick;
    float time_ms = (float)delta / 1000.0f;
    
    float rotations = (float)Target_I_count / 20.0f;
    return rotations / (time_ms / 1000.0f); // Returns calculated RPS
}

// --- Setup & Runtime Entry Configuration ---
int main() {
    stdio_init_all();
    
    // Setup Encoder Hardware Input Pins
    gpio_init(EN_L);
    gpio_set_dir(EN_L, GPIO_IN);
    gpio_pull_down(EN_L);
    
    gpio_init(EN_R);
    gpio_set_dir(EN_R, GPIO_IN);
    gpio_pull_down(EN_R);
    
    // Register global shared GPIO callback handler
    gpio_set_irq_callback(&encoder_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);
    
    // Initialize continuous PWM channels
    init_servo_pwm(PWM_LM);
    init_servo_pwm(PWM_RM);
    
    sleep_ms(2000); // Grace period to initialize tracking monitors safely
    
    while (1) {
        // Drive Forward at 1.5 Rotations Per Second for 3 seconds
        set_motor_speed(0.5f, 0.5f, 3000);
        sleep_ms(2000);
        
        // Drive Reverse at 1.0 Rotations Per Second for 2 seconds
        set_motor_speed(-0.5f, -0.5f, 2000);
        sleep_ms(5000);
    }
    
    return 0;
}