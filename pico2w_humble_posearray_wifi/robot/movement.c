#include "movement.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include <math.h>
#include <pico/stdlib.h>
#include <pico/time.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define PI 3.141592654

#include "geometry_msgs/msg/pose.h"
#include "geometry_msgs/msg/quaternion.h"
#include "hardware/pwm.h"

#include "settings.h" // External settings, view the main project folder

// Global Variables for Speed Tracking & Debouncing ---
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
float coeff_LP[4] = {-0.1777802, 0.6523898, -0.1553434, 0.08150997};
float coeff_RP[4] = {0.0814823, -0.4187451, 0.08430676, -0.1871835};
float coeff_LN[4] = {0.2038952, -0.6254874, 0.03229254, -0.1854067};
float coeff_RN[4] = {0.04240555, 0.0646009, 0.2027688, -0.006709341};

// --- Original Marco's functions ---
float eval_poly3(float x, float coeff[4]) {
  return (coeff[0] * powf(x, 3)) + (coeff[1] * powf(x, 2)) + (coeff[2] * x) +
         coeff[3];
}

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

void init_servo_pwm(uint gpio) {
  gpio_set_function(gpio, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(gpio);

  // Pico clock runs at 125MHz.
  // To get a 50Hz frequency (20ms period) with a 16-bit top wrap value of
  // 65535: Clock divider = 125,000,000 / (50 * 65536) = 38.1469
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, 38.147f);
  pwm_config_set_wrap(&config, 65535);

  pwm_init(slice_num, &config, true);
  pwm_set_gpio_level(gpio,
                     5000); // 5000 / 65535 is approx ~1.5ms pulse (Stop state)
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

float measure_motor_speed(int pwm_val, char motor, int steps) {
  I_count = 0;
  Target_I_count = 2 * steps;
  calibration_mode = true;

  uint motor_pwm_pin = (motor == 'L' || motor == 'l') ? PWM_LM : PWM_RM;
  uint sensor_pin = (motor == 'L' || motor == 'l') ? EN_L : EN_R;
  uint other_sensor = (motor == 'L' || motor == 'l') ? EN_R : EN_L;

  gpio_set_irq_enabled(other_sensor, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                       false);
  gpio_set_irq_enabled(sensor_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                       true);

  pwm_set_gpio_level(motor_pwm_pin, pwm_val);

  while (I_count <= Target_I_count) {
    sleep_ms(10);
  }
  sleep_ms(500);

  gpio_set_irq_enabled(sensor_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                       false);
  pwm_set_gpio_level(motor_pwm_pin, 5000);

  uint64_t delta = cal_current_tick - cal_last_tick;
  float time_ms = (float)delta / 1000.0f;

  float rotations = (float)Target_I_count / 20.0f;
  return rotations / (time_ms / 1000.0f); // Returns calculated RPS
}
// --- Unused functions by Marco ---

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


// Stop moving
void stop() {
  pwm_set_gpio_level(PWM_LM, 5000);
  pwm_set_gpio_level(PWM_RM, 5000);
}

// Move for a certain amount of ms
void move_ms(float RPS_L, float RPS_R, uint32_t ms_sleep) {
  int pwm_L, pwm_R;
  calculate_PWM(-RPS_L, -RPS_R, &pwm_L, &pwm_R);
  pwm_L = (RPS_L == 0.0f) ? 5000 : pwm_L;
  pwm_R = (RPS_R == 0.0f) ? 5000 : pwm_R;

  pwm_set_gpio_level(PWM_LM, pwm_L);
  pwm_set_gpio_level(PWM_RM, pwm_R);

  sleep_ms(ms_sleep);

  stop();
}

// Turn for a certain amount of ms
void turn_ms(int deg, float RPS, uint32_t ms_sleep) {
  if (!deg)
    return;

  float RPS_L, RPS_R;
  if (deg > 0) {
    // Positive degrees: Turn Right (Left wheel forward, Right wheel backward)
    RPS_L = RPS;
    RPS_R = -RPS;
  } else {
    // Negative degrees: Turn Left (Left wheel backward, Right wheel forward)
    RPS_L = -RPS;
    RPS_R = RPS;
  }

  // Your move_ms function already handles the 1-second duration
  move_ms(RPS_L, RPS_R, 10);
}

// Non-blocking move
void move_continuous(float RPS_L, float RPS_R) {
  int pwm_L, pwm_R;
  calculate_PWM(-RPS_L, -RPS_R, &pwm_L, &pwm_R);
  pwm_L = (RPS_L == 0.0f) ? 5000 : pwm_L;
  pwm_R = (RPS_R == 0.0f) ? 5000 : pwm_R;

  pwm_set_gpio_level(PWM_LM, pwm_L);
  pwm_set_gpio_level(PWM_RM, pwm_R);
}

// Non-blocking turn
void turn_continuoous(int deg, float RPS) {
  if (!deg)
    return;

  float RPS_L, RPS_R;
  if (deg > 0) {
    // Positive degrees: Turn Right (Left wheel forward, Right wheel backward)
    RPS_L = RPS;
    RPS_R = -RPS;
  } else {
    // Negative degrees: Turn Left (Left wheel backward, Right wheel forward)
    RPS_L = -RPS;
    RPS_R = RPS;
  }

  // Your move_ms function already handles the 1-second duration
  move_continuous(RPS_L, RPS_R);
}

double quaternion_to_yaw(
    geometry_msgs__msg__Quaternion
        *q) { // Function to convert quaternion's orientation to yaw
  return atan2(2.0f * (q->w * q->z + q->x * q->y),
               q->w * q->w + q->x * q->x - q->y * q->y - q->z * q->z);
  // https://robotics.stackexchange.com/questions/16471/get-yaw-from-quaternion
}

double rad_to_deg(double rad) { return rad * 180.0 / PI; }
double deg_to_rad(double deg) { return deg * (M_PI / 180.0); }

// Function to easily set a Pose and primarily its Yaw
void set_pose(geometry_msgs__msg__Pose *pose, double x, double y,
              double yaw_deg) {
  pose->position.x = x;
  pose->position.y = y;
  pose->position.z = 0;

  // Convert yaw degrees to a Z-axis Quaternion
  double yaw_rad = deg_to_rad(yaw_deg);
  pose->orientation.x = 0.0;
  pose->orientation.y = 0.0;
  pose->orientation.z = sin(yaw_rad / 2.0);
  pose->orientation.w = cos(yaw_rad / 2.0);
}

// Calculate distance between 2 Poses, from own_pos to target_pos
double calculate_distance(geometry_msgs__msg__Pose *own_pos,
                          geometry_msgs__msg__Pose *target_pos) {
  double delta_x = target_pos->position.x - own_pos->position.x;
  double delta_y = target_pos->position.y - own_pos->position.y;
  return sqrt(pow(delta_x, 2) + pow(delta_y, 2));
  // Pythagoras wowzers
}

// Calculate distance and angle from robot's Pose to target's Pose, turn or move
// forward based on DEADZONE
void move_to(geometry_msgs__msg__Pose *own_pos,
             geometry_msgs__msg__Pose *target_pos) {
  if (calculate_distance(own_pos, target_pos) <= MIN_DISTANCE) {
    stop();
    return;
  }
  double yaw = rad_to_deg(quaternion_to_yaw(&own_pos->orientation));
  double angle_to_target =
      rad_to_deg(atan2(target_pos->position.y - own_pos->position.y,
                       target_pos->position.x - own_pos->position.x));
  double delta_angle = angle_to_target - yaw;

  // Normalize delta_angle to be between -180 and 180 degrees
  while (delta_angle > 180.0)
    delta_angle -= 360.0;
  while (delta_angle < -180.0)
    delta_angle += 360.0;

  // Check if the relative error is greater than the allowed deadzone
  if (fabs(delta_angle) > DEADZONE) {
    //printf(" Decision: Turning; delta_angle: %0.2f \n", delta_angle);
    turn_continuoous(delta_angle, 1);
  } else {
    //printf(" Decision: Moving forward\n");
    move_continuous(1, 1);
  }
}