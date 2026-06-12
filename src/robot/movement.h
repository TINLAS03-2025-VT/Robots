#ifndef __MOVEMENT__
#define __MOVEMENT__

#include "geometry_msgs/msg/pose.h"
#include <pico/stdlib.h>

// --- Hardware Pins ---
#define PWM_LM 7 // Left motor
#define PWM_RM 6 // Right motor
#define EN_L 11  // Left IR encoder
#define EN_R 14  // Right IR encoder

float eval_poly3(float x, float coeff[4]);
void encoder_callback(uint gpio, uint32_t events);
void init_servo_pwm(uint gpio);
void calculate_PWM(float RPS_L, float RPS_R, int *pwm_L, int *pwm_R);
void set_motor_speed(float RPS_L, float RPS_R, uint32_t ms_sleep);
float measure_motor_speed(int pwm_val, char motor, int steps);
double map(double x, double in_min, double in_max, double out_min,
           double out_max);
void move(float RPS_L, float RPS_R);
void stop(void);
void move_ms(float RPS_L, float RPS_R, uint32_t ms_sleep);
void turn(int deg);
void move_to(geometry_msgs__msg__Pose *own_pos,
             geometry_msgs__msg__Pose *target_pos, const double move_speed_rps,
             const double turn_speed_rps);
void set_pose(geometry_msgs__msg__Pose *pose, double x, double y,
              double yaw_deg);

#endif
