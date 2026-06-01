#include <stdio.h>
#include <string.h>
#include <math.h>
#include "geometry_msgs/msg/pose.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include "interface/movement/movement.h"
#include "settings.h"

#define MOVE_TO_LOGIC_TEST 1

int main() {
    stdio_init_all();
    
    // Setup Encoder Hardware Input Pins
    gpio_init(EN_L);
    gpio_set_dir(EN_L, GPIO_IN);
    gpio_pull_down(EN_L);
    
    gpio_init(EN_R);
    gpio_set_dir(EN_R, GPIO_IN);
    gpio_pull_down(EN_R);
    
    // Initialize continuous PWM channels
    init_servo_pwm(PWM_LM);
    init_servo_pwm(PWM_RM);
    
    sleep_ms(2000); // Grace period to initialize tracking monitors safely

    geometry_msgs__msg__Pose robot;
    geometry_msgs__msg__Pose target;

    printf("=== STARTING NAVIGATION LOGIC TESTS ===\n\n");

    // Test 5: Target just outside deadzone (5~7 deg to the right)
    printf("Test 5: Target 6 deg to the right (Outside Deadzone)\n");
    set_pose(&robot,  0.0, 0.0, 0.0);
    set_pose(&target, 1.0, -0.105, 0.0); // atan2(-0.105, 1) ~ -6 degrees
    move_to(&robot, &target);
    printf("\n");

    while (1) {
        move_to(&robot, &target);
    }
    
    return 0;
}