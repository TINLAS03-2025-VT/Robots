#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int64.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_array.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "picow_udp_transports.h"
#include "pico/cyw43_arch.h"

// Movement
#include "movement.h"
#include "settings.h"

// --- WiFi credentials ---
#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_WIFI_SSID"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif

#ifndef AGENT_IP
#define AGENT_IP "YOUR_AGENT_IP"
#endif

#ifndef AGENT_PORT
#define AGENT_PORT 8888
#endif

// --- WiFi credentials ---
char ssid[] = WIFI_SSID;
char pass[] = WIFI_PASSWORD;

// Kept for compatibility with the old movement app naming.
uint agent_port = AGENT_PORT;

#ifndef POSE_CAPACITY
#define POSE_CAPACITY 8
#endif

#ifndef FRAME_ID_CAPACITY
#define FRAME_ID_CAPACITY 64
#endif

#define MAX_ROBOTS_IN_GAME 5

// --- micro-ROS objects ---
rcl_publisher_t publisher;
rcl_subscription_t posearray_subscriber;
std_msgs__msg__UInt64 pub_msg;
geometry_msgs__msg__PoseArray all_robot_positions;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_timer_t timer;
rclc_executor_t executor;

static char frame_id_buffer[FRAME_ID_CAPACITY];
static uint32_t posearray_rx_count = 0;

// Robot identifier from old movement app
uint8_t robot_num = 0;

// --- Timer callback: publishes incrementing counter ---
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void) last_call_time;
    if (timer != NULL) {
        rcl_publish(&publisher, &pub_msg, NULL);
        pub_msg.data++;
    }
}

// --- Subscription callback: receives PoseArray ---
void posearray_callback(const void *msgin)
{
    const geometry_msgs__msg__PoseArray *msg = (const geometry_msgs__msg__PoseArray *)msgin;

    posearray_rx_count++;

    printf("posearray received %lu, poses=%u\n",
           (unsigned long)posearray_rx_count,
           (unsigned int)msg->poses.size);
}

void setup_transport()
{
    rmw_uros_set_custom_transport(
        false,
        &picow_params,
        picow_udp_transport_open,
        picow_udp_transport_close,
        picow_udp_transport_write,
        picow_udp_transport_read
    );
}

void setup_ros()
{
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    rclc_publisher_init_best_effort(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt64),
        "pico_counter"
    );

    rclc_subscription_init_best_effort(
        &posearray_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseArray),
        "/robots/pos"
    );

    // Publish every 1000ms
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), timer_callback);

    geometry_msgs__msg__PoseArray__init(&all_robot_positions);
    geometry_msgs__msg__Pose__Sequence__init(&all_robot_positions.poses, MAX_ROBOTS_IN_GAME);

    all_robot_positions.header.frame_id.data = frame_id_buffer;
    all_robot_positions.header.frame_id.size = 0;
    all_robot_positions.header.frame_id.capacity = FRAME_ID_CAPACITY;
    all_robot_positions.header.frame_id.data[0] = '\0';

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(
        &executor,
        &posearray_subscriber,
        &all_robot_positions,
        &posearray_callback,
        ON_NEW_DATA
    );
}

int main()
{
    stdio_init_all();
    cyw43_arch_init_with_country(CYW43_COUNTRY_NETHERLANDS);

    // Disable CYW43 power-save mode.
    //
    // The radio defaults to a power-save mode that puts the chip to sleep
    // after a 200 ms idle timer. For periodic micro-ROS traffic at message
    // intervals slower than ~5 Hz, this imposes a wake-up penalty on every
    // message, adding ~80 ms of latency. Disabling power save keeps the
    // radio active and produces consistent latency across message rates.
    //
    // Comment out this line if you need to minimise power draw and your
    // application does not need consistent low-latency communication.
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    cyw43_arch_enable_sta_mode();
    cyw43_arch_wifi_connect_timeout_ms(ssid, pass, CYW43_AUTH_WPA2_AES_PSK, 10000);

    setup_transport();

    // Wait for the micro-ROS agent to respond. Up to 60 attempts with a
    // 1000 ms timeout each (~60 s total worst case, typically much less).
    const int timeout_ms = 1000;
    const uint8_t attempts = 60;
    rcl_ret_t ret = 0;
    int loop = 0;

    for (; loop < attempts; loop++) {
        ret = rmw_uros_ping_agent(timeout_ms, 1);
        if (ret == RCL_RET_OK) break;
    }
    if (loop == attempts) return ret;

    pub_msg.data = 0;
    setup_ros();

    // Initialize continuous PWM channels
    init_servo_pwm(PWM_LM);
    init_servo_pwm(PWM_RM);

    geometry_msgs__msg__Pose target;

    set_pose(&target, 0.0, 0.0, 0.0);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        if (all_robot_positions.poses.size > 0 &&
            (size_t)robot_num < all_robot_positions.poses.size) {
            move_to(&all_robot_positions.poses.data[robot_num], &target);
        }
    }

    cyw43_arch_deinit();
    return 0;
}
