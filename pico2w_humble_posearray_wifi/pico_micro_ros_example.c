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

#include "movement.h"

// --- WiFi credentials ---
char ssid[] = "dana";      // Edit this
char pass[] = "maaikedana"; //Edit this

#ifndef POSE_CAPACITY
#define POSE_CAPACITY 8
#endif

#ifndef FRAME_ID_CAPACITY
#define FRAME_ID_CAPACITY 64
#endif

// --- micro-ROS objects ---
rcl_publisher_t publisher;
rcl_subscription_t posearray_subscriber;
std_msgs__msg__UInt64 pub_msg;
geometry_msgs__msg__PoseArray posearray_msg;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_timer_t timer;
rclc_executor_t executor;

static char frame_id_buffer[FRAME_ID_CAPACITY];
static uint32_t posearray_rx_count = 0;

static volatile bool posearray_new_data = false;
static geometry_msgs__msg__Pose target_pose;
static int robot_num = 0;

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
    posearray_new_data = true;

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

    geometry_msgs__msg__PoseArray__init(&posearray_msg);
    geometry_msgs__msg__Pose__Sequence__init(&posearray_msg.poses, POSE_CAPACITY);

    posearray_msg.header.frame_id.data = frame_id_buffer;
    posearray_msg.header.frame_id.size = 0;
    posearray_msg.header.frame_id.capacity = FRAME_ID_CAPACITY;
    posearray_msg.header.frame_id.data[0] = '\0';

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(
        &executor,
        &posearray_subscriber,
        &posearray_msg,
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

    init_servo_pwm(PWM_LM);
    init_servo_pwm(PWM_RM);
    stop();
	
	set_pose(&target_pose, 0.0, 0.0, 0.0);

    while (true) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    if (posearray_new_data) {
        posearray_new_data = false;

        if (posearray_msg.poses.size > robot_num) {
            move_to(&posearray_msg.poses.data[robot_num], &target_pose);
        } else {
            stop();
        }
    }
}

    cyw43_arch_deinit();
    return 0;
}
