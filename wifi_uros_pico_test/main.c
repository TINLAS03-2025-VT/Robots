#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "pico/cyw43_arch.h"
#include "interface/interface.h"
#include <geometry_msgs/msg/twist.h>

//#define INTERFACE_UART
#define INTERFACE_WIFI

rcl_timer_t timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

// --- board ---
#define LED_A CYW43_WL_GPIO_LED_PIN

bool recieved_flag = false;

// --- micro-ROS ---
// Timer callback
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    (recieved_flag) ? (recieved_flag = false) : (gpio_put(LED_A, 0));
}

// Twist subscription callback
void subscription_callback(const void *msgin)
{
    recieved_flag = true;
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    gpio_put(LED_A, (msg->linear.x > 0) ? 1 : 0);
}


int main()
{
    #if defined(INTERFACE_WIFI)
        set_microros_wifi_transports("mees", "meeskees", "10.45.140.138", 8888);
    #elif defined(INTERFACE_UART)
        rmw_uros_set_custom_transport(
            true,
            NULL,
            pico_serial_transport_open,
            pico_serial_transport_close,
            pico_serial_transport_write,
            pico_serial_transport_read
        );
    #endif

    gpio_init(LED_A);
    gpio_set_dir(LED_A, GPIO_OUT);

    allocator = rcl_get_default_allocator();

    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
