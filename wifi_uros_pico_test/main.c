#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "pico/cyw43_arch.h"
#include "interface/interface.h"
#include <pico/stdlib.h>

// ROS Message types
#include <geometry_msgs/msg/pose_array.h>
#include <nav_msgs/msg/path.h>

// ROS 2 Structures
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

// Subscription for positions from Game Master
rcl_subscription_t pos_sub;
geometry_msgs__msg__PoseArray pos_msg;

// Memory allocation limits for Micro-ROS sequences
#define MAX_ROBOTS_IN_GAME 5
#define MAX_PATH_POINTS 20

// ROS Node identifiers
uint8_t robot_num = 0;
char node_name[8]; // 8 bytes
uint agent_port = 8888;

// Physical pin setup
#define LED_PIN WL_GPIO0

// Callback: Game Master sent updated positions
void pos_callback(const void *msgin)
{
    printf("Received message on the POSITION Topic! \n");
    const geometry_msgs__msg__PoseArray *msg = (const geometry_msgs__msg__PoseArray *)msgin;

    if (msg->poses.size > 0 && msg->poses.size < MAX_ROBOTS_IN_GAME)
    {
        double my_x = msg->poses.data[robot_num].position.x;
        double my_y = msg->poses.data[robot_num].position.y;
        printf("Received %d poses. Robot %d is at: (%.2f, %.2f)\n", msg->poses.size, robot_num, my_x, my_y);
    }
    else
    {
        printf("Received an empty PoseArray!\n");
    }
}

int main()
{
    stdio_init_all();

    printf("Start up... \n");

    sleep_ms(3000);

    /* Connect to WIFI */
    printf("Starting Wi-Fi and micro-ROS transport setup...\n");
    bool transport_ready = false;
    while (!transport_ready)
    {
        if (set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, agent_port) == 0)
        {
            transport_ready = true;
            printf("[SUCCESS] Wi-Fi connected and micro-ROS transport configured!\n");
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        }
        else
        {
            printf("[RETRY] Connection failed. Trying again in 2 seconds...\n");

            cyw43_arch_deinit();

            sleep_ms(2000);
        }
    }

    /* Test Micro-ROS agent connection */
    printf("Pinging MicroROS Agent... \n");
    bool agent_connected = false;
    while (!agent_connected)
    {
        // Ping once. If it fails, keep the USB active and try again
        if (rmw_uros_ping_agent(100, 3) == RCL_RET_OK)
        {
            agent_connected = true;
            printf("Successfully connected to agent!\n");
        }
        else
        {
            printf("Agent not found. Retrying...\n");
            sleep_ms(1000);
        }
    }

    allocator = rcl_get_default_allocator();

    // Memory Pre-allocation for Incoming Game Data (PoseArray)
    static geometry_msgs__msg__Pose storage_poses[MAX_ROBOTS_IN_GAME];
    pos_msg.poses.data = storage_poses;
    pos_msg.poses.capacity = MAX_ROBOTS_IN_GAME;
    pos_msg.poses.size = 0;

    // Initialize Node
    rclc_support_init(&support, 0, NULL, &allocator);
    snprintf(node_name, sizeof(node_name), "pico_%d", robot_num); // Append robot_num for the node_name
    rclc_node_init_default(&node, node_name, "", &support);

    // Initialize Subscriptions and Publishers
    rclc_subscription_init_default(
        &pos_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseArray), "robots/pos");

    // Executor Configuration:
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &pos_sub, &pos_msg, &pos_callback, ON_NEW_DATA);

    printf("Setup complete. Spinning executor...\n");
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
    }
    
    return 0;
}