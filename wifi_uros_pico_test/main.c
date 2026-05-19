#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "pico/cyw43_arch.h"
#include "interface/interface.h"

// ROS Message types
#include <geometry_msgs/msg/pose_array.h>
#include <nav_msgs/msg/path.h>

#define INTERFACE_WIFI

// ROS 2 Structures
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

// Subscription for positions from Game Master
rcl_subscription_t pos_sub;
geometry_msgs__msg__PoseArray pos_msg;

// // Publisher for this robot's intended path
// rcl_publisher_t path_pub;
// nav_msgs__msg__Path my_path_msg;

// // Subscription for the OTHER hunter's path (e.g., hunter_2)
// rcl_subscription_t other_hunter_sub;
// nav_msgs__msg__Path other_hunter_path_msg;

// Memory allocation limits for Micro-ROS sequences
#define MAX_ROBOTS_IN_GAME 5
#define MAX_PATH_POINTS 20

// ROS Node identifiers
// #define HUNTER_NUM 0
char* node_name = "robot";
char* node_namespace = "pico_0";

uint agent_port = 8888;

// Callback: Game Master sent updated positions
void pos_callback(const void *msgin)
{
    printf("Received message on the POSITION Topic! \n");
    const geometry_msgs__msg__PoseArray *msg = (const geometry_msgs__msg__PoseArray *)msgin;

    if (msg->poses.size > 0 && msg->poses.size < MAX_ROBOTS_IN_GAME) {
        double my_x = msg->poses.data[0].position.x;
        double my_y = msg->poses.data[0].position.y;
        printf("Received %d poses. Robot %s is at: (%.2f, %.2f)\n", msg->poses.size, node_namespace, my_x, my_y);
    } else {
        printf("Received an empty PoseArray!\n");
    }
}

// Callback: Received the other hunter's path
void other_hunter_path_callback(const void *msgin)
{
    const nav_msgs__msg__Path *msg = (const nav_msgs__msg__Path *)msgin;
}

int main()
{
    stdio_init_all();

    sleep_ms(3000);

    printf("Bababoeyyyy \n");

    #if defined(INTERFACE_WIFI)
        set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, agent_port);
    #endif

    allocator = rcl_get_default_allocator();

    // 1. Memory Pre-allocation for Incoming Game Data (PoseArray)
    static geometry_msgs__msg__Pose storage_poses[MAX_ROBOTS_IN_GAME];
    pos_msg.poses.data = storage_poses;
    pos_msg.poses.capacity = MAX_ROBOTS_IN_GAME;
    pos_msg.poses.size = 0;

    // // 2. Memory Pre-allocation for Incoming/Outgoing Paths
    // static geometry_msgs__msg__PoseStamped storage_my_path[MAX_PATH_POINTS];
    // my_path_msg.poses.data = storage_my_path;
    // my_path_msg.poses.capacity = MAX_PATH_POINTS;
    // my_path_msg.poses.size = 0;

    // static geometry_msgs__msg__PoseStamped storage_other_path[MAX_PATH_POINTS];
    // other_hunter_path_msg.poses.data = storage_other_path;
    // other_hunter_path_msg.poses.capacity = MAX_PATH_POINTS;
    // other_hunter_path_msg.poses.size = 0;

    // Wait for micro-ROS Agent
    if (rmw_uros_ping_agent(1000, 120) != RCL_RET_OK) return -1;

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, node_name, node_namespace, &support);

    // Initialize Subscriptions and Publishers
    rclc_subscription_init_default(
        &pos_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseArray), "pos");

    // rclc_subscription_init_default(
    //     &other_hunter_sub, &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Path), "/hunter_2/path"); // Global topic to hear peer

    // rclc_publisher_init_default(
    //     &path_pub, &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Path), "path"); // Resolves to /hunter_1/path

    // Executor Configuration: We have 2 subscriptions to manage
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &pos_sub, &pos_msg, &pos_callback, ON_NEW_DATA);
    // rclc_executor_add_subscription(&executor, &other_hunter_sub, &other_hunter_path_msg, &other_hunter_path_callback, ON_NEW_DATA);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
        
        // Loop Sequence:
        // 1. Check if new pos or peer paths arrived (handled by callbacks)
        // 2. Run pathfinding math if data updated
        // 3. Periodically publish your calculated path using:
        //    rcl_publish(&path_pub, &my_path_msg, NULL);
    }
    return 0;
}