// Standard C libs
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

// Pico SDK libs
#include <hardware/watchdog.h>
#include <pico/cyw43_arch.h>
#include <pico/float.h>
#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <pico/time.h>

// ROS2 Message defs
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_array.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

// Core micro-ROS libs
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <string.h>
#include <uxr/client/util/time.h>

// Project drivers and settings
#include "geometry_msgs/msg/pose.h"
#include "movement.h"
#include "picow_udp_transports.h"
#include "rcl/publisher.h"
#include "rclc/executor_handle.h"
#include "rclc/subscription.h"
#include "rff-algorithm.h"
#include "rmw/event.h"
#include "secrets.h"
#include "settings.h"

// Handy logging functions etc.
#include "libdiy/log/log.h"
#include "std_msgs/msg/bool.h"
#include "std_msgs/msg/int32.h"
#include "std_msgs/msg/string.h"

#ifndef FRAME_ID_CAPACITY
#define FRAME_ID_CAPACITY 64
#endif

#define NODE_BASE_NAME_LENGTH 5
#define NODE_ROBOT_NUMBER_LENGTH 3
#define NODE_NAME_NULL_CHAR 1
#define NODE_NAME_LENGTH                                                       \
  (NODE_BASE_NAME_LENGTH + NODE_ROBOT_NUMBER_LENGTH + NODE_NAME_NULL_CHAR)

// --- Status Struct ---
enum game_state_t {
  STATE_IDLE,   // Initial state, waiting for commands, not moving
  STATE_PAUSED, // Game is paused, robot should stop moving
  STATE_RUNNING // Game is running, robot should move
};

// --- Configuration & Credentials ---
static const char ssid[] = WIFI_SSID;
static const char pass[] = WIFI_PASSWORD;
static const uint8_t tag_num = TAG_NUM;

// --- micro-ROS Global Objects ---
static rcl_node_t node;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rclc_executor_t executor;
static rcl_subscription_t posearray_subscriber;
static rcl_subscription_t command_subscriber;
static rcl_subscription_t seen_subscriber;
static rcl_publisher_t ready_publisher;
std_msgs__msg__String command_msg;
std_msgs__msg__Bool seen_msg;
char command_string_buffer[COMMAND_BUFFER_CAPACITY];

// --- Shared Global Application State ---
static geometry_msgs__msg__PoseArray all_robot_positions;
static geometry_msgs__msg__Pose target;
static int runner_tag = 3; // Default runner target assignment
static char frame_id_buffer[FRAME_ID_CAPACITY];
static enum game_state_t game_state = STATE_IDLE;

// --- Telemetry & Heartbeats ---
static uint32_t posearray_rx_count = 0;
static int64_t last_own_pos_callback = 0;
static int64_t last_pos_callback = 0;

// --- Forward Declarations ---
void wifi_init(void);
int wifi_connected(void);
int wifi_reconnect(void);
void ping_agent(void);
void ros_init(void);
int get_tag_pos(geometry_msgs__msg__Pose *robot_pose_buffer,
                const geometry_msgs__msg__PoseArray *positions, int tag);
void posearray_callback(const void *msgin);
void check_connections_and_spin(void);
void process_movement_logic(void);
void ros_publish_ready(rcl_publisher_t *publisher);

// --- Lookup Functions ---
int get_tag_pose(geometry_msgs__msg__Pose *robot_pose_buffer,
                 const geometry_msgs__msg__PoseArray *positions, int tag) {
  for (size_t i = 0; i < positions->poses.size; i++) {
    if (positions->poses.data[i].position.z == tag) {
      *robot_pose_buffer = positions->poses.data[i];
      return 0; // Success
    }
  }
  return 1; // Tag not found
}

void ping_agent() {
  // Test Micro-ROS agent connection
  PRINT_DEBUG("Pinging MicroROS Agent...");

  bool agent_connected = false;
  int agent_retry_count = 0;

  while (!agent_connected) {
    // Ping agent. 100ms timeout, 3 attempts per ping.
    if (rmw_uros_ping_agent(100, 3) == RCL_RET_OK) {
      agent_connected = true;
      PRINT_SUCCESS("Agent succesfully pinged!");
    } else {
      agent_retry_count++;
      PRINT_DEBUG("Agent not found. Retrying (%d/10)...", agent_retry_count);
      sleep_ms(1000);

      if (agent_retry_count >= 10)
        HARDCHECK("Micro-ROS Agent is offline or unreachable");
    }
  }
}

// --- micro-ROS Callbacks ---
void posearray_callback(const void *msgin) {
  (void)msgin;
  last_pos_callback = uxr_millis();
  posearray_rx_count++;

  //   PRINT_DEBUG("Received positions message #%lu",
  //               (unsigned long)posearray_rx_count);

  geometry_msgs__msg__Pose own_robot_pose;
  if (get_tag_pose(&own_robot_pose, &all_robot_positions, tag_num) == 0) {
    last_own_pos_callback = uxr_millis();
  }
}

void command_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;

  if (msg == NULL || msg->data.data == NULL) {
    PRINT_DEBUG("Command received, but data buffer is empty.");
    return;
  }

  strncpy(command_string_buffer, msg->data.data, COMMAND_BUFFER_CAPACITY - 1);
  command_string_buffer[COMMAND_BUFFER_CAPACITY - 1] =
      '\0'; // Force null-termination

  PRINT_DEBUG("Command received! Message content: %s", command_string_buffer);

  if (strncmp(command_string_buffer, "start ", 6) == 0) {
    PRINT_DEBUG("MATCH: Start command");

    int runner_num = -1;
    if (sscanf(command_string_buffer + 6, "%d", &runner_num) == 1) {
      if (runner_num >= 0 &&
          runner_num < 1000) { // Arbitrary upper limit for sanity
        runner_tag = runner_num;
        PRINT_DEBUG("Runner target updated to TAG [%d]", runner_tag);

        if (runner_tag != TAG_NUM)
          sleep_ms(2000);

        game_state = STATE_RUNNING;
      } else {
        PRINT_DEBUG("Invalid runner number: %d. Must be non-negative.",
                    runner_num);
      }
    } else {
      PRINT_DEBUG("Start command missing a valid integer.");
    }
  } else if (strcmp(command_string_buffer, "pause") == 0) {
    PRINT_DEBUG("MATCH: Pause command");
    stop();
    game_state = STATE_PAUSED;
  } else if (strcmp(command_string_buffer, "resume") == 0) {
    PRINT_DEBUG("MATCH: Resume command");
    game_state = STATE_RUNNING;
  } else if (strcmp(command_string_buffer, "reset") == 0) {
    PRINT_DEBUG("MATCH: Reset command");
    runner_tag = -1; // Reset to default runner target
    game_state = STATE_IDLE;
    HARDCHECK("Resetting due to command!");
  } else {
    printf("Input: '%s' -> No match found\n", command_string_buffer);
  }
}

void seen_callback(const void *msgin) {
  const std_msgs__msg__Bool *seen_msg = (const std_msgs__msg__Bool *)msgin;

  if (seen_msg->data == true) {
    PRINT_DEBUG("Robot sees runner.");
    if (get_tag_pose(&target, &all_robot_positions, runner_tag) == 0) {
      PRINT_DEBUG(
          "Target position updated. Target coordinates: x=%.2f, y=%.2f, z=%.2f",
          target.position.x, target.position.y, target.position.z);
    } else {
      PRINT_DEBUG(
          "Runner TAG [%d] missing from updates; skipped computation cycle.",
          runner_tag);
    }
  } else {
    PRINT_DEBUG("Robot has NOT seen the runner.");
  }
}

// Initializes the subscription to the topic and sets up the
// callback, adds it to executor
void ros_create_pos_sub(rclc_executor_t *executor) {
  PRINT_DEBUG("Subscribing to: %s", POS_TOPIC);
  geometry_msgs__msg__PoseArray__init(&all_robot_positions);
  if (!geometry_msgs__msg__Pose__Sequence__init(&all_robot_positions.poses,
                                                MAX_ROBOTS_IN_GAME)) {
    HARDCHECK("Failed to allocate memory for robot positions sequence");
  }

  all_robot_positions.header.frame_id.data = frame_id_buffer;
  all_robot_positions.header.frame_id.size = 0;
  all_robot_positions.header.frame_id.capacity = FRAME_ID_CAPACITY;
  all_robot_positions.header.frame_id.data[0] = '\0';

  geometry_msgs__msg__Pose__init(&target);

  if (rclc_subscription_init_best_effort(
          &posearray_subscriber, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseArray),
          POS_TOPIC) != RCL_RET_OK) {
    HARDCHECK("Failed to initialize subscription");
  }

  if (rclc_executor_add_subscription(executor, &posearray_subscriber,
                                     &all_robot_positions, &posearray_callback,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    HARDCHECK("Failed to add subscription to executor");
  }
}

void ros_create_command_sub(rclc_executor_t *executor) {
  PRINT_DEBUG("Subscribing to: %s", ROS_COMMAND_TOPIC);

  std_msgs__msg__String__init(&command_msg);

  command_msg.data.data = command_string_buffer;
  command_msg.data.capacity = COMMAND_BUFFER_CAPACITY;
  command_msg.data.size = 0;

  if (rclc_subscription_init_default(
          &command_subscriber, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          ROS_COMMAND_TOPIC) != RCL_RET_OK) {
    HARDCHECK("Failed to initialize subscription");
  }

  if (rclc_executor_add_subscription(executor, &command_subscriber,
                                     &command_msg, &command_callback,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    HARDCHECK("Failed to add subscription to executor");
  }
}

void ros_create_seen_sub(rclc_executor_t *executor) {
  PRINT_DEBUG("Subscribing to: %s", ROS_SEEN_TOPIC);

  seen_msg.data = false;

  if (rclc_subscription_init_default(
          &seen_subscriber, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
          ROS_SEEN_TOPIC) != RCL_RET_OK) {
    HARDCHECK("Failed to initialize subscription");
  }

  if (rclc_executor_add_subscription(executor, &seen_subscriber, &seen_msg,
                                     &seen_callback,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    HARDCHECK("Failed to add subscription to executor");
  }
}

void ros_init_ready_pub(rcl_publisher_t *publisher) {
  PRINT_DEBUG("Initializing publisher for: %s", ROS_READY_TOPIC);
  if (rclc_publisher_init_default(
          publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
          ROS_READY_TOPIC) != RCL_RET_OK) {
    HARDCHECK("Failed to initialize publisher");
  }
}

void ros_publish_ready(rcl_publisher_t *publisher) {
  std_msgs__msg__Int32 msg;
  std_msgs__msg__Int32__init(&msg);
  msg.data = tag_num;

  if (rcl_publish(publisher, &msg, NULL) != RCL_RET_OK) {
    HARDCHECK("Failed to publish ready message");
  }
}

void ros_init(void) {
  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
    HARDCHECK("Failed to initialize rcl init options");
  }

  rmw_init_options_t *rmw_options =
      rcl_init_options_get_rmw_init_options(&init_options);
  rmw_uros_options_set_client_key(MICRO_ROS_CLIENT_KEY, rmw_options);

  if (rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                     &allocator) != RCL_RET_OK) {
    HARDCHECK("Failed to initialize rclc support");
  }

  char node_name[NODE_NAME_LENGTH];
  snprintf(node_name, sizeof(node_name), "pico_%d", tag_num);
  if (rclc_node_init_default(&node, node_name, "pico_namespace", &support) !=
      RCL_RET_OK) {
    HARDCHECK("Failed to initialize ROS2 Node");
  }

  if (rclc_executor_init(&executor, &support.context, 3, &allocator) !=
      RCL_RET_OK) {
    HARDCHECK("Failed to initialize executor");
  }

  ros_create_pos_sub(&executor);
  ros_create_command_sub(&executor);
  ros_create_seen_sub(&executor);

  ros_init_ready_pub(&ready_publisher);

  PRINT_SUCCESS("Entire ROS initialization complete!");
}

void wifi_init() {
  PRINT_DEBUG("Starting Wi-Fi and micro-ROS transport setup...");
  int wifi_retry_count = 0;

  while (wifi_retry_count < 5) {
    bool arch_started = false;

    if (cyw43_arch_init_with_country(CYW43_COUNTRY_NETHERLANDS) == 0) {
      arch_started = true;

      // Disable power-save mode for low latency (< 5Hz micro-ROS traffic)
      cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);
      cyw43_arch_enable_sta_mode();

      if (cyw43_arch_wifi_connect_timeout_ms(
              ssid, pass, CYW43_AUTH_WPA2_AES_PSK, 10000) == 0) {
        rmw_uros_set_custom_transport(
            false, &picow_params, picow_udp_transport_open,
            picow_udp_transport_close, picow_udp_transport_write,
            picow_udp_transport_read);

        PRINT_SUCCESS("Wi-Fi connected and micro-ROS transport configured!");
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        return;
      }
    }

    wifi_retry_count++;
    printf("[RETRY] Wi-Fi Connection failed (%d/5). \n", wifi_retry_count);

    if (arch_started) {
      cyw43_arch_deinit();
    }
    sleep_ms(2000);
  }
  HARDCHECK("Wi-Fi driver stuck or Access Point unavailable");
}

int wifi_connected(void) {
  return (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) ==
          CYW43_LINK_UP);
}

int wifi_reconnect(void) {
  printf("[WARNING] Attempting Wi-Fi reconnection...\n");
  if (cyw43_arch_wifi_connect_timeout_ms(ssid, pass, CYW43_AUTH_WPA2_AES_PSK,
                                         20000) == 0) {
    printf("[SUCCESS] Reconnected.\n");
    return 1;
  }
  printf("[ERROR] Failed to reconnect.\n");
  return 0;
}

// --- Main Processing Loops ---
void check_connections_and_spin(void) {
  if (!wifi_connected()) {
    wifi_reconnect();
  }
  // Spin executor to handle callbacks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
}

void process_movement_logic(void) {
  int64_t current_time = uxr_millis();

  // Global positioning systemic loss guard
  if ((current_time - last_pos_callback) > MAX_MILLIS_WITHOUT_ANY_POSITION) {
    stop();
    HARDCHECK("Global Safety Halt: No system updates on topic [%s]", POS_TOPIC);
  }

  // Resolve Runner Target location
  if (runner_tag != -1) {
    int tag_pos = get_tag_pose(&target, &all_robot_positions, runner_tag);
    if (tag_pos != 0) {
      PRINT_DEBUG(
          "Runner TAG [%d] missing from updates; skipped computationcycle.",
          runner_tag);
      stop();
      return;
    }
  }

  // Local Positioning loss guard
  if ((current_time - last_own_pos_callback) >
      MAX_MILLIS_WITHOUT_NEW_POSITION) {
    stop();
    return;
  }

  // Algorithm Calculation and Navigation Execute
  geometry_msgs__msg__Pose own_robot_pose;
  if (all_robot_positions.poses.size > 0 &&
      get_tag_pose(&own_robot_pose, &all_robot_positions, tag_num) == 0) {

    PRINT_DEBUG("Own pos: (%.3f, %.3f) - (%.3f, %.3f, %.3f, %.3f)",
                own_robot_pose.position.x, own_robot_pose.position.y,
                own_robot_pose.orientation.x, own_robot_pose.orientation.y,
                own_robot_pose.orientation.z, own_robot_pose.orientation.w);
    PRINT_DEBUG("Target pos: (%.3f, %.3f)", target.position.x,
                target.position.y);

    geometry_msgs__msg__Pose next_step;
    geometry_msgs__msg__Pose__init(&next_step);

    if (runner_tag == TAG_NUM) {
      calculate_hunter_move(&next_step.position, &own_robot_pose.position,
                            &all_robot_positions, &target.position,
                            &target.position);
    } else {
      // TODO: Implement runner movement logic
	  stop();
	  return;
    }

    PRINT_DEBUG("Next step: (%.3f, %.3f)", next_step.position.x,
                next_step.position.y);

    move_to(&own_robot_pose, &next_step);

    geometry_msgs__msg__Pose__fini(&next_step);
  }
}

int main(void) {
  stdio_init_all();
  sleep_ms(3000); // Allow hardware lines to settle

  // Initialize Network and ROS Middleware stack
  wifi_init();
  ping_agent();
  ros_init();

  // Hardware Actuator Initialization
  init_servo_pwm(PWM_LM);
  init_servo_pwm(PWM_RM);
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);

  srand(to_ms_since_boot(get_absolute_time()));

  last_pos_callback = uxr_millis();

  PRINT_DEBUG("Publishing READY!");
  ros_publish_ready(&ready_publisher);

  // set_pose(&target, 0.0f, 0.0f, 0.0f);

  PRINT_DEBUG("Entering active runtime loop...");
  while (true) {
    check_connections_and_spin();

    if (game_state == STATE_RUNNING) {
      process_movement_logic();
    } else if (game_state == STATE_PAUSED) {
      stop();
    } else {
      // STATE_IDLE, do nothing and wait for commands
      stop();
    }
  }

  cyw43_arch_deinit();
  return 0;
}