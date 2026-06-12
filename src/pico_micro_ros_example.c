// Standard C libs
#include <stdbool.h>
#include <stdio.h>

// Pico SDK libs
#include <hardware/watchdog.h>
#include <pico/cyw43_arch.h>
#include <pico/float.h>
#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <pico/time.h>

// ROS2 Message defs
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_array.h>
#include <std_msgs/msg/u_int64.h>

// Core micro-ROS libs
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/util/time.h>

// Project drivers and settings
#include "movement.h"
#include "picow_udp_transports.h"
#include "secrets.h"
#include "settings.h"

// Handy logging functions etc. Copied from pico_drive_test.
#include "libdiy/log/log.h"

#ifndef FRAME_ID_CAPACITY
#define FRAME_ID_CAPACITY 64
#endif

// --- WiFi credentials ---
char ssid[] = WIFI_SSID;
char pass[] = WIFI_PASSWORD;

// Kept for compatibility with the old movement app naming.
uint agent_port = AGENT_PORT;

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

// ROS Node identifiers
#define NODE_BASE_NAME_LENGTH 5
#define NODE_ROBOT_NUMBER_LENGTH 3
#define NODE_NAME_NULL_CHAR 1
#define NODE_NAME_LENGTH (NODE_BASE_NAME_LENGTH + NODE_ROBOT_NUMBER_LENGTH + NODE_NAME_NULL_CHAR)
uint8_t robot_num = ROBOT_NUM;
char node_name[NODE_NAME_LENGTH];

// Live Globals
uint8_t runner = -1; // Changes to another robot's num, which is the runner

static char frame_id_buffer[FRAME_ID_CAPACITY];
static uint32_t posearray_rx_count = 0;

// --- Timer callback: publishes incrementing counter ---
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
	(void) last_call_time;
	if (timer != NULL) {
		(void) rcl_publish(&publisher, &pub_msg, NULL);
		pub_msg.data++;
	}
}

// Callback: Game Master sent updated positions, save in all_robot_positions
int64_t last_pos_callback;
void posearray_callback(const void *msgin) {
	last_pos_callback = uxr_millis();
	(void) msgin;

	posearray_rx_count++;

	printf("\nReceived message on the POSITION Topic! \n");

	if (all_robot_positions.poses.size > robot_num) {
		printf("Own robot position num [%d]: (%.2f , %.2f) \n\n", robot_num, all_robot_positions.poses.data[robot_num].position.x, all_robot_positions.poses.data[robot_num].position.y);
	}
	else {
		printf("[WARNING] PoseArray received, but robot_num [%d] is outside poses.size [%u]\n", robot_num, (unsigned int) all_robot_positions.poses.size);
	}

	printf("posearray received %lu, poses=%u\n", (unsigned long) posearray_rx_count, (unsigned int) all_robot_positions.poses.size);
}

void setup_transport() {
	rmw_uros_set_custom_transport(
		false,&picow_params, picow_udp_transport_open,
		picow_udp_transport_close, picow_udp_transport_write,
		picow_udp_transport_read
	);
}

void ping_agent() {
	// Test Micro-ROS agent connection
	printf("Pinging MicroROS Agent... \n");

	bool agent_connected = false;
	int agent_retry_count = 0;

	while (!agent_connected) {
		// Ping agent. 100ms timeout, 3 attempts per ping.
		if (rmw_uros_ping_agent(100, 3) == RCL_RET_OK) {
			agent_connected = true;
			PRINT_SUCCESS("Agent succesfully pinged!");
		}
		else {
			agent_retry_count++;
			printf("[RETRY] Agent not found. Retrying (%d/10)...\n", agent_retry_count);
			sleep_ms(1000);

			if (agent_retry_count >= 10) HARDCHECK("Micro-ROS Agent is offline or unreachable");
		}
	}
}

void ros_init() {
	allocator = rcl_get_default_allocator();

	geometry_msgs__msg__PoseArray__init(&all_robot_positions);
	if (!geometry_msgs__msg__Pose__Sequence__init(
		&all_robot_positions.poses, MAX_ROBOTS_IN_GAME
	)) HARDCHECK("Failed to allocate memory for robot positions sequence");

	all_robot_positions.header.frame_id.data = frame_id_buffer;
	all_robot_positions.header.frame_id.size = 0;
	all_robot_positions.header.frame_id.capacity = FRAME_ID_CAPACITY;
	all_robot_positions.header.frame_id.data[0] = '\0';

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

	if (rcl_init_options_init(
		&init_options, allocator) != RCL_RET_OK
	) HARDCHECK("Failed to initialize rcl init options");

	rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	rmw_uros_options_set_client_key(MICRO_ROS_CLIENT_KEY, rmw_options);

	if (rclc_support_init_with_options(
		&support, 0, NULL, &init_options, &allocator
	) != RCL_RET_OK) HARDCHECK("Failed to initialize rclc support");

	snprintf(node_name, sizeof(node_name), "pico_%d", robot_num);
	if (rclc_node_init_default(
		&node, node_name, "pico_namespace", &support
	) != RCL_RET_OK) HARDCHECK("Failed to initialize ROS2 Node");

	if (rclc_publisher_init_best_effort(
		&publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt64), "pico_counter") != RCL_RET_OK
	) HARDCHECK("Failed to initialize publisher");

	// Initialize Subscription
	char *pos_sub_str = POS_TOPIC;
	PRINT_DEBUG("Subscribing to: %s ", pos_sub_str);
	if (rclc_subscription_init_best_effort(
			&posearray_subscriber, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseArray),
			pos_sub_str) != RCL_RET_OK
	) HARDCHECK("Failed to initialize subscription");

	// Publish every 1000ms
	if (rclc_timer_init_default2(
		&timer, &support,
		RCL_MS_TO_NS(1000), timer_callback, true) != RCL_RET_OK
	) HARDCHECK("Failed to initialize timer");

	// Initialize Executor
	if (rclc_executor_init(
		&executor, &support.context, 2, &allocator) != RCL_RET_OK
	) HARDCHECK("Failed to initialize executor");

	if (rclc_executor_add_timer(
		&executor, &timer) != RCL_RET_OK
	) HARDCHECK("Failed to add timer to executor");

	if (rclc_executor_add_subscription(
		&executor, &posearray_subscriber,
		&all_robot_positions, &posearray_callback,
		ON_NEW_DATA) != RCL_RET_OK
	) HARDCHECK("Failed to add subscription to executor");

	PRINT_SUCCESS("Entire ROS initialization complete!");
}

void wifi_init() {
	PRINT_DEBUG("Starting Wi-Fi and micro-ROS transport setup...");

	bool transport_ready = false;
	int wifi_retry_count = 0;

	while (!transport_ready) {
		bool arch_started = false;

		if (cyw43_arch_init_with_country(CYW43_COUNTRY_NETHERLANDS) == 0) {
			arch_started = true;

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

			if (cyw43_arch_wifi_connect_timeout_ms(ssid, pass, CYW43_AUTH_WPA2_AES_PSK, 10000) == 0) {
				setup_transport();
				transport_ready = true;
				PRINT_SUCCESS("Wi-Fi connected and micro-ROS transport configured!");
				cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
			}
		}

		if (!transport_ready) {
			wifi_retry_count++;
			printf("[RETRY] Connection failed (%d/5). \n", wifi_retry_count);

			if (arch_started) cyw43_arch_deinit();

			sleep_ms(2000);

			if (wifi_retry_count >= 5) HARDCHECK("Wi-Fi driver stuck or AP unavailable");
		}
	}
}

int wifi_reconnect() {
	if (cyw43_arch_wifi_connect_timeout_ms(ssid, pass, CYW43_AUTH_WPA2_AES_PSK, 20000)) {
		printf("failed to reconnect.\n");
		return 0;
	}
	else {
		printf("Connected.\n");
		return 1;
	}
}

int wifi_connected() {
	int link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
	if (link_status != CYW43_LINK_UP) {
		printf("[WARNING] Wi-Fi Link down!\n");
		return 0;
	}
	return 1;
}

void motor_test() {
	move_ms(MOVE_SPEED_RPS, 0, 1000);
	move_ms(0, MOVE_SPEED_RPS, 1000);
}

int main() {
	stdio_init_all();

	PRINT_DEBUG("Start up... ");

	sleep_ms(3000);

	motor_test();

	wifi_init(); // Try to connect to the WIFI given in "settings.h"

	ping_agent(); // Ping microros agent

	pub_msg.data = 0;
	ros_init(); // Init everything for MicroROS

	// Initialize continuous PWM channels
	init_servo_pwm(PWM_LM);
	init_servo_pwm(PWM_RM);

	geometry_msgs__msg__Pose target;

	set_pose(&target, 0.0, 0.0, 0.0); // Set target Pose for testing point to point movement

	//   PRINT_DEBUG("Testing motors");

	//   motor_test();

	//   sleep_ms(2000);

	PRINT_DEBUG("Entering loop...");

	last_pos_callback = -1; // Set last_pos_callback to -1 to prevent driving without any received position

	while (true) {
		if (!wifi_connected()) wifi_reconnect();

		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

		if (last_pos_callback == -1) {
			PRINT_DEBUG(".");
			continue;
		}

		int64_t current_time = uxr_millis();
		if (current_time - last_pos_callback > MAX_MILLIS_WITHOUT_NEW_POSITION) {
			PRINT_DEBUG("No pos received for longer than %d seconds \n", MAX_MILLIS_WITHOUT_NEW_POSITION);
			stop();
		}
		else if (all_robot_positions.poses.size > 0 && (size_t) robot_num < all_robot_positions.poses.size) {
			move_to(&all_robot_positions.poses.data[robot_num], &target, MOVE_SPEED_RPS, TURN_SPEED_RPS);
		}
	}

	cyw43_arch_deinit();
	return 0;
}
