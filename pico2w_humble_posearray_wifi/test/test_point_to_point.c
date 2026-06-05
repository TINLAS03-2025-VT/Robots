#include <stdio.h>
#include <math.h>

#define DEADZONE 5.0
#define PI 3.141592654

typedef struct {
    double x;
    double y;
    double z;
    double w;
} Quaternion;

typedef struct {
    double x;
    double y;
    double z;
} Point;

typedef struct {
    Point position;
    Quaternion orientation;
} geometry_msgs__msg__Pose;

double rad_to_deg(double rad) { return rad * (180.0 / M_PI); }
double deg_to_rad(double deg) { return deg * (M_PI / 180.0); }

// Converts a Z-axis quaternion back to a yaw angle in radians
double quaternion_to_yaw(const Quaternion* q) {
    return atan2(2.0 * (q->w * q->z + q->x * q->y), 1.0 - 2.0 * (q->y * q->y + q->z * q->z));
}

void set_pose(geometry_msgs__msg__Pose* pose, double x, double y, double yaw_deg) {
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

// Hardware mocks
void turn_raw(double delta, int p1, int p2) { printf("   [HARDWARE ACTION] -> TURNING\n"); }
void move_ms(int p1, int p2, int p3)       { printf("   [HARDWARE ACTION] -> MOVING FORWARD\n"); }

void move_to(geometry_msgs__msg__Pose *own_pos, geometry_msgs__msg__Pose *target_pos) {
    double yaw = rad_to_deg(quaternion_to_yaw(&own_pos->orientation));
    double angle_to_target = rad_to_deg(atan2(target_pos->position.y - own_pos->position.y,
                                             target_pos->position.x - own_pos->position.x));

    double delta_angle = angle_to_target - yaw;
    
    // Normalize delta_angle to be between -180 and 180 degrees
    while (delta_angle > 180.0) delta_angle -= 360.0;
    while (delta_angle < -180.0) delta_angle += 360.0;

    printf("   Yaw: %.1f° | Target Ang: %.1f° | Delta: %.1f°\n", yaw, angle_to_target, delta_angle);

    // FIXED LOGIC: Check if the relative error is greater than the allowed deadzone
    if (fabs(delta_angle) > DEADZONE) {
        turn_raw(delta_angle, 1, 0);
    } else {
        move_ms(1, 1, 0);
    }
}

// 4. Test Orchestrator
int main() {
    geometry_msgs__msg__Pose robot;
    geometry_msgs__msg__Pose target;

    printf("=== STARTING NAVIGATION LOGIC TESTS ===\n\n");

    // Test 1: Target right in front, Robot facing East (0 deg)
    printf("Test 1: Target directly in front (0 deg)\n");
    set_pose(&robot,  0.0, 0.0, 0.0);
    set_pose(&target, 1.0, 0.0, 0.0);
    move_to(&robot, &target);
    printf("\n");

    // Test 2: Target 90 degrees to the right (Clockwise, -90 deg)
    printf("Test 2: Target 90 deg to the right\n");
    set_pose(&robot,  0.0, 0.0, 0.0);
    set_pose(&target, 0.0, -1.0, 0.0);
    move_to(&robot, &target);
    printf("\n");

    // Test 3: Target 90 degrees to the left (Counter-Clockwise, 90 deg)
    printf("Test 3: Target 90 deg to the left\n");
    set_pose(&robot,  0.0, 0.0, 0.0);
    set_pose(&target, 0.0, 1.0, 0.0);
    move_to(&robot, &target);
    printf("\n");

    // Test 3.1: Target 180 degrees behind
    printf("Test 3.1: Target 180 degrees behind\n");
    set_pose(&robot,  0.0, 0.0, 0.0);
    set_pose(&target, -1.0, 0.0, 0.0);
    move_to(&robot, &target);
    printf("\n");

    // Test 4: Target inside deadzone (< 5 deg to the left)
    printf("Test 4: Target 3 deg to the left (Within Deadzone)\n");
    set_pose(&robot,  0.0, 0.0, 0.0);
    set_pose(&target, 1.0, 0.052, 0.0); // atan2(0.052, 1) ~ 3 degrees
    move_to(&robot, &target);
    printf("\n");

    // Test 5: Target just outside deadzone (5~7 deg to the right)
    printf("Test 5: Target 6 deg to the right (Outside Deadzone)\n");
    set_pose(&robot,  0.0, 0.0, 0.0);
    set_pose(&target, 1.0, -0.105, 0.0); // atan2(-0.105, 1) ~ -6 degrees
    move_to(&robot, &target);
    printf("\n");

    // Test 6: Robot rotated (Yaw 45 deg), Target directly in front of its trajectory
    printf("Test 6: Robot at 45 deg, Target at 45 deg relative to origin\n");
    set_pose(&robot,  0.0, 0.0, 45.0);
    set_pose(&target, 0.707, 0.707, 0.0); 
    move_to(&robot, &target);
    printf("\n");

    return 0;
}