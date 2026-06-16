#include <stdio.h>
#include <stdlib.h>
// #include <math.h>
#include <pico/float.h>
#include <time.h>

#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_array.h>

#include "geometry_msgs/msg/point.h"
#include "settings.h"
#include "uxr/client/util/time.h"

#ifndef ROBOT_INFLUENCE_DIST
#define ROBOT_INFLUENCE_DIST 2.0 // Distance at which robots start pushing away
#endif

#define EPSILON 0.001f

// Calculate distance between 2 Poses, from own_pos to target_pos
float get_vector_length(geometry_msgs__msg__Point *vector) {
  return sqrt(powf(vector->x, 2) + powf(vector->y, 2));
}

geometry_msgs__msg__Point
get_direction_vector(const geometry_msgs__msg__Point *point_1,
                     const geometry_msgs__msg__Point *point_2) {
  geometry_msgs__msg__Point direction_vector;

  direction_vector.x = point_2->x - point_1->x;
  direction_vector.y = point_2->y - point_1->y;
  direction_vector.z = point_2->z - point_1->z;

  return direction_vector;
}

// Helper to generate a random float between -1.0 and 1.0
float get_random_force() {
  return ((float)rand() / (float)RAND_MAX) * 2.0 - 1.0;
}

static float get_wall_force(float distance) {
  if (distance >= WALL_SAFETY_MARGIN) {
    return 0.0f;
  }
  // Prevent division by zero if the robot overshoots a boundary
  float safe_dist = (distance < EPSILON) ? EPSILON : distance;
  return (1.0f / safe_dist) * K_WALL;
}

void apply_border_repulsion(geometry_msgs__msg__Point *total_force,
                            const geometry_msgs__msg__Point *current_pos) {
  // X-Axis Field Forces
  total_force->x += get_wall_force(current_pos->x - FIELD_MIN_X);
  total_force->x -= get_wall_force(FIELD_MAX_X - current_pos->x);

  // Y-Axis Field Forces
  total_force->y += get_wall_force(current_pos->y - FIELD_MIN_Y);
  total_force->y -= get_wall_force(FIELD_MAX_Y - current_pos->y);
}

static void apply_goal_forces(geometry_msgs__msg__Point *total_force,
                              const geometry_msgs__msg__Point *start_pos,
                              const geometry_msgs__msg__Point *end_goal) {
  geometry_msgs__msg__Point goal_vector =
      get_direction_vector(start_pos, end_goal);
  float dist_to_goal = get_vector_length(&goal_vector);

  if (dist_to_goal < EPSILON) {
    return;
  }

  if (dist_to_goal > MIN_DISTANCE_TO_GOAL) {
    // Pull towards target goal
    total_force->x += (goal_vector.x / dist_to_goal) * K_GOAL_ATT;
    total_force->y += (goal_vector.y / dist_to_goal) * K_GOAL_ATT;
  } else {
    // Push away if overshooting target center radius
    total_force->x -= (goal_vector.x / dist_to_goal) * K_GOAL_REP;
    total_force->y -= (goal_vector.y / dist_to_goal) * K_GOAL_REP;
  }
}

static void
apply_obstacle_repulsion(geometry_msgs__msg__Point *total_force,
                         const geometry_msgs__msg__Point *start_pos,
                         const geometry_msgs__msg__PoseArray *obstacle_poses,
                         const geometry_msgs__msg__Point *excluded_point) {
  for (size_t i = 0; i < obstacle_poses->poses.size; i++) {
    geometry_msgs__msg__Point obstacle_pos =
        obstacle_poses->poses.data[i].position;

    // Skip self profile and specified target exclusion pins
    if (obstacle_pos.z == TAG_NUM || obstacle_pos.z == excluded_point->z) {
      continue;
    }

    geometry_msgs__msg__Point rep_vector =
        get_direction_vector(&obstacle_pos, start_pos);
    float dist_to_robot = get_vector_length(&rep_vector);

    if (dist_to_robot < EPSILON) {
      continue;
    }

    // Apply artificial physical size thresholding bounding
    if (dist_to_robot < MIN_DISTANCE_TO_ROBOT) {
      dist_to_robot = EPSILON;
    }

    // Proportional repulsion (stronger when close)
    float rep_magnitude = (1.0f / dist_to_robot);

    total_force->x += (rep_vector.x / dist_to_robot) * rep_magnitude * K_REP;
    total_force->y += (rep_vector.y / dist_to_robot) * rep_magnitude * K_REP;
  }
}

// --- Core Algorithm Entrypoint ---

void calculate_optimal_move(geometry_msgs__msg__Point *optimal_move,
                            const geometry_msgs__msg__Point *start_pos,
                            const geometry_msgs__msg__PoseArray *obstacle_poses,
                            geometry_msgs__msg__Point end_goal,
                            geometry_msgs__msg__Point excluded_point) {
  geometry_msgs__msg__Point total_force = {0.0f, 0.0f, 0.0f};

  // 1. Calculate Vector Influences
  apply_goal_forces(&total_force, start_pos, &end_goal);
  apply_obstacle_repulsion(&total_force, start_pos, obstacle_poses,
                           &excluded_point);
  apply_border_repulsion(&total_force, start_pos);

  // 2. Inject Random Walk Jitter (Stuck State Mitigation)
  static int last_calculation_time = 0;
  if (uxr_millis() - last_calculation_time > RAND_FORCE_CHANGE_INTERVAL_MS) {
    total_force.x += get_random_force() * K_RAND;
    total_force.y += get_random_force() * K_RAND;
	last_calculation_time = uxr_millis();
  }

  // 3. Normalize Vector Output Step
  float force_mag = get_vector_length(&total_force);
  if (force_mag < EPSILON) {
    force_mag = EPSILON;
  }

  optimal_move->x = start_pos->x + (total_force.x / force_mag);
  optimal_move->y = start_pos->y + (total_force.y / force_mag);
  optimal_move->z = start_pos->z; // Maintain 2D plane stability
}