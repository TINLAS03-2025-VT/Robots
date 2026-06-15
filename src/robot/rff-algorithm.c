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

#ifndef K_ATT
#define K_ATT 1.0 // Attraction weight
#endif

#ifndef K_REP
#define K_REP 5.0 // Repulsion weight
#endif

#ifndef K_RAND
#define K_RAND 0.3 // Randomness weight (chance)
#endif

#ifndef ROBOT_INFLUENCE_DIST
#define ROBOT_INFLUENCE_DIST 2.0 // Distance at which robots start pushing away
#endif

// Calculate distance between 2 Poses, from own_pos to target_pos
float get_vector_length(geometry_msgs__msg__Point *vector) {
  return sqrt(powf(vector->x, 2) + powf(vector->y, 2));
}

geometry_msgs__msg__Point
get_direction_vector(geometry_msgs__msg__Point *point_1,
                     geometry_msgs__msg__Point *point_2) {
  geometry_msgs__msg__Point direction_vector;

  direction_vector.x = point_2->x - point_1->x;
  direction_vector.y = point_2->y - point_1->y;
  direction_vector.z = point_2->z - point_1->z;

  return direction_vector;
}

// Helper to generate a random float between -1.0 and 1.0
float get_random_force() { return ((float)rand() / RAND_MAX) * 2.0 - 1.0; }

// Calculate the repulsive force for a single boundary
static float get_wall_force(float distance) {
  if (distance >= WALL_SAFETY_MARGIN) return 0.0f;
  
  // Prevent division by zero or negative distances if the robot overshoots
  float safe_distance = (distance < 0.001f) ? 1.0f : distance;

  return (1.0f / safe_distance) * K_WALL;
}

// Apply the repulsion from the border/walls to the total force
void apply_border_repulsion(geometry_msgs__msg__Point *total_force, 
                            const geometry_msgs__msg__Point *current_pos) {
  // X-axis forces: Push right from left wall (+), push left from right wall (-)
  total_force->x += get_wall_force(current_pos->x - FIELD_MIN_X);
  total_force->x -= get_wall_force(FIELD_MAX_X - current_pos->x);
  
  // Y-axis forces: Push up from bottom wall (+), push down from top wall (-)
  total_force->y += get_wall_force(current_pos->y - FIELD_MIN_Y);
  total_force->y -= get_wall_force(FIELD_MAX_Y - current_pos->y);
}

// Algorithm implementation
/*
 *	RFF Algorithm, applies Vectors (in this case geometry_msgs__msg__Point
 * is kept for standardization) and normalizes at the end
 */
void calculate_optimal_move(geometry_msgs__msg__Point *optimal_move,
                            geometry_msgs__msg__Point *start_pos,
                            geometry_msgs__msg__PoseArray *obstacle_poses,
                            geometry_msgs__msg__Point end_goal) {
  geometry_msgs__msg__Point total_force = {0.0, 0.0, 0.0};

  // ENDGOAL ATTRACTION (Pull to Goal, or Push when too close)
  geometry_msgs__msg__Point end_goal_vector =
      get_direction_vector(start_pos, &end_goal);
  float dist_to_goal = get_vector_length(&end_goal_vector);

  if (dist_to_goal > 0.001f) {
    if (dist_to_goal > MIN_DISTANCE_TO_GOAL) {
      end_goal_vector = get_direction_vector(start_pos, &end_goal); // Pull
    } else {
      end_goal_vector = get_direction_vector(&end_goal, start_pos); // Push
    }

    total_force.x += (end_goal_vector.x / dist_to_goal) * K_ATT;
    total_force.y += (end_goal_vector.y / dist_to_goal) * K_ATT;
  }

  // OBSTACLE REPULSION (Push from other robots)
  for (size_t i = 0; i < obstacle_poses->poses.size; i++) {
    geometry_msgs__msg__Point obstacle_pos =
        obstacle_poses->poses.data[i].position;

    if (obstacle_pos.z == TAG_NUM)
      continue; // Skip own pos

    // TODO: Skip runner

    geometry_msgs__msg__Point repulsive_vector =
        get_direction_vector(&obstacle_pos, start_pos);
    float dist_to_robot = get_vector_length(&repulsive_vector);
    if (dist_to_robot < 0.001f)
      continue; // Avoid nan

    float rep_magnitude =
        (1.0f /
         dist_to_robot); // Make sure the repulsion is greater when closer

    total_force.x +=
        (repulsive_vector.x / dist_to_robot) * rep_magnitude * K_REP;
    total_force.y +=
        (repulsive_vector.y / dist_to_robot) * rep_magnitude * K_REP;
  }

  // BORDER REPULSION
  apply_border_repulsion(&total_force, start_pos);

  // RANDOM FORCE (Deadlock prevention)
  total_force.x += get_random_force() * K_RAND;
  total_force.y += get_random_force() * K_RAND;

  // Final Vector: Next optimal position step
  float final_force_magnitude = get_vector_length(&total_force);
  optimal_move->x = start_pos->x + (total_force.x / final_force_magnitude);
  optimal_move->y = start_pos->y + (total_force.y / final_force_magnitude);
  optimal_move->z = start_pos->z; // Keeping Z stable for 2D move
}
