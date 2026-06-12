#include <stdio.h>
#include <stdlib.h>
// #include <math.h>
#include <time.h>
#include <pico/float.h>

#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_array.h>
#include <geometry_msgs/msg/point.h>

#include "geometry_msgs/msg/point.h"
#include "settings.h"

#ifndef K_ATT
#define K_ATT 1.0   // Attraction weight
#endif

#ifndef K_REP
#define K_REP 5.0   // Repulsion weight
#endif

#ifndef K_RAND
#define K_RAND 0.3  // Randomness weight (chance)
#endif

#ifndef ROBOT_INFLUENCE_DIST
#define ROBOT_INFLUENCE_DIST 2.0 // Distance at which robots start pushing away
#endif

// Calculate distance between 2 Poses, from own_pos to target_pos
float get_vector_length(geometry_msgs__msg__Point *vector) {
  return sqrt(pow(vector->x, 2) + pow(vector->y, 2));
}

geometry_msgs__msg__Point get_direction_vector(geometry_msgs__msg__Point *point_1,
                          geometry_msgs__msg__Point *point_2){
	geometry_msgs__msg__Point* direction_vector = geometry_msgs__msg__Point__create();
	direction_vector->x = point_1->x - point_2->x;
	direction_vector->y = point_1->y - point_2->y;
	direction_vector->z = point_1->z - point_2->z;
	return *direction_vector;
}

// Helper to generate a random float between -1.0 and 1.0
float get_random_force() {
    return ((float)rand() / RAND_MAX) * 2.0 - 1.0;
}

// Algorithm implementation
/*
*	RFF Algorithm, applies Vectors (in this case geometry_msgs__msg__Point is kept for standardization) and normalizes at the end
*/
void calculate_optimal_move(geometry_msgs__msg__Point* optimal_move, geometry_msgs__msg__PoseArray* all_robot_positions, geometry_msgs__msg__Point end_goal) {
	geometry_msgs__msg__Point* total_force = geometry_msgs__msg__Point__create();

    // ATTRACTIVE FORCE (Pull to Goal)
	geometry_msgs__msg__Point own_pos = all_robot_positions->poses.data[ROBOT_NUM].position;
	geometry_msgs__msg__Point* end_goal_vector = geometry_msgs__msg__Point__create();
    if (get_vector_length(end_goal_vector) > MIN_DISTANCE_TO_GOAL) {
		*end_goal_vector = get_direction_vector(&own_pos, &end_goal);		// Pull
    }else{
		*end_goal_vector = get_direction_vector(&end_goal, &own_pos);		// Push
	}
	total_force->x += end_goal_vector->x * K_ATT;
    total_force->y += end_goal_vector->y * K_ATT;

	geometry_msgs__msg__Point__destroy(end_goal_vector);	// Clear end_goal_vector memory
	
    // REPULSIVE FORCE (Push from other robots)
    for (size_t i = 0; i < all_robot_positions->poses.size; i++) {
		if (i == ROBOT_NUM) continue;	// Skip own number
		// TODO: Skip Runner position!!!

        geometry_msgs__msg__Point other_robot_pos = all_robot_positions->poses.data[i].position;
		geometry_msgs__msg__Point repulsive_vector = get_direction_vector(&other_robot_pos, &own_pos);
        if (get_vector_length(&repulsive_vector) > MIN_DISTANCE_TO_ROBOT) {
			total_force->x += repulsive_vector.x * K_REP;
			total_force->y += repulsive_vector.y * K_REP;
        }
    }

    // RANDOM FORCE (Deadlock prevention)
    total_force->x += get_random_force() * K_RAND;
    total_force->y += get_random_force() * K_RAND;

    // Final Vector: Next optimal position step
	float final_dir_vector_length = get_vector_length(total_force);
    optimal_move->x = own_pos.x + total_force->x / get_vector_length(total_force);
    optimal_move->y = own_pos.y + total_force->y / get_vector_length(total_force);
    optimal_move->z = own_pos.z; // Keeping Z stable for 2D move

	geometry_msgs__msg__Point__destroy(total_force);		// Clear total_force memory
}

