#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <pico/float.h>
#include <pico/double.h>

#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_array.h>

#include "settings.h"
#include "uxr/client/util/time.h"

#define DISTANCE_INVERSION_SAFEGUARD 0.001f
#define RAND_FORCE_CHANGE_INTERVAL_MS 5000

typedef geometry_msgs__msg__Point Vector3;
typedef geometry_msgs__msg__PoseArray Vector3Array;

// Calculate distance between 2 Poses, from own_pos to target_pos
static float get_vector_length(const Vector3* vector) { return sqrtf((vector->x * vector->x) + (vector->y * vector->y)); }

static Vector3 get_direction_vector(const Vector3* from, const Vector3* to) {
	Vector3 direction_vector;
	direction_vector.x = to->x - from->x;
	direction_vector.y = to->y - from->y;
	return direction_vector;
}

static void apply_goal_force(Vector3* total_force, const Vector3* current_pos, const Vector3* goal_pos) {
	if (total_force == NULL || current_pos == NULL || goal_pos == NULL) return;

	Vector3 goal_vector = get_direction_vector(current_pos, goal_pos);
	float distance_to_goal = get_vector_length(&goal_vector);
	distance_to_goal = fmaxf(distance_to_goal, DISTANCE_INVERSION_SAFEGUARD);

	Vector3 unit_goal_2d_vector = { goal_vector.x / distance_to_goal, goal_vector.y / distance_to_goal, 0.0f };

	float displacement = distance_to_goal - GOAL_TARGETED_DISTANCE;
	float force = displacement * K_GOAL;

	total_force->x += unit_goal_2d_vector.x * force;
	total_force->y += unit_goal_2d_vector.y * force;
}

static void apply_hunter_force(Vector3* total_force, const Vector3* start_pos, const Vector3Array* obstacle_poses, const Vector3* excluded_point) {
	if (total_force == NULL || start_pos == NULL || obstacle_poses == NULL || excluded_point == NULL) return;

	for (size_t i = 0; i < obstacle_poses->poses.size; i++) {
		Vector3 hunter_pos = obstacle_poses->poses.data[i].position;

		// Skip self profile and specified target exclusion pins
		if (lround(hunter_pos.z) == TAG_NUM || lround(hunter_pos.z) == lround(excluded_point->z)) continue;

		Vector3 hunter_vector = get_direction_vector(&hunter_pos, start_pos);
		float distance_to_hunter = get_vector_length(&hunter_vector);
		distance_to_hunter = fmaxf(distance_to_hunter, DISTANCE_INVERSION_SAFEGUARD);

		if (distance_to_hunter < HUNTER_SPACING) {
			Vector3 unit_hunter_2d_vector = {hunter_vector.x / distance_to_hunter, hunter_vector.y / distance_to_hunter, hunter_vector.z};

			float force = ((1.0f / (distance_to_hunter * distance_to_hunter)) - (1.0f / (HUNTER_SPACING * HUNTER_SPACING))) * K_HUNTER;
			total_force->x += unit_hunter_2d_vector.x * force;
			total_force->y += unit_hunter_2d_vector.y * force;
		}
	}
}

static float get_wall_force(float distance) {
	if (distance >= WALL_SAFETY_MARGIN) return 0.0f;
	float safe_distance = fmaxf(DISTANCE_INVERSION_SAFEGUARD, distance);
	return ((1.0f / safe_distance) - (1.0f / WALL_SAFETY_MARGIN)) * K_WALL;
}

static void apply_wall_force(Vector3* total_force, const Vector3* current_pos) {
	if (total_force == NULL || current_pos == NULL) return;

	// X-Axis Field Forces
	total_force->x += get_wall_force(current_pos->x - FIELD_MIN_X);
	total_force->x -= get_wall_force(FIELD_MAX_X - current_pos->x);

	// Y-Axis Field Forces
	total_force->y += get_wall_force(current_pos->y - FIELD_MIN_Y);
	total_force->y -= get_wall_force(FIELD_MAX_Y - current_pos->y);
}

// Helper to generate a random float between -1.0 and 1.0
static float get_random_force() { return ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f; }

// --- Core Algorithm Entrypoint ---

void calculate_optimal_move(Vector3* optimal_move, const Vector3* start_pos, const Vector3Array* obstacle_poses,const Vector3* end_goal, const Vector3* excluded_point) {
	if (optimal_move == NULL || start_pos == NULL || obstacle_poses == NULL || end_goal == NULL || excluded_point == NULL) return;

	Vector3 total_force = {0.0f, 0.0f, 0.0f};

	// 1. Calculate Vector Influences
	apply_goal_force(&total_force, start_pos, end_goal);

	apply_hunter_force(&total_force, start_pos, obstacle_poses, excluded_point);

	apply_wall_force(&total_force, start_pos);

	// 2. Inject Random Walk Jitter (Stuck State Mitigation)
	static int64_t last_calculation_time = 0;
	static Vector3 static_rand_force = {0.0f, 0.0f, 0.0f};

	int64_t current_time = uxr_millis();
	if (current_time - last_calculation_time > RAND_FORCE_CHANGE_INTERVAL_MS) {
		static_rand_force.x = get_random_force() * K_RAND;
		static_rand_force.y = get_random_force() * K_RAND;
		last_calculation_time = current_time;
	}

	total_force.x += static_rand_force.x;
	total_force.y += static_rand_force.y;

	// 3. Normalize Vector Output Step
	float force_mag = fmaxf(get_vector_length(&total_force), DISTANCE_INVERSION_SAFEGUARD);

	optimal_move->x = start_pos->x + (total_force.x / force_mag);
	optimal_move->y = start_pos->y + (total_force.y / force_mag);
	optimal_move->z = start_pos->z; // Maintain 2D plane stability
}