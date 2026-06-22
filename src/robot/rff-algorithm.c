#include "rff-algorithm.h"

#include <math.h>
#include <pico/double.h>
#include <pico/float.h>
#include <stdio.h>
#include <stdlib.h>
#include <pico/float.h>
#include <pico/double.h>

#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_array.h>

#include "geometry_msgs/msg/pose_array.h"
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

static void apply_jitter_force(Vector3 *total_force) {
  if (total_force == NULL)
    return;

  static int64_t last_jitter_time = 0;
  static Vector3 current_jitter = {0.0f, 0.0f, 0.0f};
  int64_t current_time = uxr_millis();

  if (current_time - last_jitter_time > RAND_FORCE_CHANGE_INTERVAL_MS) {
    float angle = ((float)rand() / (float)RAND_MAX) * 2.0f * M_PI;
    float r = sqrtf((float)rand() / (float)RAND_MAX);
    current_jitter.x = (cosf(angle) * r) * K_RAND;
    current_jitter.y = (sinf(angle) * r) * K_RAND;
    last_jitter_time = current_time;
  }

  total_force->x += current_jitter.x;
  total_force->y += current_jitter.y;
}

// Helper to generate a random float between -1.0 and 1.0
static float get_random_force() { return ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f; }

// --- Core Algorithm Entrypoint ---

void calculate_hunter_move(Vector3* optimal_move, const Vector3* start_pos, const Vector3Array* obstacle_poses,const Vector3* end_goal, const Vector3* excluded_point) {
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

// Static arena corner targets mapping to your field size rules
static const geometry_msgs__msg__Point _runner_corners[] = {
    {FIELD_MIN_X + 0.2f, FIELD_MIN_Y + 0.2f, 0.0f}, // Bottom Left
    {FIELD_MAX_X - 0.2f, FIELD_MIN_Y + 0.2f, 0.0f}, // Bottom Right
    {FIELD_MAX_X - 0.2f, FIELD_MAX_Y - 0.2f, 0.0f}, // Top Right
    {FIELD_MIN_X + 0.2f, FIELD_MAX_Y - 0.2f, 0.0f}  // Top Left
};
#define RUNNER_CORNERS_COUNT                                                   \
  (sizeof(_runner_corners) / sizeof(_runner_corners[0]))

// --- 1. The nextPoint Mirror ---
Vector3 next_point(const Vector3 *target,
           const Vector3 *pos,
           const Vector3Array *obstacle_poses) {
  Vector3 force = {0.0f, 0.0f, 0.0f};

  apply_goal_force(&force, pos, target);

  apply_hunter_force(&force, pos, obstacle_poses, target);

  apply_wall_force(&force, pos);

  apply_jitter_force(&force);

  // 5. Fallback: keep moving forward if all forces cancel out completely
  float sqr_mag = (force.x * force.x) + (force.y * force.y);
  if (sqr_mag < 0.001f) {
    force.x =
        1.0f; // Default forward heading equivalent (Vector3.forward/right)
    force.y = 0.0f;
    sqr_mag = 1.0f;
  }

  // Return pos + force.normalized
  float force_mag = sqrtf(sqr_mag);
  Vector3 next;
  next.x = pos->x + (force.x / force_mag);
  next.y = pos->y + (force.y / force_mag);
  next.z = pos->z;
  return next;
}

// --- 2. The Runner Brain Mirror ---
void calculate_runner_move(
    Vector3 *optimal_move,
    const Vector3 *start_pos,
    const Vector3Array *obstacle_poses) {
  if (optimal_move == NULL || start_pos == NULL || obstacle_poses == NULL)
    return;

  Vector3 flee_direction = {0.0f, 0.0f, 0.0f};
  bool hunter_nearby = false;

  // Evaluate proximity of hunters
  for (size_t i = 0; i < obstacle_poses->poses.size; i++) {
    Vector3 hunter_pos =
        obstacle_poses->poses.data[i].position;
    int hunter_id = (int)lround(hunter_pos.z);

    if (hunter_id == TAG_NUM)
      continue; // Skip self

    // distance = Vector3.Distance(transform.position, kvp.Value)
    Vector3 diff = {start_pos->x - hunter_pos.x,
                     start_pos->y - hunter_pos.y, 0.0f};
    float dist = get_vector_length(&diff);

    if (dist < HUNTER_FLEE_RADIUS) {
      hunter_nearby = true;
      // fleeDirection += transform.position - kvp.Value
      flee_direction.x += diff.x;
      flee_direction.y += diff.y;
    }
  }

  static int current_corner_idx = -1;
  static int64_t last_corner_change_time = 0;
  int64_t current_time = uxr_millis();

  if (hunter_nearby) {
    last_corner_change_time =
        0; // Force a new corner after fleeing escaping danger

    // target = transform.position + fleeDirection.normalized * HunterFleeRadius
    float flee_mag =
        fmaxf(get_vector_length(&flee_direction), DISTANCE_INVERSION_SAFEGUARD);
    Vector3 target;
    target.x =
        start_pos->x + ((flee_direction.x / flee_mag) * HUNTER_FLEE_RADIUS);
    target.y =
        start_pos->y + ((flee_direction.y / flee_mag) * HUNTER_FLEE_RADIUS);
    target.z = start_pos->z;

    *optimal_move = next_point(&target, start_pos, obstacle_poses);
    return;
  }

  // Patrolling logic (if safe)
  bool timer_expired =
      (current_time - last_corner_change_time) > CORNER_CHANGE_INTERVAL_MS;

  if (current_corner_idx < 0 || timer_expired) {
    int new_index;
    do {
      new_index = rand() % RUNNER_CORNERS_COUNT;
    } while (new_index == current_corner_idx && RUNNER_CORNERS_COUNT > 1);

    current_corner_idx = new_index;
    last_corner_change_time = current_time;
  }

  // driveTo(nextPoint(_corners[_currentCornerIndex]))
  *optimal_move = next_point(&_runner_corners[current_corner_idx], start_pos,
                             obstacle_poses);
}