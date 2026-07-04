#include "rff-algorithm.h"

#include <math.h>
#include <pico/double.h>
#include <pico/float.h>
#include <stdio.h>
#include <stdlib.h>

#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_array.h>

//SB: review - geometry_msgs/msg/point.h and geometry_msgs/msg/pose_array.h are both
//already included above (angle-bracket form). these includes just
//pull in the same headers a second time. Harmless with include guards but
//worth trimming, same note as in main.c and movement.c.
#include "geometry_msgs/msg/point.h"
#include "geometry_msgs/msg/pose_array.h"
#include "settings.h"
#include "uxr/client/util/time.h"

#define DISTANCE_INVERSION_SAFEGUARD 0.001f
#define RAND_FORCE_CHANGE_INTERVAL_MS 5000

typedef geometry_msgs__msg__Point Vector3;
typedef geometry_msgs__msg__PoseArray Vector3Array;

// Calculate distance between 2 Poses, from own_pos to target_pos
static float get_vector_length(const Vector3 *vector) {
  return sqrtf((vector->x * vector->x) + (vector->y * vector->y));
}

//SB: review - It might be usefull to add a comment explaining why the Z value isnt used. readability
static Vector3 get_direction_vector(const Vector3 *from, const Vector3 *to) {
  Vector3 direction_vector;
  direction_vector.x = to->x - from->x;
  direction_vector.y = to->y - from->y;
  return direction_vector;
}

static void apply_goal_force(Vector3 *total_force, const Vector3 *current_pos,
                             const Vector3 *goal_pos) {
  if (total_force == NULL || current_pos == NULL || goal_pos == NULL)
    return;

  Vector3 goal_vector = get_direction_vector(current_pos, goal_pos);
  float distance_to_goal = get_vector_length(&goal_vector);
  distance_to_goal = fmaxf(distance_to_goal, DISTANCE_INVERSION_SAFEGUARD);

  Vector3 unit_goal_2d_vector = {goal_vector.x / distance_to_goal,
                                 goal_vector.y / distance_to_goal, 0.0f};

  float displacement = distance_to_goal - GOAL_TARGETED_DISTANCE;
  float force = displacement * K_GOAL;

  total_force->x += unit_goal_2d_vector.x * force;
  total_force->y += unit_goal_2d_vector.y * force;
}

static void apply_hunter_force(Vector3 *total_force, const Vector3 *start_pos,
                               const Vector3Array *obstacle_poses,
                               const Vector3 *target_pos) {
  if (total_force == NULL || start_pos == NULL || obstacle_poses == NULL ||
      target_pos == NULL)
    return;

  for (size_t i = 0; i < obstacle_poses->poses.size; i++) {
    Vector3 hunter_pos = obstacle_poses->poses.data[i].position;

    //SB: review - this exclusion check (skip self tag and target's tag) duplicates what
    //SB: review   hunter_array_filter() already does at the caller in
    //SB: review   rff_algorithm_apply_forces() before this function is ever invoked with
    //SB: review   filtered_hunters. Right now it's just redundant work on an
    //SB: review   already-filtered list; if hunter_array_filter's filtering gets fixed/
    //SB: review   changed later, having the exclusion logic duplicated in two places
    //SB: review   means both need to stay in sync, which is an easy thing to miss.
    //SB: review   Worth picking one place to do this filtering.
    // Skip self profile and specified target exclusion pins
    if (lround(hunter_pos.z) == TAG_NUM ||
        lround(hunter_pos.z) == lround(target_pos->z))
      continue;

    Vector3 hunter_vector = get_direction_vector(&hunter_pos, start_pos);
    float distance_to_hunter = get_vector_length(&hunter_vector);
    distance_to_hunter =
        fmaxf(distance_to_hunter, DISTANCE_INVERSION_SAFEGUARD);

    if (distance_to_hunter < HUNTER_SPACING) {
      Vector3 unit_hunter_2d_vector = {hunter_vector.x / distance_to_hunter,
                                       hunter_vector.y / distance_to_hunter,
                                       hunter_vector.z};

      float force = ((1.0f / (distance_to_hunter * distance_to_hunter)) -
                     (1.0f / (HUNTER_SPACING * HUNTER_SPACING))) *
                    K_HUNTER;
      total_force->x += unit_hunter_2d_vector.x * force;
      total_force->y += unit_hunter_2d_vector.y * force;
    }
  }
}

static float get_wall_force(float distance) {
  if (distance >= WALL_SAFETY_MARGIN)
    return 0.0f;
  float safe_distance = fmaxf(DISTANCE_INVERSION_SAFEGUARD, distance);
  return ((1.0f / safe_distance) - (1.0f / WALL_SAFETY_MARGIN)) * K_WALL;
}

static void apply_wall_force(Vector3 *total_force, const Vector3 *current_pos) {
  if (total_force == NULL || current_pos == NULL)
    return;

  // X-Axis Field Forces
  total_force->x += get_wall_force(current_pos->x - FIELD_MIN_X);
  total_force->x -= get_wall_force(FIELD_MAX_X - current_pos->x);

  // Y-Axis Field Forces
  total_force->y += get_wall_force(current_pos->y - FIELD_MIN_Y);
  total_force->y -= get_wall_force(FIELD_MAX_Y - current_pos->y);
}

// Generates a random float between 0.0 and 1.0
static inline float get_random_normalized() {
    return (float)rand() / (float)RAND_MAX;
}

static void apply_jitter_force(Vector3 *total_force) {
    if (total_force == NULL) return;

    static int64_t last_jitter_time = 0;
    static Vector3 current_jitter = {0.0f, 0.0f, 0.0f};
    int64_t current_time = uxr_millis();

    // Only recalculate jitter if the tracking interval has expired
    if (current_time - last_jitter_time > RAND_FORCE_CHANGE_INTERVAL_MS) {
        // Calculate a random point within a unit circle (Polar Distribution)
        float angle = get_random_normalized() * 2.0f * M_PI;
        float radius = sqrtf(get_random_normalized());
        
        current_jitter.x = cosf(angle) * radius * K_RAND;
        current_jitter.y = sinf(angle) * radius * K_RAND;
        
        last_jitter_time = current_time;
    }

    total_force->x += current_jitter.x;
    total_force->y += current_jitter.y;
}

static void hunter_array_filter(Vector3Array *filtered_array,
							   const Vector3Array *original_array,
							   int exclude_tag, int exclude_runner_tag) {
  if (filtered_array == NULL || original_array == NULL)
	return;

  size_t filtered_count = 0;
  for (size_t i = 0; i < original_array->poses.size; i++) {
	Vector3 current_pos = original_array->poses.data[i].position;
	long current_tag = lround(current_pos.z);

	if (current_tag != exclude_tag && current_tag != exclude_runner_tag) {
	  filtered_array->poses.data[filtered_count].position = current_pos;
	  filtered_count++;
	}
  }
  filtered_array->poses.size = filtered_count;
}

static void rff_algorithm_apply_forces(Vector3 *next_step,
                                       const Vector3 *own_pos,
                                       const Vector3Array *hunter_poses,
                                       const Vector3 *target_pos) {

  if (next_step == NULL || own_pos == NULL || hunter_poses == NULL ||
      target_pos == NULL)
    return;

  geometry_msgs__msg__Point force_buffer = {0.0f, 0.0f, 0.0f};

  // Apply goal force towards the runner's position
  apply_goal_force(&force_buffer, own_pos, target_pos);

  // Filter and apply hunter forces, excluding self and runner tag
  Vector3Array filtered_hunters;
  geometry_msgs__msg__PoseArray__init(&filtered_hunters);
  hunter_array_filter(&filtered_hunters, hunter_poses, TAG_NUM, lround(target_pos->z));
  apply_hunter_force(&force_buffer, own_pos, &filtered_hunters, target_pos);
  geometry_msgs__msg__PoseArray__fini(&filtered_hunters);

  //SB: review - this always passes lround(target_pos->z) as the
  //SB: review   tag to exclude from hunter forces. That's meaningful when target_pos is
  //SB: review   really another robot's pose (z holds its tag, as in
  //SB: review   calculate_hunter_move_2). But calculate_runner_move_2 also calls this with
  //SB: review   target_pos pointing at a static corner (z=0.0f) or a projected flee point
  //SB: review   (z=start_pos->z, i.e. own tag). In the corner case, if any real robot ever
  //SB: review   had tag 0 it would be silently excluded from hunter-avoidance forces here.
  //SB: review   Probably fine given your tag numbering, but worth a comment noting the
  //SB: review   assumption ("target_pos->z is only meaningful as a tag when target_pos is
  //SB: review   an actual robot") so it doesn't bite later if tag numbering changes.
  // Apply wall forces and random jitter
  apply_wall_force(&force_buffer, own_pos);
  apply_jitter_force(&force_buffer);

  float force_mag =
      fmaxf(get_vector_length(&force_buffer), DISTANCE_INVERSION_SAFEGUARD);
  next_step->x = own_pos->x + (force_buffer.x / force_mag);
  next_step->y = own_pos->y + (force_buffer.y / force_mag);
  next_step->z = own_pos->z; // Maintain 2D plane stability
}

// --- Core Algorithm Entrypoint ---

void calculate_hunter_move_2(
    geometry_msgs__msg__Point *optimal_move,
    const geometry_msgs__msg__Point *start_pos,
    const geometry_msgs__msg__PoseArray *all_robot_positions,
    const geometry_msgs__msg__Point *runner_position) {
  if (optimal_move == NULL || start_pos == NULL ||
      all_robot_positions == NULL || runner_position == NULL)
    return;

  rff_algorithm_apply_forces(optimal_move, start_pos, all_robot_positions,
                             runner_position);
}

static const geometry_msgs__msg__Point _runner_corners[] = {
    {1.0f, 5.0f, 0.0f}, // Unity (1, 0, 5) -> ROS (X=1, Y=5)
    {9.0f, 5.0f, 0.0f}, // Unity (9, 0, 5) -> ROS (X=9, Y=5)
    {5.0f, 1.0f, 0.0f}, // Unity (5, 0, 1) -> ROS (X=5, Y=1)
    {5.0f, 9.0f, 0.0f}  // Unity (5, 0, 9) -> ROS (X=5, Y=9)
};
#define RUNNER_CORNERS_COUNT                                                   \
  (sizeof(_runner_corners) / sizeof(_runner_corners[0]))

void calculate_runner_move_2(
    geometry_msgs__msg__Point *optimal_move,
    const geometry_msgs__msg__Point *start_pos,
    const geometry_msgs__msg__PoseArray *all_robot_positions) {
  if (optimal_move == NULL || start_pos == NULL || all_robot_positions == NULL)
    return;

  if (RUNNER_CORNERS_COUNT == 0) return;

  Vector3 flee_direction = {0.0f, 0.0f, 0.0f};
  bool hunter_nearby = false;

  // Evaluate proximity of hunters
  for (size_t i = 0; i < all_robot_positions->poses.size; i++) {
    Vector3 hunter_pos = all_robot_positions->poses.data[i].position;
    int robot_id = (int)lround(hunter_pos.z);

    if (robot_id == TAG_NUM)
      continue; // Skip self

    // Note: Checking x and y for 2D field planar distance matching your setup
    Vector3 diff = {start_pos->x - hunter_pos.x, start_pos->y - hunter_pos.y, 0.0f};
    float dist = get_vector_length(&diff);

    if (dist < HUNTER_FLEE_RADIUS) {
      hunter_nearby = true;
      flee_direction.x += diff.x;
      flee_direction.y += diff.y;
    }
  }

  static int current_corner_idx = -1;
  static int64_t last_corner_change_time = 0;
  int64_t current_time = uxr_millis();

  if (hunter_nearby) {
    last_corner_change_time = 0; // Force a new corner selection after escaping danger

    // Calculate normalized flee direction
    float flee_len = get_vector_length(&flee_direction);
    Vector3 flee_norm = {0.0f, 0.0f, 0.0f};
    if (flee_len > DISTANCE_INVERSION_SAFEGUARD) {
      flee_norm.x = flee_direction.x / flee_len;
      flee_norm.y = flee_direction.y / flee_len;
    }

    // Calculate vector pointing to center of the field
    float field_center_x = (FIELD_MIN_X + FIELD_MAX_X) / 2.0f;
    float field_center_y = (FIELD_MIN_Y + FIELD_MAX_Y) / 2.0f;
    Vector3 to_center = {field_center_x - start_pos->x, field_center_y - start_pos->y, 0.0f};
    
    float center_len = get_vector_length(&to_center);
    Vector3 center_norm = {0.0f, 0.0f, 0.0f};
    if (center_len > DISTANCE_INVERSION_SAFEGUARD) {
      center_norm.x = to_center.x / center_len;
      center_norm.y = to_center.y / center_len;
    }

    // Blend forces exactly like Unity: flee_norm + (to_center_norm * 0.8f)
    Vector3 blended = {
        flee_norm.x + (center_norm.x * 0.8f),
        flee_norm.y + (center_norm.y * 0.8f),
        0.0f
    };

    // 4. Normalize the blended vector
    float blended_len = get_vector_length(&blended);
    Vector3 blended_norm = {0.0f, 0.0f, 0.0f};
    if (blended_len > DISTANCE_INVERSION_SAFEGUARD) {
      blended_norm.x = blended.x / blended_len;
      blended_norm.y = blended.y / blended_len;
    }

    // 5. Build final projection target point and forward to your field force algorithm
    geometry_msgs__msg__Point target;
    target.x = start_pos->x + (blended_norm.x * HUNTER_FLEE_RADIUS);
    target.y = start_pos->y + (blended_norm.y * HUNTER_FLEE_RADIUS);
    target.z = start_pos->z;

    rff_algorithm_apply_forces(optimal_move, start_pos, all_robot_positions, &target);
    return;
  }

  // Patrolling corner logic (Runs safely if no enemies are inside the radius)
  bool timer_expired = (current_time - last_corner_change_time) > CORNER_CHANGE_INTERVAL_MS;

  if (current_corner_idx < 0 || timer_expired) {
    int new_index;
    do {
      new_index = rand() % RUNNER_CORNERS_COUNT;
    } while (new_index == current_corner_idx && RUNNER_CORNERS_COUNT > 1);

    current_corner_idx = new_index;
    last_corner_change_time = current_time;
  }

  rff_algorithm_apply_forces(optimal_move, start_pos, all_robot_positions, &_runner_corners[current_corner_idx]);
}

void calculate_roam_move(
    geometry_msgs__msg__Point *optimal_move,
    const geometry_msgs__msg__Point *start_pos,
    const geometry_msgs__msg__PoseArray *all_robot_positions) {
    
  if (optimal_move == NULL || start_pos == NULL || all_robot_positions == NULL)
    return;

  static bool has_roam_target = false;
  static Vector3 roam_target = {0.0f, 0.0f, 0.0f};

  // Check if we need to select a new target
  bool target_reached = false;
  if (has_roam_target) {
    Vector3 diff = {roam_target.x - start_pos->x, roam_target.y - start_pos->y, 0.0f};
    if (get_vector_length(&diff) < ROAMING_TARGET_CHANGE_DISTANCE) {
      target_reached = true;
    }
  }

  if (!has_roam_target || target_reached) {
    roam_target.x = 1.0f + (get_random_normalized() * 8.0f);
    roam_target.y = 1.0f + (get_random_normalized() * 8.0f);
    roam_target.z = (float)TAG_NUM; // Assigning own tag to avoid self-exclusion errors
    
    has_roam_target = true;

    #ifdef PICO_BOARD
    printf("[RobotBrain:%d] New roam target: (%f, %f)\n", TAG_NUM, roam_target.x, roam_target.y);
    #endif
  }

  // Pass the target to the force application pipeline (acts like driveTo(nextPoint(...)))
  rff_algorithm_apply_forces(optimal_move, start_pos, all_robot_positions, &roam_target);
}
//SB: review - commented code should be removed.

// void calculate_hunter_move(Vector3 *optimal_move, const Vector3 *start_pos,
//                            const Vector3Array *obstacle_poses,
//                            const Vector3 *end_goal,
//                            const Vector3 *excluded_point) {
//   if (optimal_move == NULL || start_pos == NULL || obstacle_poses == NULL ||
//       end_goal == NULL || excluded_point == NULL)
//     return;

//   Vector3 total_force = {0.0f, 0.0f, 0.0f};

//   // 1. Calculate Vector Influences
//   apply_goal_force(&total_force, start_pos, end_goal);

//   apply_hunter_force(&total_force, start_pos, obstacle_poses, excluded_point);

//   apply_wall_force(&total_force, start_pos);

//   // 2. Inject Random Walk Jitter (Stuck State Mitigation)
//   static int64_t last_calculation_time = 0;
//   static Vector3 static_rand_force = {0.0f, 0.0f, 0.0f};

//   int64_t current_time = uxr_millis();
//   if (current_time - last_calculation_time > RAND_FORCE_CHANGE_INTERVAL_MS) {
//     static_rand_force.x = get_random_force() * K_RAND;
//     static_rand_force.y = get_random_force() * K_RAND;
//     last_calculation_time = current_time;
//   }

//   total_force.x += static_rand_force.x;
//   total_force.y += static_rand_force.y;

//   // 3. Normalize Vector Output Step
//   float force_mag =
//       fmaxf(get_vector_length(&total_force), DISTANCE_INVERSION_SAFEGUARD);

//   optimal_move->x = start_pos->x + (total_force.x / force_mag);
//   optimal_move->y = start_pos->y + (total_force.y / force_mag);
//   optimal_move->z = start_pos->z; // Maintain 2D plane stability
// }

// // Static arena corner targets mapping to your field size rules
// static const geometry_msgs__msg__Point _runner_corners[] = {
//     {FIELD_MIN_X + 0.2f, FIELD_MIN_Y + 0.2f, 0.0f}, // Bottom Left
//     {FIELD_MAX_X - 0.2f, FIELD_MIN_Y + 0.2f, 0.0f}, // Bottom Right
//     {FIELD_MAX_X - 0.2f, FIELD_MAX_Y - 0.2f, 0.0f}, // Top Right
//     {FIELD_MIN_X + 0.2f, FIELD_MAX_Y - 0.2f, 0.0f}  // Top Left
// };
// #define RUNNER_CORNERS_COUNT                                                   \
//   (sizeof(_runner_corners) / sizeof(_runner_corners[0]))

// // --- nextPoint Mirror from Simulation's runner algorithm ---
// Vector3 next_point(const Vector3 *target, const Vector3 *pos,
//                    const Vector3Array *obstacle_poses) {
//   Vector3 force = {0.0f, 0.0f, 0.0f};

//   apply_goal_force(&force, pos, target);

//   apply_hunter_force(&force, pos, obstacle_poses, target);

//   apply_wall_force(&force, pos);

//   apply_jitter_force(&force);

//   // 5. Fallback: keep moving forward if all forces cancel out completely
//   float sqr_mag = (force.x * force.x) + (force.y * force.y);
//   if (sqr_mag < 0.001f) {
//     force.x =
//         1.0f; // Default forward heading equivalent (Vector3.forward/right)
//     force.y = 0.0f;
//     sqr_mag = 1.0f;
//   }

//   // Return pos + force.normalized
//   float force_mag = sqrtf(sqr_mag);
//   Vector3 next;
//   next.x = pos->x + (force.x / force_mag);
//   next.y = pos->y + (force.y / force_mag);
//   next.z = pos->z;
//   return next;
// }

// // --- The Runner Brain Mirror from the simulation ---
// void calculate_runner_move(Vector3 *optimal_move, const Vector3 *start_pos,
//                            const Vector3Array *obstacle_poses) {
//   if (optimal_move == NULL || start_pos == NULL || obstacle_poses == NULL)
//     return;

//   Vector3 total_force = {0.0f, 0.0f, 0.0f};
//   bool hunter_nearby = false;

//   // Evaluate proximity of hunters
//   for (size_t i = 0; i < obstacle_poses->poses.size; i++) {
//     Vector3 hunter_pos = obstacle_poses->poses.data[i].position;
//     int hunter_id = (int)lround(hunter_pos.z);

//     if (hunter_id == TAG_NUM)
//       continue; // Skip self

//     // distance = Vector3.Distance(transform.position, kvp.Value)
//     Vector3 diff = {start_pos->x - hunter_pos.x, start_pos->y - hunter_pos.y,
//                     0.0f};
//     float dist = get_vector_length(&diff);

//     if (dist < HUNTER_FLEE_RADIUS) {
//       hunter_nearby = true;
//       // fleeDirection += transform.position - kvp.Value
//       total_force.x += diff.x;
//       total_force.y += diff.y;
//     }
//   }

//   apply_hunter_force(&total_force, start_pos, obstacle_poses, &total_force);

//   static int current_corner_idx = -1;
//   static int64_t last_corner_change_time = 0;
//   int64_t current_time = uxr_millis();

//   if (hunter_nearby) {
//     last_corner_change_time =
//         0; // Force a new corner after fleeing escaping danger

//     // target = transform.position + fleeDirection.normalized * HunterFleeRadius
//     float flee_mag =
//         fmaxf(get_vector_length(&total_force), DISTANCE_INVERSION_SAFEGUARD);
//     Vector3 target;
//     target.x = start_pos->x + ((total_force.x / flee_mag) * HUNTER_FLEE_RADIUS);
//     target.y = start_pos->y + ((total_force.y / flee_mag) * HUNTER_FLEE_RADIUS);
//     target.z = start_pos->z;

//     *optimal_move = next_point(&target, start_pos, obstacle_poses);
//     return;
//   }

//   // Patrolling logic (if safe)
//   bool timer_expired =
//       (current_time - last_corner_change_time) > CORNER_CHANGE_INTERVAL_MS;

//   if (current_corner_idx < 0 || timer_expired) {
//     int new_index;
//     do {
//       new_index = rand() % RUNNER_CORNERS_COUNT;
//     } while (new_index == current_corner_idx && RUNNER_CORNERS_COUNT > 1);

//     current_corner_idx = new_index;
//     last_corner_change_time = current_time;
//   }

//   // driveTo(nextPoint(_corners[_currentCornerIndex]))
//   *optimal_move = next_point(&_runner_corners[current_corner_idx], start_pos,
//                              obstacle_poses);
// }