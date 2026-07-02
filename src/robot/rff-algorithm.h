#ifndef RFF_ALGORITHM_H
#define RFF_ALGORITHM_H

#include "geometry_msgs/msg/point.h"
#include "geometry_msgs/msg/pose_array.h"

// void calculate_hunter_move(Vector3* optimal_move, const Vector3* start_pos,
// const Vector3Array* obstacle_poses,const Vector3* end_goal, const Vector3*
// excluded_point);
void calculate_hunter_move(
    geometry_msgs__msg__Point *optimal_move,
    const geometry_msgs__msg__Point *start_pos,
    const geometry_msgs__msg__PoseArray *all_robot_positions,
    const geometry_msgs__msg__Point *end_goal,
    const geometry_msgs__msg__Point *excluded_point);

// Force calculations for the Runner persona
void calculate_runner_move(geometry_msgs__msg__Point *optimal_move,
                           const geometry_msgs__msg__Point *start_pos,
                           const geometry_msgs__msg__PoseArray *obstacle_poses);

void calculate_hunter_move_2(
    geometry_msgs__msg__Point *optimal_move,
    const geometry_msgs__msg__Point *start_pos,
    const geometry_msgs__msg__PoseArray *all_robot_positions,
    const geometry_msgs__msg__Point *runner_position);

void calculate_runner_move_2(
    geometry_msgs__msg__Point *optimal_move,
    const geometry_msgs__msg__Point *start_pos,
    const geometry_msgs__msg__PoseArray *all_robot_positions);

void calculate_roam_move(
    geometry_msgs__msg__Point *optimal_move,
    const geometry_msgs__msg__Point *start_pos,
    const geometry_msgs__msg__PoseArray *all_robot_positions);

#endif