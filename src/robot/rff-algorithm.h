#ifndef RFF_ALGORITHM_H
#define RFF_ALGORITHM_H

#include "geometry_msgs/msg/point.h"
#include "geometry_msgs/msg/pose_array.h"

void calculate_optimal_move(geometry_msgs__msg__Point* optimal_move, const geometry_msgs__msg__Point* start_pos, const geometry_msgs__msg__PoseArray* all_robot_positions, const geometry_msgs__msg__Point* end_goal, const geometry_msgs__msg__Point* excluded_point);

#endif