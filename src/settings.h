#ifndef __SETTINGS__
#define __SETTINGS__

#include <stdint.h>

#define ROBOT_NUM 0

// Movement settings
#define DEADZONE 10.0    	// Deadzone degrees, minimal difference to turn
#define MOVE_SPEED_RPS 0.4
#define TURN_SPEED_RPS 0.1
#define MIN_DISTANCE 3  	// Distance at which to stop moving to a point

#define MIN_DISTANCE_TO_ROBOT 0.2	// Min distance needed to push away from robot
#define MIN_DISTANCE_TO_GOAL 0.2	// Min distance needed to pull towards the goal

// ROS Topic identifiers
#define POS_TOPIC "/robots/pos"
#define MAX_MILLIS_WITHOUT_NEW_POSITION 500

#define MAX_ROBOTS_IN_GAME 5

#endif