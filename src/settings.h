#ifndef __SETTINGS__
#define __SETTINGS__

#include <stdint.h>

#define TAG_NUM 3
#define MAX_ROBOTS_IN_GAME 5

/* 	Pathfinding algorithm settings
*	These settings were tweaked to work well on a field of 1x1 meters, having 
*/
#define K_GOAL 1.2f 					// Attraction/repulsion weight of goal
#define GOAL_TARGETED_DISTANCE 2.0f		// Ideal radius around the runner

#define K_HUNTER 1.5f 					// Hunter repulsion weight
#define HUNTER_SPACING 2.8f				// Ideal distance between brother hunters

#define K_WALL 2.5f						// Wall repulsion, always higher than total hunter repulsion
#define WALL_SAFETY_MARGIN 1.2f			// Distance from wall where repulsion kicks in

#define K_RAND 0.7f 					// Randomness weight (chance)

#define FIELD_MIN_X 0.0f
#define FIELD_MAX_X 10.0f
#define FIELD_MIN_Y 0.0f
#define FIELD_MAX_Y 10.0f

// Movement settings
#define DEADZONE 10.0f    		// Deadzone degrees, minimal difference to turn
#define MOVE_SPEED_RPS 0.4f		// Moving speed forward
#define MAX_TURNING_RPS 0.6f		// Max turn speed
#define MIN_TURNING_RPS 0.05f

// No effect, because it's continuous
#define MIN_MOVE_DISTANCE 0.3f  	// Distance at which to stop moving straight to a point
#define MAX_MOVE_DISTANCE_BACKWARD MIN_MOVE_DISTANCE + 1	// Maximum distance from which it won't simply drive backwards

// ROS Topic identifiers
#define POS_TOPIC "/robots/pos"
#define MAX_MILLIS_WITHOUT_NEW_POSITION 500		// Not receiving their own tag's position
#define MAX_MILLIS_WITHOUT_ANY_POSITION	5000		// Receiving nothing on the position callback at all


#endif