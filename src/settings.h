#ifndef __SETTINGS__
#define __SETTINGS__

#include <stdint.h>

#define TAG_NUM 3
#define MAX_ROBOTS_IN_GAME 5

/* 	Pathfinding algorithm settings
*	These settings were tweaked to work well on a field of 1x1 meters, having 
*/
#define K_GOAL_ATT 2.0 		// Attraction weight of goal
#define K_GOAL_REP K_REP * MAX_ROBOTS_IN_GAME + 1.0f	// Goal repulsion, always higher than total hunter repulsion
#define K_REP 2.2 		// Repulsion weight
#define K_WALL K_REP * MAX_ROBOTS_IN_GAME + 1.0f		// Wall repulsion, always higher than total hunter repulsion
#define K_RAND 0.7 		// Randomness weight (chance)
#define RAND_FORCE_CHANGE_INTERVAL_MS 4000

#define MIN_DISTANCE_TO_ROBOT 1.5
#define MIN_DISTANCE_TO_GOAL 1.5	// Distance at which goal stops pulling and pushes away.
#define WALL_SAFETY_MARGIN 0.4f  // Distance from wall where repulsion kicks in

//#define MIN_DISTANCE_TO_ROBOT 0.2	// Min distance needed to push away from robot

#define FIELD_MIN_X 0.0f
#define FIELD_MAX_X 10.0f
#define FIELD_MIN_Y 0.0f
#define FIELD_MAX_Y 10.0f

// Movement settings
#define DEADZONE 10.0    		// Deadzone degrees, minimal difference to turn
#define MOVE_SPEED_RPS 0.4		// Moving speed forward	
#define MAX_TURNING_RPS 0.6		// Max turn speed
#define MIN_TURNING_RPS 0.05

// No effect, because it's continuous
#define MIN_MOVE_DISTANCE 0.3  	// Distance at which to stop moving straight to a point
#define MAX_MOVE_DISTANCE_BACKWARD MIN_MOVE_DISTANCE + 1	// Maximum distance from which it won't simply drive backwards

// ROS Topic identifiers
#define POS_TOPIC "/robots/pos"
#define MAX_MILLIS_WITHOUT_NEW_POSITION 500		// Not receiving their own tag's position
#define MAX_MILLIS_WITHOUT_ANY_POSITION	5000		// Receiving nothing on the position callback at all


#endif