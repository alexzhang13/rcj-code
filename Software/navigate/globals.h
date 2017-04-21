#ifndef _MASZE_GLOBALS_H_
#define _MASZE_GLOBALS_H_

#include "navigate_defs.h"
#include <stdint.h>

const int32_t MAZE_MAX_VAL = 100000000; // used as the maximum weight in maze: wall
const int32_t MAZE_NO_ACCESS = 50000000;    // use to indicate black plate
const int32_t MAZE_OBSTACLE = 10000;    // use to indicate movable obstables
const int32_t MAZE_MIN_VAL = 10;        // open to neighbor cell
const int32_t MAZE_FAILED_WT = 1000;    // add trial weights if robot fails
const int32_t MAZE_to_STAIR  = 1; //500;		// wall to stair. Make it 1 if wants to go first
#endif
