#pragma once
#include <stdio.h>
#include "config/config.h"

/***************************/
/* BroadPhase CC FUNCTION */
/***************************/
// --- Bounding box around trajectory segment. Checks if bounding box overlaps with obstacle.
__device__ bool isBroadPhaseValid(float *bbMin, float *bbMax, float *obs);

/***************************/
/* Motion Validity CC FUNCTION */
/***************************/
__device__ bool isMotionValid(float *x0, float *x1, float *bbMin, float *bbMax, float *obstacles, int obstaclesCount);