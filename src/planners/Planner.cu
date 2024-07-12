#include "planners/Planner.cuh"
#include "config/config.h"

Planner::Planner(float ws, int numDisc, int maxTreeSize, float goalThreshold, int maxIterations)
    : h_ws_(ws), h_numDisc_(numDisc), h_maxTreeSize_(maxTreeSize), h_goalThreshold_(goalThreshold), h_maxIterations_(maxIterations)
{
    if(VERBOSE)
        {
            printf("/***************************/\n");
            printf("/* Workspace Dimension: %d */\n", DIM);
            printf("/* Workspace Size: %f */\n", h_ws_);
            printf("/* Discretization steps in trajectory: %d */\n", h_numDisc_);
            printf("/* Max Tree Size: %d */\n", h_maxTreeSize_);
            printf("/* Goal Distance Threshold: %f */\n", h_goalThreshold_);
            printf("/* Max Planning Iterations: %d */\n", h_maxIterations_);
        }
}
