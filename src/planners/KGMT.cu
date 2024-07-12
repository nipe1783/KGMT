#include "planners/KGMT.cuh"
#include "config/config.h"

KGMT::KGMT(float h_ws, int h_numDisc, int h_maxTreeSize, float h_goalThreshold, int h_maxIterations)
    : Planner(h_ws, h_numDisc, h_maxTreeSize, h_goalThreshold, h_maxIterations)
{
    h_graph_ = Graph(h_ws_);
    if(VERBOSE)
        {
            printf("/* Planner Type: KGMT */\n");
            printf("/* Number of R1 Vertices: %d */\n", NUM_R1_VERTICES);
            printf("/* Number of R2 Vertices: %d */\n", NUM_R2_VERTICES);
            printf("/***************************/\n");
        }
}

void KGMT::plan(const float* h_initial, const float* h_goal)
{
    int itr = 0;
    while(itr < h_maxIterations_)
        {
            itr++;
        }
}

__global__ void updateGrid() {}