#include "planners/KGMT.cuh"

KGMT::KGMT(const float* h_ws, const int h_size_ws, int numDisc = 10, int maxTreeSize = 30000, bool verbose)
    : Planner(h_ws, h_size_ws, numDisc, maxTreeSize, verbose)
{
    if(verbose)
        {
            printf("/***************************/\n");
            printf("/* Planner Type: KGMT */\n");
            printf("/***************************/\n");
        }
};

void KGMT::plan(const float* h_initial, const float* h_goal)
{
    printf("KGMT::plan\n");
};