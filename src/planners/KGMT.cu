#include "planners/KGMT.cuh"

KGMT::KGMT(const float* h_ws, const int h_size_ws, bool verbose) : Planner(h_ws, h_size_ws, verbose)
{
    if(verbose)
        {
            printf("/***************************/\n");
            printf("/* KGMT */\n");
            printf("/***************************/\n");
        }
};

void KGMT::plan(const float* h_initial, const float* h_goal)
{
    printf("KGMT::plan\n");
};