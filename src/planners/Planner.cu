#include "planners/Planner.cuh"

Planner::Planner(const float* h_ws, const int h_size_ws, int numDisc, int maxTreeSize, bool verbose)
    : h_ws_(const_cast<float*>(h_ws)), h_size_ws_(h_size_ws), h_numDisc_(numDisc), h_maxTreeSize_(maxTreeSize)
{
    if(verbose)
        {
            printf("/***************************/\n");
            printf("/* Workspace Dimsnion: %d */\n", h_size_ws);
            for(int i = 0; i < h_size_ws; i++) printf("/* %f */\n", h_ws_[i]);
            printf("/***************************/\n");
            printf("/* Discretization steps in trajectory: %d */\n", h_numDisc_);
            printf("/* Max Tree Size: %d */\n", h_maxTreeSize_);
            printf("/***************************/\n");
        }
};
