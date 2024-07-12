#include "planners/Planner.cuh"

Planner::Planner(const float* h_ws, const int h_size_ws, bool verbose) : h_ws_(const_cast<float*>(h_ws)), h_size_ws_(h_size_ws)
{
    if(verbose)
        {
            printf("/***************************/\n");
            printf("/* Workspace Dimsnion: %d */\n", h_size_ws);
            for(int i = 0; i < h_size_ws; i++) printf("/* %f */\n", h_ws_[i]);
            printf("/***************************/\n");
        }
};
