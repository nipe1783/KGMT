#pragma once
#include <stdio.h>

class Planner
{
public:
    // --- constructor ---
    Planner() = default;
    Planner(const float* h_ws, const int h_size_ws, int numDisc = 10, int maxTreeSize = 30000, bool verbose = false);

    // methods
    virtual void plan(const float* h_initial, const float* h_goal) = 0;

    // --- fields ---
    float* h_ws_;
    int h_size_ws_;
    int h_numDisc_;
    int h_maxTreeSize_;
};