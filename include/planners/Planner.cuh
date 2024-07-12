#pragma once
#include <stdio.h>
#include "config/config.h"

class Planner
{
public:
    // --- constructor ---
    Planner() = default;
    Planner(const float h_ws, int numDisc = 10, int maxTreeSize = 30000, float goalThreshold = 0.5, int maxIterations = 100);

    // methods
    virtual void plan(const float* h_initial, const float* h_goal) = 0;

    // --- fields ---
    float h_ws_, h_goalThreshold_;
    int h_numDisc_, h_maxTreeSize_, h_maxIterations_, h_treeSize_ = 0;
};
