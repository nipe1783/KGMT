#pragma once
#include "planners/Planner.cuh"

class KGMT : public Planner
{
public:
    // --- constructor ---
    KGMT() = default;
    KGMT(const float* h_ws, const int h_size_ws, int numDisc = 10, int maxTreeSize = 30000, bool verbose = false);
    // --- methods ---
    void plan(const float* h_initial, const float* h_goal) override;
};