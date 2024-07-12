#pragma once
#include <stdio.h>

class Planner
{
public:
    // --- constructor ---
    Planner() = default;
    Planner(const float* h_ws, const int h_size_ws, bool verbose = false);

    // methods
    virtual void plan(const float* h_initial, const float* h_goal) = 0;

    // --- fields ---
    float* h_ws_;
    int h_size_ws_;
};