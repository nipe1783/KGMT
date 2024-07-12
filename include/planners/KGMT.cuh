#pragma once
#include "planners/Planner.cuh"
#include "graphs/Graph.cuh"

class KGMT : public Planner
{
public:
    // --- constructor ---
    KGMT() = default;
    KGMT(float ws, int numDisc = 10, int maxTreeSize = 30000, float goalThreshold = 0.5, int maxIterations = 100);

    // --- methods ---
    /***************************/
    /* Kinodynamic Motion Planning FUNCTION */
    /***************************/
    void plan(const float* h_initial, const float* h_goal) override;

    // --- fields ---
    Graph h_graph_;
};