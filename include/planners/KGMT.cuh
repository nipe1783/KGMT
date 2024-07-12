#pragma once
#include "planners/Planner.cuh"
#include "graphs/Graph.cuh"

class KGMT : public Planner
{
public:
    /**************************** CONSTRUCTORS ****************************/
    KGMT() = default;
    KGMT(float ws, int numDisc = 10, int maxTreeSize = 30000, float goalThreshold = 0.5, int maxIterations = 100);

    /****************************    METHODS    ****************************/
    void plan(float* h_initial, float* h_goal) override;
    void propagateFrontier();

    /****************************    FIELDS    ****************************/
    // --- host fields ---
    Graph graph_;
    int h_frontierSize_;
    float* h_sampleScoreThreshold_ = new float(0.0);

    // --- device fields ---
    thrust::device_vector<bool> d_frontier_;
    float* d_sampleScoreThreshold_ptr_;
    bool* d_frontier_ptr_;
};

/**************************** DEVICE FUNCTIONS ****************************/
__global__ void propagateFrontier_kernel(bool* frontier, float* treeSamples);