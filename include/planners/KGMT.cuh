#pragma once
#include "planners/Planner.cuh"
#include "graphs/Graph.cuh"

class KGMT : public Planner
{
public:
    /**************************** CONSTRUCTORS ****************************/
    KGMT();

    /****************************    METHODS    ****************************/
    void plan(float* h_initial, float* h_goal, float* d_obstacles_ptr, uint h_obstaclesCount) override;
    void propagateFrontier(float* d_obstacles_ptr, uint h_obstaclesCount);
    void updateFrontier();

    /****************************    FIELDS    ****************************/
    // --- host fields ---
    Graph graph_;
    uint h_frontierSize_;
    float* h_sampleScoreThreshold_ = new float(0.0);

    // --- device fields ---
    thrust::device_vector<bool> d_frontier_;
    thrust::device_vector<uint> d_activeFrontierIdxs_, d_unexploredSamplesParentIdxs_, d_frontierScanIdx_;
    thrust::device_vector<float> d_unexploredSamples_;
    float *d_sampleScoreThreshold_ptr_, *d_unexploredSamples_ptr_;
    bool* d_frontier_ptr_;
    uint *d_activeFrontierIdxs_ptr_, *d_unexploredSamplesParentIdxs_ptr_;
};

/**************************** DEVICE FUNCTIONS ****************************/

/***************************/
/* PROPAGATE FRONTIER KERNEL  */
/***************************/
// --- Propagates current frontier. Builds new frontier. ---
// --- One Block Per Frontier Sample ---
__global__ void
propagateFrontier_kernel(bool* frontier, uint* activeFrontierIdxs, float* treeSamples, float* unexploredSamples, uint frontierSize,
                         curandState* randomSeeds, uint* unexploredSamplesParentIdxs, float* obstacles, int obstaclesCount,
                         int* vertexCounter, int* activeSubVertices, int* validVertexCounter, float* vertexScores, bool* newFrontier);