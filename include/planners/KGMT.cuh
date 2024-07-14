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
    void writeDeviceVectorsToCSV();

    /****************************    FIELDS    ****************************/
    // --- host fields ---
    Graph graph_;
    uint h_frontierSize_, h_frontierNextSize_, h_activeBlockSize_;
    float* h_sampleScoreThreshold_ = new float(0.0);

    // --- device fields ---
    thrust::device_vector<bool> d_frontier_, d_frontierNext_;
    thrust::device_vector<uint> d_activeFrontierIdxs_, d_frontierScanIdx_;
    thrust::device_vector<int> d_unexploredSamplesParentIdxs_;
    thrust::device_vector<float> d_unexploredSamples_;
    float *d_sampleScoreThreshold_ptr_, *d_unexploredSamples_ptr_;
    bool *d_frontier_ptr_, *d_frontierNext_ptr_;
    uint *d_activeFrontierIdxs_ptr_, *d_frontierScanIdx_ptr_;
    int* d_unexploredSamplesParentIdxs_ptr_;
};

/**************************** DEVICE FUNCTIONS ****************************/

/***************************/
/* PROPAGATE FRONTIER KERNEL 1 */
/***************************/
// --- Propagates current frontier. Builds new frontier. ---
// --- One Block Per Frontier Sample ---
__global__ void propagateFrontier_kernel1(bool* frontier, uint* activeFrontierIdxs, float* treeSamples, float* unexploredSamples,
                                          uint frontierSize, curandState* randomSeeds, int* unexploredSamplesParentIdxs, float* obstacles,
                                          int obstaclesCount, int* activeVertices, int* vertexCounter, int* activeSubVertices,
                                          int* validVertexCounter, float* vertexScores, bool* frontierNext);

__global__ void propagateFrontier_kernel2(uint* activeFrontierIdxs, bool* frontier, float* treeSamples, int iterations,
                                          float* unexploredSamples, int* unexploredSamplesParentIdxs, curandState* randomSeeds,
                                          float* obstacles, int obstaclesCount, int* vertexCounter, int* validVertexCounter,
                                          int* activeVertices, int* activeSubVertices, float* vertexScores, bool* frontierNext);

__global__ void updateFrontier_kernel(bool* frontier, uint* activeFrontierNextIdxs, uint frontierNextSize, float* xGoal, int treeSize,
                                      float* unexploredSamples, float* treeSamples, int* unexploredSamplesParentIdxs,
                                      int* treeSamplesParentIdxs, float* treeSampleCosts, float* costToGoal);