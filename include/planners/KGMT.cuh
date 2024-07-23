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
    void planBench(float* h_initial, float* h_goal, float* d_obstacles_ptr, uint h_obstaclesCount, int benchItr);
    void propagateFrontier(float* d_obstacles_ptr, uint h_obstaclesCount);
    void updateFrontier();
    void updateGraphTotalCount();
    void updateGraphValidCount();
    void updateGraphSubVerticesOccupancy();
    void writeDeviceVectorsToCSV(int itr);
    void writeExecutionTimeToCSV(double time);

    /****************************    FIELDS    ****************************/
    // --- host fields ---
    Graph graph_;
    uint h_frontierSize_, h_frontierNextSize_, h_activeBlockSize_;

    // --- device fields ---
    thrust::device_vector<bool> d_frontier_, d_frontierNext_;
    thrust::device_vector<uint> d_activeFrontierIdxs_, d_frontierScanIdx_;
    thrust::device_vector<int> d_unexploredSamplesParentIdxs_;
    thrust::device_vector<float> d_unexploredSamples_, d_goalSample_;
    float *d_unexploredSamples_ptr_, *d_goalSample_ptr_;
    bool *d_frontier_ptr_, *d_frontierNext_ptr_;
    uint *d_activeFrontierIdxs_ptr_, *d_frontierScanIdx_ptr_;
    int* d_unexploredSamplesParentIdxs_ptr_;

    // --- Graph Count Helpers ---
    thrust::device_vector<int> d_unexploredSamplesVertices_, d_updateGraphCounter_, d_updateGraphKeys_, d_updateGraphKeysCounter_,
      d_updateGraphTempKeys_, d_updateGraphTempKeysCounter_, d_unexploredSamplesValidVertices_, d_updateGraphValidCounter_,
      d_updateGraphValidKeys_, d_updateGraphValidKeysCounter_, d_updateGraphValidTempKeys_, d_updateGraphValidTempKeysCounter_,
      d_unexploredSamplesSubVertices_;
    int *d_unexploredSamplesVertices_ptr_, *d_updateGraphCounter_ptr_, *d_updateGraphKeys_ptr_, *d_updateGraphKeysCounter_ptr_,
      *d_updateGraphTempKeys_ptr_, *d_updateGraphTempKeysCounter_ptr_, *d_unexploredSamplesValidVertices_ptr_,
      *d_updateGraphValidCounter_ptr_, *d_updateGraphValidKeys_ptr_, *d_updateGraphValidKeysCounter_ptr_, *d_updateGraphValidTempKeys_ptr_,
      *d_updateGraphValidTempKeysCounter_ptr_, *d_unexploredSamplesSubVertices_ptr_;

    thrust::device_vector<bool> d_updateGraphSubKeysCounter_;
    bool* d_updateGraphSubKeysCounter_ptr_;
};

/**************************** DEVICE FUNCTIONS ****************************/

/***************************/
/* PROPAGATE FRONTIER KERNEL 1 */
/***************************/
// --- Propagates current frontier. Builds new frontier. ---
// --- One Block Per Frontier Sample ---
__global__ void
propagateFrontier_kernel1(bool* frontier, uint* activeFrontierIdxs, float* treeSamples, float* unexploredSamples, uint frontierSize,
                          curandState* randomSeeds, int* unexploredSamplesParentIdxs, float* obstacles, int obstaclesCount,
                          int* activeSubVertices, float* vertexScores, bool* frontierNext, int* unexploredSamplesVertices,
                          int* unexploredSamplesValidVertices, int* unexploredSamplesSubVertices);

__global__ void
propagateFrontier_kernel2(bool* frontier, uint* activeFrontierIdxs, float* treeSamples, float* unexploredSamples, uint frontierSize,
                          curandState* randomSeeds, int* unexploredSamplesParentIdxs, float* obstacles, int obstaclesCount,
                          int* activeSubVertices, float* vertexScores, bool* frontierNext, int* unexploredSamplesVertices,
                          int* unexploredSamplesValidVertices, int* unexploredSamplesSubVertices, int iterations, int unexploredSize);

__global__ void updateFrontier_kernel(bool* frontier, bool* frontierNext, uint* activeFrontierNextIdxs, uint frontierNextSize, float* xGoal,
                                      int treeSize, float* unexploredSamples, float* treeSamples, int* unexploredSamplesParentIdxs,
                                      int* treeSamplesParentIdxs, float* treeSampleCosts, float* costToGoal);