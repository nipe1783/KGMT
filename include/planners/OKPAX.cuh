#pragma once
#include "planners/Planner.cuh"
#include "graphs/OKPAXRegions.cuh"

class OKPAX : public Planner
{
public:
    /**************************** CONSTRUCTORS ****************************/
    OKPAX();

    /****************************    METHODS    ****************************/
    void plan(float* h_initial, float* h_goal, float* d_obstacles_ptr, uint h_obstaclesCount) override;
    float planOptimize(float* h_initial, float* h_goal, float* d_obstacles_ptr, uint h_obstaclesCount);
    void planDataCollect(float* h_initial, float* h_goal, float* d_obstacles_ptr, uint h_obstaclesCount, int benchItr);
    void propagateFrontier(float* d_obstacles_ptr, uint h_obstaclesCount);
    void updateFrontier();
    void getControlPathsToGoal();
    void writeExecutionTimeToCSV(double time);
    void resetPlanner(float* h_initial, float* h_goal);

    // Methods for exctracting data to CSV:
    void writeDeviceVectorsToCSV(int itr = 0);
    void writeSolutionsToCSV(int itr = 0);
    void writeSolutionCostsToCSV(int itr = 0);

    /****************************    FIELDS    ****************************/
    // --- host fields ---
    OKPAXRegions graph_;
    uint h_frontierSize_, h_frontierNextSize_, h_activeBlockSize_, h_frontierRepeatSize_, h_propIterations_;
    float h_fAccept_, h_minCost_;

    // --- device fields ---
    thrust::device_vector<bool> d_frontier_, d_frontierNext_, d_goalSet_, d_pruned_;
    thrust::device_vector<uint> d_activeFrontierIdxs_, d_frontierScanIdx_, d_activeFrontierRepeatCount_, d_frontierRepeatScanIdx_,
      d_activeFrontierRepeatIdxs_, d_goalSetScanIdx_, d_goalSetIdxs_, d_treeInactiveIterations_;
    thrust::device_vector<int> d_unexploredSamplesParentIdxs_, d_treeXR1s_, d_frontierNextXR1s_;
    thrust::device_vector<float> d_unexploredSamples_, d_goalSample_;
    float *d_unexploredSamples_ptr_, *d_goalSample_ptr_;
    bool *d_frontier_ptr_, *d_frontierNext_ptr_, *d_goalSet_ptr_, *d_pruned_ptr_;
    uint *d_activeFrontierIdxs_ptr_, *d_frontierScanIdx_ptr_, *d_activeFrontierRepeatCount_ptr_, *d_frontierRepeatScanIdx_ptr_,
      *d_activeFrontierRepeatIdxs_ptr_, *d_goalSetScanIdx_ptr_, *d_goalSetIdxs_ptr_, *d_treeInactiveIterations_ptr_;
    int *d_unexploredSamplesParentIdxs_ptr_, *d_treeXR1s_ptr_, *d_frontierNextXR1s_ptr_;
    float* d_minCost_ptr_;
};

/**************************** DEVICE FUNCTIONS ****************************/

/***************************/
/* PROPAGATE FRONTIER KERNEL 1 */
/***************************/
// --- Propagates current frontier. Builds new frontier. ---
// --- One Block Per Frontier Sample ---
__global__ void OKPAX_propagateFrontier_kernel1(bool* frontier, uint* activeFrontierIdxs, float* treeSamples, float* unexploredSamples,
                                                uint frontierSize, curandState* randomSeeds, int* unexploredSamplesParentIdxs,
                                                float* obstacles, int obstaclesCount, bool* frontierNext, float* treeSampleCosts,
                                                float* minCostsR1, int* frontierNextXR1s, float* unexploredSampleCosts);

__global__ void
OKPAX_propagateFrontier_kernel2(bool* frontier, uint* activeFrontierIdxs, float* treeSamples, float* unexploredSamples, uint frontierSize,
                                curandState* randomSeeds, int* unexploredSamplesParentIdxs, float* obstacles, int obstaclesCount,
                                bool* frontierNext, int iterations, float* treeSampleCosts, float* minCostsR1, int* frontierNextXR1s,
                                float* unexploredSampleCosts);

__global__ void
OKPAX_updateFrontier_kernel(bool* frontier, bool* frontierNext, uint* activeFrontierNextIdxs, uint frontierNextSize, float* xGoal,
                            int treeSize, float* unexploredSamples, float* treeSamples, int* unexploredSamplesParentIdxs,
                            int* treeSamplesParentIdxs, float* treeSampleCosts, uint* activeFrontierRepeatCount, curandState* randomSeeds,
                            float* controlPathToGoal, bool* goalSet, int* iterations, int iteration, float* minCostsR1, int* treeXR1,
                            int* frontierNextXR1s, float* minCost, float* unexploredSampleCosts, bool* pruned);

__global__ void OKPAX_pruningTree_kernel(int treeSize, int* treeSamplesParentIdxs, float* treeSampleCosts, bool* goalSet, float* minCostsR1,
                                         int* treeXR1s, bool* pruned, uint* inactiveIterations);

__global__ void
OKPAX_pruningFrontier_kernel(uint* activeFrontierNextIdxs, uint frontierNextSize, int* unexploredSamplesParentIdxs,
                             int* treeSamplesParentIdxs, float* treeSampleCosts, float* minCostsR1, int* frontierNextXR1s, int* treeXR1s,
                             bool* frontierNext, float* unexploredSampleCosts, bool* pruned, uint* inactiveIterations);

__global__ void
OKPAX_getControlPathsToGoal_kernel(float* controlPathsToGoal, float* treeSamples, int* treeSamplesParentIdxs, uint* goalSetIdxs,
                                   int goalSetSize, float* pathCosts, float* treeSampleCosts, int* iterations);