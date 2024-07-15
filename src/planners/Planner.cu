#include "planners/Planner.cuh"
#include "config/config.h"

Planner::Planner()
{
    d_treeSamples_           = thrust::device_vector<float>(MAX_TREE_SIZE * SAMPLE_DIM);
    d_treeSamplesParentIdxs_ = thrust::device_vector<int>(MAX_TREE_SIZE);
    d_treeSampleCosts_       = thrust::device_vector<float>(MAX_TREE_SIZE);

    d_treeSamples_ptr_           = thrust::raw_pointer_cast(d_treeSamples_.data());
    d_treeSamplesParentIdxs_ptr_ = thrust::raw_pointer_cast(d_treeSamplesParentIdxs_.data());
    d_treeSampleCosts_ptr_       = thrust::raw_pointer_cast(d_treeSampleCosts_.data());

    h_gridSize_ = iDivUp(MAX_TREE_SIZE, h_blockSize_);

    cudaMalloc(&d_randomSeeds_ptr_, MAX_TREE_SIZE * sizeof(curandState));
    thrust::fill(d_treeSamplesParentIdxs_.begin(), d_treeSamplesParentIdxs_.end(), -1);

    cudaMalloc(&d_costToGoal_ptr_, sizeof(float));

    if(VERBOSE)
        {
            printf("/***************************/\n");
            printf("/* Workspace Dimension: %d */\n", DIM);
            printf("/* Workspace Size: %f */\n", WS_SIZE);
            printf("/* Discretization steps in trajectory: %d */\n", NUM_DISC);
            printf("/* Max Tree Size: %d */\n", MAX_TREE_SIZE);
            printf("/* Goal Distance Threshold: %f */\n", GOAL_THRESH);
            printf("/* Max Planning Iterations: %d */\n", MAX_ITER);
        }
}

__global__ void initializeRandomSeeds_kernel(curandState* randomSeeds, int numSeeds, int seed)
{
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    if(tid < numSeeds)
        {
            curand_init(seed, tid, 0, &randomSeeds[tid]);
        }
}

void Planner::initializeRandomSeeds(int seed)
{
    int blockSize = 512;  // TODO: Check if this is the optimal block size
    initializeRandomSeeds_kernel<<<iDivUp(MAX_TREE_SIZE, blockSize), blockSize>>>(d_randomSeeds_ptr_, MAX_TREE_SIZE, seed);
}

__global__ void findInd(uint numSamples, bool* S, uint* scanIdx, uint* activeS)
{
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if(tid >= numSamples) return;
    if(!S[tid]) return;
    activeS[scanIdx[tid]] = tid;
}

__global__ void findInd(uint numSamples, uint* S, uint* scanIdx, uint* activeS)
{
    int node = blockIdx.x * blockDim.x + threadIdx.x;
    if(node >= numSamples) return;
    if(!S[node]) return;
    activeS[scanIdx[node]] = node;
}