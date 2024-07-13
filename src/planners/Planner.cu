#include "planners/Planner.cuh"
#include "config/config.h"

Planner::Planner()
{
    d_treeSamples_     = thrust::device_vector<float>(MAX_TREE_SIZE * SAMPLE_DIM);
    d_treeSamples_ptr_ = thrust::raw_pointer_cast(d_treeSamples_.data());

    cudaMalloc(&d_randomSeeds_ptr_, MAX_TREE_SIZE * sizeof(curandState));

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