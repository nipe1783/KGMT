#include "planners/Planner.cuh"
#include "config/config.h"

Planner::Planner(int h_desiredTreeSize)
{
    h_desiredTreeSize_ = h_desiredTreeSize;
    h_maxTreeSize_     = h_desiredTreeSize;
    h_gridSize_        = iDivUp(h_maxTreeSize_, h_blockSize_);

    d_treeSamples_           = thrust::device_vector<float>(h_maxTreeSize_ * SAMPLE_DIM);
    d_treeSamplesParentIdxs_ = thrust::device_vector<int>(h_maxTreeSize_);
    d_treeSampleCosts_       = thrust::device_vector<float>(h_maxTreeSize_);
    d_controlPathToGoal_     = thrust::device_vector<float>(MAX_ITER * SAMPLE_DIM);
    d_randomSeeds_           = thrust::device_vector<curandState>(h_maxTreeSize_);

    d_treeSamples_ptr_           = thrust::raw_pointer_cast(d_treeSamples_.data());
    d_treeSamplesParentIdxs_ptr_ = thrust::raw_pointer_cast(d_treeSamplesParentIdxs_.data());
    d_treeSampleCosts_ptr_       = thrust::raw_pointer_cast(d_treeSampleCosts_.data());
    d_controlPathToGoal_ptr_     = thrust::raw_pointer_cast(d_controlPathToGoal_.data());
    d_randomSeeds_ptr_           = thrust::raw_pointer_cast(d_randomSeeds_.data());

    cudaMalloc(&d_costToGoal_ptr_, sizeof(float));
    cudaMalloc(&d_pathToGoal_ptr_, sizeof(int));

    h_controlPathToGoal_ = new float[SAMPLE_DIM * MAX_ITER];

    if(VERBOSE)
        {
            printf("/***************************/\n");
            printf("/* Workspace Dimension: %d */\n", W_DIM);
            printf("/* Workspace Size: %f */\n", W_SIZE);
            printf("/* Maximum discretization steps in propagation: %d */\n", MAX_PROPAGATION_DURATION);
            printf("/* Propagation step Size: %f */\n", STEP_SIZE);
            printf("/* Max Tree Size: %d */\n", h_maxTreeSize_);
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
    int blockSize = 32;
    initializeRandomSeeds_kernel<<<iDivUp(h_maxTreeSize_, blockSize), blockSize>>>(d_randomSeeds_ptr_, h_maxTreeSize_, seed);
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

__global__ void repeatInd(uint numSamples, uint* activeS, uint* C, uint* prefixSum, uint* repeatedInd)
{
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if(tid >= numSamples) return;

    uint index    = activeS[tid];
    uint count    = C[index];
    uint startPos = prefixSum[index];
    for(uint i = 0; i < count; ++i)
        {
            if(startPos + i >= numSamples) return;
            repeatedInd[startPos + i] = index;
        }
}