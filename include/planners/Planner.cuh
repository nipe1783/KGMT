#pragma once
#include <stdio.h>
#include "config/config.h"
#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/iterator/constant_iterator.h>
#include <curand_kernel.h>
#include "helper/helper.cuh"
#include "statePropagator/statePropagator.cuh"
#include <filesystem>

class Planner
{
public:
    /**************************** CONSTRUCTORS ****************************/
    Planner();

    /****************************    METHODS    ****************************/
    virtual void plan(float* h_initial, float* h_goal, float* d_obstacles_ptr, uint h_obstaclesCount) = 0;
    void initializeRandomSeeds(int seed);

    /****************************    FIELDS    ****************************/
    // --- host fields ---
    uint h_treeSize_ = 0, h_itr_ = 0, h_blockSize_ = 512, h_gridSize_;

    // --- device fields ---
    thrust::device_vector<float> d_treeSamples_, d_treeSampleCosts_;
    thrust::device_vector<int> d_treeSamplesParentIdxs_;

    float *d_treeSamples_ptr_, *d_treeSampleCosts_ptr_;
    int* d_treeSamplesParentIdxs_ptr_;

    curandState* d_randomSeeds_ptr_;
};

/**************************** DEVICE FUNCTIONS ****************************/

/***************************/
/* INIT RANDOM SEEDS KERNEL */
/***************************/
// --- used to generate random values when propagating frontier on GPU. ---
__global__ void initializeRandomSeeds_kernel(curandState* randomSeeds, int numSeeds, int seed);

/***************************/
/* FIND INDICES BOOL KERNEL */
/***************************/
// --- Finds active indices in a boolean array. ---
__global__ void findInd(uint numSamples, bool* S, uint* scanIdx, uint* activeS);

/***************************/
/* FIND INDICES INT KERNEL */
/***************************/
// --- Finds active indices in an integer array. ---
__global__ void findInd(uint numSamples, uint* S, uint* scanIdx, uint* activeS);