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
    uint h_treeSize_ = 0;

    // --- device fields ---
    thrust::device_vector<float> d_treeSamples_;
    float* d_treeSamples_ptr_;

    curandState* d_randomSeeds_ptr_;
};

/**************************** DEVICE FUNCTIONS ****************************/

/***************************/
/* INIT RANDOM SEEDS KERNEL */
/***************************/
// --- used to generate random values when propagating frontier on GPU. ---
__global__ void initializeRandomSeeds_kernel(curandState* randomSeeds, int numSeeds, int seed);
