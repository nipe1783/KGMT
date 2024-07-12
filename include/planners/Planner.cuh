#pragma once
#include <stdio.h>
#include "config/config.h"
#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/iterator/constant_iterator.h>
#include "helper/helper.cuh"

class Planner
{
public:
    /**************************** CONSTRUCTORS ****************************/
    Planner() = default;
    Planner(const float h_ws, int numDisc = 10, int maxTreeSize = 30000, float goalThreshold = 0.5, int maxIterations = 100);

    /****************************    METHODS    ****************************/
    virtual void plan(float* h_initial, float* h_goal) = 0;

    /****************************    FIELDS    ****************************/
    // --- host fields ---
    float h_ws_, h_goalThreshold_;
    int h_numDisc_, h_maxTreeSize_, h_maxIterations_, h_treeSize_ = 0;

    // --- device fields ---
    thrust::device_vector<float> d_treeSamples_;
    float* d_treeSamples_ptr_;
};
