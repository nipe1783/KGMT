#pragma once
#include <stdio.h>
#include <vector>
#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/count.h>
#include "helper/helper.cuh"

class OKPAXRegions
{
public:
    // --- constructor ---
    OKPAXRegions() = default;
    OKPAXRegions(float h_ws);

    // --- host fields ---
    int h_blockSize_ = 512;

    // --- device fields ---
    thrust::device_vector<float> d_minCostsR1_;
    float* d_minCostsR1_ptr_;
};

/**************************** DEVICE FUNCTIONS ****************************/
__host__ __device__ int OKPAX_getRegion(float* coord);