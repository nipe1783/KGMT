#include <iostream>
#include "planners/KGMT.cuh"
#include "../../../../miniconda3/envs/cudf_dev/x86_64-conda-linux-gnu/sysroot/usr/include/unistd.h"
int main(void)
{
    float h_initial[SAMPLE_DIM] = {0.3, 0.02, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          h_goal[SAMPLE_DIM]    = {.7, .95, .9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KGMT kgmt;

    // TODO: clean this up
    int numObstacles = 1;
    float* d_obstacles;
    std::vector<float> obstacles = readObstaclesFromCSV("../include/config/obstacles/pillars/obstacles.csv", numObstacles, DIM);
    cudaMalloc(&d_obstacles, numObstacles * 2 * DIM * sizeof(float));
    cudaMemcpy(d_obstacles, obstacles.data(), numObstacles * 2 * DIM * sizeof(float), cudaMemcpyHostToDevice);
    kgmt.planBench(h_initial, h_goal, d_obstacles, numObstacles, 0);
    cudaFree(d_obstacles);
    return 0;
}