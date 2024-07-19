#include <iostream>
#include "planners/KGMT.cuh"
int main(void)
{
    // float h_initial[SAMPLE_DIM] = {.5, .5, 0.0, 0.0, 0.0, 0.0, 0.0}, h_goal[SAMPLE_DIM] = {2, 18, 0.0, 0.0, 0.0, 0.0, 0.0};
    float h_initial[SAMPLE_DIM] = {0.3, 0.02, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          h_goal[SAMPLE_DIM]    = {.7, .95, .9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KGMT kgmt;

    // TODO: clean this up
    int numObstacles = 1;
    float* d_obstacles;
    std::vector<float> obstacles = readObstaclesFromCSV("../include/config/obstacles/pillars/obstacles.csv", numObstacles, DIM);
    cudaMalloc(&d_obstacles, numObstacles * 2 * DIM * sizeof(float));
    cudaMemcpy(d_obstacles, obstacles.data(), numObstacles * 2 * DIM * sizeof(float), cudaMemcpyHostToDevice);

    kgmt.plan(h_initial, h_goal, d_obstacles, numObstacles);

    cudaFree(d_obstacles);
    return 0;
    return 0;
}