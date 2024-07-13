#include <iostream>
#include "planners/KGMT.cuh"
int main(void)
{
    float h_initial[SAMPLE_DIM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, h_goal[SAMPLE_DIM] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    KGMT kgmt;

    // TODO: clean this up
    int numObstacles = 1;
    float* d_obstacles;
    std::vector<float> obstacles = readObstaclesFromCSV("../include/config/obstacles/obstacles.csv", numObstacles, DIM);
    cudaMalloc(&d_obstacles, numObstacles * 2 * DIM * sizeof(float));
    cudaMemcpy(d_obstacles, obstacles.data(), numObstacles * 2 * DIM * sizeof(float), cudaMemcpyHostToDevice);
    printDeviceVector(d_obstacles, numObstacles * 2 * DIM);

    kgmt.plan(h_initial, h_goal, d_obstacles, numObstacles);

    cudaFree(d_obstacles);
    return 0;
    return 0;
}