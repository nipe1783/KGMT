#include <iostream>
#include "planners/KGMT.cuh"

int main(void)
{
    // --- Remove Previous Bench Data ---
    system("rm -rf Data/*");

    float h_initial[SAMPLE_DIM] = {0.0, 0.02, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          h_goal[SAMPLE_DIM]    = {.9, .95, .9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KGMT kgmt;

    int numObstacles;
    float* d_obstacles;

    // --- Load Workspace Obstacles ---
    std::vector<float> obstacles = readObstaclesFromCSV("../include/config/obstacles/narrowPassage/obstacles.csv", numObstacles, DIM);

    // --- Transfer Obstacles to device ---
    cudaMalloc(&d_obstacles, numObstacles * 2 * DIM * sizeof(float));
    cudaMemcpy(d_obstacles, obstacles.data(), numObstacles * 2 * DIM * sizeof(float), cudaMemcpyHostToDevice);

    // --- Execute planner ---
    kgmt.planBench(h_initial, h_goal, d_obstacles, numObstacles, 0);

    // --- Free memory ---
    cudaFree(d_obstacles);
    return 0;
}