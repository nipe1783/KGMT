#include <iostream>
#include "planners/KGMT.cuh"
int main(void)
{
    float h_initial[SAMPLE_DIM] = {0.3, 0.02, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          h_goal[SAMPLE_DIM]    = {.7, .95, .9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KGMT kgmt;

    int numObstacles;
    float* d_obstacles;

    // --- Load Workspace Obstacles ---
    std::vector<float> obstacles = readObstaclesFromCSV("../include/config/obstacles/trees/obstacles.csv", numObstacles, DIM);

    // --- Transfer Obstacles to device ---
    cudaMalloc(&d_obstacles, numObstacles * 2 * DIM * sizeof(float));
    cudaMemcpy(d_obstacles, obstacles.data(), numObstacles * 2 * DIM * sizeof(float), cudaMemcpyHostToDevice);

    // --- Execute planner N times ---
    int N = 1000;
    for(int i = 0; i < N; i++)
        {
            // --- Execute planner ---
            kgmt.plan(h_initial, h_goal, d_obstacles, numObstacles);
        }

    // --- Free memory ---
    cudaFree(d_obstacles);
    return 0;
}