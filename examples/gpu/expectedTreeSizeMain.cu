#include <iostream>
#include "planners/KGMT.cuh"
int main(void)
{
    // --- Remove Previous Bench Data ---
    system("rm -rf Data/*");

    float h_initial[SAMPLE_DIM] = {10.0, 8, 5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          h_goal[SAMPLE_DIM]    = {80, 95.0, 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // float h_initial[SAMPLE_DIM] = {.100, .80, .05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    //       h_goal[SAMPLE_DIM]    = {.800, .950, .900, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KGMT kgmt(100000);

    int numObstacles;
    float* d_obstacles;

    // --- Load Workspace Obstacles ---
    std::vector<float> obstacles = readObstaclesFromCSV("../include/config/obstacles/quadTrees/obstacles.csv", numObstacles, W_DIM);

    // --- Transfer Obstacles to device ---
    cudaMalloc(&d_obstacles, numObstacles * 2 * W_DIM * sizeof(float));
    cudaMemcpy(d_obstacles, obstacles.data(), numObstacles * 2 * W_DIM * sizeof(float), cudaMemcpyHostToDevice);

    // --- Execute planner N times ---
    int N = 50;
    for(int i = 0; i < N; i++)
        {
            // --- Execute planner ---
            kgmt.planBench(h_initial, h_goal, d_obstacles, numObstacles, i);
        }

    // --- Free memory ---
    cudaFree(d_obstacles);
    return 0;
}