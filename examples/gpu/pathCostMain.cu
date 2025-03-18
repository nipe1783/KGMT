#include <iostream>
#include "planners/OKPAX.cuh"
#include "planners/KPAX.cuh"
#include "cpu/planners/OMPL_Planner.h"

int main(void)
{
    // --- Remove Previous Bench Data ---
    system("rm -rf Data/*");

    // float h_initial[SAMPLE_DIM] = {.1, .080, .05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    //       h_goal[SAMPLE_DIM]    = {.80, .950, .900, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float h_initial[SAMPLE_DIM] = {10.0, 8, 5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          h_goal[SAMPLE_DIM]    = {80, 95.0, 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    int numObstacles;
    float* d_obstacles;

    // --- Load Workspace Obstacles ---
    std::vector<float> obstacles = readObstaclesFromCSV("../include/config/obstacles/quadNarrowPassage/obstacles.csv", numObstacles, W_DIM);

    // --- Transfer Obstacles to device ---
    cudaMalloc(&d_obstacles, numObstacles * 2 * W_DIM * sizeof(float));
    cudaMemcpy(d_obstacles, obstacles.data(), numObstacles * 2 * W_DIM * sizeof(float), cudaMemcpyHostToDevice);

    // --- Execute planner ---
    int N = 100;
    std::vector<float> pathCosts(N);
    OKPAX kpax;
    for(int i = 0; i < N; i++)
        {
            pathCosts[i] = kpax.planBenchmark(h_initial, h_goal, d_obstacles, numObstacles, i);
        }

    float sum  = std::accumulate(pathCosts.begin(), pathCosts.end(), 0.0);
    float mean = sum / pathCosts.size();
    printf("Mean path cost: %f\n", mean);

    // --- Free memory ---
    cudaFree(d_obstacles);
    return 0;
}