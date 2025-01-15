#include <iostream>
#include "planners/KGMT.cuh"
#include "cpu/planners/OMPL_Planner.h"

int main(void)
{
    // --- Remove Previous Bench Data ---
    system("rm -rf Data/*");

    float h_initial[SAMPLE_DIM] = {.100, .080, .05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          h_goal[SAMPLE_DIM]    = {.800, .950, .900, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // float h_initial[SAMPLE_DIM] = {10.0, 8, 5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    //       h_goal[SAMPLE_DIM]    = {80, 95.0, 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KGMT kgmt;

    int numObstacles;
    float* d_obstacles;

    // --- Load Workspace Obstacles ---
    std::vector<float> obstacles = readObstaclesFromCSV("../include/config/obstacles/house/obstacles.csv", numObstacles, W_DIM);

    // --- Transfer Obstacles to device ---
    cudaMalloc(&d_obstacles, numObstacles * 2 * W_DIM * sizeof(float));
    cudaMemcpy(d_obstacles, obstacles.data(), numObstacles * 2 * W_DIM * sizeof(float), cudaMemcpyHostToDevice);

    // --- OMPL obj to calculate path cost ---
    OMPL_Planner omplCost;

    // --- Execute planner ---
    int N = 1;
    for(int i = 0; i < N; i++)
        {
            kgmt.planDataCollect(h_initial, h_goal, d_obstacles, numObstacles, i);

            std::string path = "/home/nicolas/dev/research/KGMT/build/Data/ControlPathToGoal/ControlPathToGoal" + std::to_string(i) +
                               "/controlPathToGoal.csv";
            omplCost.computePathCost(h_initial, h_goal, obstacles.data(), numObstacles, 0.0, path);
        }

    // --- Free memory ---
    cudaFree(d_obstacles);
    return 0;
}