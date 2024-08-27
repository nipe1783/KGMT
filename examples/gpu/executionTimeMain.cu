#include <iostream>
#include "planners/KGMT.cuh"

int main(void)
{
    // --- Remove Previous Bench Data ---
    system("rm -rf Data/*");

    float h_initial[SAMPLE_DIM] = {10.0, 8, 5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          h_goal[SAMPLE_DIM]    = {80, 95.0, 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    KGMT kgmt;

    int numObstacles;
    float* d_obstacles;

    // --- Load Workspace Obstacles ---
    std::vector<float> obstacles = readObstaclesFromCSV("../include/config/obstacles/quadHouse/obstacles.csv", numObstacles, W_DIM);

    // --- Transfer Obstacles to device ---
    cudaMalloc(&d_obstacles, numObstacles * 2 * W_DIM * sizeof(float));
    cudaMemcpy(d_obstacles, obstacles.data(), numObstacles * 2 * W_DIM * sizeof(float), cudaMemcpyHostToDevice);

    // --- Execute planner N times ---
    int N = 50;
    cudaEvent_t start, stop;
    float milliseconds = 0;

    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    for(int i = 0; i < N; i++)
        {
            cudaEventRecord(start);

            // --- Execute planner ---
            kgmt.plan(h_initial, h_goal, d_obstacles, numObstacles);

            cudaEventRecord(stop);
            cudaEventSynchronize(stop);

            cudaEventElapsedTime(&milliseconds, start, stop);
            std::cout << "Execution time: " << milliseconds / 1000.0 << " seconds" << std::endl;
        }

    // --- Free memory ---
    cudaFree(d_obstacles);
    cudaEventDestroy(start);
    cudaEventDestroy(stop);

    return 0;
}
