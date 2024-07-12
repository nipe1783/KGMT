#include "planners/KGMT.cuh"
#include "config/config.h"

KGMT::KGMT(float h_ws, int h_numDisc, int h_maxTreeSize, float h_goalThreshold, int h_maxIterations)
    : Planner(h_ws, h_numDisc, h_maxTreeSize, h_goalThreshold, h_maxIterations)
{
    graph_ = Graph(h_ws_);

    cudaMalloc(&d_sampleScoreThreshold_ptr_, sizeof(float));
    cudaMemcpy(d_sampleScoreThreshold_ptr_, h_sampleScoreThreshold_, sizeof(float), cudaMemcpyHostToDevice);

    d_frontier_ = thrust::device_vector<bool>(h_maxTreeSize_);
    d_frontier_ptr_ = thrust::raw_pointer_cast(d_frontier_.data());

    if(VERBOSE)
        {
            printf("/* Planner Type: KGMT */\n");
            printf("/* Number of R1 Vertices: %d */\n", NUM_R1_VERTICES);
            printf("/* Number of R2 Vertices: %d */\n", NUM_R2_VERTICES);
            printf("/***************************/\n");
        }
}

void KGMT::plan(float* h_initial, float* h_goal)
{
    double t_kgmtStart = std::clock();
    cudaMemcpy(d_treeSamples_ptr_, h_initial, SAMPLE_DIM * sizeof(float), cudaMemcpyHostToDevice);
    h_treeSize_ = 0;

    int itr = 0;
    while(itr < h_maxIterations_)
        {
            graph_.updateVertices(d_sampleScoreThreshold_ptr_);
            propagateFrontier();
            itr++;
        }
    std::cout << "time inside KGMT is " << (std::clock() - t_kgmtStart) / (double)CLOCKS_PER_SEC << std::endl;
    printf("Iteration %d, Tree size %d\n", itr, h_treeSize_);
}

/***************************/
/* PROPAGATE FRONTIER KERNEL  */
/***************************/
// --- Propagates current frontier. Builds new frontier. ---
__global__ void propagateFrontier_kernel(bool* frontier, float* treeSamples)
{
    printf("Propagating Frontier\n");
}

void KGMT::propagateFrontier()
{
    propagateFrontier_kernel<<<1, 1>>>(d_frontier_ptr_, d_treeSamples_ptr_);
}
