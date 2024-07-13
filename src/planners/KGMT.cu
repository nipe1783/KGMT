#include "planners/KGMT.cuh"
#include "config/config.h"

KGMT::KGMT()
{
    graph_ = Graph(WS_SIZE);

    cudaMalloc(&d_sampleScoreThreshold_ptr_, sizeof(float));
    cudaMemcpy(d_sampleScoreThreshold_ptr_, h_sampleScoreThreshold_, sizeof(float), cudaMemcpyHostToDevice);

    d_frontier_                    = thrust::device_vector<bool>(MAX_TREE_SIZE);
    d_activeFrontierIdxs_          = thrust::device_vector<uint>(MAX_TREE_SIZE);
    d_unexploredSamples_           = thrust::device_vector<float>(MAX_TREE_SIZE * SAMPLE_DIM);
    d_unexploredSamplesParentIdxs_ = thrust::device_vector<uint>(MAX_TREE_SIZE);
    d_frontierScanIdx_             = thrust::device_vector<uint>(MAX_TREE_SIZE);

    d_frontier_ptr_                    = thrust::raw_pointer_cast(d_frontier_.data());
    d_activeFrontierIdxs_ptr_          = thrust::raw_pointer_cast(d_activeFrontierIdxs_.data());
    d_unexploredSamples_ptr_           = thrust::raw_pointer_cast(d_unexploredSamples_.data());
    d_unexploredSamplesParentIdxs_ptr_ = thrust::raw_pointer_cast(d_unexploredSamplesParentIdxs_.data());

    if(VERBOSE)
        {
            printf("/* Planner Type: KGMT */\n");
            printf("/* Number of R1 Vertices: %d */\n", NUM_R1_VERTICES);
            printf("/* Number of R2 Vertices: %d */\n", NUM_R2_VERTICES);
            printf("/***************************/\n");
        }
}

void KGMT::plan(float* h_initial, float* h_goal, float* d_obstacles_ptr, uint h_obstaclesCount)
{
    double t_kgmtStart = std::clock();

    cudaMemcpy(d_treeSamples_ptr_, h_initial, SAMPLE_DIM * sizeof(float), cudaMemcpyHostToDevice);
    initializeRandomSeeds(time(NULL));
    thrust::fill(d_frontier_.begin(), d_frontier_.begin() + 1, true);

    h_treeSize_ = 0;
    int itr     = 0;
    while(itr < MAX_ITER)
        {
            graph_.updateVertices(d_sampleScoreThreshold_ptr_);
            propagateFrontier(d_obstacles_ptr, h_obstaclesCount);
            updateFrontier();
            itr++;
        }

    std::cout << "time inside KGMT is " << (std::clock() - t_kgmtStart) / (double)CLOCKS_PER_SEC << std::endl;
    printf("Iteration %d, Tree size %d\n", itr, h_treeSize_);
}

/***************************/
/* PROPAGATE FRONTIER KERNEL  */
/***************************/
// --- Propagates current frontier. Builds new frontier. ---
// --- One Block Per Frontier Sample ---
__global__ void
propagateFrontier_kernel(bool* frontier, uint* activeFrontierIdxs, float* treeSamples, float* unexploredSamples, uint frontierSize,
                         curandState* randomSeeds, uint* unexploredSamplesParentIdxs, float* obstacles, int obstaclesCount,
                         int* vertexCounter, int* activeSubVertices, int* validVertexCounter, float* vertexScores, bool* newFrontier)
{
    if(blockIdx.x >= frontierSize) return;
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    // --- Load Frontier Sample Idx into shared memory.  ---
    __shared__ int s_x0Idx;
    if(threadIdx.x == 0)
        {
            s_x0Idx           = activeFrontierIdxs[blockIdx.x];
            frontier[s_x0Idx] = false;
        }
    __syncthreads();

    // --- Load Frontier Sample into shared memory. ---
    __shared__ float s_x0[SAMPLE_DIM];
    if(threadIdx.x < SAMPLE_DIM) s_x0[threadIdx.x] = treeSamples[s_x0Idx * SAMPLE_DIM + threadIdx.x];
    __syncthreads();

    // --- Propagate Sample and add it to unexplored sample set. ---
    float* x1                        = &unexploredSamples[tid * SAMPLE_DIM];
    unexploredSamplesParentIdxs[tid] = s_x0Idx;
    curandState randSeed             = randomSeeds[tid];
    bool valid                       = propagateAndCheck(s_x0, x1, &randSeed, obstacles, obstaclesCount);
    int x1Vertex                     = (DIM == 2) ? getVertex(x1[0], x1[1]) : getVertex(x1[0], x1[1], x1[2]);
    int x1SubVertex                  = (DIM == 2) ? getSubVertex(x1[0], x1[1], x1Vertex) : getSubVertex(x1[0], x1[1], x1[2], x1Vertex);

    // --- Update Graph sample count and populate newFrontier ---
    atomicAdd(&vertexCounter[x1Vertex], 1);
    if(valid)
        {
            atomicAdd(&validVertexCounter[x1Vertex], 1);
            if(curand_uniform(&randSeed) < vertexScores[x1Vertex] || activeSubVertices[x1SubVertex] == 0) newFrontier[tid] = true;
            if(activeSubVertices[x1SubVertex] == 0) atomicAdd(&activeSubVertices[x1SubVertex], 1);
        }

    randomSeeds[tid] = randSeed;
}

void KGMT::propagateFrontier(float* d_obstacles_ptr, uint h_obstaclesCount)
{
    thrust::exclusive_scan(d_frontier_.begin(), d_frontier_.end(), d_frontierScanIdx_.begin(), 0, thrust::plus<uint>());
    h_frontierSize_ = d_frontierScanIdx_[MAX_TREE_SIZE - 1];
    (d_frontier_[MAX_TREE_SIZE - 1]) ? ++h_frontierSize_ : 0;
    propagateFrontier_kernel<<<1, 7>>>(d_frontier_ptr_, d_activeFrontierIdxs_ptr_, d_treeSamples_ptr_, d_unexploredSamples_ptr_,
                                       h_frontierSize_, d_randomSeeds_ptr_, d_unexploredSamplesParentIdxs_ptr_, d_obstacles_ptr,
                                       h_obstaclesCount, graph_.d_counterArray_ptr_, graph_.d_activeSubVertices_ptr_,
                                       graph_.d_validCounterArray_ptr_, graph_.d_vertexScoreArray_ptr_, d_frontier_ptr_);
}

void KGMT::updateFrontier() {}
