#include "planners/KGMT.cuh"
#include "config/config.h"

KGMT::KGMT()
{
    graph_ = Graph(WS_SIZE);

    cudaMalloc(&d_sampleScoreThreshold_ptr_, sizeof(float));
    cudaMemcpy(d_sampleScoreThreshold_ptr_, h_sampleScoreThreshold_, sizeof(float), cudaMemcpyHostToDevice);

    d_frontier_                    = thrust::device_vector<bool>(MAX_TREE_SIZE);
    d_frontierNext_                = thrust::device_vector<bool>(MAX_TREE_SIZE);
    d_activeFrontierIdxs_          = thrust::device_vector<uint>(MAX_TREE_SIZE);
    d_unexploredSamples_           = thrust::device_vector<float>(MAX_TREE_SIZE * SAMPLE_DIM);
    d_unexploredSamplesParentIdxs_ = thrust::device_vector<int>(MAX_TREE_SIZE);
    d_frontierScanIdx_             = thrust::device_vector<uint>(MAX_TREE_SIZE);

    d_frontier_ptr_                    = thrust::raw_pointer_cast(d_frontier_.data());
    d_frontierNext_ptr_                = thrust::raw_pointer_cast(d_frontierNext_.data());
    d_activeFrontierIdxs_ptr_          = thrust::raw_pointer_cast(d_activeFrontierIdxs_.data());
    d_unexploredSamples_ptr_           = thrust::raw_pointer_cast(d_unexploredSamples_.data());
    d_unexploredSamplesParentIdxs_ptr_ = thrust::raw_pointer_cast(d_unexploredSamplesParentIdxs_.data());
    d_frontierScanIdx_ptr_             = thrust::raw_pointer_cast(d_frontierScanIdx_.data());

    h_activeBlockSize_ = 32;

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

    h_treeSize_ = 1;
    h_itr_      = 0;
    while(h_itr_ < MAX_ITER)
        {
            graph_.updateVertices(d_sampleScoreThreshold_ptr_);
            propagateFrontier(d_obstacles_ptr, h_obstaclesCount);
            updateFrontier();
            h_itr_++;

            writeDeviceVectorsToCSV();
        }

    std::cout << "time inside KGMT is " << (std::clock() - t_kgmtStart) / (double)CLOCKS_PER_SEC << std::endl;
    printf("Iteration %d, Tree size %d\n", h_itr_, h_treeSize_);

    // move vectors to csv to be plotted.
    copyAndWriteVectorToCSV(d_treeSamples_, "samples.csv", MAX_TREE_SIZE, SAMPLE_DIM);
    copyAndWriteVectorToCSV(d_unexploredSamples_, "unexploredSamples.csv", MAX_TREE_SIZE, SAMPLE_DIM);
    copyAndWriteVectorToCSV(d_treeSamplesParentIdxs_, "parentRelations.csv", MAX_TREE_SIZE, 1);
}

void KGMT::propagateFrontier(float* d_obstacles_ptr, uint h_obstaclesCount)
{
    // --- Find indices and size of frontier. ---
    thrust::exclusive_scan(d_frontier_.begin(), d_frontier_.end(), d_frontierScanIdx_.begin(), 0, thrust::plus<uint>());
    h_frontierSize_ = d_frontierScanIdx_[MAX_TREE_SIZE - 1];
    (d_frontier_[MAX_TREE_SIZE - 1]) ? ++h_frontierSize_ : 0;
    findInd<<<h_gridSize_, h_blockSize_>>>(MAX_TREE_SIZE, d_frontier_ptr_, d_frontierScanIdx_ptr_, d_activeFrontierIdxs_ptr_);

    int gridSize = iDivUp(h_frontierSize_ * h_activeBlockSize_, h_activeBlockSize_);
    if(h_activeBlockSize_ * gridSize > MAX_TREE_SIZE - h_treeSize_)
        {
            printf("V2\n");
            int remaining  = MAX_TREE_SIZE - h_treeSize_;
            int iterations = int(float(remaining) / float(h_frontierSize_));
            gridSize       = int(floor(MAX_TREE_SIZE / h_activeBlockSize_));
            propagateFrontier_kernel2<<<gridSize, h_activeBlockSize_>>>(
              d_activeFrontierIdxs_ptr_, d_frontier_ptr_, d_treeSamples_ptr_, iterations, d_unexploredSamples_ptr_,
              d_unexploredSamplesParentIdxs_ptr_, d_randomSeeds_ptr_, d_obstacles_ptr, h_obstaclesCount, graph_.d_counterArray_ptr_,
              graph_.d_validCounterArray_ptr_, graph_.d_activeVertices_ptr_, graph_.d_activeSubVertices_ptr_,
              graph_.d_vertexScoreArray_ptr_, d_frontierNext_ptr_);
        }
    else
        {
            printf("V1\n");
            // --- Propagate Frontier. Block Size threads per sample. ---
            propagateFrontier_kernel1<<<gridSize, h_activeBlockSize_>>>(
              d_frontier_ptr_, d_activeFrontierIdxs_ptr_, d_treeSamples_ptr_, d_unexploredSamples_ptr_, h_frontierSize_, d_randomSeeds_ptr_,
              d_unexploredSamplesParentIdxs_ptr_, d_obstacles_ptr, h_obstaclesCount, graph_.d_activeVertices_ptr_,
              graph_.d_counterArray_ptr_, graph_.d_activeSubVertices_ptr_, graph_.d_validCounterArray_ptr_, graph_.d_vertexScoreArray_ptr_,
              d_frontierNext_ptr_);
        }
}

/***************************/
/* PROPAGATE FRONTIER KERNEL  */
/***************************/
// --- Propagates current frontier. Builds new frontier. ---
// --- One Block Per Frontier Sample ---
__global__ void propagateFrontier_kernel1(bool* frontier, uint* activeFrontierIdxs, float* treeSamples, float* unexploredSamples,
                                          uint frontierSize, curandState* randomSeeds, int* unexploredSamplesParentIdxs, float* obstacles,
                                          int obstaclesCount, int* activeVertices, int* vertexCounter, int* activeSubVertices,
                                          int* validVertexCounter, float* vertexScores, bool* frontierNext)
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

    // --- Update Graph sample count and populate next Frontier ---
    atomicAdd(&vertexCounter[x1Vertex], 1);
    if(valid)
        {
            atomicAdd(&validVertexCounter[x1Vertex], 1);
            if(curand_uniform(&randSeed) < vertexScores[x1Vertex] || activeSubVertices[x1SubVertex] == 0) frontierNext[tid] = true;
            if(activeVertices[x1Vertex] == 0) atomicExch(&activeVertices[x1Vertex], 1);
            if(activeSubVertices[x1SubVertex] == 0) atomicExch(&activeSubVertices[x1SubVertex], 1);
        }

    randomSeeds[tid] = randSeed;
}

/***************************/
/* FRONTIER PROPAGATION KERNEL 2 */
/***************************/
// --- 1 thread per sample. iterates n times. ---
__global__ void propagateFrontier_kernel2(uint* activeFrontierIdxs, bool* frontier, float* treeSamples, int iterations,
                                          float* unexploredSamples, int* unexploredSamplesParentIdxs, curandState* randomSeeds,
                                          float* obstacles, int obstaclesCount, int* vertexCounter, int* validVertexCounter,
                                          int* activeVertices, int* activeSubVertices, float* vertexScores, bool* frontierNext)
{
    int tid   = blockIdx.x * blockDim.x + threadIdx.x;
    int x0Idx = activeFrontierIdxs[tid];

    if(!frontier[x0Idx]) return;
    frontier[x0Idx] = false;

    float* x0 = &treeSamples[x0Idx * SAMPLE_DIM];
    for(int i = 0; i < iterations; i++)
        {
            // --- Propagate Sample and add it to unexplored sample set. ---
            int x1Idx                          = tid * iterations + i;  // --- Index of new sample in unexplored samples ---
            curandState randSeed               = randomSeeds[x1Idx];
            float* x1                          = &unexploredSamples[x1Idx * SAMPLE_DIM];  // --- New sample ---
            unexploredSamplesParentIdxs[x1Idx] = x0Idx;                                   // --- Parent of new sample ---
            bool valid                         = propagateAndCheck(x0, x1, &randSeed, obstacles, obstaclesCount);
            int x1Vertex                       = (DIM == 2) ? getVertex(x1[0], x1[1]) : getVertex(x1[0], x1[1], x1[2]);
            int x1SubVertex = (DIM == 2) ? getSubVertex(x1[0], x1[1], x1Vertex) : getSubVertex(x1[0], x1[1], x1[2], x1Vertex);

            // --- Update Graph sample count and populate next Frontier ---
            atomicAdd(&vertexCounter[x1Vertex], 1);
            if(valid)
                {
                    atomicAdd(&validVertexCounter[x1Vertex], 1);
                    if(curand_uniform(&randSeed) < vertexScores[x1Vertex] || activeSubVertices[x1SubVertex] == 0) frontierNext[tid] = true;
                    if(activeVertices[x1Vertex] == 0) atomicExch(&activeVertices[x1Vertex], 1);
                    if(activeSubVertices[x1SubVertex] == 0) atomicExch(&activeSubVertices[x1SubVertex], 1);
                }
            randomSeeds[x1Idx] = randSeed;
        }
}

/***************************/
/* FRONTIER UPDATE KERNEL */
/***************************/
// --- Adds previous frontier to the tree and builds new frontier. ---
__global__ void updateFrontier_kernel(bool* frontier, uint* activeFrontierNextIdxs, uint frontierNextSize, float* xGoal, int treeSize,
                                      float* unexploredSamples, float* treeSamples, int* unexploredSamplesParentIdxs,
                                      int* treeSamplesParentIdxs, float* treeSampleCosts, float* costToGoal)
{
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    __shared__ float s_xGoal[SAMPLE_DIM];
    if(threadIdx.x < SAMPLE_DIM) s_xGoal[threadIdx.x] = xGoal[threadIdx.x];
    __syncthreads();

    if(tid >= frontierNextSize) return;

    // --- Update Tree ---
    int x1TreeIdx                    = treeSize + tid;               // --- Index of new tree sample ---
    int x1UnexploredIdx              = activeFrontierNextIdxs[tid];  // --- Index of sample in unexplored sample set ---
    float* x1                        = &unexploredSamples[x1UnexploredIdx * SAMPLE_DIM];  // --- sample from unexplored set ---
    int x0Idx                        = unexploredSamplesParentIdxs[x1UnexploredIdx];      // --- parent of the unexplored sample ---
    treeSamplesParentIdxs[x1TreeIdx] = x0Idx;  // --- Transfer parent of unexplored sample to tree ---
    for(int i = 0; i < SAMPLE_DIM; i++) treeSamples[x1TreeIdx * SAMPLE_DIM + i] = x1[i];  // --- Transfer unexplored sample to tree ---
    treeSampleCosts[x1TreeIdx] = x1[SAMPLE_DIM - 1] + treeSampleCosts[x0Idx];             // --- Update cost of new sample ---

    // --- Update Frontier ---
    frontier[x1TreeIdx] = true;

    // --- Goal Criteria Check ---
    if(distance(x1, s_xGoal) < GOAL_THRESH) costToGoal[0] = treeSampleCosts[x1TreeIdx];
}

void KGMT::updateFrontier()
{
    // --- Find indices and size of the next frontier ---
    thrust::exclusive_scan(d_frontierNext_.begin(), d_frontierNext_.end(), d_frontierScanIdx_.begin(), 0, thrust::plus<uint>());
    h_frontierNextSize_ = d_frontierScanIdx_[MAX_TREE_SIZE - 1];
    findInd<<<h_gridSize_, h_blockSize_>>>(MAX_TREE_SIZE, d_frontierNext_ptr_, d_frontierScanIdx_ptr_, d_activeFrontierIdxs_ptr_);

    // --- Update Frontier ---
    updateFrontier_kernel<<<iDivUp(h_frontierNextSize_, h_blockSize_), h_blockSize_>>>(
      d_frontier_ptr_, d_activeFrontierIdxs_ptr_, h_frontierNextSize_, d_treeSamples_ptr_, h_treeSize_, d_unexploredSamples_ptr_,
      d_treeSamples_ptr_, d_unexploredSamplesParentIdxs_ptr_, d_treeSamplesParentIdxs_ptr_, d_treeSampleCosts_ptr_,
      d_sampleScoreThreshold_ptr_);

    // --- Update Tree Size ---
    h_treeSize_ += h_frontierNextSize_;
}

void KGMT::writeDeviceVectorsToCSV()
{
    std::ostringstream filename;
    std::filesystem::create_directories("Data");
    std::filesystem::create_directories("Data/Samples");
    std::filesystem::create_directories("Data/UnexploredSamples");
    std::filesystem::create_directories("Data/Parents");
    std::filesystem::create_directories("Data/R1Scores");
    std::filesystem::create_directories("Data/R1Avail");
    std::filesystem::create_directories("Data/R1");
    std::filesystem::create_directories("Data/G");
    std::filesystem::create_directories("Data/GNew");
    filename.str("");
    filename << "Data/Samples/samples" << h_itr_ << ".csv";
    copyAndWriteVectorToCSV(d_treeSamples_, filename.str(), MAX_TREE_SIZE, SAMPLE_DIM);
    filename.str("");
    filename << "Data/Parents/parents" << h_itr_ << ".csv";
    copyAndWriteVectorToCSV(d_treeSamplesParentIdxs_, filename.str(), MAX_TREE_SIZE, 1);
    filename.str("");
    filename << "Data/R1Scores/R1Scores" << h_itr_ << ".csv";
    copyAndWriteVectorToCSV(graph_.d_vertexScoreArray_, filename.str(), NUM_R1_VERTICES, 1);
    filename.str("");
    filename << "Data/R1Avail/R1Avail" << h_itr_ << ".csv";
    copyAndWriteVectorToCSV(graph_.d_activeVertices_, filename.str(), NUM_R1_VERTICES, 1);
    filename.str("");
    filename << "Data/R1/R1" << h_itr_ << ".csv";
    copyAndWriteVectorToCSV(graph_.d_counterArray_, filename.str(), NUM_R1_VERTICES, 1);
    filename.str("");
    filename << "Data/UnexploredSamples/unexploredSamples" << h_itr_ << ".csv";
    copyAndWriteVectorToCSV(d_unexploredSamples_, filename.str(), MAX_TREE_SIZE, SAMPLE_DIM);
}
