#include "planners/KGMT.cuh"
#include "config/config.h"

KGMT::KGMT()
{
    graph_ = Graph(WS_SIZE);

    d_frontier_                    = thrust::device_vector<bool>(MAX_TREE_SIZE);
    d_frontierNext_                = thrust::device_vector<bool>(MAX_TREE_SIZE);
    d_activeFrontierIdxs_          = thrust::device_vector<uint>(MAX_TREE_SIZE);
    d_unexploredSamples_           = thrust::device_vector<float>(MAX_TREE_SIZE * SAMPLE_DIM);
    d_unexploredSamplesParentIdxs_ = thrust::device_vector<int>(MAX_TREE_SIZE);
    d_frontierScanIdx_             = thrust::device_vector<uint>(MAX_TREE_SIZE);
    d_goalSample_                  = thrust::device_vector<float>(SAMPLE_DIM);

    d_frontier_ptr_                    = thrust::raw_pointer_cast(d_frontier_.data());
    d_frontierNext_ptr_                = thrust::raw_pointer_cast(d_frontierNext_.data());
    d_activeFrontierIdxs_ptr_          = thrust::raw_pointer_cast(d_activeFrontierIdxs_.data());
    d_unexploredSamples_ptr_           = thrust::raw_pointer_cast(d_unexploredSamples_.data());
    d_unexploredSamplesParentIdxs_ptr_ = thrust::raw_pointer_cast(d_unexploredSamplesParentIdxs_.data());
    d_frontierScanIdx_ptr_             = thrust::raw_pointer_cast(d_frontierScanIdx_.data());
    d_goalSample_ptr_                  = thrust::raw_pointer_cast(d_goalSample_.data());

    h_activeBlockSize_ = 32;

    // --- GRAPH HELPERS ---
    d_unexploredSamplesVertices_  = thrust::device_vector<int>(MAX_TREE_SIZE);
    d_updateGraphCounter_         = thrust::device_vector<int>(MAX_TREE_SIZE);
    d_updateGraphKeys_            = thrust::device_vector<int>(NUM_R1_VERTICES + 1);
    d_updateGraphKeysCounter_     = thrust::device_vector<int>(NUM_R1_VERTICES + 1);
    d_updateGraphTempKeys_        = thrust::device_vector<int>(MAX_TREE_SIZE);
    d_updateGraphTempKeysCounter_ = thrust::device_vector<int>(MAX_TREE_SIZE);

    d_unexploredSamplesVertices_ptr_  = thrust::raw_pointer_cast(d_unexploredSamplesVertices_.data());
    d_updateGraphCounter_ptr_         = thrust::raw_pointer_cast(d_updateGraphCounter_.data());
    d_updateGraphKeys_ptr_            = thrust::raw_pointer_cast(d_updateGraphKeys_.data());
    d_updateGraphKeysCounter_ptr_     = thrust::raw_pointer_cast(d_updateGraphKeysCounter_.data());
    d_updateGraphTempKeys_ptr_        = thrust::raw_pointer_cast(d_updateGraphTempKeys_.data());
    d_updateGraphTempKeysCounter_ptr_ = thrust::raw_pointer_cast(d_updateGraphTempKeysCounter_.data());

    d_unexploredSamplesValidVertices_  = thrust::device_vector<int>(MAX_TREE_SIZE);
    d_updateGraphValidCounter_         = thrust::device_vector<int>(MAX_TREE_SIZE);
    d_updateGraphValidKeys_            = thrust::device_vector<int>(NUM_R1_VERTICES + 1);
    d_updateGraphValidKeysCounter_     = thrust::device_vector<int>(NUM_R1_VERTICES + 1);
    d_updateGraphValidTempKeys_        = thrust::device_vector<int>(MAX_TREE_SIZE);
    d_updateGraphValidTempKeysCounter_ = thrust::device_vector<int>(MAX_TREE_SIZE);

    d_unexploredSamplesValidVertices_ptr_  = thrust::raw_pointer_cast(d_unexploredSamplesValidVertices_.data());
    d_updateGraphValidCounter_ptr_         = thrust::raw_pointer_cast(d_updateGraphValidCounter_.data());
    d_updateGraphValidKeys_ptr_            = thrust::raw_pointer_cast(d_updateGraphValidKeys_.data());
    d_updateGraphValidKeysCounter_ptr_     = thrust::raw_pointer_cast(d_updateGraphValidKeysCounter_.data());
    d_updateGraphValidTempKeys_ptr_        = thrust::raw_pointer_cast(d_updateGraphValidTempKeys_.data());
    d_updateGraphValidTempKeysCounter_ptr_ = thrust::raw_pointer_cast(d_updateGraphValidTempKeysCounter_.data());

    d_unexploredSamplesSubVertices_ = thrust::device_vector<int>(MAX_TREE_SIZE);
    d_updateGraphSubKeysCounter_    = thrust::device_vector<bool>(NUM_R2_VERTICES + 1);

    d_unexploredSamplesSubVertices_ptr_ = thrust::raw_pointer_cast(d_unexploredSamplesSubVertices_.data());
    d_updateGraphSubKeysCounter_ptr_    = thrust::raw_pointer_cast(d_updateGraphSubKeysCounter_.data());

    thrust::fill(d_unexploredSamplesVertices_.begin(), d_unexploredSamplesVertices_.end(), NUM_R1_VERTICES + 1);
    thrust::fill(d_updateGraphCounter_.begin(), d_updateGraphCounter_.end(), 1);
    thrust::sequence(d_updateGraphKeys_.begin(), d_updateGraphKeys_.end(), 0);
    thrust::fill(d_updateGraphKeysCounter_.begin(), d_updateGraphKeysCounter_.end(), 0);

    thrust::fill(d_unexploredSamplesValidVertices_.begin(), d_unexploredSamplesValidVertices_.end(), NUM_R1_VERTICES + 1);
    thrust::fill(d_updateGraphValidCounter_.begin(), d_updateGraphValidCounter_.end(), 1);
    thrust::sequence(d_updateGraphValidKeys_.begin(), d_updateGraphValidKeys_.end(), 0);
    thrust::fill(d_updateGraphValidKeysCounter_.begin(), d_updateGraphValidKeysCounter_.end(), 0);

    thrust::fill(d_unexploredSamplesSubVertices_.begin(), d_unexploredSamplesSubVertices_.end(), NUM_R2_VERTICES + 1);
    thrust::fill(d_updateGraphSubKeysCounter_.begin(), d_updateGraphSubKeysCounter_.end(), false);

    // --- END GRAPH HELPERS ---

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
    cudaMemcpy(d_goalSample_ptr_, h_goal, SAMPLE_DIM * sizeof(float), cudaMemcpyHostToDevice);
    initializeRandomSeeds(time(NULL));
    thrust::fill(d_frontier_.begin(), d_frontier_.begin() + 1, true);

    h_treeSize_ = 1;
    h_itr_      = 0;

    while(h_itr_ < MAX_ITER)
        {
            h_itr_++;
            graph_.updateVertices(d_updateGraphKeysCounter_ptr_, d_updateGraphValidKeysCounter_ptr_);
            propagateFrontier(d_obstacles_ptr, h_obstaclesCount);
            updateGraphTotalCount();
            updateGraphValidCount();
            updateGraphSubVerticesOccupancy();
            updateFrontier();
            // writeDeviceVectorsToCSV();
            if(h_costToGoal_ != 0)
                {
                    printf("Goal Reached: %f\n", h_costToGoal_);
                    break;
                }
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
    if(h_activeBlockSize_ * gridSize > (MAX_TREE_SIZE - h_treeSize_))
        {
            int iterations = std::min(int(float(MAX_TREE_SIZE - h_treeSize_) / float(h_frontierSize_)), int(h_activeBlockSize_));
            gridSize       = int(floor(MAX_TREE_SIZE / h_activeBlockSize_));
            // --- Propagate Frontier. One thread per sample. Iterates n times. ---
            propagateFrontier_kernel2<<<gridSize, h_activeBlockSize_>>>(
              d_frontier_ptr_, d_activeFrontierIdxs_ptr_, d_treeSamples_ptr_, d_unexploredSamples_ptr_, h_frontierSize_, d_randomSeeds_ptr_,
              d_unexploredSamplesParentIdxs_ptr_, d_obstacles_ptr, h_obstaclesCount, graph_.d_activeSubVertices_ptr_,
              graph_.d_vertexScoreArray_ptr_, d_frontierNext_ptr_, d_unexploredSamplesVertices_ptr_, d_unexploredSamplesValidVertices_ptr_,
              d_unexploredSamplesSubVertices_ptr_, iterations);
        }
    else
        {
            // --- Propagate Frontier. Block Size threads per sample. ---
            propagateFrontier_kernel1<<<gridSize, h_activeBlockSize_>>>(
              d_frontier_ptr_, d_activeFrontierIdxs_ptr_, d_treeSamples_ptr_, d_unexploredSamples_ptr_, h_frontierSize_, d_randomSeeds_ptr_,
              d_unexploredSamplesParentIdxs_ptr_, d_obstacles_ptr, h_obstaclesCount, graph_.d_activeSubVertices_ptr_,
              graph_.d_vertexScoreArray_ptr_, d_frontierNext_ptr_, d_unexploredSamplesVertices_ptr_, d_unexploredSamplesValidVertices_ptr_,
              d_unexploredSamplesSubVertices_ptr_);
        }
}

/***************************/
/* PROPAGATE FRONTIER KERNEL  */
/***************************/
// --- Propagates current frontier. Builds new frontier. ---
// --- One Block Per Frontier Sample ---
__global__ void
propagateFrontier_kernel1(bool* frontier, uint* activeFrontierIdxs, float* treeSamples, float* unexploredSamples, uint frontierSize,
                          curandState* randomSeeds, int* unexploredSamplesParentIdxs, float* obstacles, int obstaclesCount,
                          bool* activeSubVertices, float* vertexScores, bool* frontierNext, int* unexploredSamplesVertices,
                          int* unexploredSamplesValidVertices, int* unexploredSamplesSubVertices)
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
    unexploredSamplesVertices[tid] = x1Vertex;
    if(valid)
        {
            unexploredSamplesValidVertices[tid] = x1Vertex;
            unexploredSamplesSubVertices[tid]   = x1SubVertex;
            if(curand_uniform(&randSeed) < vertexScores[x1Vertex] || !activeSubVertices[x1SubVertex]) frontierNext[tid] = true;
        }

    randomSeeds[tid] = randSeed;
}

/***************************/
/* FRONTIER PROPAGATION KERNEL 2 */
/***************************/
// --- 1 thread per sample. iterates n times. ---
__global__ void
propagateFrontier_kernel2(bool* frontier, uint* activeFrontierIdxs, float* treeSamples, float* unexploredSamples, uint frontierSize,
                          curandState* randomSeeds, int* unexploredSamplesParentIdxs, float* obstacles, int obstaclesCount,
                          bool* activeSubVertices, float* vertexScores, bool* frontierNext, int* unexploredSamplesVertices,
                          int* unexploredSamplesValidVertices, int* unexploredSamplesSubVertices, int iterations)
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
            unexploredSamplesVertices[x1Idx] = x1Vertex;
            if(valid)
                {
                    unexploredSamplesValidVertices[x1Idx] = x1Vertex;
                    unexploredSamplesSubVertices[x1Idx]   = x1SubVertex;
                    if(curand_uniform(&randSeed) < vertexScores[x1Vertex] || !activeSubVertices[x1SubVertex]) frontierNext[x1Idx] = true;
                }
            randomSeeds[x1Idx] = randSeed;
        }
}

/***************************/
/* FRONTIER UPDATE KERNEL */
/***************************/
// --- Adds previous frontier to the tree and builds new frontier. ---
__global__ void updateFrontier_kernel(bool* frontier, bool* frontierNext, uint* activeFrontierNextIdxs, uint frontierNextSize, float* xGoal,
                                      int treeSize, float* unexploredSamples, float* treeSamples, int* unexploredSamplesParentIdxs,
                                      int* treeSamplesParentIdxs, float* treeSampleCosts, float* costToGoal)
{
    int tid           = blockIdx.x * blockDim.x + threadIdx.x;
    frontierNext[tid] = false;

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
    treeSampleCosts[x1TreeIdx] = distance(x1, s_xGoal);                                   // --- Update cost of new sample ---

    // --- Update Frontier ---
    frontier[x1TreeIdx] = true;

    // --- Goal Criteria Check ---
    if(treeSampleCosts[x1TreeIdx] < GOAL_THRESH) costToGoal[0] = treeSampleCosts[x1TreeIdx];
}

void KGMT::updateFrontier()
{
    // --- Find indices and size of the next frontier ---
    thrust::exclusive_scan(d_frontierNext_.begin(), d_frontierNext_.end(), d_frontierScanIdx_.begin(), 0, thrust::plus<uint>());
    h_frontierNextSize_ = d_frontierScanIdx_[MAX_TREE_SIZE - 1];
    findInd<<<h_gridSize_, h_blockSize_>>>(MAX_TREE_SIZE, d_frontierNext_ptr_, d_frontierScanIdx_ptr_, d_activeFrontierIdxs_ptr_);

    // --- Update Frontier ---
    updateFrontier_kernel<<<std::min(int(h_frontierNextSize_), int(floor(MAX_TREE_SIZE / h_blockSize_))), h_blockSize_>>>(
      d_frontier_ptr_, d_frontierNext_ptr_, d_activeFrontierIdxs_ptr_, h_frontierNextSize_, d_goalSample_ptr_, h_treeSize_,
      d_unexploredSamples_ptr_, d_treeSamples_ptr_, d_unexploredSamplesParentIdxs_ptr_, d_treeSamplesParentIdxs_ptr_,
      d_treeSampleCosts_ptr_, d_costToGoal_ptr_);

    // --- Check for goal criteria ---
    cudaMemcpy(&h_costToGoal_, d_costToGoal_ptr_, sizeof(float), cudaMemcpyDeviceToHost);

    // --- Update Tree Size ---
    h_treeSize_ += h_frontierNextSize_;
}

void KGMT::updateGraphTotalCount()
{
    // --- Sort Vertices of new samples. ---
    thrust::sort(d_unexploredSamplesVertices_.begin(), d_unexploredSamplesVertices_.end());

    auto new_end =
      thrust::reduce_by_key(d_unexploredSamplesVertices_.begin(), d_unexploredSamplesVertices_.end(), d_updateGraphCounter_.begin(),
                            d_updateGraphTempKeys_.begin(), d_updateGraphTempKeysCounter_.begin());

    int numUniqueVertices = new_end.first - d_updateGraphTempKeys_.begin();

    thrust::scatter(thrust::device, d_updateGraphTempKeysCounter_.begin(), d_updateGraphTempKeysCounter_.begin() + numUniqueVertices,
                    d_updateGraphTempKeys_.begin(), d_updateGraphKeysCounter_.begin());

    thrust::fill(d_unexploredSamplesVertices_.begin(), d_unexploredSamplesVertices_.end(), NUM_R1_VERTICES + 1);
    thrust::fill(d_updateGraphCounter_.begin(), d_updateGraphCounter_.end(), 1);
}

void KGMT::updateGraphValidCount()
{
    // --- Sort Vertices of new samples. ---
    thrust::sort(d_unexploredSamplesValidVertices_.begin(), d_unexploredSamplesValidVertices_.end());

    auto new_end = thrust::reduce_by_key(d_unexploredSamplesValidVertices_.begin(), d_unexploredSamplesValidVertices_.end(),
                                         d_updateGraphValidCounter_.begin(), d_updateGraphValidTempKeys_.begin(),
                                         d_updateGraphValidTempKeysCounter_.begin());

    int numUniqueVertices = new_end.first - d_updateGraphValidTempKeys_.begin();

    thrust::scatter(thrust::device, d_updateGraphValidTempKeysCounter_.begin(),
                    d_updateGraphValidTempKeysCounter_.begin() + numUniqueVertices, d_updateGraphValidTempKeys_.begin(),
                    d_updateGraphValidKeysCounter_.begin());

    thrust::fill(d_unexploredSamplesValidVertices_.begin(), d_unexploredSamplesValidVertices_.end(), NUM_R1_VERTICES + 1);
    thrust::fill(d_updateGraphValidCounter_.begin(), d_updateGraphValidCounter_.end(), 1);
}

struct MarkPresence
{
    bool* d_updateGraphSubKeysCounter;

    MarkPresence(bool* ptr) : d_updateGraphSubKeysCounter(ptr) {}

    __device__ void operator()(const int& index) const
    {
        if(index >= 0)
            {
                d_updateGraphSubKeysCounter[index] = true;
            }
    }
};

void KGMT::updateGraphSubVerticesOccupancy()
{
    thrust::sort(d_unexploredSamplesSubVertices_.begin(), d_unexploredSamplesSubVertices_.end());

    auto new_end             = thrust::unique(d_unexploredSamplesSubVertices_.begin(), d_unexploredSamplesSubVertices_.end());
    int numUniqueSubVertices = new_end - d_unexploredSamplesSubVertices_.begin();

    thrust::for_each(d_unexploredSamplesSubVertices_.begin(), d_unexploredSamplesSubVertices_.begin() + numUniqueSubVertices,
                     MarkPresence(d_updateGraphSubKeysCounter_ptr_));

    thrust::transform(graph_.d_activeSubVertices_.begin(), graph_.d_activeSubVertices_.end(), d_updateGraphSubKeysCounter_.begin(),
                      graph_.d_activeSubVertices_.begin(), thrust::logical_or<bool>());

    thrust::fill(d_unexploredSamplesSubVertices_.begin(), d_unexploredSamplesSubVertices_.end(), NUM_R2_VERTICES + 1);
}

void KGMT::writeDeviceVectorsToCSV()
{
    std::ostringstream filename;
    std::filesystem::create_directories("Data");
    std::filesystem::create_directories("Data/Samples");
    std::filesystem::create_directories("Data/UnexploredSamples");
    std::filesystem::create_directories("Data/Parents");
    std::filesystem::create_directories("Data/R1Scores");
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
    filename << "Data/R1/R1" << h_itr_ << ".csv";
    copyAndWriteVectorToCSV(graph_.d_counterArray_, filename.str(), NUM_R1_VERTICES, 1);
    filename.str("");
    filename << "Data/UnexploredSamples/unexploredSamples" << h_itr_ << ".csv";
    copyAndWriteVectorToCSV(d_unexploredSamples_, filename.str(), MAX_TREE_SIZE, SAMPLE_DIM);
}
