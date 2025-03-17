#include "planners/Planner.cuh"
#include "config/config.h"

Planner::Planner()
{
    d_treeSamples_           = thrust::device_vector<float>(MAX_TREE_SIZE * SAMPLE_DIM);
    d_treeSamplesParentIdxs_ = thrust::device_vector<int>(MAX_TREE_SIZE);
    d_treeSampleCosts_       = thrust::device_vector<float>(MAX_TREE_SIZE);
    d_unexploredSampleCosts_ = thrust::device_vector<float>(MAX_TREE_SIZE);
    d_goalSet_               = thrust::device_vector<int>(MAX_TREE_SIZE);
    d_controlPathsToGoal_    = thrust::device_vector<float>(MAX_ITER * SAMPLE_DIM);
    d_pathCosts_             = thrust::device_vector<float>(3 * MAX_SOL_SET_SIZE);
    d_iterations_            = thrust::device_vector<int>(MAX_TREE_SIZE);

    d_treeSamples_ptr_           = thrust::raw_pointer_cast(d_treeSamples_.data());
    d_treeSamplesParentIdxs_ptr_ = thrust::raw_pointer_cast(d_treeSamplesParentIdxs_.data());
    d_treeSampleCosts_ptr_       = thrust::raw_pointer_cast(d_treeSampleCosts_.data());
    d_unexploredSampleCosts_ptr_ = thrust::raw_pointer_cast(d_unexploredSampleCosts_.data());
    d_goalSet_ptr_               = thrust::raw_pointer_cast(d_goalSet_.data());
    d_controlPathsToGoal_ptr_    = thrust::raw_pointer_cast(d_controlPathsToGoal_.data());
    d_pathCosts_ptr_             = thrust::raw_pointer_cast(d_pathCosts_.data());
    d_iterations_ptr_            = thrust::raw_pointer_cast(d_iterations_.data());

    h_gridSize_ = iDivUp(MAX_TREE_SIZE, h_blockSize_);

    cudaMalloc(&d_randomSeeds_ptr_, MAX_TREE_SIZE * sizeof(curandState));
    cudaMalloc(&d_costToGoal_ptr_, sizeof(float));
    cudaMalloc(&d_pathToGoal_ptr_, sizeof(int));

    h_controlPathsToGoal_ = new float[SAMPLE_DIM * MAX_ITER];

    if(VERBOSE)
        {
            printf("/***************************/\n");
            printf("/* Workspace Dimension: %d */\n", W_DIM);
            printf("/* Workspace Size: %f */\n", W_SIZE);
            printf("/* Maximum discretization steps in propagation: %d */\n", MAX_PROPAGATION_DURATION);
            printf("/* Propagation step Size: %f */\n", STEP_SIZE);
            printf("/* Max Tree Size: %d */\n", MAX_TREE_SIZE);
            printf("/* Goal Distance Threshold: %f */\n", GOAL_THRESH);
            printf("/* Max Planning Iterations: %d */\n", MAX_ITER);
        }
}

void Planner::writeDeviceVectorsToCSV(int itr)
{
    std::ostringstream filename;
    bool append = h_itr_ != 0;

    // Create necessary directories
    std::filesystem::create_directories("Data");
    std::filesystem::create_directories("Data/Samples/Samples" + std::to_string(itr));
    std::filesystem::create_directories("Data/Parents/Parents" + std::to_string(itr));
    std::filesystem::create_directories("Data/TotalCountPerVertex/TotalCountPerVertex" + std::to_string(itr));
    std::filesystem::create_directories("Data/ValidCountPerVertex/ValidCountPerVertex" + std::to_string(itr));
    std::filesystem::create_directories("Data/Frontier/Frontier" + std::to_string(itr));
    std::filesystem::create_directories("Data/FrontierRepeatCount/FrontierRepeatCount" + std::to_string(itr));
    std::filesystem::create_directories("Data/VertexScores/VertexScores" + std::to_string(itr));
    std::filesystem::create_directories("Data/FrontierSize/FrontierSize" + std::to_string(itr));
    std::filesystem::create_directories("Data/TreeSize/TreeSize" + std::to_string(itr));
    std::filesystem::create_directories("Data/ExpandedNodes/ExpandedNodes" + std::to_string(itr));
    std::filesystem::create_directories("Data/ControlPathsToGoal/ControlPathsToGoal" + std::to_string(itr));
    std::filesystem::create_directories("Data/goalSet/goalSet" + std::to_string(itr));
    std::filesystem::create_directories("Data/treeSampleCosts/treeSampleCosts" + std::to_string(itr));
    std::filesystem::create_directories("Data/minCosts/minCosts" + std::to_string(itr));
    std::filesystem::create_directories("Data/pathCosts/pathCosts" + std::to_string(itr));

    // Write Samples
    filename.str("");
    filename << "Data/Samples/Samples" << itr << "/samples" << h_itr_ << ".csv";
    copyAndWriteVectorToCSV(d_treeSamples_, filename.str(), MAX_TREE_SIZE, SAMPLE_DIM, append);

    // Write Goal Set
    filename.str("");
    filename << "Data/goalSet/goalSet" << itr << "/goalSet" << h_itr_ << ".csv";
    copyAndWriteVectorToCSV(d_goalSet_, filename.str(), MAX_TREE_SIZE, 1, false);

    // Write Parents
    filename.str("");
    filename << "Data/Parents/Parents" << itr << "/parents" << h_itr_ << ".csv";
    copyAndWriteVectorToCSV(d_treeSamplesParentIdxs_, filename.str(), MAX_TREE_SIZE, 1, append);

    // Write Tree Size
    filename.str("");
    filename << "Data/TreeSize/TreeSize" << itr << "/treeSize.csv";
    writeValueToCSV(h_treeSize_, filename.str());

    // Write Control Path to Goal
    filename.str("");
    filename << "Data/ControlPathsToGoal/ControlPathsToGoal" << itr << "/controlPathsToGoal.csv";
    copyAndWriteVectorToCSV(d_controlPathsToGoal_, filename.str(), MAX_ITER, SAMPLE_DIM, false);

    // Write Tree Sample Costs
    filename.str("");
    filename << "Data/treeSampleCosts/treeSampleCosts" << itr << "/treeSampleCosts.csv";
    copyAndWriteVectorToCSV(d_treeSampleCosts_, filename.str(), MAX_TREE_SIZE, 1, false);

    // Write Path Costs
    filename.str("");
    filename << "Data/pathCosts/pathCosts" << itr << "/pathCosts.csv";
    copyAndWriteVectorToCSV(d_pathCosts_, filename.str(), 2, 1, false);
}

void Planner::writeSolutionsToCSV(int itr)
{
    std::ostringstream filename;
    std::filesystem::create_directories("Data/ControlPathsToGoal/ControlPathsToGoal" + std::to_string(itr));
    filename.str("");
    filename << "Data/ControlPathsToGoal/ControlPathsToGoal" << itr << "/controlPathsToGoal.csv";
    copyAndWriteVectorToCSV(d_controlPathsToGoal_, filename.str(), MAX_ITER, SAMPLE_DIM, false);
}

void Planner::writeSolutionCostsToCSV(int itr)
{
    std::ostringstream filename;
    std::filesystem::create_directories("Data/PathCosts");
    filename.str("");
    filename << "Data/PathCosts/pathCosts" << itr << ".csv";
    copyAndWriteVectorToCSV(d_pathCosts_, filename.str(), MAX_SOL_SET_SIZE, 3, false);
}

void Planner::writeIterationTimeToCSV(const std::vector<float>& iterationTimes, int itr)
{
    std::filesystem::path dirPath = "Data/IterationTime";
    std::filesystem::create_directories(dirPath);
    std::filesystem::path filePath = dirPath / ("IterationTime" + std::to_string(itr) + ".csv");
    std::ofstream file(filePath, std::ios_base::out);
    for(const auto& time : iterationTimes)
        {
            file << time << std::endl;
        }

    file.close();
}

void Planner::writeExecutionTimeToCSV(double time)
{
    std::ostringstream filename;
    std::filesystem::create_directories("Data");
    std::filesystem::create_directories("Data/ExecutionTime");
    filename.str("");
    filename << "Data/ExecutionTime/executionTime.csv";
    writeValueToCSV(time, filename.str());
}

__global__ void initializeRandomSeeds_kernel(curandState* randomSeeds, int numSeeds, int seed)
{
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    if(tid < numSeeds)
        {
            curand_init(seed, tid, 0, &randomSeeds[tid]);
        }
}

void Planner::initializeRandomSeeds(int seed)
{
    int blockSize = 32;
    initializeRandomSeeds_kernel<<<iDivUp(MAX_TREE_SIZE, blockSize), blockSize>>>(d_randomSeeds_ptr_, MAX_TREE_SIZE, seed);
}

__global__ void findInd(uint numSamples, bool* S, uint* scanIdx, uint* activeS)
{
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if(tid >= numSamples) return;
    if(!S[tid]) return;
    activeS[scanIdx[tid]] = tid;
}

__global__ void findInd(uint numSamples, uint* S, uint* scanIdx, uint* activeS)
{
    int node = blockIdx.x * blockDim.x + threadIdx.x;
    if(node >= numSamples) return;
    if(!S[node]) return;
    activeS[scanIdx[node]] = node;
}

__global__ void repeatInd(uint numSamples, uint* activeS, uint* C, uint* prefixSum, uint* repeatedInd)
{
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if(tid >= numSamples) return;

    uint index    = activeS[tid];
    uint count    = C[index];
    uint startPos = prefixSum[index];
    for(uint i = 0; i < count; ++i)
        {
            repeatedInd[startPos + i] = index;
        }
}