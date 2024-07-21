#include "graphs/Graph.cuh"
#include "config/config.h"

Graph::Graph(const float ws)
{
    h_numEdges_ = (DIM == 2) ? pow(R1, 2) * 4 : pow(R1, 3) * 6;
    h_vertexArray_.resize((DIM == 2) ? pow(R1, 2) : pow(R1, 3));
    constructVertexArray();
    constructEdgeArray();
    constructFromVertices();
    constructToVertices();
    if(VERBOSE)
        {
            printf("/***************************/\n");
            printf("/* Graph Dimension: %d */\n", DIM);
            printf("/* Number of Edges: %d */\n", h_numEdges_);
            printf("/***************************/\n");
        }

    d_validCounterArray_     = thrust::device_vector<int>(NUM_R1_VERTICES);
    d_counterArray_          = thrust::device_vector<int>(NUM_R1_VERTICES);
    d_vertexScoreArray_      = thrust::device_vector<float>(NUM_R1_VERTICES);
    d_activeVerticesScanIdx_ = thrust::device_vector<int>(NUM_R1_VERTICES);
    d_activeSubVertices_     = thrust::device_vector<bool>(NUM_R2_VERTICES);

    d_validCounterArray_ptr_ = thrust::raw_pointer_cast(d_validCounterArray_.data());
    d_counterArray_ptr_      = thrust::raw_pointer_cast(d_counterArray_.data());
    d_vertexScoreArray_ptr_  = thrust::raw_pointer_cast(d_vertexScoreArray_.data());
    d_activeSubVertices_ptr_ = thrust::raw_pointer_cast(d_activeSubVertices_.data());
}

void Graph::constructVertexArray()
{
    int edgeIdx = 0;
    for(int i = 0; i < R1; ++i)
        {
            for(int j = 0; j < R1; ++j)
                {
                    for(int k = 0; k < (DIM == 3 ? R1 : 1); ++k)
                        {
                            int currentNode             = (DIM == 2) ? i * R1 + j : (i * R1 * R1) + (j * R1) + k;
                            h_vertexArray_[currentNode] = edgeIdx;

                            // Calculate the number of edges for the current node
                            int edges = 0;
                            if(DIM == 2)
                                {
                                    if(i > 0) edges++;
                                    if(j > 0) edges++;
                                    if(i < R1 - 1) edges++;
                                    if(j < R1 - 1) edges++;
                                }
                            else if(DIM == 3)
                                {
                                    if(i > 0) edges++;
                                    if(j > 0) edges++;
                                    if(k > 0) edges++;
                                    if(i < R1 - 1) edges++;
                                    if(j < R1 - 1) edges++;
                                    if(k < R1 - 1) edges++;
                                }

                            edgeIdx += edges;
                        }
                }
        }
}

void Graph::constructEdgeArray()
{
    for(int i = 0; i < R1; ++i)
        {
            for(int j = 0; j < R1; ++j)
                {
                    for(int k = 0; k < (DIM == 3 ? R1 : 1); ++k)
                        {
                            if(DIM == 2)
                                {
                                    if(i > 0) h_edgeArray_.push_back((i - 1) * R1 + j);
                                    if(j > 0) h_edgeArray_.push_back(i * R1 + (j - 1));
                                    if(i < R1 - 1) h_edgeArray_.push_back((i + 1) * R1 + j);
                                    if(j < R1 - 1) h_edgeArray_.push_back(i * R1 + (j + 1));
                                }
                            if(DIM == 3)
                                {
                                    if(i > 0) h_edgeArray_.push_back(((i - 1) * R1 * R1) + (j * R1) + k);
                                    if(j > 0) h_edgeArray_.push_back((i * R1 * R1) + ((j - 1) * R1) + k);
                                    if(k > 0) h_edgeArray_.push_back((i * R1 * R1) + (j * R1) + (k - 1));
                                    if(i < R1 - 1) h_edgeArray_.push_back(((i + 1) * R1 * R1) + (j * R1) + k);
                                    if(j < R1 - 1) h_edgeArray_.push_back((i * R1 * R1) + ((j + 1) * R1) + k);
                                    if(k < R1 - 1) h_edgeArray_.push_back((i * R1 * R1) + (j * R1) + (k + 1));
                                }
                        }
                }
        }
}

void Graph::constructFromVertices()
{
    for(int i = 0; i < R1; ++i)
        {
            for(int j = 0; j < R1; ++j)
                {
                    for(int k = 0; k < (DIM == 3 ? R1 : 1); ++k)
                        {
                            int currentVertex = (DIM == 2) ? i * R1 + j : (i * R1 * R1) + (j * R1) + k;
                            if(DIM == 2)
                                {
                                    if(i > 0) h_fromVertices_.push_back(currentVertex);
                                    if(j > 0) h_fromVertices_.push_back(currentVertex);
                                    if(i < R1 - 1) h_fromVertices_.push_back(currentVertex);
                                    if(j < R1 - 1) h_fromVertices_.push_back(currentVertex);
                                }
                            if(DIM == 3)
                                {
                                    if(i > 0) h_fromVertices_.push_back(currentVertex);
                                    if(j > 0) h_fromVertices_.push_back(currentVertex);
                                    if(k > 0) h_fromVertices_.push_back(currentVertex);
                                    if(i < R1 - 1) h_fromVertices_.push_back(currentVertex);
                                    if(j < R1 - 1) h_fromVertices_.push_back(currentVertex);
                                    if(k < R1 - 1) h_fromVertices_.push_back(currentVertex);
                                }
                        }
                }
        }
}

void Graph::constructToVertices()
{
    for(int i = 0; i < R1; ++i)
        {
            for(int j = 0; j < R1; ++j)
                {
                    for(int k = 0; k < (DIM == 3 ? R1 : 1); ++k)
                        {
                            if(DIM == 2)
                                {
                                    if(i > 0) h_toVertices_.push_back((i - 1) * R1 + j);
                                    if(j > 0) h_toVertices_.push_back(i * R1 + (j - 1));
                                    if(i < R1 - 1) h_toVertices_.push_back((i + 1) * R1 + j);
                                    if(j < R1 - 1) h_toVertices_.push_back(i * R1 + (j + 1));
                                }
                            if(DIM == 3)
                                {
                                    if(i > 0) h_toVertices_.push_back(((i - 1) * R1 * R1) + (j * R1) + k);
                                    if(j > 0) h_toVertices_.push_back((i * R1 * R1) + ((j - 1) * R1) + k);
                                    if(k > 0) h_toVertices_.push_back((i * R1 * R1) + (j * R1) + (k - 1));
                                    if(i < R1 - 1) h_toVertices_.push_back(((i + 1) * R1 * R1) + (j * R1) + k);
                                    if(j < R1 - 1) h_toVertices_.push_back((i * R1 * R1) + ((j + 1) * R1) + k);
                                    if(k < R1 - 1) h_toVertices_.push_back((i * R1 * R1) + (j * R1) + (k + 1));
                                }
                        }
                }
        }
}

__host__ __device__ int getVertex(float x, float y)
{
    int cellX = static_cast<int>(x / R1_SIZE);
    int cellY = static_cast<int>(y / R1_SIZE);

    if(cellX >= 0 && cellX < R1 && cellY >= 0 && cellY < R1)
        {
            return cellY * R1 + cellX;
        }
    return -1;
}

__host__ __device__ int getVertex(float x, float y, float z)
{
    int cellX = static_cast<int>(x / R1_SIZE);
    int cellY = static_cast<int>(y / R1_SIZE);
    int cellZ = static_cast<int>(z / R1_SIZE);
    if(cellX >= 0 && cellX < R1 && cellY >= 0 && cellY < R1 && (cellZ >= 0 && cellZ < R1))
        {
            return (cellY * R1 + cellX) * R1 + cellZ;
        }
    return -1;
}

__host__ __device__ int getSubVertex(float x, float y, int r1)
{
    if(r1 == -1) return -1;
    int cellX_R2 = static_cast<int>((x - (r1 % R1) * R1_SIZE) / R2_SIZE);
    int cellY_R2 = static_cast<int>((y - (r1 / R1) * R1_SIZE) / R2_SIZE);
    if(cellX_R2 >= 0 && cellX_R2 < R2 && cellY_R2 >= 0 && cellY_R2 < R2)
        {
            return r1 * (R2 * R2) + (cellY_R2 * R2 + cellX_R2);
        }
    return -1;
}

__host__ __device__ int getSubVertex(float x, float y, float z, int r1)
{
    if(r1 == -1) return -1;

    // Calculate base cell coordinates in the R1 grid
    int cellY_base = r1 / (R1 * R1);
    int cellX_base = (r1 / R1) % R1;
    int cellZ_base = r1 % R1;

    // Calculate sub-cell coordinates within the R1 cell
    int cellX_R2 = static_cast<int>((x - cellX_base * R1_SIZE) / R2_SIZE);
    int cellY_R2 = static_cast<int>((y - cellY_base * R1_SIZE) / R2_SIZE);
    int cellZ_R2 = static_cast<int>((z - cellZ_base * R1_SIZE) / R2_SIZE);

    // Check if the sub-cell coordinates are within valid bounds
    if(cellX_R2 >= 0 && cellX_R2 < R2 && cellY_R2 >= 0 && cellY_R2 < R2 && cellZ_R2 >= 0 && cellZ_R2 < R2)
        {
            // Return the flattened index of the sub-cell
            return r1 * (R2 * R2 * R2) + (cellZ_R2 * R2 * R2) + (cellY_R2 * R2) + cellX_R2;
        }
    return -1;
}

__host__ __device__ int hashEdge(int key, int size)
{
    return key % size;
}

__host__ __device__ int getEdge(int fromVertex, int toVertex, int* hashTable, int numEdges)
{
    int key  = fromVertex * 100000 + toVertex;
    int hash = hashEdge(key, numEdges);
    while(hashTable[2 * hash] != key)
        {
            if(hashTable[2 * hash] == -1)
                {
                    return -1;
                }
            hash = (hash + 1) % numEdges;
        }
    return hashTable[2 * hash + 1];
}

/***************************/
/* VERTICES UPDATE KERNEL  */
/***************************/
// --- Updates Vertex Scores for device graph vectors. Determines new threshold score for future samples in expansion set. ---
__global__ void updateVertices_kernel(bool* activeSubVertices, int* validCounterArray, int* counterArray, float* vertexScores,
                                      int* updateGraphKeysCounter, int* updateGraphValidKeysCounter)
{
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    if(tid >= NUM_R1_VERTICES) return;

    __shared__ float s_totalScore;
    float score = 0.0;

    counterArray[tid]                = counterArray[tid] + updateGraphKeysCounter[tid];
    validCounterArray[tid]           = validCounterArray[tid] + updateGraphValidKeysCounter[tid];
    updateGraphKeysCounter[tid]      = 0;
    updateGraphValidKeysCounter[tid] = 0;

    if(validCounterArray[tid] > 0)
        {
            int numValidSamples = validCounterArray[tid];
            float coverage      = 0;

            // --- Thread loops through all sub vertices to determine vertex coverage. ---
            for(int i = tid * R2_PER_R1; i < (tid + 1) * R2_PER_R1; ++i)
                {
                    coverage += activeSubVertices[i];
                }

            coverage /= R2_PER_R1;

            // --- From OMPL Syclop ref: https://ompl.kavrakilab.org/classompl_1_1control_1_1Syclop.html---
            score = pow(((EPSILON + numValidSamples) / (EPSILON + numValidSamples + (counterArray[tid] - numValidSamples))), 4) /
                    ((1 + coverage) * (1 + pow(counterArray[tid], 2)));
        }

    // --- Sum scores from each thread to determine score threshold ---
    typedef cub::BlockReduce<float, NUM_R1_VERTICES> BlockReduceFloatT;
    __shared__ typename BlockReduceFloatT::TempStorage tempStorageFloat;
    float blockSum = BlockReduceFloatT(tempStorageFloat).Sum(score);

    if(threadIdx.x == 0)
        {
            s_totalScore = blockSum;
        }
    __syncthreads();

    // --- Update vertex scores ---
    if(validCounterArray[tid] == 0)
        {
            vertexScores[tid] = 1.0f;
        }
    else
        {
            vertexScores[tid] = score / s_totalScore;
        }
}

void Graph::updateVertices(int* d_updateGraphKeysCounter_ptr, int* d_updateGraphValidKeysCounter_ptr)
{
    // --- Update vertex scores and sampleScoreThreshold ---
    updateVertices_kernel<<<1, NUM_R1_VERTICES>>>(d_activeSubVertices_ptr_, d_validCounterArray_ptr_, d_counterArray_ptr_,
                                                  d_vertexScoreArray_ptr_, d_updateGraphKeysCounter_ptr, d_updateGraphValidKeysCounter_ptr);
}