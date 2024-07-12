#include "graphs/Graph.cuh"
#include "config/config.h"

Graph::Graph(const float ws)
{
    h_r1Size_ = ws / R1;
    h_r2Size_ = ws / (R1 * R2);
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
    // TODO: Delete Comments
    // if(VERBOSE)
    //     {
    //         printf("/***************************/\n");
    //         printf("/* Vertex Array */\n");
    //         for(int i = 0; i < h_vertexArray_.size(); ++i)
    //             {
    //                 printf("Vertex %d: %d\n", i, h_vertexArray_[i]);
    //             }
    //         printf("/***************************/\n");
    //     }

    // if(VERBOSE)
    //     {
    //         printf("/***************************/\n");
    //         printf("/* Edge Array */\n");
    //         for(int i = 0; i < h_edgeArray_.size(); ++i)
    //             {
    //                 printf("Edge %d: %d\n", i, h_edgeArray_[i]);
    //             }
    //         printf("/***************************/\n");
    //     }

    // if(VERBOSE)
    //     {
    //         printf("/***************************/\n");
    //         printf("/* From->To Vertices */\n");
    //         for(int i = 0; i < h_fromVertices_.size(); ++i)
    //             {
    //                 printf("From Vertex %d: To Vertex %d\n", h_fromVertices_[i], h_toVertices_[i]);
    //             }
    //     }

    d_activeVertices_ = thrust::device_vector<int>(NUM_R1_VERTICES);
    d_validCounterArray_ = thrust::device_vector<int>(NUM_R1_VERTICES);
    d_counterArray_ = thrust::device_vector<int>(NUM_R1_VERTICES);
    d_vertexScoreArray_ = thrust::device_vector<float>(NUM_R1_VERTICES);
    d_activeVerticesScanIdx_ = thrust::device_vector<int>(NUM_R1_VERTICES);

    d_activeVertices_ptr_ = thrust::raw_pointer_cast(d_activeVertices_.data());
    d_validCounterArray_ptr_ = thrust::raw_pointer_cast(d_validCounterArray_.data());
    d_counterArray_ptr_ = thrust::raw_pointer_cast(d_counterArray_.data());
    d_vertexScoreArray_ptr_ = thrust::raw_pointer_cast(d_vertexScoreArray_.data());
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
                            int currentNode = (DIM == 2) ? i * R1 + j : (i * R1 * R1) + (j * R1) + k;
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

__host__ __device__ int getVertex(float x, float y, float r1Size)
{
    int cellX = static_cast<int>(x / r1Size);
    int cellY = static_cast<int>(y / r1Size);

    if(cellX >= 0 && cellX < R1 && cellY >= 0 && cellY < R1)
        {
            return cellY * R1 + cellX;
        }
    return -1;
}

__host__ __device__ int getVertex(float x, float y, float z, float r1Size)
{
    int cellX = static_cast<int>(x / r1Size);
    int cellY = static_cast<int>(y / r1Size);
    int cellZ = static_cast<int>(z / r1Size);

    if(cellX >= 0 && cellX < R1 && cellY >= 0 && cellY < R1 && (cellZ >= 0 && cellZ < R1))
        {
            return (cellY * R1 + cellX) * R1 + cellZ;
        }
    return -1;
}

__host__ __device__ int getSubVertex(float x, float y, int r1, float r1Size, float r2Size)
{
    if(r1 == -1)
        {
            return -1;
        }

    int cellX_R2 = static_cast<int>((x - (r1 % R1) * r1Size) / r2Size);
    int cellY_R2 = static_cast<int>((y - (r1 / R1) * r1Size) / r2Size);
    if(cellX_R2 >= 0 && cellX_R2 < R2 && cellY_R2 >= 0 && cellY_R2 < R2)
        {
            return r1 * (R2 * R2) + (cellY_R2 * R2 + cellX_R2);
        }
    return -1;
}

__host__ __device__ int getSubVertex(float x, float y, float z, int r1, float r1Size, float r2Size)
{
    if(r1 == -1)
        {
            return -1;
        }

    int cellX_R2 = static_cast<int>((x - (r1 / (R1 * R1)) * r1Size) / r2Size);
    int cellY_R2 = static_cast<int>((y - ((r1 / R1) % R1) * r1Size) / r2Size);
    int cellZ_R2 = static_cast<int>((z - (r1 % R1) * r1Size) / r2Size);
    if(cellX_R2 >= 0 && cellX_R2 < R2 && cellY_R2 >= 0 && cellY_R2 < R2 && cellZ_R2 >= 0 && cellZ_R2 < R2)
        {
            return r1 * (R2 * R2 * R2) + ((cellY_R2 * R2 + cellX_R2) * R2 + cellZ_R2);
        }
    return -1;
}

__host__ __device__ int hashEdge(int key, int size)
{
    return key % size;
}

__host__ __device__ int getEdge(int fromVertex, int toVertex, int* hashTable, int numEdges)
{
    int key = fromVertex * 100000 + toVertex;
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
__global__ void updateVertices_kernel(float* vertexScoreArray, int* activeVertices, int* activeSubVertices, int* validCounterArray,
                                      int* counterArray, int numActiveVertices, float* vertexScores, float* sampleScoreThreshold)
{
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    if(tid >= NUM_R1_VERTICES)
        {
            return;
        }

    __shared__ float s_totalScore;
    float score = 0.0;

    if(activeVertices[tid] != 0)
        {
            int numValidSamples = validCounterArray[tid];
            float coverage = 0;
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
            sampleScoreThreshold[0] = s_totalScore / numActiveVertices;
        }
    __syncthreads();

    // --- Update vertex scores ---
    if(activeVertices[tid] == 0)
        {
            vertexScores[tid] = 1.0f;
        }
    else
        {
            vertexScores[tid] = score / s_totalScore;
        }
}

void Graph::updateVertices(float* d_sampleScoreThreshold_ptr)
{
    // --- Determine number of active vertices in graph ---
    thrust::exclusive_scan(d_activeVertices_.begin(), d_activeVertices_.end(), d_activeVerticesScanIdx_.begin(), 0, thrust::plus<int>());
    h_numActiveVertices_ = d_activeVerticesScanIdx_[NUM_R1_VERTICES - 1];

    // --- Update vertex scores and sampleScoreThreshold ---
    updateVertices_kernel<<<1, NUM_R1_VERTICES>>>(d_vertexScoreArray_ptr_, d_activeVertices_ptr_, d_activeSubVertices_ptr_,
                                                  d_validCounterArray_ptr_, d_counterArray_ptr_, h_numActiveVertices_,
                                                  d_vertexScoreArray_ptr_, d_sampleScoreThreshold_ptr);
}