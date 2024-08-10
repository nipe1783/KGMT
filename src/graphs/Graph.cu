#include "graphs/Graph.cuh"
#include "config/config.h"

Graph::Graph(const float ws)
{
    if(VERBOSE)
        {
            printf("/***************************/\n");
            printf("/* Graph Dimension: %d */\n", DIM);
            printf("/***************************/\n");
        }

    d_validCounterArray_     = thrust::device_vector<int>(NUM_R1_VERTICES);
    d_counterArray_          = thrust::device_vector<int>(NUM_R1_VERTICES);
    d_vertexScoreArray_      = thrust::device_vector<float>(NUM_R1_VERTICES);
    d_activeVerticesScanIdx_ = thrust::device_vector<int>(NUM_R1_VERTICES);
    d_activeSubVertices_     = thrust::device_vector<int>(NUM_R2_VERTICES);
    d_minValueInRegion_      = thrust::device_vector<float>(NUM_R1_VERTICES * STATE_DIM);

    d_validCounterArray_ptr_ = thrust::raw_pointer_cast(d_validCounterArray_.data());
    d_counterArray_ptr_      = thrust::raw_pointer_cast(d_counterArray_.data());
    d_vertexScoreArray_ptr_  = thrust::raw_pointer_cast(d_vertexScoreArray_.data());
    d_activeSubVertices_ptr_ = thrust::raw_pointer_cast(d_activeSubVertices_.data());
    d_minValueInRegion_ptr_  = thrust::raw_pointer_cast(d_minValueInRegion_.data());

    initializeRegions();
}

void Graph::initializeRegions()
{
    initializeRegions_kernel<<<iDivUp(NUM_R1_VERTICES, h_blockSize_), h_blockSize_>>>(d_minValueInRegion_ptr_);
}

/***************************/
/* INITIALIZE REGIONS KERNEL */
/***************************/
// --- one thread per R1 region ---
__global__ void initializeRegions_kernel(float* minValueInRegion)
{
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    if(tid >= NUM_R1_VERTICES) return;

    int wRegion = tid % (W_R1_LENGTH * W_R1_LENGTH * W_R1_LENGTH);
    int wIndex[DIM];
    int temp = wRegion;
    for(int i = DIM - 1; i >= 0; --i)
        {
            wIndex[i] = temp % W_R1_LENGTH;
            temp /= W_R1_LENGTH;
        }

    for(int i = 0; i < DIM; ++i)
        {
            minValueInRegion[tid * STATE_DIM + i] = W_MIN + wIndex[i] * W_R1_SIZE;
        }

    int aRegion = (tid / (W_R1_LENGTH * W_R1_LENGTH * W_R1_LENGTH)) % (C_R1_LENGTH * C_R1_LENGTH);
    int aIndex[C_DIM];
    temp = aRegion;
    for(int i = C_DIM - 1; i >= 0; --i)
        {
            aIndex[i] = temp % C_R1_LENGTH;
            temp /= C_R1_LENGTH;
        }
    for(int i = 0; i < C_DIM; ++i)
        {
            minValueInRegion[tid * STATE_DIM + DIM + i] = C_MIN + aIndex[i] * C_R1_SIZE;
        }

    int vRegion = (tid / (W_R1_LENGTH * W_R1_LENGTH * W_R1_LENGTH * C_R1_LENGTH * C_R1_LENGTH)) % V_R1_LENGTH;
    int vIndex[V_DIM];
    temp = vRegion;
    for(int i = V_DIM - 1; i >= 0; --i)
        {
            vIndex[i] = temp % V_R1_LENGTH;
            temp /= V_R1_LENGTH;
        }
    for(int i = 0; i < V_DIM; ++i)
        {
            minValueInRegion[tid * STATE_DIM + DIM + C_DIM + i] = V_MIN + vIndex[i] * V_R1_SIZE;
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
    if(cellX >= 0 && cellX < R1 && cellY >= 0 && cellY < R1 && cellZ >= 0 && cellZ < R1)
        {
            return cellX * R1 * R1 + cellY * R1 + cellZ;
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
    int cellX_base = r1 / (R1 * R1);
    int cellY_base = (r1 / R1) % R1;
    int cellZ_base = r1 % R1;

    // Calculate sub-cell coordinates within the R1 cell
    int cellX_R2 = static_cast<int>((x - cellX_base * R1_SIZE) / R2_SIZE);
    int cellY_R2 = static_cast<int>((y - cellY_base * R1_SIZE) / R2_SIZE);
    int cellZ_R2 = static_cast<int>((z - cellZ_base * R1_SIZE) / R2_SIZE);

    // Check if the sub-cell coordinates are within valid bounds
    if(cellX_R2 >= 0 && cellX_R2 < R2 && cellY_R2 >= 0 && cellY_R2 < R2 && cellZ_R2 >= 0 && cellZ_R2 < R2)
        {
            // Return the flattened index of the sub-cell
            return r1 * (R2 * R2 * R2) + (cellX_R2 * R2 * R2) + (cellY_R2 * R2) + cellZ_R2;
        }
    return -1;
}

__host__ __device__ int getRegion(float* coord)
{
    // --- Workspace ---
    int wRegion = 0;
    int factor  = 1;
    int index;
    for(int i = W_DIM - 1; i >= 0; --i)
        {
            index = (int)(W_R1_LENGTH * (coord[i] - W_MIN) / (W_MAX - W_MIN));
            if(index >= W_R1_LENGTH) index = W_R1_LENGTH - 1;

            wRegion += factor * index;
            factor *= W_R1_LENGTH;
        }

    if(V_DIM == 1 && C_DIM == 1)
        {
            return wRegion;
        }

    // --- Attitude ---
    int aRegion = 0;
    factor      = 1;
    for(int i = C_DIM - 1; i >= 0; --i)
        {
            index = (int)(C_R1_LENGTH * (coord[i + DIM] - C_MIN) / (C_MAX - C_MIN));
            if(index >= C_R1_LENGTH) index = C_R1_LENGTH - 1;

            aRegion += factor * index;
            factor *= C_R1_LENGTH;
        }

    // --- Velocity ---
    int vRegion = 0;
    factor      = 1;
    for(int i = V_DIM - 1; i >= 0; --i)
        {
            index = (int)(V_R1_LENGTH * (coord[i + DIM + C_DIM] - V_MIN) / (V_MAX - V_MIN));
            if(index >= V_R1_LENGTH) index = V_R1_LENGTH - 1;

            vRegion += factor * index;
            factor *= V_R1_LENGTH;
        }

    return wRegion * C_R1_LENGTH * C_R1_LENGTH * V_R1_LENGTH + aRegion * V_R1_LENGTH + vRegion;
}

__device__ int getSubRegion(float* coord, int r1, float* minRegion)
{
    // --- Workspace ---
    int wRegion = 0;
    int factor  = 1;
    int index;

    for(int i = DIM - 1; i >= 0; --i)
        {
            index = (int)(W_R2_LENGTH * (coord[i] - minRegion[r1 * STATE_DIM + i]) / (W_R1_SIZE));
            if(index >= W_R2_LENGTH) index = W_R2_LENGTH - 1;

            wRegion += factor * index;
            factor *= W_R2_LENGTH;
        }

    // --- Attitude ---
    int aRegion = 0;
    factor      = 1;
    for(int i = C_DIM - 1; i >= 0; --i)
        {
            index = (int)(C_R2_LENGTH * (coord[i + DIM] - minRegion[r1 * STATE_DIM + i + DIM]) / (C_R1_SIZE));
            if(index >= C_R2_LENGTH) index = C_R2_LENGTH - 1;

            aRegion += factor * index;
            factor *= C_R2_LENGTH;
        }

    // --- Velocity ---
    int vRegion = 0;
    factor      = 1;
    for(int i = V_DIM - 1; i >= 0; --i)
        {
            index = (int)(V_R2_LENGTH * (coord[i + DIM + C_DIM] - minRegion[r1 * STATE_DIM + i + DIM + C_DIM]) / (V_R1_SIZE));
            if(index >= V_R2_LENGTH) index = V_R2_LENGTH - 1;

            vRegion += factor * index;
            factor *= V_R2_LENGTH;
        }

    return r1 * NUM_R2_PER_R1 + (wRegion * C_R2_LENGTH * C_R2_LENGTH * V_R2_LENGTH + aRegion * V_R2_LENGTH + vRegion);
}

/***************************/
/* VERTICES UPDATE KERNEL  */
/***************************/
// --- Updates Vertex Scores for device graph vectors. Determines new threshold score for future samples in expansion set. ---
__global__ void updateVertices_kernel(int* activeSubVertices, int* validCounterArray, int* counterArray, float* vertexScores)
{
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    if(tid >= NUM_R1_VERTICES - 1) return;

    __shared__ float s_totalScore;
    float score = 0.0;

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
            float freeVol =
              (EPSILON + numValidSamples) / (EPSILON + numValidSamples + (counterArray[tid] - numValidSamples)) * pow(R1_SIZE, DIM);
            score = pow(freeVol, 4) / ((1 + coverage) * (1 + pow(counterArray[tid], 2)));
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
            // TODO: check if adding epsilon is ok.
            vertexScores[tid] = EPSILON + (score / s_totalScore);
        }
}

void Graph::updateVertices()
{
    // --- Update vertex scores and sampleScoreThreshold ---
    updateVertices_kernel<<<1, NUM_R1_VERTICES>>>(d_activeSubVertices_ptr_, d_validCounterArray_ptr_, d_counterArray_ptr_,
                                                  d_vertexScoreArray_ptr_);
}