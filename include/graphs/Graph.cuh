#pragma once
#include <stdio.h>
#include <vector>
#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/iterator/constant_iterator.h>
#include "helper/helper.cuh"

class Graph
{
public:
    // --- constructor ---
    Graph() = default;
    Graph(float h_ws);

    // --- host fields ---
    int h_numEdges_, h_numActiveVertices_;
    std::vector<int> h_fromVertices_, h_toVertices_, h_vertexArray_, h_edgeArray_, h_weightArray_;

    // --- device fields ---
    thrust::device_vector<int> d_activeVertices_, d_activeSubVertices_, d_validCounterArray_, d_counterArray_, d_activeVerticesScanIdx_;
    thrust::device_vector<float> d_vertexScoreArray_;
    float* d_vertexScoreArray_ptr_;
    int *d_activeVertices_ptr_, *d_activeSubVertices_ptr_, *d_validCounterArray_ptr_, *d_counterArray_ptr_, *d_activeVerticesScanIdx_ptr_;

    /****************************    METHODS    ****************************/
    void updateVertices(float* d_sampleScoreThreshold_ptr);

private:
    /**************************** METHODS ****************************/
    void constructFromVertices();
    void constructToVertices();
    void constructVertexArray();
    void constructEdgeArray();
};

/**************************** DEVICE FUNCTIONS ****************************/
// --- Given an x, y, z point. Returns the vertex on the graph the point belongs to ---
__host__ __device__ int getVertex(float x, float y);
__host__ __device__ int getVertex(float x, float y, float z);

// --- Given an x, y, z point. Returns the sub vertex that the point belongs to. ---
__host__ __device__ int getSubVertex(float x, float y, int r1);
__host__ __device__ int getSubVertex(float x, float y, float z, int r1);

// --- Given two graph vertices, Returns which edge of the graph it corresponds to. ---
__host__ __device__ int getEdge(int fromVertex, int toVertex, int* hashTable, int numEdges);

// --- Hashing Function for edge lookup. ---
__host__ __device__ int hashEdge(int key, int size);

/***************************/
/* VERTICES UPDATE KERNEL */
/***************************/
// --- Updates Vertex Scores for device graph vectors. Determines new threshold score for future samples in expansion set. ---
__global__ void updateVertices_kernel(float* vertexScoreArray, int* activeVertices, int* activeSubVertices, int* validCounterArray,
                                      int* counterArray, int numActiveVertices, float* vertexScores, float* sampleScoreThreshold);