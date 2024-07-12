#pragma once
#include <stdio.h>
#include <vector>

class Graph
{
public:
    // --- constructor ---
    Graph() = default;
    Graph(float h_ws);

    // --- fields ---
    int h_numEdges_;
    float h_r1Size_, h_r2Size_;
    std::vector<int> h_fromVertices_, h_toVertices_, h_vertexArray_, h_edgeArray_, h_weightArray_;

private:
    // --- methods ---
    void constructFromVertices();
    void constructToVertices();
    void constructVertexArray();
    void constructEdgeArray();
};

// --- Device Functions ---
__host__ __device__ int getVertex(float x, float y, float r1Size);
__host__ __device__ int getVertex(float x, float y, float z, float r1Size);
__host__ __device__ int getSubVertex(float x, float y, int r1, float r1Size, float r2Size);
__host__ __device__ int getSubVertex(float x, float y, float z, int r1, float r1Size, float r2Size);
__host__ __device__ int getEdge(int fromVertex, int toVertex, int* hashTable, int numEdges);
__host__ __device__ int hashEdge(int key, int size);