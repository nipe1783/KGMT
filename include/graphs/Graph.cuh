#pragma once
#include <stdio.h>
#include <vector>

class Graph
{
public:
    // --- constructor ---
    Graph() = default;
    Graph(float h_ws, const int dim, const int r1, const int r2);

    // --- fields ---
    int h_numEdges_;
    float h_r1Size_, h_r2Size_;
    std::vector<int> h_fromVertices_, h_toVertices_, h_vertexArray_, h_edgeArray_, h_weightArray_;

private:
    // --- methods ---
    void constructFromVertices(const int dim, const int r);
    void constructToVertices(const int dim, const int r);
    void constructVertexArray(const int dim, const int r);
    void constructEdgeArray(const int dim, const int r);
};