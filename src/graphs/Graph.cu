#include "graphs/Graph.cuh"
#include "config/config.h"

Graph::Graph(const float ws, const int dim, const int r1, const int r2)
{
    h_r1Size_ = ws / r1;
    h_r2Size_ = ws / (r1 * r2);
    h_numEdges_ = (dim == 2) ? pow(r1, 2) * 4 : pow(r1, 3) * 6;
    h_vertexArray_.resize((dim == 2) ? pow(r1, 2) : pow(r1, 3));
    constructVertexArray(dim, r1);
    constructEdgeArray(dim, r1);
    constructFromVertices(dim, r1);
    constructToVertices(dim, r1);
    if(VERBOSE)
        {
            printf("/* Graph Dimension: %d */\n", dim);
            printf("/* Number of Edges: %d */\n", h_numEdges_);
        }
}

void Graph::constructVertexArray(const int dim, const int r)
{
    int edgeIdx = 0;
    for(int i = 0; i < r; ++i)
        {
            for(int j = 0; j < r; ++j)
                {
                    for(int k = 0; k < (dim == 3 ? r : 1); ++k)
                        {
                            int currentNode = (dim == 2) ? i * r + j : (i * r * r) + (j * r) + k;
                            h_vertexArray_[currentNode] = edgeIdx;

                            // Calculate the number of edges for the current node
                            int edges = 0;
                            if(dim == 2)
                                {
                                    if(i > 0) edges++;
                                    if(j > 0) edges++;
                                    if(i < r - 1) edges++;
                                    if(j < r - 1) edges++;
                                }
                            else if(dim == 3)
                                {
                                    if(i > 0) edges++;
                                    if(j > 0) edges++;
                                    if(k > 0) edges++;
                                    if(i < r - 1) edges++;
                                    if(j < r - 1) edges++;
                                    if(k < r - 1) edges++;
                                }

                            edgeIdx += edges;
                        }
                }
        }
}

void Graph::constructEdgeArray(const int dim, const int r)
{
    for(int i = 0; i < r; ++i)
        {
            for(int j = 0; j < r; ++j)
                {
                    for(int k = 0; k < (dim == 3 ? r : 1); ++k)
                        {
                            if(dim == 2)
                                {
                                    if(i > 0) h_edgeArray_.push_back((i - 1) * r + j);
                                    if(j > 0) h_edgeArray_.push_back(i * r + (j - 1));
                                    if(i < r - 1) h_edgeArray_.push_back((i + 1) * r + j);
                                    if(j < r - 1) h_edgeArray_.push_back(i * r + (j + 1));
                                }
                            if(dim == 3)
                                {
                                    if(i > 0) h_edgeArray_.push_back(((i - 1) * r * r) + (j * r) + k);
                                    if(j > 0) h_edgeArray_.push_back((i * r * r) + ((j - 1) * r) + k);
                                    if(k > 0) h_edgeArray_.push_back((i * r * r) + (j * r) + (k - 1));
                                    if(i < r - 1) h_edgeArray_.push_back(((i + 1) * r * r) + (j * r) + k);
                                    if(j < r - 1) h_edgeArray_.push_back((i * r * r) + ((j + 1) * r) + k);
                                    if(k < r - 1) h_edgeArray_.push_back((i * r * r) + (j * r) + (k + 1));
                                }
                        }
                }
        }
}

void Graph::constructFromVertices(const int dim, const int r)
{
    for(int i = 0; i < r; ++i)
        {
            for(int j = 0; j < r; ++j)
                {
                    for(int k = 0; k < (dim == 3 ? r : 1); ++k)
                        {
                            int currentVertex = (dim == 2) ? i * r + j : (i * r * r) + (j * r) + k;
                            if(dim == 2)
                                {
                                    if(i > 0) h_fromVertices_.push_back(currentVertex);
                                    if(j > 0) h_fromVertices_.push_back(currentVertex);
                                    if(i < r - 1) h_fromVertices_.push_back(currentVertex);
                                    if(j < r - 1) h_fromVertices_.push_back(currentVertex);
                                }
                            if(dim == 3)
                                {
                                    if(i > 0) h_fromVertices_.push_back(currentVertex);
                                    if(j > 0) h_fromVertices_.push_back(currentVertex);
                                    if(k > 0) h_fromVertices_.push_back(currentVertex);
                                    if(i < r - 1) h_fromVertices_.push_back(currentVertex);
                                    if(j < r - 1) h_fromVertices_.push_back(currentVertex);
                                    if(k < r - 1) h_fromVertices_.push_back(currentVertex);
                                }
                        }
                }
        }
}

void Graph::constructToVertices(const int dim, const int r)
{
    for(int i = 0; i < r; ++i)
        {
            for(int j = 0; j < r; ++j)
                {
                    for(int k = 0; k < (dim == 3 ? r : 1); ++k)
                        {
                            if(dim == 2)
                                {
                                    if(i > 0) h_toVertices_.push_back((i - 1) * r + j);
                                    if(j > 0) h_toVertices_.push_back(i * r + (j - 1));
                                    if(i < r - 1) h_toVertices_.push_back((i + 1) * r + j);
                                    if(j < r - 1) h_toVertices_.push_back(i * r + (j + 1));
                                }
                            if(dim == 3)
                                {
                                    if(i > 0) h_toVertices_.push_back(((i - 1) * r * r) + (j * r) + k);
                                    if(j > 0) h_toVertices_.push_back((i * r * r) + ((j - 1) * r) + k);
                                    if(k > 0) h_toVertices_.push_back((i * r * r) + (j * r) + (k - 1));
                                    if(i < r - 1) h_toVertices_.push_back(((i + 1) * r * r) + (j * r) + k);
                                    if(j < r - 1) h_toVertices_.push_back((i * r * r) + ((j + 1) * r) + k);
                                    if(k < r - 1) h_toVertices_.push_back((i * r * r) + (j * r) + (k + 1));
                                }
                        }
                }
        }
}