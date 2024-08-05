#include "cpu/planners/OMPL_Planner.h"
#include <iostream>

std::vector<float> readObstaclesFromCSV_CPU(const std::string& filename, int& numObstacles, int workspaceDim)
{
    std::vector<float> obstacles;
    std::ifstream file(filename);

    if(!file.is_open())
        {
            std::cerr << "Error opening file: " << filename << std::endl;
            exit(1);
        }

    std::string line;
    while(std::getline(file, line))
        {
            std::stringstream ss(line);
            float value;
            while(ss >> value)
                {
                    obstacles.push_back(value);
                    if(ss.peek() == ',') ss.ignore();
                }
        }

    file.close();
    numObstacles = obstacles.size() / (2 * workspaceDim);
    return obstacles;
}

int main(void)
{
    float h_initial[SAMPLE_DIM] = {0.3, 0.02, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          h_goal[SAMPLE_DIM]    = {.7, .95, .9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    int numObstacles = 1;
    float* d_obstacles;
    std::vector<float> obstacles = readObstaclesFromCSV_CPU("../include/config/obstacles/pillars/obstacles.csv", numObstacles, DIM);

    OMPL_Planner rrt;
    rrt.planParallelRRT(h_initial, h_goal, obstacles.data(), numObstacles, 0.0);

    return 0;
}