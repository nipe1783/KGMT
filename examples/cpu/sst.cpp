#include "cpu/planners/OMPL_Planner.h"
#include <iostream>
#include <filesystem>

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
    // --- Remove Previous Bench Data ---
    system("rm -rf Data/*");
    OMPL_Planner planner;

    float h_initial[SAMPLE_DIM] = {.1, .080, .05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          h_goal[SAMPLE_DIM]    = {.8, .950, .900, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    int numObstacles = 1;
    float* d_obstacles;
    std::vector<float> obstacles = readObstaclesFromCSV_CPU("../include/config/obstacles/zigZag/obstacles.csv", numObstacles, W_DIM);

    for(int i = 0; i < 100; i++)
        {
            planner.planSST(h_initial, h_goal, obstacles.data(), numObstacles, 0.0);
            std::filesystem::rename("solution_times_and_costs.csv", "Data/zigZag_solution_times_and_costs" + std::to_string(i) + ".csv");
        }

    obstacles = readObstaclesFromCSV_CPU("../include/config/obstacles/house/obstacles.csv", numObstacles, W_DIM);
    
    for(int i = 0; i < 100; i++)
        {
            planner.planSST(h_initial, h_goal, obstacles.data(), numObstacles, 0.0);
            std::filesystem::rename("solution_times_and_costs.csv", "Data/house_solution_times_and_costs" + std::to_string(i) + ".csv");
        }
        

    obstacles = readObstaclesFromCSV_CPU("../include/config/obstacles/trees/obstacles.csv", numObstacles, W_DIM);

    for(int i = 0; i < 100; i++)
        {
            planner.planSST(h_initial, h_goal, obstacles.data(), numObstacles, 0.0);
            std::filesystem::rename("solution_times_and_costs.csv", "Data/trees_solution_times_and_costs" + std::to_string(i) + ".csv");
        }

    obstacles = readObstaclesFromCSV_CPU("../include/config/obstacles/narrowPassage/obstacles.csv", numObstacles, W_DIM);

    for(int i = 0; i < 100; i++)
        {
            planner.planSST(h_initial, h_goal, obstacles.data(), numObstacles, 0.0);
            std::filesystem::rename("solution_times_and_costs.csv", "Data/narrowPassage_solution_times_and_costs" + std::to_string(i) + ".csv");
        }



    return 0;
}