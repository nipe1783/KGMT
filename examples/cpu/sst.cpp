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

void moveFiles(int iteration, int threadCount, std::string name)
{
    int fileIndex = 0;

    for (auto &entry : std::filesystem::directory_iterator(std::filesystem::current_path()))
    {
        if (entry.is_regular_file())
        {
            // The old filename, e.g. "solution_times_and_costs_140425260144384.csv"
            std::string oldName = entry.path().filename().string();

            const std::string prefix = "solution_times_and_costs_";

            // Check if it starts with the prefix and ends with ".csv"
            if (oldName.rfind(prefix, 0) == 0 &&  // starts with prefix
                oldName.size() > prefix.size() &&
                oldName.compare(oldName.size() - 4, 4, ".csv") == 0)
            {
                // Example global index if you want 0..(threadCount*iterations - 1)
                int globalIndex = iteration * threadCount + fileIndex;

                // Build the new filename
                // e.g. "house_solution_times_and_costs0.csv", then 1.csv, etc.
                std::string newName = name + "_solution_times_and_costs"
                                    + std::to_string(globalIndex) + ".csv";

                // Move it into the "Data/" directory
                newName = "Data/" + newName;

                // Actually rename (move) the file
                std::filesystem::rename(entry.path(), newName);

                fileIndex++;
            }
        }
    }
}


int main(void)
{
    // --- Remove Previous Bench Data ---
    system("rm -rf Data/*");
    OMPL_Planner planner;

    // float h_initial[SAMPLE_DIM] = {.1, .080, .05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    //       h_goal[SAMPLE_DIM]    = {.8, .950, .900, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float h_initial[SAMPLE_DIM] = {10.0, 8, 5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          h_goal[SAMPLE_DIM]    = {80, 95.0, 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    int numObstacles = 1;
    float* d_obstacles;
    
    std::vector<float> obstacles = readObstaclesFromCSV_CPU("../include/config/obstacles/quadHouse/obstacles.csv", numObstacles, W_DIM);
    
    for(int i = 0; i < 5; i++)
        {
            planner.planSST(h_initial, h_goal, obstacles.data(), numObstacles, 0.0);
            moveFiles(i, 20, "house");
        }

    obstacles = readObstaclesFromCSV_CPU("../include/config/obstacles/quadZigZag/obstacles.csv", numObstacles, W_DIM);

    for(int i = 0; i < 5; i++)
        {
            planner.planSST(h_initial, h_goal, obstacles.data(), numObstacles, 0.0);
            moveFiles(i, 20, "zigZag");
        }

    obstacles = readObstaclesFromCSV_CPU("../include/config/obstacles/quadTrees/obstacles.csv", numObstacles, W_DIM);

    for(int i = 0; i < 5; i++)
        {
            planner.planSST(h_initial, h_goal, obstacles.data(), numObstacles, 0.0);
            moveFiles(i, 20, "trees");
        }

    obstacles = readObstaclesFromCSV_CPU("../include/config/obstacles/quadNarrowPassage/obstacles.csv", numObstacles, W_DIM);

    for(int i = 0; i < 5; i++)
        {
            planner.planSST(h_initial, h_goal, obstacles.data(), numObstacles, 0.0);
            moveFiles(i, 20, "narrowPassage");
        }



    return 0;
}