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
    // float h_initial[SAMPLE_DIM] = {.100, .080, .05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    //       h_goal[SAMPLE_DIM]    = {.800, .950, .900, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float h_initial[SAMPLE_DIM] = {10.0, 8, 5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          h_goal[SAMPLE_DIM]    = {80, 95.0, 90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    int numObstacles;
    std::vector<float> obstacles;

    OMPL_Planner rrt;
    int N = 50;

    std::string sourceDir, targetDir;

    /***************************/
    /* 12DQuad TREES */
    /***************************/

    // --- Remove Previous Bench Data ---
    system("rm -rf Data/*");

    obstacles = readObstaclesFromCSV_CPU("../include/config/obstacles/quadTrees/obstacles.csv", numObstacles, W_DIM);

    // --- RRT ---
    for(int i = 0; i < N; i++)
        {
            rrt.planParallelRRT(h_initial, h_goal, obstacles.data(), numObstacles, 0.00);
        }

    sourceDir = "/home/nicolas/dev/research/KGMT/build/Data";
    targetDir = "/home/nicolas/dev/research/KGMT/benchmarking/rrtParallel/12DQuad/trees/Data";

    if(!std::filesystem::exists(targetDir))
        {
            std::filesystem::create_directories(targetDir);
        }

    system(("mv " + sourceDir + "/* " + targetDir + "/").c_str());
    std::filesystem::create_directories(sourceDir);

    // --- EST ---
    for(int i = 0; i < N; i++)
        {
            rrt.planParallelEST(h_initial, h_goal, obstacles.data(), numObstacles, 0.00);
        }

    targetDir = "/home/nicolas/dev/research/KGMT/benchmarking/estParallel/12DQuad/trees/Data";

    if(!std::filesystem::exists(targetDir))
        {
            std::filesystem::create_directories(targetDir);
        }

    system(("mv " + sourceDir + "/* " + targetDir + "/").c_str());
    std::filesystem::create_directories(sourceDir);

    // --- PDST ---
    for(int i = 0; i < N; i++)
        {
            rrt.planParallelPDST(h_initial, h_goal, obstacles.data(), numObstacles, 0.00);
        }

    targetDir = "/home/nicolas/dev/research/KGMT/benchmarking/pdstParallel/12DQuad/trees/Data";

    if(!std::filesystem::exists(targetDir))
        {
            std::filesystem::create_directories(targetDir);
        }

    system(("mv " + sourceDir + "/* " + targetDir + "/").c_str());
    std::filesystem::create_directories(sourceDir);

    /***************************/
    /* 12DQuad NARROW PASSAGE */
    /***************************/

    // --- Remove Previous Bench Data ---
    system("rm -rf Data/*");

    obstacles = readObstaclesFromCSV_CPU("../include/config/obstacles/quadNarrowPassage/obstacles.csv", numObstacles, W_DIM);

    // --- RRT ---
    for(int i = 0; i < N; i++)
        {
            rrt.planParallelRRT(h_initial, h_goal, obstacles.data(), numObstacles, 0.00);
        }

    sourceDir = "/home/nicolas/dev/research/KGMT/build/Data";
    targetDir = "/home/nicolas/dev/research/KGMT/benchmarking/rrtParallel/12DQuad/narrowPassage/Data";

    if(!std::filesystem::exists(targetDir))
        {
            std::filesystem::create_directories(targetDir);
        }

    system(("mv " + sourceDir + "/* " + targetDir + "/").c_str());
    std::filesystem::create_directories(sourceDir);

    // --- EST ---
    for(int i = 0; i < N; i++)
        {
            rrt.planParallelEST(h_initial, h_goal, obstacles.data(), numObstacles, 0.00);
        }

    targetDir = "/home/nicolas/dev/research/KGMT/benchmarking/estParallel/12DQuad/narrowPassage/Data";

    if(!std::filesystem::exists(targetDir))
        {
            std::filesystem::create_directories(targetDir);
        }

    system(("mv " + sourceDir + "/* " + targetDir + "/").c_str());
    std::filesystem::create_directories(sourceDir);

    // --- PDST ---
    for(int i = 0; i < N; i++)
        {
            rrt.planParallelPDST(h_initial, h_goal, obstacles.data(), numObstacles, 0.00);
        }

    targetDir = "/home/nicolas/dev/research/KGMT/benchmarking/pdstParallel/12DQuad/narrowPassage/Data";

    if(!std::filesystem::exists(targetDir))
        {
            std::filesystem::create_directories(targetDir);
        }

    system(("mv " + sourceDir + "/* " + targetDir + "/").c_str());
    std::filesystem::create_directories(sourceDir);

    /***************************/
    /* HOUSE */
    /***************************/

    // --- Remove Previous Bench Data ---
    system("rm -rf Data/*");

    // --- RRT ---
    for(int i = 0; i < N; i++)
        {
            rrt.planParallelRRT(h_initial, h_goal, obstacles.data(), numObstacles, 0.00);
        }

    sourceDir = "/home/nicolas/dev/research/KGMT/build/Data";
    targetDir = "/home/nicolas/dev/research/KGMT/benchmarking/rrtParallel/12DQuad/house/Data";

    if(!std::filesystem::exists(targetDir))
        {
            std::filesystem::create_directories(targetDir);
        }

    system(("mv " + sourceDir + "/* " + targetDir + "/").c_str());
    std::filesystem::create_directories(sourceDir);

    // --- EST ---
    for(int i = 0; i < N; i++)
        {
            rrt.planParallelEST(h_initial, h_goal, obstacles.data(), numObstacles, 0.00);
        }

    targetDir = "/home/nicolas/dev/research/KGMT/benchmarking/estParallel/12DQuad/house/Data";

    if(!std::filesystem::exists(targetDir))
        {
            std::filesystem::create_directories(targetDir);
        }

    system(("mv " + sourceDir + "/* " + targetDir + "/").c_str());
    std::filesystem::create_directories(sourceDir);

    // --- PDST ---
    for(int i = 0; i < N; i++)
        {
            rrt.planParallelPDST(h_initial, h_goal, obstacles.data(), numObstacles, 0.00);
        }

    targetDir = "/home/nicolas/dev/research/KGMT/benchmarking/pdstParallel/12DQuad/house/Data";

    if(!std::filesystem::exists(targetDir))
        {
            std::filesystem::create_directories(targetDir);
        }

    system(("mv " + sourceDir + "/* " + targetDir + "/").c_str());
    std::filesystem::create_directories(sourceDir);

    return 0;
}