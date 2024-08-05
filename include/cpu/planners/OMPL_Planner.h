#pragma once

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include "ompl/control/ODESolver.h"
#include <ompl/base/Goal.h>
#include <stdio.h>
#include "config/config.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/util/Console.h"
#include <iostream>
#include <filesystem>
#include <fstream>
#include "cpu/collisionCheck/CollisionCheck.h"
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <omp.h>
#include <thread>
#include <chrono>
#include <ompl/base/PlannerData.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace fs = std::filesystem;

class OMPL_Planner
{
public:
    // constructor and destructor:
    OMPL_Planner();
    ~OMPL_Planner();

    // fields:
    float safetyMargin_;
    int obstaclesCount_;
    float* obstacles_;

    // methods:
    void planParallelPDST(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin);
    void planParallelRRT(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin);
    void planParallelEST(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin);
    void planRRT(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin);
    void planEST(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin);
    void planPDST(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin);
    oc::SimpleSetupPtr kinodynamicSimpleSetUp(const float* initial, const float* goal);

    ob::StateSpacePtr createStateSpace();
    oc::ControlSpacePtr createControlSpace(ob::StateSpacePtr& space);
};