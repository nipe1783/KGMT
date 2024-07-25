#pragma once

#include <ompl/control/planners/pdst/PDST.h>
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

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace fs = std::filesystem;

class PDST_6DI
{
public:
    // constructor and destructor:
    PDST_6DI();
    ~PDST_6DI();

    // fields:
    ob::RealVectorBounds boundsPos_, boundsVel_;
    float safetyMargin_;
    int obstaclesCount_;
    float* obstacles_;

    // methods:
    void plan(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin);
    oc::SimpleSetupPtr kinodynamicSimpleSetUp(const float* initial, const float* goal);

    ob::StateSpacePtr createStateSpace();
    oc::ControlSpacePtr createControlSpace(ob::StateSpacePtr &space);

};