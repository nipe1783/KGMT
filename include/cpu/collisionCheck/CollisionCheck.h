#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "config/config.h"

namespace ob = ompl::base;

class CollisionCheck : public ob::StateValidityChecker
{
    public:
        CollisionCheck(const ob::SpaceInformationPtr &si, float* obstacles, const int obstaclesCount, float &safetyMargin);
        bool isValid(const ob::State *state) const override;

    private:
        const ob::SpaceInformation *si_;
        float safetyMargin_;
        int obstaclesCount_;
        float* obstacles_;
};