#include "cpu/collisionCheck/CollisionCheck.h"

CollisionCheck::CollisionCheck(const ob::SpaceInformationPtr &si, float* obstacles, int obstaclesCount, float &safetyMargin) :
    ob::StateValidityChecker(si)
    {
        si_ = si.get();
        safetyMargin_ = safetyMargin;
        obstaclesCount_ = obstaclesCount;
        obstacles_ = obstacles;
    }

bool CollisionCheck::isValid(const ob::State *state) const
{
    assert(DIM == 3);
    
    auto compState = state->as<ob::CompoundStateSpace::StateType>();
    auto xyState = compState->as<ob::RealVectorStateSpace::StateType>(0);
    const double x = xyState->values[0];
    const double y = xyState->values[1];
    const double z = xyState->values[2];

    if (!si_->satisfiesBounds(state))
        return false;

    for(int obsIdx = 0; obsIdx < obstaclesCount_; ++obsIdx)
        {
            float obs[2 * DIM];
            for(int d = 0; d < DIM; ++d)
                {
                    obs[d]       = obstacles_[obsIdx * 2 * DIM + d];
                    obs[DIM + d] = obstacles_[obsIdx * 2 * DIM + DIM + d];
                }
            if(x >= obs[0] - safetyMargin_ && x <= obs[3] + safetyMargin_ &&
               y >= obs[1] - safetyMargin_ && y <= obs[4] + safetyMargin_ &&
               z >= obs[2] - safetyMargin_ && z <= obs[5] + safetyMargin_)
                return false;
        }
    return true;
}