/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ryan Luna */

#ifndef OMPL_CONTROL_PLANNERS_ModEST_ModEST_
#define OMPL_CONTROL_PLANNERS_ModEST_ModEST_

#include "ompl/datastructures/Grid.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/PDF.h"
#include <unordered_map>
#include <vector>

namespace ompl
{
    namespace control
    {
        class ModEST : public base::Planner
        {
        public:
            // --- Modifications ---
            int iterations_ = 0;

            ModEST(const SpaceInformationPtr &si);

            ~ModEST() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            double getGoalBias() const
            {
                return goalBias_;
            }

            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            double getRange() const
            {
                return maxDistance_;
            }

            void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
            {
                projectionEvaluator_ = projectionEvaluator;
            }

            void setProjectionEvaluator(const std::string &name)
            {
                projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
            }

            const base::ProjectionEvaluatorPtr &getProjectionEvaluator() const
            {
                return projectionEvaluator_;
            }

            void setup() override;

            void getPlannerData(base::PlannerData &data) const override;

        protected:
            class Motion
            {
            public:
                Motion() = default;

                Motion(const SpaceInformation *si) : state(si->allocState()), control(si->allocControl()) {}

                ~Motion() = default;

                base::State *state{nullptr};

                Control *control{nullptr};

                unsigned int steps{0};

                Motion *parent{nullptr};
            };

            struct MotionInfo;

            using GridCell = Grid<MotionInfo>::Cell;

            using CellPDF = PDF<GridCell *>;

            struct MotionInfo
            {
                Motion *operator[](unsigned int i)
                {
                    return motions_[i];
                }
                const Motion *operator[](unsigned int i) const
                {
                    return motions_[i];
                }
                void push_back(Motion *m)
                {
                    motions_.push_back(m);
                }
                unsigned int size() const
                {
                    return motions_.size();
                }
                bool empty() const
                {
                    return motions_.empty();
                }
                std::vector<Motion *> motions_;
                CellPDF::Element *elem_;
            };

            struct TreeData
            {
                TreeData() = default;

                Grid<MotionInfo> grid{0};

                unsigned int size{0};
            };

            void freeMemory();

            void addMotion(Motion *motion);

            Motion *selectMotion();

            base::ValidStateSamplerPtr sampler_;

            DirectedControlSamplerPtr controlSampler_;

            const SpaceInformation *siC_;

            TreeData tree_;

            base::ProjectionEvaluatorPtr projectionEvaluator_;

            double goalBias_{0.05};

            double maxDistance_{0.};

            RNG rng_;

            CellPDF pdf_;

            Motion *lastGoalMotion_{nullptr};
        };
    }  // namespace control
}  // namespace ompl

#endif