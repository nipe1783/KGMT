/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Rice University
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

/* Author: Jonathan Sobieski, Mark Moll */

#ifndef OMPL_CONTROL_PLANNERS_ModPDST_ModPDST_
#define OMPL_CONTROL_PLANNERS_ModPDST_ModPDST_

#include <utility>

#include "ompl/base/Planner.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/control/PathControl.h"
#include "ompl/control/PlannerData.h"
#include "ompl/datastructures/BinaryHeap.h"

namespace ompl
{
    namespace control
    {
        class ModPDST : public base::Planner
        {
        public:
            // --- Modifications ---
            int iterations_ = 0;
            // ---  ---

            ModPDST(const SpaceInformationPtr &si);

            ~ModPDST() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            void clear() override;
            void setup() override;

            void getPlannerData(base::PlannerData &data) const override;

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

            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }
            double getGoalBias() const
            {
                return goalBias_;
            }

        protected:
            struct Cell;
            struct Motion;

            struct MotionCompare
            {
                bool operator()(Motion *p1, Motion *p2) const
                {
                    // lowest priority means highest score
                    return p1->score() < p2->score();
                }
            };

            struct Motion
            {
            public:
                Motion(base::State *startState, base::State *endState, Control *control, unsigned int controlDuration, double priority,
                       Motion *parent)
                    : startState_(startState)
                    , endState_(endState)
                    , control_(control)
                    , controlDuration_(controlDuration)
                    , priority_(priority)
                    , parent_(parent)
                    , cell_(nullptr)
                    , heapElement_(nullptr)
                    , isSplit_(false)
                {}
                Motion(base::State *state)
                    : startState_(state)
                    , endState_(state)
                    , control_(nullptr)
                    , controlDuration_(0)
                    , priority_(0.)
                    , parent_(nullptr)
                    , cell_(nullptr)
                    , heapElement_(nullptr)
                    , isSplit_(false)
                {}
                double score() const
                {
                    return priority_ / cell_->volume_;
                }
                void updatePriority()
                {
                    priority_ = priority_ * 2. + 1.;
                }

                base::State *startState_;
                base::State *endState_;
                control::Control *control_;
                unsigned int controlDuration_;
                double priority_;
                Motion *parent_;
                Cell *cell_;
                BinaryHeap<Motion *, MotionCompare>::Element *heapElement_;
                bool isSplit_;
            };

            struct Cell
            {
                Cell(double volume, base::RealVectorBounds bounds, unsigned int splitDimension = 0)
                    : volume_(volume)
                    , splitDimension_(splitDimension)
                    , splitValue_(0.0)
                    , left_(nullptr)
                    , right_(nullptr)
                    , bounds_(std::move(bounds))
                {}

                ~Cell()
                {
                    if(left_ != nullptr)
                        {
                            delete left_;
                            delete right_;
                        }
                }

                void subdivide(unsigned int spaceDimension);

                Cell *stab(const Eigen::Ref<Eigen::VectorXd> &projection) const
                {
                    auto *containingCell = const_cast<Cell *>(this);
                    while(containingCell->left_ != nullptr)
                        {
                            if(projection[containingCell->splitDimension_] <= containingCell->splitValue_)
                                containingCell = containingCell->left_;
                            else
                                containingCell = containingCell->right_;
                        }
                    return containingCell;
                }
                void addMotion(Motion *motion)
                {
                    motions_.push_back(motion);
                    motion->cell_ = this;
                }

                unsigned int size() const
                {
                    unsigned int sz = 1;
                    if(left_ != nullptr) sz += left_->size() + right_->size();
                    return sz;
                }

                double volume_;
                unsigned int splitDimension_;
                double splitValue_;
                Cell *left_;
                Cell *right_;
                base::RealVectorBounds bounds_;
                std::vector<Motion *> motions_;
            };

            void
            addMotion(Motion *motion, Cell *cell, base::State *, base::State *, Eigen::Ref<Eigen::VectorXd>, Eigen::Ref<Eigen::VectorXd>);
            void updateHeapElement(Motion *motion)
            {
                if(motion->heapElement_ != nullptr)
                    priorityQueue_.update(motion->heapElement_);
                else
                    motion->heapElement_ = priorityQueue_.insert(motion);
            }
            Motion *propagateFrom(Motion *motion, base::State *, base::State *);
            unsigned int findDurationAndAncestor(Motion *motion, base::State *state, base::State *scratch, Motion *&ancestor) const;

            void freeMemory();

            base::StateSamplerPtr sampler_;
            DirectedControlSamplerPtr controlSampler_;
            const SpaceInformation *siC_;
            // Random number generator
            RNG rng_;
            std::vector<Motion *> startMotions_;
            BinaryHeap<Motion *, MotionCompare> priorityQueue_;
            Cell *bsp_{nullptr};
            base::ProjectionEvaluatorPtr projectionEvaluator_;
            double goalBias_{0.05};
            base::GoalSampleableRegion *goalSampler_{nullptr};
            unsigned int iteration_{1};
            Motion *lastGoalMotion_{nullptr};
        };
    }  // namespace control
}  // namespace ompl

#endif