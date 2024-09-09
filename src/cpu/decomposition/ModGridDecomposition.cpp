/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Rice University
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

/* Author: Mark Moll */

#include "cpu/decomposition/ModGridDecomposition.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;

static ompl::base::RealVectorBounds bounds3(int dim, float max)
{
    ompl::base::RealVectorBounds b(3);
    b.low[0]  = 0;
    b.low[1]  = 0;
    b.low[2]  = 0;
    b.high[0] = max;
    b.high[1] = max;
    b.high[2] = max;
    // OMPL_INFORM("bounds3: %f %f %f %f %f %f", b.low[0], b.low[1], b.low[2], b.high[0], b.high[1], b.high[2]);

    return b;
}

ompl::control::ModGridDecomposition::ModGridDecomposition(int len, int dim, float max) : GridDecomposition(len, dim, bounds3(dim, max)) {}

void ompl::control::ModGridDecomposition::project(const ompl::base::State *s, std::vector<double> &coord) const
{
    auto compState = s->as<ob::CompoundStateSpace::StateType>();
    auto xyState   = compState->as<ob::RealVectorStateSpace::StateType>(0);
    const double x = xyState->values[0];
    const double y = xyState->values[1];
    const double z = xyState->values[2];
    coord.resize(3);
    coord[0] = x;
    coord[1] = y;
    coord[2] = z;
    // OMPL_INFORM("project: (%f, %f, %f)", x, y, z);
}

void ompl::control::ModGridDecomposition::sampleFullState(const ompl::base::StateSamplerPtr &sampler, const std::vector<double> &coord,
                                                          ompl::base::State *s) const
{
    auto compState = s->as<ob::CompoundStateSpace::StateType>();
    auto xyState   = compState->as<ob::RealVectorStateSpace::StateType>(0);
    double *pos    = xyState->values;
    pos[0]         = coord[0];
    pos[1]         = coord[1];
    pos[2]         = coord[2];
}