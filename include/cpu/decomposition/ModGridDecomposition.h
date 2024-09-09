#ifndef OMPL_CONTROL_PLANNERS_SYCLOP_ModGRIDDECOMPOSITION_
#define OMPL_CONTROL_PLANNERS_SYCLOP_ModGRIDDECOMPOSITION_

#include <cstdlib>
#include <memory>
#include <unordered_map>
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/State.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"
#include "ompl/util/RandomNumbers.h"

namespace ompl
{
    namespace control
    {
        class ModGridDecomposition : public GridDecomposition
        {
        public:
            // Constructor declared but not defined in the header
            ModGridDecomposition(int len, int dim, float max);

            void project(const ompl::base::State *s, std::vector<double> &coord) const override;
            void sampleFullState(const ompl::base::StateSamplerPtr &sampler, const std::vector<double> &coord,
                                 ompl::base::State *s) const override;
        };
    }  // namespace control
}  // namespace ompl

#endif  // OMPL_CONTROL_PLANNERS_SYCLOP_ModGRIDDECOMPOSITION_
