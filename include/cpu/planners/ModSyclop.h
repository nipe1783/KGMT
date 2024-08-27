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

/* Author: Matt Maly */

#ifndef OMPL_CONTROL_PLANNERS_ModSYCLOP_ModSYCLOP_
#define OMPL_CONTROL_PLANNERS_ModSYCLOP_ModSYCLOP_

#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"
#include "ompl/datastructures/PDF.h"
#include "ompl/util/Hash.h"
#include <functional>
#include <map>
#include <utility>
#include <vector>

namespace ompl
{
    namespace control
    {
        class ModSyclop : public base::Planner
        {
        public:
            using EdgeCostFactorFn = std::function<double(int, int)>;

            using LeadComputeFn = std::function<void(int, int, std::vector<int> &)>;

            ModSyclop(const SpaceInformationPtr &si, DecompositionPtr d, const std::string &plannerName)
                : ompl::base::Planner(si, plannerName), siC_(si.get()), decomp_(std::move(d)), covGrid_(Defaults::COVGRID_LENGTH, decomp_)
            {
                specs_.approximateSolutions = true;

                Planner::declareParam<int>("free_volume_samples", this, &ModSyclop::setNumFreeVolumeSamples,
                                           &ModSyclop::getNumFreeVolumeSamples, "10000:10000:500000");
                Planner::declareParam<int>("num_region_expansions", this, &ModSyclop::setNumRegionExpansions,
                                           &ModSyclop::getNumRegionExpansions, "10:10:500");
                Planner::declareParam<int>("num_tree_expansions", this, &ModSyclop::setNumTreeExpansions, &ModSyclop::getNumTreeExpansions,
                                           "0:1:100");
                Planner::declareParam<double>("prob_abandon_lead_early", this, &ModSyclop::setProbAbandonLeadEarly,
                                              &ModSyclop::getProbAbandonLeadEarly, "0.:.05:1.");
                Planner::declareParam<double>("prob_add_available_regions", this, &ModSyclop::setProbAddingToAvailableRegions,
                                              &ModSyclop::getProbAddingToAvailableRegions, "0.:.05:1.");
                Planner::declareParam<double>("prob_shortest_path_lead", this, &ModSyclop::setProbShortestPathLead,
                                              &ModSyclop::getProbShortestPathLead, "0.:.05:1.");
            }

            ~ModSyclop() override = default;

            void setup() override;

            void clear() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void setLeadComputeFn(const LeadComputeFn &compute);

            void addEdgeCostFactor(const EdgeCostFactorFn &factor);

            void clearEdgeCostFactors();

            int getNumFreeVolumeSamples() const
            {
                return numFreeVolSamples_;
            }

            void setNumFreeVolumeSamples(int numSamples)
            {
                numFreeVolSamples_ = numSamples;
            }

            double getProbShortestPathLead() const
            {
                return probShortestPath_;
            }

            void setProbShortestPathLead(double probability)
            {
                probShortestPath_ = probability;
            }

            double getProbAddingToAvailableRegions() const
            {
                return probKeepAddingToAvail_;
            }

            void setProbAddingToAvailableRegions(double probability)
            {
                probKeepAddingToAvail_ = probability;
            }

            int getNumRegionExpansions() const
            {
                return numRegionExpansions_;
            }

            void setNumRegionExpansions(int regionExpansions)
            {
                numRegionExpansions_ = regionExpansions;
            }

            int getNumTreeExpansions() const
            {
                return numTreeSelections_;
            }

            void setNumTreeExpansions(int treeExpansions)
            {
                numTreeSelections_ = treeExpansions;
            }

            double getProbAbandonLeadEarly() const
            {
                return probAbandonLeadEarly_;
            }

            void setProbAbandonLeadEarly(double probability)
            {
                probAbandonLeadEarly_ = probability;
            }

            struct Defaults
            {
                static const int NUM_FREEVOL_SAMPLES   = 100000;
                static const int COVGRID_LENGTH        = 128;
                static const int NUM_REGION_EXPANSIONS = 100;
                static const int NUM_TREE_SELECTIONS   = 1;
                // C++ standard prohibits non-integral static const member initialization
                // These constants are set in Syclop.cpp.  C++11 standard changes this
                // with the constexpr keyword, but for compatibility this is not done.
                static const double PROB_ABANDON_LEAD_EARLY /*= 0.25*/;
                static const double PROB_KEEP_ADDING_TO_AVAIL /*= 0.50*/;
                static const double PROB_SHORTEST_PATH /*= 0.95*/;
            };

        protected:
            class Motion
            {
            public:
                Motion() = default;

                Motion(const SpaceInformation *si) : state(si->allocState()), control(si->allocControl()) {}
                virtual ~Motion() = default;
                base::State *state{nullptr};
                Control *control{nullptr};
                const Motion *parent{nullptr};
                unsigned int steps{0};
            };

            class Region
            {
            public:
                Region()          = default;
                virtual ~Region() = default;

                Region(const Region &)            = default;
                Region &operator=(const Region &) = default;
                Region(Region &&)                 = default;
                Region &operator=(Region &&)      = default;

                void clear()
                {
                    motions.clear();
                    covGridCells.clear();
                    pdfElem = nullptr;
                }

                std::set<int> covGridCells;
                std::vector<Motion *> motions;
                double volume;
                double freeVolume;
                double percentValidCells;
                double weight;
                double alpha;
                int index;
                unsigned int numSelections;
                PDF<int>::Element *pdfElem;
            };

            class Adjacency
            {
            public:
                Adjacency()          = default;
                virtual ~Adjacency() = default;
                void clear()
                {
                    covGridCells.clear();
                }
                std::set<int> covGridCells;
                const Region *source;
                const Region *target;
                double cost;
                int numLeadInclusions;
                int numSelections;
                bool empty;
            };

            virtual Motion *addRoot(const base::State *s) = 0;

            virtual void selectAndExtend(Region &region, std::vector<Motion *> &newMotions) = 0;

            inline const Region &getRegionFromIndex(const int rid) const
            {
                return graph_[boost::vertex(rid, graph_)];
            }

            int numFreeVolSamples_{Defaults::NUM_FREEVOL_SAMPLES};

            double probShortestPath_{Defaults::PROB_SHORTEST_PATH};

            double probKeepAddingToAvail_{Defaults::PROB_KEEP_ADDING_TO_AVAIL};

            int numRegionExpansions_{Defaults::NUM_REGION_EXPANSIONS};

            int numTreeSelections_{Defaults::NUM_TREE_SELECTIONS};

            double probAbandonLeadEarly_{Defaults::PROB_ABANDON_LEAD_EARLY};

            const SpaceInformation *siC_;

            DecompositionPtr decomp_;

            RNG rng_;

        private:
            struct HashRegionPair
            {
                size_t operator()(const std::pair<int, int> &p) const
                {
                    std::size_t hash = std::hash<int>()(p.first);
                    hash_combine(hash, p.second);
                    return hash;
                }
            };

            class CoverageGrid : public GridDecomposition
            {
            public:
                CoverageGrid(const int len, const DecompositionPtr &d)
                    : GridDecomposition(len, d->getDimension(), d->getBounds()), decomp(d)
                {}

                ~CoverageGrid() override = default;

                void project(const base::State *s, std::vector<double> &coord) const override
                {
                    decomp->project(s, coord);
                }

                void sampleFullState(const base::StateSamplerPtr & /*sampler*/, const std::vector<double> & /*coord*/,
                                     base::State * /*s*/) const override
                {}

            protected:
                const DecompositionPtr &decomp;
            };

            using RegionGraph    = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Region, Adjacency>;
            using Vertex         = boost::graph_traits<RegionGraph>::vertex_descriptor;
            using VertexIter     = boost::graph_traits<RegionGraph>::vertex_iterator;
            using VertexIndexMap = boost::property_map<RegionGraph, boost::vertex_index_t>::type;
            using EdgeIter       = boost::graph_traits<RegionGraph>::edge_iterator;

            friend class DecompositionHeuristic;

            class DecompositionHeuristic : public boost::astar_heuristic<RegionGraph, double>
            {
            public:
                DecompositionHeuristic(const ModSyclop *s, const Region &goal) : syclop(s), goalRegion(goal) {}

                double operator()(Vertex v)
                {
                    const Region &region = syclop->getRegionFromIndex(v);
                    return region.alpha * goalRegion.alpha;
                }

            private:
                const ModSyclop *syclop;
                const Region &goalRegion;
            };

            struct found_goal
            {};

            class GoalVisitor : public boost::default_astar_visitor
            {
            public:
                GoalVisitor(const int goal) : goalRegion(goal) {}
                void examine_vertex(Vertex v, const RegionGraph & /*g*/)
                {
                    if(static_cast<int>(v) == goalRegion) throw found_goal();
                }

            private:
                const int goalRegion;
            };

            class RegionSet
            {
            public:
                int sampleUniform()
                {
                    if(empty()) return -1;
                    return regions.sample(rng.uniform01());
                }
                void insert(const int r)
                {
                    if(regToElem.count(r) == 0)
                        regToElem[r] = regions.add(r, 1);
                    else
                        {
                            PDF<int>::Element *elem = regToElem[r];
                            regions.update(elem, regions.getWeight(elem) + 1);
                        }
                }
                void clear()
                {
                    regions.clear();
                    regToElem.clear();
                }
                std::size_t size() const
                {
                    return regions.size();
                }
                bool empty() const
                {
                    return regions.empty();
                }

            private:
                RNG rng;
                PDF<int> regions;
                std::unordered_map<int, PDF<int>::Element *> regToElem;
            };

            void initRegion(Region &r);

            void setupRegionEstimates();

            void updateRegion(Region &r);

            void initEdge(Adjacency &adj, const Region *source, const Region *target);

            void setupEdgeEstimates();

            void updateEdge(Adjacency &a);

            bool updateCoverageEstimate(Region &r, const base::State *s);

            bool updateConnectionEstimate(const Region &c, const Region &d, const base::State *s);

            void buildGraph();

            void clearGraphDetails();

            int selectRegion();

            void computeAvailableRegions();

            void defaultComputeLead(int startRegion, int goalRegion, std::vector<int> &lead);

            double defaultEdgeCost(int r, int s);

            LeadComputeFn leadComputeFn;
            std::vector<int> lead_;
            PDF<int> availDist_;
            std::vector<EdgeCostFactorFn> edgeCostFactors_;
            CoverageGrid covGrid_;
            RegionGraph graph_;
            bool graphReady_{false};
            std::unordered_map<std::pair<int, int>, Adjacency *, HashRegionPair> regionsToEdge_;
            unsigned int numMotions_{0};
            RegionSet startRegions_;
            RegionSet goalRegions_;
        };
    }  // namespace control
}  // namespace ompl

#endif