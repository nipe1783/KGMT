#include "cpu/planners/RRT_6DI.h"

class GoalRegionWithinSphere : public ob::GoalRegion {
public:
    GoalRegionWithinSphere(const ob::SpaceInformationPtr &si, const std::vector<double> &goal, double radius)
        : ob::GoalRegion(si), goal_(goal), radius_(radius) {
        threshold_ = radius_;
    }

    virtual bool isSatisfied(const ob::State *st) const override {
        const ob::CompoundState *state = st->as<ob::CompoundState>();
        const ob::RealVectorStateSpace::StateType *pos = state->as<ob::RealVectorStateSpace::StateType>(0);

        double dist = 0.0;
        for (size_t i = 0; i < goal_.size(); ++i) {
            double diff = pos->values[i] - goal_[i];
            dist += diff * diff;
        }
        dist = std::sqrt(dist);

        return dist <= radius_;
    }

    virtual double distanceGoal(const ob::State *st) const override {
        float x = st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        float y = st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
        float z = st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[2];
        return sqrt(pow(x - goal_[0], 2) + pow(y - goal_[1], 2) + pow(z - goal_[2], 2));
    }

private:
    std::vector<double> goal_;
    double radius_;
};

void write2sys(const oc::SimpleSetupPtr problem)
{
    fs::path sol_dir = "solutions/";
    fs::create_directories(sol_dir);

    std::string fileName = "solutions/Output.csv";
    auto filePath = fs::current_path() / fs::path(fileName);
    std::ofstream file(filePath);
    const oc::PathControl p = problem->getSolutionPath();
    p.printAsMatrix(file);
}

void doubleIntegratorODE(const oc::ODESolver::StateType &q, const oc::Control *control, oc::ODESolver::StateType &qdot)
{
    // q: [x, y, z, vx, vy, vz]
    // control: [ax, ay, az]

    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    qdot.resize(q.size());
    // Position derivatives
    qdot[0] = q[3];
    qdot[1] = q[4];
    qdot[2] = q[5];
    // Velocity derivatives
    qdot[3] = u[0];
    qdot[4] = u[1];
    qdot[5] = u[2];
}


ob::StateSpacePtr RRT_6DI::createStateSpace() {

    // Create a compound state space
    ob::StateSpacePtr space = std::make_shared<ob::CompoundStateSpace>();

    // Add a 3-dimensional RealVectorStateSpace for position (x, y, z)
    auto positionSpace = std::make_shared<ob::RealVectorStateSpace>(3);

    // Add a 3-dimensional RealVectorStateSpace for velocity (vx, vy, vz)
    auto velocitySpace = std::make_shared<ob::RealVectorStateSpace>(3);

    // Combine the spaces into a compound state space
    space->as<ob::CompoundStateSpace>()->addSubspace(positionSpace, 1.0);
    space->as<ob::CompoundStateSpace>()->addSubspace(velocitySpace, 1.0);
    space->as<ob::CompoundStateSpace>()->lock();

    // Set the bounds for the position space
    ob::RealVectorBounds posBounds(3);
    posBounds.setLow(0, 0.0);   // x lower bound
    posBounds.setHigh(0, WS_SIZE);  // x upper bound
    posBounds.setLow(1, 0.0);   // y lower bound
    posBounds.setHigh(1, WS_SIZE);  // y upper bound
    posBounds.setLow(2, 0.0);   // z lower bound
    posBounds.setHigh(2, WS_SIZE);  // z upper bound
    positionSpace->setBounds(posBounds);

    // Set the bounds for the velocity space
    ob::RealVectorBounds velBounds(3);
    velBounds.setLow(DI_MIN_VEL);
    velBounds.setHigh(DI_MAX_VEL);
    velocitySpace->setBounds(velBounds);

    return space;
}

oc::ControlSpacePtr RRT_6DI::createControlSpace(ob::StateSpacePtr &space) {
    // Create a 3D control space
   auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 3));
   
   // set the bounds for the control space
    ob::RealVectorBounds cbounds(3);
    cbounds.setLow(DI_MIN_ACC);
    cbounds.setHigh(DI_MAX_ACC);
    cspace->setBounds(cbounds);
    return cspace;
}

oc::SimpleSetupPtr RRT_6DI::kinodynamicSimpleSetUp(const float* initial, const float* goal)
{
    ob::StateSpacePtr space = createStateSpace();
    oc::ControlSpacePtr cspace = createControlSpace(space);
    auto ss = std::make_shared<oc::SimpleSetup>(cspace);
    ss->setStateValidityChecker(ob::StateValidityCheckerPtr(new CollisionCheck(ss->getSpaceInformation(), obstacles_, obstaclesCount_, safetyMargin_)));

    auto odeFunction = std::bind(&doubleIntegratorODE, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), odeFunction);


    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));
    ss->getSpaceInformation()->setPropagationStepSize(STEP_SIZE);
    ss->getSpaceInformation()->setMinMaxControlDuration(1, MAX_PROPAGATION_DURATION);

    // set starting state:
    ob::ScopedState<> start(space);
    start[0] = initial[0]; // x
    start[1] = initial[1]; // y
    start[2] = initial[2]; // z
    start[3] = initial[3]; // vx
    start[4] = initial[4]; // vy
    start[5] = initial[5]; // vz

    // set goal region:
    OMPL_INFORM("goal x: %f, y: %f, z: %f", goal[0], goal[1], goal[2]);
    ob::GoalPtr goalRegion (new GoalRegionWithinSphere(ss->getSpaceInformation(), {goal[0], goal[1], goal[2]}, GOAL_THRESH));

    // set the start and goal states
    ss->setStartState(start);
    ss->setGoal(goalRegion);

    OMPL_INFORM("Successfully Setup the problem instance");
    return ss;
}

void RRT_6DI::plan(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin) {
    
    safetyMargin_ = safetyMargin;
    obstacles_ = obstacles;
    obstaclesCount_ = numObstacles;
    OMPL_INFORM("numObstacles: %d", obstaclesCount_);

    // create simple setup object
    oc::SimpleSetupPtr ss = kinodynamicSimpleSetUp(initial, goal);
    oc::PathControl pathOmpl(ss->getSpaceInformation()); // create empty path

    // set planner
    auto planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
    ss->setPlanner(planner);
    
    // run automated setup routine
    ss->getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    ss->setup();
    ss->print();
    
    ob::PlannerStatus solved = ss->solve(30.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        pathOmpl = ss->getSolutionPath();
        write2sys(ss);
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}

RRT_6DI::RRT_6DI() : boundsPos_(DIM/2), boundsVel_(DIM/2){};
RRT_6DI::~RRT_6DI(){};
