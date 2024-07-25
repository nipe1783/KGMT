#include "cpu/planners/OMPL_Planner.h"

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

class CustomProjection : public ob::ProjectionEvaluator
{
public:
    CustomProjection(const ob::StateSpace* space) : ob::ProjectionEvaluator(space)
    {
    }

    virtual unsigned int getDimension() const override
    {
        return 2;
    }

    virtual void defaultCellSizes() override
    {
        cellSizes_.resize(2);
        cellSizes_[0] = 0.1;
        cellSizes_[1] = 0.1;
    }

    virtual void project(const ob::State* state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const auto* compoundState = state->as<ob::CompoundState>();
        const auto* positionState = compoundState->as<ob::RealVectorStateSpace::StateType>(0);
        projection(0) = positionState->values[0];
        projection(1) = positionState->values[1];
    }
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
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    qdot.resize(q.size());
    if (MODEL == 1) {
        // q: [x, y, z, vx, vy, vz]
        // control: [ax, ay, az]
        qdot[0] = q[3];
        qdot[1] = q[4];
        qdot[2] = q[5];
        qdot[3] = u[0];
        qdot[4] = u[1];
        qdot[5] = u[2];
    } else if (MODEL == 2) {   
        // q: [x, y, z, yaw, pitch, v]
        // control: [yawRate, pitchRate, acc]
        qdot[0] = q[5] * cos(q[4]) * cos(q[3]);
        qdot[1] = q[5] * cos(q[4]) * sin(q[3]);
        qdot[2] = q[5] * sin(q[4]);
        qdot[3] = u[0]; // yawRate
        qdot[4] = u[1]; // pitchRate
        qdot[5] = u[2]; // acceleration
    }
}

ob::StateSpacePtr OMPL_Planner::createStateSpace() {

    ob::StateSpacePtr space = std::make_shared<ob::CompoundStateSpace>();

    if (MODEL == 1) {
        // Double integrator model
        auto positionSpace = std::make_shared<ob::RealVectorStateSpace>(3);
        auto velocitySpace = std::make_shared<ob::RealVectorStateSpace>(3);
        space->as<ob::CompoundStateSpace>()->addSubspace(positionSpace, 1.0);
        space->as<ob::CompoundStateSpace>()->addSubspace(velocitySpace, 1.0);

        // Set the bounds
        ob::RealVectorBounds posBounds(3);
        posBounds.setLow(0, 0.0);   
        posBounds.setHigh(0, WS_SIZE);  
        posBounds.setLow(1, 0.0);   
        posBounds.setHigh(1, WS_SIZE);  
        posBounds.setLow(2, 0.0);   
        posBounds.setHigh(2, WS_SIZE);  
        positionSpace->setBounds(posBounds);

        ob::RealVectorBounds velBounds(3);
        velBounds.setLow(DI_MIN_VEL);
        velBounds.setHigh(DI_MAX_VEL);
        velocitySpace->setBounds(velBounds);
    } else if (MODEL == 2) {
        // Dubins airplane model
        auto positionSpace = std::make_shared<ob::RealVectorStateSpace>(3);
        auto orientationSpace = std::make_shared<ob::RealVectorStateSpace>(2);
        auto velocitySpace = std::make_shared<ob::RealVectorStateSpace>(1);
        space->as<ob::CompoundStateSpace>()->addSubspace(positionSpace, 1.0);
        space->as<ob::CompoundStateSpace>()->addSubspace(orientationSpace, 1.0);
        space->as<ob::CompoundStateSpace>()->addSubspace(velocitySpace, 1.0);

        // Set the bounds for the state space
        ob::RealVectorBounds posBounds(3);
        posBounds.setLow(0, 0.0);
        posBounds.setHigh(0, WS_SIZE);
        posBounds.setLow(1, 0.0);
        posBounds.setHigh(1, WS_SIZE);
        posBounds.setLow(2, 0.0);
        posBounds.setHigh(2, WS_SIZE);
        positionSpace->setBounds(posBounds);

        ob::RealVectorBounds orientBounds(2);
        orientBounds.setLow(0, DUBINS_AIRPLANE_MIN_YAW);
        orientBounds.setHigh(0, DUBINS_AIRPLANE_MAX_YAW);
        orientBounds.setLow(1, DUBINS_AIRPLANE_MIN_PITCH);
        orientBounds.setHigh(1, DUBINS_AIRPLANE_MAX_YAW);
        orientationSpace->setBounds(orientBounds);

        ob::RealVectorBounds velBounds(1);
        velBounds.setLow(DUBINS_AIRPLANE_MIN_VEL);
        velBounds.setHigh(DUBINS_AIRPLANE_MAX_VEL);
        velocitySpace->setBounds(velBounds);
    }

    space->as<ob::CompoundStateSpace>()->lock();
    return space;
}

oc::ControlSpacePtr OMPL_Planner::createControlSpace(ob::StateSpacePtr &space) {
    auto cspace = std::make_shared<oc::RealVectorControlSpace>(space, 3);
    ob::RealVectorBounds cbounds(3);

    if (MODEL == 1) {
        cbounds.setLow(0, DI_MIN_ACC);
        cbounds.setHigh(0, DI_MAX_ACC);
        cbounds.setLow(1, DI_MIN_ACC);
        cbounds.setHigh(1, DI_MAX_ACC);
        cbounds.setLow(2, DI_MIN_ACC);
        cbounds.setHigh(2, DI_MAX_ACC);
    } else if (MODEL == 2) {
        cbounds.setLow(0, DUBINS_AIRPLANE_MIN_YR);
        cbounds.setHigh(0, DUBINS_AIRPLANE_MAX_YR);
        cbounds.setLow(1, DUBINS_AIRPLANE_MIN_PR);
        cbounds.setHigh(1, DUBINS_AIRPLANE_MAX_PR);
        cbounds.setLow(2, DUBINS_AIRPLANE_MIN_ACC);
        cbounds.setHigh(2, DUBINS_AIRPLANE_MAX_ACC);
    }

    cspace->setBounds(cbounds);
    return cspace;
}

oc::SimpleSetupPtr OMPL_Planner::kinodynamicSimpleSetUp(const float* initial, const float* goal)
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
    space->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new CustomProjection(space.get())));

    // set starting state:
    ob::ScopedState<> start(space);
    start[0] = initial[0]; // x
    start[1] = initial[1]; // y
    start[2] = initial[2]; // z

    if (MODEL == 1) {
        start[3] = initial[3]; // vx
        start[4] = initial[4]; // vy
        start[5] = initial[5]; // vz
    } else if (MODEL == 2) {
        start[3] = initial[3]; // yaw
        start[4] = initial[4]; // pitch
        start[5] = initial[5]; // velocity
    }

    // set goal region:
    OMPL_INFORM("goal x: %f, y: %f, z: %f", goal[0], goal[1], goal[2]);
    ob::GoalPtr goalRegion(new GoalRegionWithinSphere(ss->getSpaceInformation(), {goal[0], goal[1], goal[2]}, GOAL_THRESH));

    // set the start and goal states
    ss->setStartState(start);
    ss->setGoal(goalRegion);

    OMPL_INFORM("Successfully Setup the problem instance");
    return ss;
}

void OMPL_Planner::planRRT(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin) {
    
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

void OMPL_Planner::planPDST(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin) {
    
    safetyMargin_ = safetyMargin;
    obstacles_ = obstacles;
    obstaclesCount_ = numObstacles;
    OMPL_INFORM("numObstacles: %d", obstaclesCount_);

    // create simple setup object
    oc::SimpleSetupPtr ss = kinodynamicSimpleSetUp(initial, goal);
    oc::PathControl pathOmpl(ss->getSpaceInformation()); // create empty path

    // set planner
    auto planner = std::make_shared<oc::PDST>(ss->getSpaceInformation());
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

void OMPL_Planner::planEST(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin) {
    
    safetyMargin_ = safetyMargin;
    obstacles_ = obstacles;
    obstaclesCount_ = numObstacles;
    OMPL_INFORM("numObstacles: %d", obstaclesCount_);

    // create simple setup object
    oc::SimpleSetupPtr ss = kinodynamicSimpleSetUp(initial, goal);
    oc::PathControl pathOmpl(ss->getSpaceInformation()); // create empty path

    // set planner
    auto planner = std::make_shared<oc::EST>(ss->getSpaceInformation());
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


OMPL_Planner::OMPL_Planner() : boundsPos_(DIM/2), boundsVel_(DIM/2){};
OMPL_Planner::~OMPL_Planner(){};
