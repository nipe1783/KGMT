#include "cpu/planners/OMPL_Planner.h"

void setNumberOfThreads(unsigned int numThreads)
{
    omp_set_num_threads(numThreads);
    OMPL_INFORM("Using %u threads for planning.", numThreads);
}

template <typename T>
inline void writeValueToCSV(const T& value, const std::string& filename)
{
    std::ofstream file;
    file.open(filename, std::ios_base::app);

    if constexpr(std::is_floating_point_v<std::decay_t<decltype(value)>>)
        {
            file << std::fixed << std::setprecision(10);
        }

    file << value << std::endl;
    file.close();
}

void writeExecutionTimeToCSV(double time)
{
    std::ostringstream filename;
    std::filesystem::create_directories("Data");
    std::filesystem::create_directories("Data/ExecutionTime");
    filename.str("");
    filename << "Data/ExecutionTime/executionTime.csv";
    writeValueToCSV(time, filename.str());
}

void writeIterationsToCSV(int iterations)
{
    std::ostringstream filename;
    std::filesystem::create_directories("Data");
    std::filesystem::create_directories("Data/Iterations");
    filename.str("");
    filename << "Data/Iterations/iterations.csv";
    writeValueToCSV(iterations, filename.str());
}

void writeNumVerticesToCSV(int numVertices)
{
    std::ostringstream filename;
    std::filesystem::create_directories("Data");
    std::filesystem::create_directories("Data/Vertices");
    filename.str("");
    filename << "Data/Vertices/vertices.csv";
    writeValueToCSV(numVertices, filename.str());
}

class GoalRegionWithinSphere : public ob::GoalRegion
{
public:
    GoalRegionWithinSphere(const ob::SpaceInformationPtr& si, const std::vector<double>& goal, double radius)
        : ob::GoalRegion(si), goal_(goal), radius_(radius)
    {
        threshold_ = radius_;
    }

    virtual bool isSatisfied(const ob::State* st) const override
    {
        const ob::CompoundState* state                 = st->as<ob::CompoundState>();
        const ob::RealVectorStateSpace::StateType* pos = state->as<ob::RealVectorStateSpace::StateType>(0);

        double dist = 0.0;
        for(size_t i = 0; i < goal_.size(); ++i)
            {
                double diff = pos->values[i] - goal_[i];
                dist += diff * diff;
            }
        dist = std::sqrt(dist);

        return dist <= radius_;
    }

    virtual double distanceGoal(const ob::State* st) const override
    {
        float x = st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        float y = st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
        float z = st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[2];
        return sqrt(pow(x - goal_[0], 2) + pow(y - goal_[1], 2) + pow(z - goal_[2], 2));
    }

private:
    std::vector<double> goal_;
    double radius_;
};

/***************************/
/* SPACE DECOMPOSITION */
/***************************/
// --- Uses the same space decomposition as KGMT. ---
class CustomProjection : public ob::ProjectionEvaluator
{
public:
    CustomProjection(const ob::StateSpace* space) : ob::ProjectionEvaluator(space) {}

    virtual unsigned int getDimension() const override
    {
        if(MODEL == 1)
            {
                return W_DIM + V_DIM;
            }
        else if(MODEL == 2)
            {
                return W_DIM + C_DIM;
            }
        else if(MODEL == 3)
            {
                return W_DIM + C_DIM + V_DIM;
            }
    }

    virtual void defaultCellSizes() override
    {
        if(MODEL == 1)
            {
                cellSizes_.resize(W_DIM + V_DIM);
                cellSizes_[0] = (W_MAX - W_MIN) / W_R1_LENGTH;
                cellSizes_[1] = (W_MAX - W_MIN) / W_R1_LENGTH;
                cellSizes_[2] = (W_MAX - W_MIN) / W_R1_LENGTH;
                cellSizes_[3] = (V_MAX - V_MIN) / V_R1_LENGTH;
                cellSizes_[4] = (V_MAX - V_MIN) / V_R1_LENGTH;
                cellSizes_[5] = (V_MAX - V_MIN) / V_R1_LENGTH;
            }
        else if(MODEL == 2)
            {
                cellSizes_.resize(W_DIM + C_DIM);
                cellSizes_[0] = (W_MAX - W_MIN) / W_R1_LENGTH;
                cellSizes_[1] = (W_MAX - W_MIN) / W_R1_LENGTH;
                cellSizes_[2] = (W_MAX - W_MIN) / W_R1_LENGTH;
                cellSizes_[3] = (C_MAX - C_MIN) / C_R1_LENGTH;
                cellSizes_[4] = (C_MAX - C_MIN) / C_R1_LENGTH;
            }
        else if(MODEL == 3)
            {
                cellSizes_.resize(W_DIM + C_DIM + V_DIM);
                cellSizes_[0] = (W_MAX - W_MIN) / W_R1_LENGTH;
                cellSizes_[1] = (W_MAX - W_MIN) / W_R1_LENGTH;
                cellSizes_[2] = (W_MAX - W_MIN) / W_R1_LENGTH;
                cellSizes_[3] = (C_MAX - C_MIN) / C_R1_LENGTH;
                cellSizes_[4] = (C_MAX - C_MIN) / C_R1_LENGTH;
                cellSizes_[5] = (C_MAX - C_MIN) / C_R1_LENGTH;
                cellSizes_[6] = (V_MAX - V_MIN) / V_R1_LENGTH;
                cellSizes_[7] = (V_MAX - V_MIN) / V_R1_LENGTH;
                cellSizes_[8] = (V_MAX - V_MIN) / V_R1_LENGTH;
            }
    }

    virtual void project(const ob::State* state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const auto* compoundState = state->as<ob::CompoundState>();
        const auto* positionState = compoundState->as<ob::RealVectorStateSpace::StateType>(0);

        if(MODEL == 1)
            {
                const auto* velocityState = compoundState->as<ob::RealVectorStateSpace::StateType>(1);
                projection(0)             = positionState->values[0];
                projection(1)             = positionState->values[1];
                projection(2)             = positionState->values[2];
                projection(3)             = velocityState->values[0];
                projection(4)             = velocityState->values[1];
                projection(5)             = velocityState->values[2];
            }
        else if(MODEL == 2)
            {
                const auto* configurationState = compoundState->as<ob::RealVectorStateSpace::StateType>(1);
                projection(0)                  = positionState->values[0];
                projection(1)                  = positionState->values[1];
                projection(2)                  = positionState->values[2];
                projection(3)                  = configurationState->values[0];
                projection(4)                  = configurationState->values[1];
            }
        else if(MODEL == 3)
            {
                const auto* configurationState = compoundState->as<ob::RealVectorStateSpace::StateType>(1);
                const auto* velocityState      = compoundState->as<ob::RealVectorStateSpace::StateType>(2);
                projection(0)                  = positionState->values[0];
                projection(1)                  = positionState->values[1];
                projection(2)                  = positionState->values[2];
                projection(3)                  = configurationState->values[0];
                projection(4)                  = configurationState->values[1];
                projection(5)                  = configurationState->values[2];
                projection(6)                  = velocityState->values[0];
                projection(7)                  = velocityState->values[1];
                projection(8)                  = velocityState->values[2];
            }
    }
};

void write2sys(const oc::SimpleSetupPtr problem)
{
    fs::path sol_dir = "solutions/";
    fs::create_directories(sol_dir);

    std::string fileName = "solutions/Output.csv";
    auto filePath        = fs::current_path() / fs::path(fileName);
    std::ofstream file(filePath);
    const oc::PathControl p = problem->getSolutionPath();
    p.printAsMatrix(file);
}

void doubleIntegratorODE(const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    const double* u = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    qdot.resize(q.size());
    if(MODEL == 1)
        {
            // q: [x, y, z, vx, vy, vz]
            // control: [ax, ay, az]
            qdot[0] = q[3];
            qdot[1] = q[4];
            qdot[2] = q[5];
            qdot[3] = u[0];
            qdot[4] = u[1];
            qdot[5] = u[2];
        }
    else if(MODEL == 2)
        {
            // q: [x, y, z, yaw, pitch, v]
            // control: [yawRate, pitchRate, acc]
            qdot[0] = q[5] * cos(q[4]) * cos(q[3]);
            qdot[1] = q[5] * cos(q[4]) * sin(q[3]);
            qdot[2] = q[5] * sin(q[4]);
            qdot[3] = u[0];  // yawRate
            qdot[4] = u[1];  // pitchRate
            qdot[5] = u[2];  // acceleration
        }
    else if(MODEL == 3)
        {
            // q: [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]
            // control: [Zc, Lc, Mc, Nc]
            double phi   = q[3];
            double theta = q[4];
            double psi   = q[5];
            double u_val = q[6];
            double v     = q[7];
            double w     = q[8];
            double p     = q[9];
            double q_val = q[10];
            double r     = q[11];
            qdot[0]      = cos(theta) * cos(psi) * u_val + (sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi)) * v +
                      (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) * w;
            qdot[1] = cos(theta) * sin(psi) * u_val + (sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi)) * v +
                      (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) * w;
            qdot[2]   = -sin(theta) * u_val + sin(phi) * cos(theta) * v + cos(phi) * cos(theta) * w;
            qdot[3]   = p + (q_val * sin(phi) + r * cos(phi)) * tan(theta);
            qdot[4]   = q_val * cos(phi) - r * sin(phi);
            qdot[5]   = (q_val * sin(phi) + r * cos(phi)) / cos(theta);
            float XYZ = -NU * sqrt(u_val * u_val + v * v + w * w);
            qdot[6]   = (r * v - q_val * w) - GRAVITY * sin(theta) + MASS_INV * XYZ * u_val;
            qdot[7]   = (p * w - r * u_val) + GRAVITY * cos(theta) * sin(phi) + MASS_INV * XYZ * v;
            qdot[8]   = (q_val * u_val - p * v) + GRAVITY * cos(theta) * cos(phi) + MASS_INV * XYZ * w + MASS_INV * u[0];

            float LMN = -MU * sqrt(p * p + q_val * q_val + r * r);
            qdot[9]   = (IY - IZ) / IX * q_val * r + (1 / IX) * LMN * p + (1 / IX) * u[1];
            qdot[10]  = (IZ - IX) / IY * p * r + (1 / IY) * LMN * q_val + (1 / IY) * u[2];
            qdot[11]  = (IX - IY) / IZ * p * q_val + (1 / IZ) * LMN * r + (1 / IZ) * u[3];
        }
}

ob::StateSpacePtr OMPL_Planner::createStateSpace()
{
    ob::StateSpacePtr space = std::make_shared<ob::CompoundStateSpace>();

    if(MODEL == 1)
        {
            // --- Double Integrator ---
            auto positionSpace = std::make_shared<ob::RealVectorStateSpace>(3);
            auto velocitySpace = std::make_shared<ob::RealVectorStateSpace>(3);
            space->as<ob::CompoundStateSpace>()->addSubspace(positionSpace, 1.0);
            space->as<ob::CompoundStateSpace>()->addSubspace(velocitySpace, 1.0);

            ob::RealVectorBounds posBounds(3);
            posBounds.setLow(0, 0.0);
            posBounds.setHigh(0, W_SIZE);
            posBounds.setLow(1, 0.0);
            posBounds.setHigh(1, W_SIZE);
            posBounds.setLow(2, 0.0);
            posBounds.setHigh(2, W_SIZE);
            positionSpace->setBounds(posBounds);

            ob::RealVectorBounds velBounds(3);
            velBounds.setLow(V_MIN);
            velBounds.setHigh(V_MAX);
            velocitySpace->setBounds(velBounds);
        }
    else if(MODEL == 2)
        {
            // --- Dubins Airplane ---
            auto positionSpace    = std::make_shared<ob::RealVectorStateSpace>(3);
            auto orientationSpace = std::make_shared<ob::RealVectorStateSpace>(2);
            auto velocitySpace    = std::make_shared<ob::RealVectorStateSpace>(1);
            space->as<ob::CompoundStateSpace>()->addSubspace(positionSpace, 1.0);
            space->as<ob::CompoundStateSpace>()->addSubspace(orientationSpace, 1.0);
            space->as<ob::CompoundStateSpace>()->addSubspace(velocitySpace, 1.0);

            ob::RealVectorBounds posBounds(3);
            posBounds.setLow(0, 0.0);
            posBounds.setHigh(0, W_SIZE);
            posBounds.setLow(1, 0.0);
            posBounds.setHigh(1, W_SIZE);
            posBounds.setLow(2, 0.0);
            posBounds.setHigh(2, W_SIZE);
            positionSpace->setBounds(posBounds);

            ob::RealVectorBounds orientBounds(2);
            orientBounds.setLow(0, DUBINS_AIRPLANE_MIN_YAW);
            orientBounds.setHigh(0, DUBINS_AIRPLANE_MAX_YAW);
            orientBounds.setLow(1, DUBINS_AIRPLANE_MIN_PITCH);
            orientBounds.setHigh(1, DUBINS_AIRPLANE_MAX_YAW);
            orientationSpace->setBounds(orientBounds);

            ob::RealVectorBounds velBounds(1);
            velBounds.setLow(V_MIN);
            velBounds.setHigh(V_MAX);
            velocitySpace->setBounds(velBounds);
        }
    else if(MODEL == 3)
        {
            // --- 12D Quad ---
            auto positionSpace    = std::make_shared<ob::RealVectorStateSpace>(3);
            auto orientationSpace = std::make_shared<ob::RealVectorStateSpace>(3);
            auto velocitySpace    = std::make_shared<ob::RealVectorStateSpace>(3);
            auto angularVelSpace  = std::make_shared<ob::RealVectorStateSpace>(3);
            space->as<ob::CompoundStateSpace>()->addSubspace(positionSpace, 1.0);
            space->as<ob::CompoundStateSpace>()->addSubspace(orientationSpace, 1.0);
            space->as<ob::CompoundStateSpace>()->addSubspace(velocitySpace, 1.0);
            space->as<ob::CompoundStateSpace>()->addSubspace(angularVelSpace, 1.0);

            ob::RealVectorBounds posBounds(3);
            posBounds.setLow(0, 0.0);
            posBounds.setHigh(0, W_SIZE);
            posBounds.setLow(1, 0.0);
            posBounds.setHigh(1, W_SIZE);
            posBounds.setLow(2, 0.0);
            posBounds.setHigh(2, W_SIZE);
            positionSpace->setBounds(posBounds);

            ob::RealVectorBounds orientBounds(3);
            orientBounds.setLow(0, QUAD_MIN_YAW);
            orientBounds.setHigh(0, QUAD_MAX_YAW);
            orientBounds.setLow(1, QUAD_MIN_PITCH);
            orientBounds.setHigh(1, QUAD_MAX_PITCH);
            orientBounds.setLow(2, QUAD_MIN_ROLL);
            orientBounds.setHigh(2, QUAD_MAX_ROLL);
            orientationSpace->setBounds(orientBounds);

            ob::RealVectorBounds velBounds(3);
            velBounds.setLow(V_MIN);
            velBounds.setHigh(V_MAX);
            velocitySpace->setBounds(velBounds);

            ob::RealVectorBounds angVelBounds(3);
            angVelBounds.setLow(QUAD_MIN_ANGLE_RATE);
            angVelBounds.setHigh(QUAD_MAX_ANGLE_RATE);
            angularVelSpace->setBounds(angVelBounds);
        }

    space->as<ob::CompoundStateSpace>()->lock();
    return space;
}

oc::ControlSpacePtr OMPL_Planner::createControlSpace(ob::StateSpacePtr& space)
{
    auto cspace = std::make_shared<oc::RealVectorControlSpace>(space, CONTROL_DIM);
    ob::RealVectorBounds cbounds(CONTROL_DIM);

    if(MODEL == 1)
        {
            // --- Double Integrator ---
            cbounds.setLow(0, A_MIN);
            cbounds.setHigh(0, A_MAX);
            cbounds.setLow(1, A_MIN);
            cbounds.setHigh(1, A_MAX);
            cbounds.setLow(2, A_MIN);
            cbounds.setHigh(2, A_MAX);
        }
    else if(MODEL == 2)
        {
            // --- Dubins Airplane ---
            cbounds.setLow(0, DUBINS_AIRPLANE_MIN_YR);
            cbounds.setHigh(0, DUBINS_AIRPLANE_MAX_YR);
            cbounds.setLow(1, DUBINS_AIRPLANE_MIN_PR);
            cbounds.setHigh(1, DUBINS_AIRPLANE_MAX_PR);
            cbounds.setLow(2, A_MIN);
            cbounds.setHigh(2, A_MAX);
        }
    else if(MODEL == 3)
        {
            // --- 12 QUAD ---
            cbounds.setLow(0, QUAD_MIN_Zc);
            cbounds.setHigh(0, QUAD_MAX_Zc);
            cbounds.setLow(1, QUAD_MIN_Lc);
            cbounds.setHigh(1, QUAD_MAX_Lc);
            cbounds.setLow(2, QUAD_MIN_Mc);
            cbounds.setHigh(2, QUAD_MAX_Mc);
            cbounds.setLow(3, QUAD_MIN_Nc);
            cbounds.setHigh(3, QUAD_MAX_Nc);
        }

    cspace->setBounds(cbounds);
    return cspace;
}

oc::SimpleSetupPtr OMPL_Planner::kinodynamicSimpleSetUp(const float* initial, const float* goal)
{
    ob::StateSpacePtr space    = createStateSpace();
    oc::ControlSpacePtr cspace = createControlSpace(space);
    auto ss                    = std::make_shared<oc::SimpleSetup>(cspace);
    ss->setStateValidityChecker(
      ob::StateValidityCheckerPtr(new CollisionCheck(ss->getSpaceInformation(), obstacles_, obstaclesCount_, safetyMargin_)));

    auto odeFunction = std::bind(&doubleIntegratorODE, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    auto odeSolver   = std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), odeFunction);
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));
    ss->getSpaceInformation()->setPropagationStepSize(STEP_SIZE);
    ss->getSpaceInformation()->setMinMaxControlDuration(1, MAX_PROPAGATION_DURATION);
    space->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new CustomProjection(space.get())));

    // set starting state:
    ob::ScopedState<> start(space);
    if(MODEL == 1 || MODEL == 2)
        {
            start[0] = initial[0];
            start[1] = initial[1];
            start[2] = initial[2];
            start[3] = initial[3];  // vx
            start[4] = initial[4];  // vy
            start[5] = initial[5];  // vz
        }
    else if(MODEL == 3)
        {
            start[0]  = initial[0];
            start[1]  = initial[1];
            start[2]  = initial[2];
            start[3]  = initial[3];
            start[4]  = initial[4];
            start[5]  = initial[5];
            start[6]  = initial[6];
            start[7]  = initial[7];
            start[8]  = initial[8];
            start[9]  = initial[9];
            start[10] = initial[10];
            start[11] = initial[11];
        }

    // --- Setting goal region ---
    OMPL_INFORM("goal x: %f, y: %f, z: %f", goal[0], goal[1], goal[2]);
    ob::GoalPtr goalRegion(new GoalRegionWithinSphere(ss->getSpaceInformation(), {goal[0], goal[1], goal[2]}, GOAL_THRESH));
    ss->setStartState(start);
    ss->setGoal(goalRegion);

    OMPL_INFORM("Successfully Setup the problem instance");
    return ss;
}

void OMPL_Planner::planRRT(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin)
{
    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    safetyMargin_   = safetyMargin;
    obstacles_      = obstacles;
    obstaclesCount_ = numObstacles;
    OMPL_INFORM("numObstacles: %d", obstaclesCount_);

    oc::SimpleSetupPtr ss = kinodynamicSimpleSetUp(initial, goal);
    oc::PathControl pathOmpl(ss->getSpaceInformation());

    // --- Setting Planner ---
    auto planner = std::make_shared<oc::ModRRT>(ss->getSpaceInformation());
    ss->setPlanner(planner);
    ss->getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    ss->setup();

    // --- Solving Problem ---
    auto start = std::chrono::high_resolution_clock::now();

    ob::PlannerStatus solved = ss->solve(100.0);

    auto end                              = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    double elapsedTime                    = elapsed.count();

    if(solved)
        {
            std::cout << "Found solution:" << std::endl;
            pathOmpl = ss->getSolutionPath();
            write2sys(ss);
            writeExecutionTimeToCSV(elapsedTime);
            writeIterationsToCSV(planner->iterations_);
            ompl::base::PlannerData data(ss->getSpaceInformation());
            planner->getPlannerData(data);
            writeNumVerticesToCSV(data.numVertices());
        }
    else
        {
            std::cout << "No solution found" << std::endl;
        }
}

void OMPL_Planner::planPDST(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin)
{
    safetyMargin_   = safetyMargin;
    obstacles_      = obstacles;
    obstaclesCount_ = numObstacles;
    OMPL_INFORM("numObstacles: %d", obstaclesCount_);

    oc::SimpleSetupPtr ss = kinodynamicSimpleSetUp(initial, goal);
    oc::PathControl pathOmpl(ss->getSpaceInformation());

    // --- Setting Planner ---
    auto planner = std::make_shared<oc::ModPDST>(ss->getSpaceInformation());
    ss->setPlanner(planner);
    ss->getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    ss->setup();
    ss->print();

    // --- Solving Problem ---
    auto start = std::chrono::high_resolution_clock::now();

    ob::PlannerStatus solved = ss->solve(100.0);

    auto end                              = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    double elapsedTime                    = elapsed.count();

    if(solved)
        {
            std::cout << "Found solution:" << std::endl;
            pathOmpl = ss->getSolutionPath();
            write2sys(ss);
            writeExecutionTimeToCSV(elapsedTime);
            writeIterationsToCSV(planner->iterations_);
            ompl::base::PlannerData data(ss->getSpaceInformation());
            planner->getPlannerData(data);
            writeNumVerticesToCSV(data.numVertices());
            planner->clear();
        }
    else
        {
            std::cout << "No solution found" << std::endl;
        }
}

void OMPL_Planner::planEST(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin)
{
    safetyMargin_   = safetyMargin;
    obstacles_      = obstacles;
    obstaclesCount_ = numObstacles;
    OMPL_INFORM("numObstacles: %d", obstaclesCount_);

    oc::SimpleSetupPtr ss = kinodynamicSimpleSetUp(initial, goal);
    oc::PathControl pathOmpl(ss->getSpaceInformation());

    // --- Setting Planner ---
    auto planner = std::make_shared<oc::ModEST>(ss->getSpaceInformation());
    ss->setPlanner(planner);
    ss->getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    ss->setup();
    ss->print();

    // --- Solving Problem ---
    auto start = std::chrono::high_resolution_clock::now();

    ob::PlannerStatus solved = ss->solve(100.0);

    auto end                              = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    double elapsedTime                    = elapsed.count();

    if(solved)
        {
            std::cout << "Found solution:" << std::endl;
            pathOmpl = ss->getSolutionPath();
            write2sys(ss);
            writeExecutionTimeToCSV(elapsedTime);
            writeIterationsToCSV(planner->iterations_);
            ompl::base::PlannerData data(ss->getSpaceInformation());
            planner->getPlannerData(data);
            writeNumVerticesToCSV(data.numVertices());
        }
    else
        {
            std::cout << "No solution found" << std::endl;
        }
}

void OMPL_Planner::planParallelRRT(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin)
{
    try
        {
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

            // --- Setting thread number to maximum available ---
            unsigned int numThreads = std::thread::hardware_concurrency();
            if(numThreads == 0) numThreads = 1;

            // Set the number of threads to use
            setNumberOfThreads(numThreads);

            safetyMargin_   = safetyMargin;
            obstacles_      = obstacles;
            obstaclesCount_ = numObstacles;
            OMPL_INFORM("numObstacles: %d", obstaclesCount_);

            // create simple setup object
            oc::SimpleSetupPtr ss = kinodynamicSimpleSetUp(initial, goal);
            oc::PathControl pathOmpl(ss->getSpaceInformation());
            ompl::tools::ParallelPlan pp(ss->getProblemDefinition());

            // --- Creating numThread planners ---
            std::vector<std::shared_ptr<oc::ModRRT>> planners;
            for(unsigned int i = 0; i < numThreads; ++i)
                {
                    auto planner = std::make_shared<oc::ModRRT>(ss->getSpaceInformation());
                    pp.addPlanner(planner);
                    planners.push_back(planner);
                }
            ss->getSpaceInformation()->setStateValidityCheckingResolution(0.005);
            ss->setup();

            // --- Solving Problem ---
            auto start = std::chrono::high_resolution_clock::now();

            ompl::base::PlannerStatus solved = pp.solve(100.0, false);

            auto end                              = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            double elapsedTime                    = elapsed.count();

            if(solved)
                {
                    std::cout << "Found solution in " << elapsedTime << " seconds." << std::endl;
                    writeExecutionTimeToCSV(elapsedTime);

                    int totalIterations = 0;
                    int numVertices     = 0;
                    for(size_t i = 0; i < planners.size(); ++i)
                        {
                            ompl::base::PlannerData data(ss->getSpaceInformation());
                            planners[i]->getPlannerData(data);
                            totalIterations += planners[i]->iterations_;
                            numVertices += data.numVertices();
                            planners[i]->clear();
                        }
                    writeIterationsToCSV(totalIterations);
                    writeNumVerticesToCSV(numVertices);
                    write2sys(ss);
                }
            else
                {
                    std::cout << "No solution found" << std::endl;
                }
        }
    catch(const std::exception& e)
        {
            std::cerr << "Exception caught: " << e.what() << std::endl;
        }
}

void OMPL_Planner::planParallelEST(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin)
{
    try
        {
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

            // --- Setting thread number to maximum available ---
            unsigned int numThreads = std::thread::hardware_concurrency();
            if(numThreads == 0) numThreads = 1;

            // Set the number of threads to use
            setNumberOfThreads(numThreads);

            safetyMargin_   = safetyMargin;
            obstacles_      = obstacles;
            obstaclesCount_ = numObstacles;
            OMPL_INFORM("numObstacles: %d", obstaclesCount_);

            // create simple setup object
            oc::SimpleSetupPtr ss = kinodynamicSimpleSetUp(initial, goal);
            oc::PathControl pathOmpl(ss->getSpaceInformation());
            ompl::tools::ParallelPlan pp(ss->getProblemDefinition());

            // --- Creating numThread planners ---
            std::vector<std::shared_ptr<oc::ModEST>> planners;
            for(unsigned int i = 0; i < numThreads; ++i)
                {
                    auto planner = std::make_shared<oc::ModEST>(ss->getSpaceInformation());
                    pp.addPlanner(planner);
                    planners.push_back(planner);
                }
            ss->getSpaceInformation()->setStateValidityCheckingResolution(0.0005);
            ss->setup();

            // --- Solving Problem ---
            auto start = std::chrono::high_resolution_clock::now();

            ompl::base::PlannerStatus solved = pp.solve(100.0, false);

            auto end                              = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            double elapsedTime                    = elapsed.count();

            if(solved)
                {
                    std::cout << "Found solution in " << elapsedTime << " seconds." << std::endl;
                    writeExecutionTimeToCSV(elapsedTime);

                    int totalIterations = 0;
                    int numVertices     = 0;
                    for(size_t i = 0; i < planners.size(); ++i)
                        {
                            ompl::base::PlannerData data(ss->getSpaceInformation());
                            planners[i]->getPlannerData(data);
                            totalIterations += planners[i]->iterations_;
                            numVertices += data.numVertices();
                            planners[i]->clear();
                        }
                    writeIterationsToCSV(totalIterations);
                    writeNumVerticesToCSV(numVertices);
                    write2sys(ss);
                }
            else
                {
                    std::cout << "No solution found" << std::endl;
                }
        }
    catch(const std::exception& e)
        {
            std::cerr << "Exception caught: " << e.what() << std::endl;
        }
}

void OMPL_Planner::planParallelPDST(const float* initial, const float* goal, float* obstacles, int numObstacles, float safetyMargin)
{
    try
        {
            ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

            // --- Setting thread number to maximum available ---
            unsigned int numThreads = std::thread::hardware_concurrency();
            if(numThreads == 0) numThreads = 1;

            // Set the number of threads to use
            setNumberOfThreads(numThreads);

            safetyMargin_   = safetyMargin;
            obstacles_      = obstacles;
            obstaclesCount_ = numObstacles;
            OMPL_INFORM("numObstacles: %d", obstaclesCount_);

            // create simple setup object
            oc::SimpleSetupPtr ss = kinodynamicSimpleSetUp(initial, goal);
            oc::PathControl pathOmpl(ss->getSpaceInformation());
            ompl::tools::ParallelPlan pp(ss->getProblemDefinition());

            // --- Creating numThread planners ---
            std::vector<std::shared_ptr<oc::ModPDST>> planners;
            for(unsigned int i = 0; i < numThreads; ++i)
                {
                    auto planner = std::make_shared<oc::ModPDST>(ss->getSpaceInformation());
                    pp.addPlanner(planner);
                    planners.push_back(planner);
                }
            ss->getSpaceInformation()->setStateValidityCheckingResolution(0.005);
            ss->setup();

            // --- Solving Problem ---
            auto start = std::chrono::high_resolution_clock::now();

            ompl::base::PlannerStatus solved = pp.solve(100.0, false);

            auto end                              = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            double elapsedTime                    = elapsed.count();

            if(solved)
                {
                    std::cout << "Found solution in " << elapsedTime << " seconds." << std::endl;
                    writeExecutionTimeToCSV(elapsedTime);

                    int totalIterations = 0;
                    int numVertices     = 0;
                    for(size_t i = 0; i < planners.size(); ++i)
                        {
                            ompl::base::PlannerData data(ss->getSpaceInformation());
                            planners[i]->getPlannerData(data);
                            totalIterations += planners[i]->iterations_;
                            numVertices += data.numVertices();
                            planners[i]->clear();
                        }
                    writeIterationsToCSV(totalIterations);
                    writeNumVerticesToCSV(numVertices);
                    write2sys(ss);
                }
            else
                {
                    std::cout << "No solution found" << std::endl;
                }
        }
    catch(const std::exception& e)
        {
            std::cerr << "Exception caught: " << e.what() << std::endl;
        }
}

OMPL_Planner::OMPL_Planner(){};
OMPL_Planner::~OMPL_Planner(){};
