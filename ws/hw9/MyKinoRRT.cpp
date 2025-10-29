#include "MyKinoRRT.h"

#include "CSpace.h"
#include "MyAStar.h"

// Runge-Kutta 4th order integrator template
template<typename AgentType>
Eigen::VectorXd RK4(AgentType &agent, Eigen::VectorXd& state, const Eigen::VectorXd& control, const double dt)
{
    // Intermediate states
    Eigen::VectorXd w1 = agent.dynamics(state, control);
    Eigen::VectorXd tempState = state + (0.5 * dt * w1);
    Eigen::VectorXd w2 = agent.dynamics(tempState, control);
    tempState = state + (0.5 * dt * w2);
    Eigen::VectorXd w3 = agent.dynamics(tempState, control);
    tempState = state + (dt * w3);
    Eigen::VectorXd w4 = agent.dynamics(tempState, control);

    // Propagate
    return state + ((dt/6.0) * (w1 + 2*w2 + 2*w3 + w4));
}

// Kinodynamic RRT
amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent)
{
    // Assign agent dimensions
    agent.agent_dim = problem.agent_dim;

    // Build CSpace
    KinoDynamicCSpace cspace(problem);

    // Add initial configurations as the root of the tree graph and initialize the node index
    this->nodes[0] = problem.q_init;
    this->controls[0] = Eigen::VectorXd::Zero(problem.u_bounds.size());
    this->durations[0] = 0;
    size_t nodeIdx = 1;

    // Find mean of goal bounds and set as the target goal
    Eigen::VectorXd q_goal_mean = Eigen::VectorXd(problem.q_goal.size());
    for (int i = 0; i < problem.q_goal.size(); i++)
    {
        q_goal_mean[i] = 0.5*(problem.q_goal[i].first + problem.q_goal[i].second);
    }

    // Take samples until we find a valid trajectory or hit the max number of samples
    bool trajFound = false;
    for (int i = 0; i < this->nSample; i++)
    {
        // Set up random number generation to determine whether sample is goal_state or random
        static std::random_device rd_p;
        static std::mt19937 gen_p(rd_p());
        static std::uniform_real_distribution<double> dist_p(0, 1);
        double rand = dist_p(gen_p);
        Eigen::VectorXd randState;

        if (rand <= this->pGoal)// Choose goal_state with probability pGoal
        {
            randState = q_goal_mean;
        }
        else // Choose a random configuration with probability 1-pGoal
        {
            // Generate vector with values between 0 and 1, as ::Random() generates numbers between -1 and 1
            Eigen::VectorXd randVec = Eigen::VectorXd::Random(problem.q_init.size()) * 0.5 + Eigen::VectorXd::Ones(problem.q_init.size()) * 0.5;
            // Apply CSpace range
            randState = cspace.lowerBounds() + (cspace.upperBounds() - cspace.lowerBounds()).cwiseProduct(randVec);
        }

        // Find nearest configuration to the sample point
        double minDist = std::numeric_limits<double>::infinity();
        size_t nearestNodeIdx = 0;
        for (size_t j = 0; j < nodeIdx; j++)
        {
            Eigen::VectorXd nodeState = this->nodes[j];
            double dist = (randState - nodeState).norm();
            if (dist < minDist)
            {
                minDist = dist;
                nearestNodeIdx = j;
            }
        }

        // Construct a unit vector from the nearest tree node to the random node and construct a path that's rConnect along the unit vector
        Eigen::VectorXd rB = randState - this->nodes[nearestNodeIdx];
        Eigen::VectorXd rHatB = rB / rB.norm();
        Eigen::VectorXd goalPoint = this->nodes[nearestNodeIdx] + this->rConnect * rHatB;

        // Sample random controls to try and reach the goal point
            // Initialize tracking variables
        minDist = std::numeric_limits<double>::infinity();
        Eigen::VectorXd bestControl;
        double bestDt;
            // Find control max and min
        Eigen::VectorXd controlMin = Eigen::VectorXd(problem.u_bounds.size());
        for (int k = 0; k < problem.u_bounds.size(); k++)
        {
            controlMin[k] = problem.u_bounds[k].first;
        }
        Eigen::VectorXd controlMax = Eigen::VectorXd(problem.u_bounds.size());
        for (int k = 0; k < problem.u_bounds.size(); k++)
        {
            controlMax[k] = problem.u_bounds[k].second;
        }
            // Sample random controls
        for (int j = 0; j < this->uSample; j++)
        {
                // Generate vector between 0 and 1
            Eigen::VectorXd randVec = Eigen::VectorXd::Random(problem.u_bounds.size()) * 0.5 + Eigen::VectorXd::Ones(problem.u_bounds.size()) * 0.5;
                // Apply control range
            Eigen::VectorXd control = controlMin + (controlMax - controlMin).cwiseProduct(randVec);
                // Generate a random time duration between the provided range
            static std::random_device rd_dt;
            static std::mt19937 gen_dt(rd_dt());
            std::uniform_real_distribution<double> dist_dt(problem.dt_bounds.first, problem.dt_bounds.second);
            double dt = dist_dt(gen_dt);
                // Start at the nearest node
            Eigen::VectorXd testPoint = this->nodes[nearestNodeIdx];
            agent.propagate(testPoint, control, dt);
            double dist = (goalPoint - testPoint).norm();
            if (dist < minDist)
            {
                bestControl = control;
                bestDt = dt;
            }
        }

        // Test the subpath for collisions and add it to the graph if collision free
        bool collided = false;
        Eigen::VectorXd testPoint;
        for (double t = 0; t <= bestDt; t += bestDt/5.0)
        {
            // Incrementally integrate the dynamics and check for collisions
            testPoint = this->nodes[nearestNodeIdx];
            agent.propagate(testPoint, bestControl, t);
            if (!isWithinBounds(testPoint, cspace) || cspace.inCollision(testPoint))
            {
                collided = true;
                break;
            }
        }

        // Connect nodes if they didn't collide and add to the node map
        if (!collided)
        {
                // Add successful test point to nodes map
            Eigen::VectorXd newState = this->nodes[nearestNodeIdx];
            agent.propagate(newState, bestControl, bestDt);
            this->nodes[nodeIdx] = newState;
            this->controls[nodeIdx] = bestControl;
            this->durations[nodeIdx] = bestDt;

                // Connect with edge lengths of 1 for RRT
            this->graphPtr->connect(nearestNodeIdx, nodeIdx, 1);

            // Check for path solution
            trajFound = true;
            for (int j = 0; j < problem.q_goal.size(); j++)
            {
                Eigen::VectorXd currentQ = this->nodes[nodeIdx];
                if (currentQ[j] < problem.q_goal[j].first || currentQ[j] > problem.q_goal[j].second)
                {
                    trajFound = false;
                    break;
                }
            }

            // If trajectory was found, exit the sampling loop
            if (trajFound)
            {
                break;
            }

            // Increment node index
            nodeIdx++;
        }
    }

    // Add goal state to graph and graph search for the solution
    amp::KinoPath path;
    if (trajFound)
    {
        // Add goal state as a node
        //this->nodes[nodeIdx+1] = q_goal_mean;
        //this->controls[nodeIdx+1] = Eigen::VectorXd::Zero(problem.u_bounds.size());
        //this->durations[nodeIdx+1] = 0;
        //this->graphPtr->connect(nodeIdx, nodeIdx+1, 1);

        // Run A* on the tree
        amp::ShortestPathProblem problemRRT(this->graphPtr, 0, nodeIdx);
        MyAStarAlgo algo;
        MyAStarAlgo::GraphSearchResult result = algo.search(problemRRT, amp::SearchHeuristic());

        if (result.success)
        {
            // Add nodes in sequence
            for (const auto &node : result.node_path)
            {
                path.waypoints.push_back(nodes[node]);
                path.controls.push_back(controls[node]);
                path.durations.push_back(durations[node]);
            }
            path.valid = true;
        }
        else
        {
            path.waypoints.push_back(problem.q_init);
            path.controls.push_back(Eigen::VectorXd::Zero(problem.u_bounds.size()));
            path.durations.push_back(0);
            path.valid = false;
        }

    }
    else
    {
        path.waypoints.push_back(problem.q_init);
        path.controls.push_back(Eigen::VectorXd::Zero(problem.u_bounds.size()));
        path.durations.push_back(0);
        path.valid = false;
    }

    return path;
}

// Single Integrator
void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, const double dt)
{
    state = RK4(*this, state, control, dt);
};

Eigen::VectorXd MySingleIntegrator::dynamics(Eigen::VectorXd &state, const Eigen::VectorXd &control)
{
    return control;
}

// First order unicycle
void MyFirstOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, const double dt)
{
    state = RK4(*this, state, control, dt);
}

Eigen::VectorXd MyFirstOrderUnicycle::dynamics(Eigen::VectorXd &state, const Eigen::VectorXd &control)
{
    // Extract dimensions
    double radius = this->agent_dim.length/2.0;

    // Extract states used in dynamics
    double theta = state[2];

    // Calculate rates of change
    double xDot = control[0]*radius*std::cos(theta);
    double yDot = control[0]*radius*std::sin(theta);
    double thetaDot = control[1];

    // Assign output
    Eigen::VectorXd dX = Eigen::VectorXd(state.size());
    dX << xDot, yDot, thetaDot;

    return dX;
}

// Second order unicycle
void MySecondOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, const double dt)
{
    state = RK4(*this, state, control, dt);
}

Eigen::VectorXd MySecondOrderUnicycle::dynamics(Eigen::VectorXd &state, const Eigen::VectorXd &control)
{
    // Extract dimensions
    double radius = this->agent_dim.length/2.0;

    // Extract states used in dynamics
    double theta = state[2];
    double sigma = state[3];
    double omega = state[4];

    // Calculate rates of change
    double xDot = sigma*radius*std::cos(theta);
    double yDot = sigma*radius*std::sin(theta);
    double thetaDot = omega;
    double sigmaDot = control[0];
    double omegaDot = control[1];

    // Assign output
    Eigen::VectorXd dX = Eigen::VectorXd(state.size());
    dX << xDot, yDot, thetaDot, sigmaDot, omegaDot;

    return dX;
}

// Simple car
void MySimpleCar::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, const double dt)
{
    state = RK4(*this, state, control, dt);
}

Eigen::VectorXd MySimpleCar::dynamics(Eigen::VectorXd &state, const Eigen::VectorXd &control)
{
    // Extract dimensions used in dynamics
    double length = this->agent_dim.length;
    double width = this->agent_dim.width;

    // Extract states used in dynamics
    double theta = state[2];
    double v = state[3];
    double phi = state[4];

    // Calculate rates of change
    double xDot = v*std::cos(theta);
    double yDot = v*std::sin(theta);
    double thetaDot = (v/length)*std::tan(phi);
    double vDot = control[0];
    double phiDot = control[1];

    // Assign output
    Eigen::VectorXd dX = Eigen::VectorXd(state.size());
    dX << xDot, yDot, thetaDot, vDot, phiDot;

    return dX;
}
