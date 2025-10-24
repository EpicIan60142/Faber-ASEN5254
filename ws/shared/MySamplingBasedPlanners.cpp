#include "MySamplingBasedPlanners.h"

#include "CSpace.h"
#include "MyAStar.h"

amp::Path GenericPRM::plan(const Eigen::VectorXd &init_state, const Eigen::VectorXd &goal_state, const amp::ConfigurationSpace &cspace, const int nSample, const double rConnect, const bool smooth, std::shared_ptr<amp::Graph<double>> &graph, std::map<amp::Node, Eigen::VectorXd> &nodes)
{
        // Make graph pointer
    std::shared_ptr<amp::Graph<double>> graphPtr = graph;

        // Step 1: Add init_state and goal_state to graph
    nodes[0] = init_state;
    nodes[1] = goal_state;

        // Step 2: Sample random configurations
            // Start node index at 2
    size_t nodeIdx = 2;
            // Sample nSample new configurations
    for (int i = 0; i < nSample; i++)
    {
            // Generate vector with values between 0 and 1, as ::Random() generates numbers between -1 and 1
        Eigen::VectorXd randVec = Eigen::VectorXd::Random(init_state.size()) * 0.5 + Eigen::VectorXd::Ones(init_state.size()) * 0.5;
            // Apply CSpace range
        Eigen::VectorXd randState = cspace.lowerBounds() + (cspace.upperBounds() - cspace.lowerBounds()).cwiseProduct(randVec);

            // If sample doesn't collide, add it to the vertex set
        if (!cspace.inCollision(randState))
        {
            nodes[nodeIdx] = randState;
            nodeIdx++;
        }
    }

        // Step 3: Connect samples if they are within rConnect of each other
            // Loop through all nodes
    for (int i = 0; i < nodeIdx; i++)
    {
            // Assign "from" node
        amp::Node from = i;
            // Check distance from node to every other node
        for (int j = 0; j < nodeIdx; j++)
        {
            if (i == j)
            {
                continue;
            }
                // Assign "to" node
            amp::Node to = j;

                // Calculate distance between nodes
            double dist = (nodes[from] - nodes[to]).norm();
                // If nodes are close enough, attempt to connect them
            if (dist < rConnect)
            {
                    // Step incrementally along the vector connecting each node and exit if a collision occurs
                bool collided = false;
                Eigen::VectorXd rB = nodes[to] - nodes[from];
                for (double t = 0; t <= 1; t += 0.01)
                {
                    Eigen::VectorXd candidatePoint = nodes[from] + t * rB;
                    if (cspace.inCollision(candidatePoint))
                    {
                        collided = true;
                        break;
                    }
                }
                    // Connect nodes if they didn't collide
                if (!collided)
                {
                        // Connect with edge lengths of 1 for PRM
                    graphPtr->connect(from, to, 1);
                }
            }
        }
    }

    //graphPtr->print();

        // Step 4: Graph search for a path from q_init to q_goal
    amp::ShortestPathProblem problem(graphPtr, 0, 1);
    MyAStarAlgo algo;
    MyAStarAlgo::GraphSearchResult result = algo.search(problem, amp::SearchHeuristic());

        // Step 5: Attempt to smooth path if smoothing is enabled
    if (smooth && result.success)
    {
            // Get number of nodes in original path
        const size_t nNodes = result.node_path.size();
            // Set up random number generation
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<size_t> dist(1, nNodes);
            // Set up a smaller graph just made up of the valid path
        std::shared_ptr<amp::Graph<double>> smoothGraphPtr = std::make_shared<amp::Graph<double>>();
        smoothGraphPtr->clear();
        for (int i = 0; i < nNodes-1; i++)
        {
            auto it = result.node_path.begin();
            std::advance(it, i);
            const amp::Node node1 = *it;
            it = result.node_path.begin();
            std::advance(it, i+1);
            const amp::Node node2 = *it;
            smoothGraphPtr->connect(node1, node2, 1);
        }

        //smoothGraphPtr->print();

            // Try to directly connect nodes in the path
        for (int i = 0; i < nSample; i++)
        {
                // Generate index of valid path nodes to try and connect
            const size_t node1Idx = dist(gen);
            const size_t node2Idx = dist(gen);

                // Get node corresponding to that index
            auto it = result.node_path.begin();
            std::advance(it, node1Idx);
            const amp::Node node1 = *it;
            it = result.node_path.begin();
            std::advance(it, node2Idx);
            const amp::Node node2 = *it;

            bool collided = false;
            Eigen::VectorXd rB = nodes[node2] - nodes[node1];
            for (double t = 0; t <= 1; t += 0.001)
            {
                Eigen::VectorXd candidatePoint = nodes[node1] + t * rB;
                if (cspace.inCollision(candidatePoint))
                {
                    collided = true;
                    break;
                }
            }
            // Connect nodes if they didn't collide
            if (!collided)
            {
                // Connect with edge lengths of 1 for PRM
                smoothGraphPtr->connect(node1, node2, 1);
            }
        }

        //smoothGraphPtr->print();

            // Rerun the search algorithm and update graph pointer
        graphPtr = smoothGraphPtr;
        amp::ShortestPathProblem smoothProblem(graphPtr, 0, 1);
        result = algo.search(smoothProblem, amp::SearchHeuristic());
    }

    amp::Path path;

    if (result.success)
    {
            // Index through nodes
        for (const auto &node : result.node_path)
        {
            path.waypoints.push_back(nodes[node]);
        }
        path.valid = true;
    }
    else
    {
        path.waypoints.push_back(init_state);

        path.valid = false;
    }


    return path;
}

amp::Path GenericRRT::plan(const Eigen::VectorXd &init_state, const Eigen::VectorXd &goal_state, const amp::ConfigurationSpace &cspace, const int nSample, const double rConnect, const double pGoal, const double epsilon, std::shared_ptr<amp::Graph<double>> &graph, std::map<amp::Node, Eigen::VectorXd> &nodes)
{
        // Make graph pointer
    std::shared_ptr<amp::Graph<double>> graphPtr = graph;

        // Add init_state as the root of the tree graph and initialize the node index
    nodes[0] = init_state;
    size_t nodeIdx = 1;

        // Take samples until a solution is found or we hit the max number of samples
    bool pathFound = false;
    for (int i = 0; i < nSample; i++)
    {
            // Set up random number generation to determine whether sample is goal_state or random
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_real_distribution<double> dist(0, 1);
        double rand = dist(gen);
        Eigen::VectorXd randState;
                // Choose goal_state with probability pGoal
        if (rand <= pGoal)
        {
            randState = goal_state;
        }
                // Choose a random configuration with probability 1-pGoal
        else
        {
                // Generate vector with values between 0 and 1, as ::Random() generates numbers between -1 and 1
            Eigen::VectorXd randVec = Eigen::VectorXd::Random(init_state.size()) * 0.5 + Eigen::VectorXd::Ones(init_state.size()) * 0.5;
                // Apply CSpace range
            randState = cspace.lowerBounds() + (cspace.upperBounds() - cspace.lowerBounds()).cwiseProduct(randVec);
        }

            // Find nearest configuration to the sample point
        double minDist = std::numeric_limits<double>::infinity();
        size_t nearestNodeIdx = 0;
        for (size_t j = 0; j < nodeIdx; j++)
        {
            Eigen::VectorXd nodeState = nodes[j];
            double dist = (randState - nodeState).norm();
            if (dist < minDist)
            {
                minDist = dist;
                nearestNodeIdx = j;
            }
        }

            // Construct a unit vector from the nearest tree node to the random node and construct a path that's rConnect along the unit vector
        Eigen::VectorXd rB = randState - nodes[nearestNodeIdx];
        Eigen::VectorXd rHatB = rB / rB.norm();
        Eigen::VectorXd candidatePoint = nodes[nearestNodeIdx] + rConnect * rHatB;

            // Test the path for collisions and add it to the graph if collision free
        bool collided = false;
        Eigen::VectorXd stepVector = candidatePoint - nodes[nearestNodeIdx];
        for (double t = 0; t <= 1; t += 0.05)
        {
                // Incrementally step along the unit vector and test for collisions or out of bounds
            Eigen::VectorXd testPoint = nodes[nearestNodeIdx] + t * stepVector;
            if (!isWithinBounds(testPoint, cspace) || cspace.inCollision(testPoint))
            {
                collided = true;
                break;
            }
        }

            // Connect nodes if they didn't collide and add to the node map
        if (!collided)
        {
                // Add candidate point to nodes map
            nodes[nodeIdx] = candidatePoint;

                // Connect with edge lengths of 1 for RRT
            graphPtr->connect(nearestNodeIdx, nodeIdx, 1);

                // Check for path solution
            if ((goal_state - nodes[nodeIdx]).norm() < epsilon)
            {
                pathFound = true;
                break;
            }
                 // Increment node index
            nodeIdx++;
        }
    }

        // Add goal state to graph and graph search for the solution
    amp::Path path;
    if (pathFound)
    {
            // Add goal state as a node
        nodes[nodeIdx+1] = goal_state;
        graphPtr->connect(nodeIdx, nodeIdx+1, 1);

            // Run A* on the tree
        amp::ShortestPathProblem problem(graphPtr, 0, nodeIdx+1);
        MyAStarAlgo algo;
        MyAStarAlgo::GraphSearchResult result = algo.search(problem, amp::SearchHeuristic());

        if (result.success)
        {
                // Add nodes in sequence
            for (const auto &node : result.node_path)
            {
                path.waypoints.push_back(nodes[node]);
            }
            path.valid = true;
        }
        else
        {
            path.waypoints.push_back(init_state);
            path.valid = false;
        }

    }
    else
    {
        path.waypoints.push_back(init_state);
        path.valid = false;
    }

    return path;
}

// Implement your PRM algorithm here
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    // Make Cspace constructor and construct cspace
        // Make cells with specified side lengths for this planner
    std::size_t n_x1_cells = (problem.x_max - problem.x_min) / (gridSize);
    std::size_t n_x2_cells = (problem.y_max - problem.y_min) / (gridSize);

        // Make 2D grid CSpace constructor and construct it
    std::shared_ptr<MyPointAgentCSConstructor> point_agent_ctor = std::make_shared<MyPointAgentCSConstructor>(n_x1_cells, n_x2_cells);
    std::unique_ptr<amp::GridCSpace2D> cspace = point_agent_ctor->construct(problem);

        // Make an adapter to pass the 2D grid to the generic planning method
    GridCSpaceAdapter adapter(*cspace);

    // Call generic planner
    amp::Path path_nd = GenericPRM::plan(problem.q_init, problem.q_goal, adapter, nSample, rConnect, smooth, graphPtr, nodes);

    // Convert path to 2D
    std::vector<Eigen::Vector2d> waypoints_2d = path_nd.getWaypoints2D();
    amp::Path2D path_2d;
    path_2d.valid = path_nd.valid;
    for (const Eigen::Vector2d &waypoint_2d : waypoints_2d)
    {
        path_2d.waypoints.push_back(waypoint_2d);
    }

    // Return the 2D path
    return path_2d;
}

// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    // Make Cspace constructor and construct cspace
        // Make cells with specified side lengths for this planner
    std::size_t n_x1_cells = (problem.x_max - problem.x_min) / (gridSize);
    std::size_t n_x2_cells = (problem.y_max - problem.y_min) / (gridSize);

    // Make 2D grid CSpace constructor and construct it
    std::shared_ptr<MyPointAgentCSConstructor> point_agent_ctor = std::make_shared<MyPointAgentCSConstructor>(n_x1_cells, n_x2_cells);
    std::unique_ptr<amp::GridCSpace2D> cspace = point_agent_ctor->construct(problem);

    // Make an adapter to pass the 2D grid to the generic planning method
    GridCSpaceAdapter adapter(*cspace);

    // Call generic planner
    amp::Path path_nd = GenericRRT::plan(problem.q_init, problem.q_goal, adapter, nSample, rConnect, pGoal, epsilon, graphPtr, nodes);

    // Convert path to 2D
    std::vector<Eigen::Vector2d> waypoints_2d = path_nd.getWaypoints2D();
    amp::Path2D path_2d;
    path_2d.valid = path_nd.valid;
    for (const Eigen::Vector2d &waypoint_2d : waypoints_2d)
    {
        path_2d.waypoints.push_back(waypoint_2d);
    }

    // Return the 2D path
    return path_2d;
}
