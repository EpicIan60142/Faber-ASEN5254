#include "MyMultiAgentPlanners.h"

#include "CSpace.h"
#include "MySamplingBasedPlanners.h"

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    // Create the meta-state
    Eigen::VectorXd meta_q_init;
    Eigen::VectorXd meta_q_goal;
    int metaSize = 0;

        // Calculate total number of states
    for (const amp::CircularAgentProperties &agent : problem.agent_properties)
    {
        metaSize += agent.q_init.size();
    }

        // Resize the meta-state
    meta_q_init.resize(metaSize);
    meta_q_goal.resize(metaSize);

        // Fill in the meta-state
    int currentIndex = 0;
    for (const amp::CircularAgentProperties &agent : problem.agent_properties)
    {
        int agentSize = agent.q_init.size();
        meta_q_init.segment(currentIndex, agentSize) = agent.q_init;
        meta_q_goal.segment(currentIndex, agentSize) = agent.q_goal;
        currentIndex += agentSize;
    }

    // Create multi-agent C-space
    MultiAgentCSpace cspace(problem.agent_properties, amp::Environment2D(problem.x_min, problem.x_max, problem.y_min, problem.y_max, problem.obstacles));

    // Run Goal biased RRT planner
    GenericRRT rrtPlanner;
    amp::Path path = rrtPlanner.plan(meta_q_init, meta_q_goal, cspace, nSample, rConnect, pGoal, epsilon, graphPtr, nodes);

    if (path.waypoints.size() < 2)
    {
        path.valid = false;
    }

    // Assign each agent's path from the meta-state path
        // Define a multi-agent path object
    amp::MultiAgentPath2D multiAgentPath;
        // Loop through each agent and pull out its path from the meta-agent
    for (int i = 0; i < problem.agent_properties.size(); i++)
    {
        amp::Path2D agent_path;
        for (int j = 0; j < path.waypoints.size(); j++)
        {
            int agentSize = 2; //problem.agent_properties[i].q_init.size();
            Eigen::VectorXd agentPos = path.waypoints[j].segment(agentSize*i, agentSize);
            agent_path.waypoints.push_back(agentPos);
        }
        agent_path.waypoints.push_back(problem.agent_properties[i].q_goal);
        multiAgentPath.agent_paths.push_back(agent_path);
    }
    multiAgentPath.valid = path.valid;
    return multiAgentPath;
}

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    // Define vector for storing already calculated paths
    std::vector<amp::Path> previousPaths;

    // Randomize agent priorities
        // Define random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
        // Populate priority list
    std::vector<int> priorityOrder;
    for (int i = 0; i < problem.agent_properties.size(); i++)
    {
        priorityOrder.push_back(i);
    }
        // Shuffle the list
    std::shuffle(priorityOrder.begin(), priorityOrder.end(), gen);

    // Add initial conditions to the previous paths vector
    for (const amp::CircularAgentProperties& agent : problem.agent_properties)
    {
        amp::Path initPath;
        initPath.waypoints.push_back(agent.q_init);
        previousPaths.push_back(initPath);
    }

    // Plan each agent sequentially
    bool tryAgain = true;
    int tries = 0;
    while (tryAgain && tries < maxTries)
    {
        for (int i = 0; i < priorityOrder.size(); i++)
        {
            // Reset looping variable
            tryAgain = false;

            // Pull out the first agent index
            int agentIdx = priorityOrder[i];

            // Create decoupled multi-agent C-space for this agent
            DecoupledAgentCSpace cspace(problem.agent_properties,
                                        amp::Environment2D(problem.x_min, problem.x_max, problem.y_min, problem.y_max, problem.obstacles),
                                        previousPaths,
                                        agentIdx);

            // Plan path for the current agent
                // Extract initial and goal states
            Eigen::VectorXd q_init = problem.agent_properties[agentIdx].q_init;
            Eigen::VectorXd q_goal = problem.agent_properties[agentIdx].q_goal;
                // Clear graph and nodes
            graphPtr->clear();
            nodes.clear();
                // Run RRT planner
            GenericRRT rrtPlanner;
            amp::Path path = rrtPlanner.plan(q_init, q_goal, cspace, nSample, rConnect, pGoal, epsilon, graphPtr, nodes);

            // If path was valid, add it to previous paths. Otherwise, try again with different priority order
            if (path.valid)
            {
                previousPaths[agentIdx] = path; //
            }
            else
            {
                tryAgain = true;
                tries++;
                break;
            }
        }
        if (tryAgain)
        {
            std::shuffle(priorityOrder.begin(), priorityOrder.end(), gen);
        }
    }

    // Add final paths to the multi-agent path
        // Define multi-agent path
    amp::MultiAgentPath2D multiAgentPath;
        // Loop through each agent and pull out its path from the stored paths vector
    for (int i = 0; i < problem.agent_properties.size(); i++)
    {
        amp::Path2D agent_path;
        for (int j = 0; j < previousPaths[i].waypoints.size(); j++)
        {
            Eigen::VectorXd agentPos = previousPaths[i].waypoints[j];
            agent_path.waypoints.push_back(agentPos);
        }
        agent_path.waypoints.push_back(problem.agent_properties[i].q_goal);
        multiAgentPath.agent_paths.push_back(agent_path);
    }
    multiAgentPath.valid = previousPaths.back().valid; // If we populated the last agent's path, all of them are valid

    return multiAgentPath;
}