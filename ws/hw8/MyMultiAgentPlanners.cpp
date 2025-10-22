#include "MyMultiAgentPlanners.h"

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    // Create a meta path
    amp::Path path;

    // Create the meta-state
    Eigen::VectorXd meta_q_init;
    int metaSize = 0;

        // Calculate total number of states
    for (const amp::CircularAgentProperties &agent : problem.agent_properties)
    {
        metaSize += agent.q_init.size();
    }

        // Resize the meta-state
    meta_q_init.resize(metaSize);

        // Fill in the meta-state
    int currentIndex = 0;
    for (const amp::CircularAgentProperties &agent : problem.agent_properties)
    {
        int agentSize = agent.q_init.size();
        meta_q_init.segment(currentIndex, agentSize) = agent.q_init;
        currentIndex += agentSize;
    }

    // Generate an initial C-space

    // Assign each agent's path from the meta-state path
        // Define a multi-agent path object
    amp::MultiAgentPath2D multiAgentPath;
        // Loop through each agent and pull out it's path from the meta-agent
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        amp::Path2D agent_path;
        agent_path.waypoints = {agent.q_init, agent.q_goal};
        multiAgentPath.agent_paths.push_back(agent_path);
    }
    return multiAgentPath;
}

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        amp::Path2D agent_path;
        agent_path.waypoints = {agent.q_init, agent.q_goal};
        path.agent_paths.push_back(agent_path);
    }
    return path;
}