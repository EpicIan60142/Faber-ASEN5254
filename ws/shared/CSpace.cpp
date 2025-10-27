//
// Created by ianmf on 9/24/25.
//

#include "CSpace.h"
#include "ObstacleChecker.h"

// Function for checking if a point is within N-dimensional C space bounds
bool isWithinBounds(const Eigen::VectorXd &point, const amp::ConfigurationSpace &cspace)
{
    for (int i = 0; i < point.size(); i++)
    {
        if (point[i] < cspace.lowerBounds()[i] || point[i] > cspace.upperBounds()[i])
        {
            return false;
        }
    }
    return true;

}

// Override this method for determining which cspace cell a point is inside of
std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const
{
    // Implement your discretization procedure here, such that the point (x0, x1) lies within the returned cell

        // get number of cells for each parameter in this space
    std::pair<std::size_t, std::size_t> cellNums = size();
    std::size_t x0Cells = cellNums.first;
    std::size_t x1Cells = cellNums.second;

        // Get bounds for this space
    std::pair<double, double> x0Bounds = this->x0Bounds();
    double x0Min = x0Bounds.first;
    double x0Max = x0Bounds.second;

    std::pair<double, double> x1Bounds = this->x1Bounds();
    double x1Min = x1Bounds.first;
    double x1Max = x1Bounds.second;

        // Compute where the provided point falls in the grid
    double x0Ratio = (x0 - x0Min) / (x0Max - x0Min);
    double x1Ratio = (x1 - x1Min) / (x1Max - x1Min);

        // Clamp cell index to valid range if x0, x1 was out of bounds
    std::size_t cell_x = std::min(static_cast<std::size_t>(x0Ratio * x0Cells), x0Cells - 1);
    std::size_t cell_y = std::min(static_cast<std::size_t>(x1Ratio * x1Cells), x1Cells - 1);

    return {cell_x, cell_y};
}

// Function for determining a configuration from a cell
Eigen::Vector2d MyGridCSpace2D::getPointFromCell(std::pair<std::size_t, std::size_t> cell) const
{
        // Extract x0 and x1 bounds
    std::pair<double, double> x0Bounds = this->x0Bounds();
    std::pair<double, double> x1Bounds = this->x1Bounds();

        // Calculate midpoint of given cell
    double x0 = ((cell.first + 0.5)/this->size().first)*(x0Bounds.second - x0Bounds.first) + x0Bounds.first;
    double x1 = ((cell.second + 0.5)/this->size().second)*(x1Bounds.second - x1Bounds.first) + x1Bounds.first;

        // Return as a vector
    return Eigen::Vector2d(x0, x1);
}


// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env)
{
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer.
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0, 2*M_PI, 0, 2*M_PI);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    // Create an obstacle checker object for primitive evaluation
    ObstacleChecker obsCheck;
    obsCheck.setObstacles(env.obstacles);

    // Loop through all configurations and calculate joint vertices for robot
    std::pair<std::size_t, std::size_t> cellNums = cspace.size();
    for (std::size_t i = 0; i < cellNums.first; i++)
    {
        for (std::size_t j = 0; j < cellNums.second; j++)
        {
                // Extract midpoint of each configuration cell as theta1 and theta2
            double theta1 = (i + 0.5) * (2*M_PI / cellNums.first);
            double theta2 = (j + 0.5) * (2*M_PI / cellNums.second);

                // Store joint vertices
            std::vector<Eigen::Vector2d> joints;

                // Create configuration state
            amp::ManipulatorState state(2);
            state << theta1, theta2;

            std::pair<std::size_t, std::size_t> cellIdx = cspace.getCellFromPoint(theta1, theta2);

                // Propagate forward kinematics
            joints.push_back(manipulator.getJointLocation(state, 0)); // Base location
            joints.push_back(manipulator.getJointLocation(state, 1)); // End of first link
            joints.push_back(manipulator.getJointLocation(state, 2)); // End of second link

                // Check each line for collisions
            for (int i = 0; i < manipulator.nLinks(); i++)
            {
                    // Extract arm vertices
                Eigen::Vector2d firstVertex = joints[i];
                Eigen::Vector2d secondVertex = joints[i+1];

                    // Test joints for collisions
                //bool firstCollide = obsCheck.evaluatePrimitives(firstVertex, false);
                bool endCollide = obsCheck.evaluatePrimitives(secondVertex, false);

                if (endCollide)
                {
                    cspace(cellIdx.first, cellIdx.second) = true;
                    break;
                }

                    // Test in between points for collisions
                double lineLength = (secondVertex - firstVertex).norm();
                for (double dr = 0; dr < lineLength; dr += 0.05)
                {
                    Eigen::Vector2d candidatePoint = firstVertex + dr * (secondVertex - firstVertex).normalized();

                    bool pointCollide = obsCheck.evaluatePrimitives(candidatePoint, false);

                    if (pointCollide)
                    {
                        cspace(cellIdx.first, cellIdx.second) = true;
                    }
                }
            }
        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer.
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_x1_cells, m_x2_cells, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    // Create an obstacle checker object for primitive evaluation
    ObstacleChecker obsCheck;
    obsCheck.setObstacles(env.obstacles);

    // Loop through all configurations and calculate whether we collide with an obstacle or not
    std::pair<std::size_t, std::size_t> cellNums = cspace.size();
    for (std::size_t i = 0; i < cellNums.first; i++)
    {
        for (std::size_t j = 0; j < cellNums.second; j++)
        {
                // Extract midpoint of each configuration cell as x1 and x2, and assign it as the candidate point to check
            double x1 = (i + 0.5) * ((env.x_max - env.x_min) / cellNums.first) + env.x_min;
            double x2 = (j + 0.5) * ((env.y_max - env.y_min) / cellNums.second) + env.y_min;
            Eigen::Vector2d candidatePoint(x1, x2);

                // Get the corresponding cell index for this point
            std::pair<std::size_t, std::size_t> cellIdx = cspace.getCellFromPoint(x1, x2);

                // Check point for collisions
            bool collided = obsCheck.evaluatePrimitives(candidatePoint, false);

                // Assign cspace collision
            if (collided)
            {
                cspace(i, j) = true;
            }

        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

// Checks whether any agents collide with obstacles in the environment in a centralized planner
bool MultiAgentCSpace::agentEnvCollision(const Eigen::VectorXd& metaState) const
{
    // Copy the obstacle checker object, otherwise obsCheck.checkPointInRadius will throw an error for some reason...
    ObstacleChecker obsCheck2 = obsCheck;

    // Loop over all agents and check if they collide with obstacles
    for (int i = 0; i < numAgents; i++)
    {
        // Extract agent position
        Eigen::Vector2d agentPos = metaState.segment(agentSize*i, agentSize);

        // Loop over obstacles and exit if the robot collides
        for (const auto &obstacle : env.obstacles)
        {
            // Check if the provided agent state is within its radius to an obstacle
            if (obsCheck2.checkPointInRadius(agentPos, obstacle, agentProps[i].radius))
            {
                return true;
            }
        }
    }
    return false;
}

// Checks whether any agents collide with other agents in a centralized planner
bool MultiAgentCSpace::agentAgentCollision(const Eigen::VectorXd& metaState) const
{
    // Choose starting agent
    for (int i = 0; i < numAgents; ++i)
    {
        // Get first agent position
        Eigen::Vector2d pos_i = metaState.segment(i * agentSize, agentSize);

        // Check all other agents
        for (int j = i + 1; j < numAgents; ++j)
        {
            // Get second position
            Eigen::Vector2d pos_j = metaState.segment(j * agentSize, agentSize);

            // Calculate distance between agent centers
            double distance = (pos_i - pos_j).norm();
            double min_distance = agentProps[i].radius + agentProps[j].radius;

            // If agents are closer than the sum of their radii, they've collided
            if (distance < min_distance)
            {
                return true;
            }
        }
    }
    return false;
}

// Checks whether an agent collides with obstacles in the environment
bool DecoupledAgentCSpace::agentEnvCollision(const Eigen::VectorXd &q) const
{
    // Copy the obstacle checker object, otherwise obsCheck.checkPointInRadius will throw an error for some reason...
    ObstacleChecker obsCheck2 = obsCheck;

    // Check if current configuration collides with obstacles in xy space
    Eigen::Vector2d agentPos = q.segment(0, 2);
    for (const auto &obstacle : env.obstacles)
    {
        // Check if the provided agent state is within its radius to an obstacle
        if (obsCheck2.checkPointInRadius(agentPos, obstacle, agentProps[agentIdx].radius))
        {
            return true;
        }
    }
    return false;
}

// Checks whether an agent collides with a previous path at a specific moment in time
bool DecoupledAgentCSpace::agentAgentCollision(const Eigen::VectorXd &q, int time) const
{
    // Loop over existing paths
    for (int i = 0; i < agentPaths.size(); i++)
    {
        // Don't check for collisions with self
        if (i == agentIdx) continue;

        // Don't check for collisions with unplanned agents
        //if (agentPaths[i].waypoints.size() <= 1) continue;

        // Pull out path of the other agent at this timestep, or at the end of its path
        Eigen::VectorXd otherPos;
        if (time >= agentPaths[i].waypoints.size())
        {
            otherPos = agentPaths[i].waypoints.back();
        }
        else
        {
            otherPos = agentPaths[i].waypoints[time];
        }

        // Calculate distance between agent centers
        double dist = (q - otherPos.segment(0,2)).norm();
        double minDist = agentProps[agentIdx].radius + agentProps[i].radius;

        // If distance between centers is less than the sum of their radii, they've collided
        if (dist < minDist)
        {
            return true;
        }
    }
    return false;
}



