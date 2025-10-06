//
// Created by ianmf on 9/24/25.
//

#include "CSpace.h"
#include "ObstacleChecker.h"

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
    std::size_t cell_x = x0Ratio * x0Cells;
    std::size_t cell_y = x1Ratio * x1Cells;

    return {cell_x, cell_y};
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
                cspace(cellIdx.first, cellIdx.second) = true;
            }

        }
    }

    /*
    //std::cout << "Constructing C-space for point agent" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    cspace(0, 0) = true;
    cspace(1, 1) = true;
    cspace(2, 2) = true;
    cspace(3, 3) = true;
    cspace(4, 4) = true;
    cspace(0, 4) = true;
    cspace(1, 3) = true;
    cspace(3, 1) = true;
    cspace(4, 0) = true;
    */

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

