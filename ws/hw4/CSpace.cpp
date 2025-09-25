//
// Created by ianmf on 9/24/25.
//

#include "CSpace.h"
#include "ObstacleChecker.h"

// Override this method for returning whether or not a point is in collision

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

        // Declare cell index objects
    std::size_t cell_x = 0; // x index of cell
    std::size_t cell_y = 0; // y index of cell

        // loop through each cell until we find where the point lies
    for (int i = 0; i < x0Cells; i++) // Loop x0
    {
            // Find x0 bounds for this cell
        double x0CellMin = x0Min + i * (x0Max - x0Min) / x0Cells;
        double x0CellMax = x0Min + (i + 1) * (x0Max - x0Min) / x0Cells;
        for (int j = 0; j < x1Cells; j++) // Loop x1
        {
                // Find x1 bounds for this cell
            double x1CellMin = x1Min + j * (x1Max - x1Min) / x1Cells;
            double x1CellMax = x1Min + (j + 1) * (x1Max - x1Min) / x1Cells;
                // If x0 is within both x0 and x1 bounds, we've found the cell it's in
            if (x0CellMin <= x0 && x0 <= x0CellMax && x1CellMin <= x1 && x1 <= x1CellMax)
            {
                cell_x = i;
                cell_y = j;

                return {cell_x, cell_y};
            }
        }
    }
    // If the point is not in any cell, return the cell that contains the point (x0, x1)
    // (e.g. return (0, 0) if x0 < m_x_min or x1 < m_y_min

    return {cell_x, cell_y};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env)
{
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer.
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    // Create an obstacle checker object for primitive evaluation
    ObstacleChecker obsCheck;
    obsCheck.setObstacles(env.obstacles);

    // Apply minkowski sum to convert workspace obstacles to c-space
    std::vector<amp::Obstacle2D> cSpaceObs; // Create vector for storing c-space obstacles

        // Loop through all configurations and calculate joint vertices for robot
    for (double theta1 = 0; theta1 < 2 * M_PI; theta1 += 0.01)
    {
        for (double theta2 = 0; theta2 < 2*M_PI; theta2 += 0.01)
        {
                // Create configuration state
            amp::ManipulatorState state(2);
            state << theta1, theta2;

                // Propagate forward kinematics
            Eigen::Vector2d baseLocation = manipulator.getJointLocation(state, 0);
            Eigen::Vector2d joint1End = manipulator.getJointLocation(state, 1);
            Eigen::Vector2d joint2End = manipulator.getJointLocation(state, 2);

                // Invert robot (get -A)
            baseLocation *= -1;
            joint1End *= -1;
            joint2End *= -1;

                // Sort robot and obstacle vertices in counter-clockwise order, starting with smallest y coordinate


                // Run Minkowski sum on each obstacle in workspace to get corresponding c space obstacle


        }
    }

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
        // Loop through each grid point and check if any c space obstacle vertices/edges fall inside the grid. If so, mark as a collision




    cspace(1, 3) = true;
    cspace(3, 3) = true;
    cspace(0, 1) = true;
    cspace(1, 0) = true;
    cspace(2, 0) = true;
    cspace(3, 0) = true;
    cspace(4, 1) = true;

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}