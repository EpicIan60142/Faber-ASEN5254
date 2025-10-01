#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
        // Create potential function object
    MyPotentialFunction potential{*this, problem};

        // Create path
    amp::Path2D path;

        // Add initial point
    path.waypoints.push_back(problem.q_init);

        // Calculate gradient and follow it back
    Eigen::Vector2d gradient = potential.getGradient(path.waypoints.back());
    int loopCount = 0;
    while (gradient.norm() > epsilon && loopCount < maxLoopCount)
    {
        path.waypoints.push_back(path.waypoints.back() - alpha*gradient);

        gradient = potential.getGradient(path.waypoints.back());

        loopCount++;
    }

        // Add goal point
    path.waypoints.push_back(problem.q_goal);

    return path;
}
