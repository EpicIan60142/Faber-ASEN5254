#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
        // Make sure we have Qstars for each obstacle in the problem
    if (problem.obstacles.size() - this->getQstar().size() > 0)
    {
        while (this->getQstar().size() < problem.obstacles.size())
        {
            this->addQstar(0.25);
        }
    }

        // Create potential function object
    MyPotentialFunction potential{*this, problem};

        // Create path
    amp::Path2D path;

        // Add initial point
    path.waypoints.push_back(problem.q_init);

        // Calculate gradient and follow it back
    Eigen::Vector2d candidatePoint = path.waypoints.back();
    int loopCount = 0;
    while ((problem.q_goal - candidatePoint).norm() > epsilon && loopCount < maxLoopCount)
    {
        Eigen::Vector2d gradient = potential.getGradient(candidatePoint);

            // We haven't reached the goal but the gradient is small, likely a local min. Thus, add a random perturbation to get out
        if (gradient.norm() < epsilon)
        {
            candidatePoint += 0.1*Eigen::Vector2d::Random();
            continue;
        }

        candidatePoint -= alpha*gradient;

        path.waypoints.push_back(candidatePoint);

        loopCount++;
    }

        // Add goal point
    path.waypoints.push_back(problem.q_goal);

    return path;
}
