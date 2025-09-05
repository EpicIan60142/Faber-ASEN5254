#include "Bug1.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug1::plan(const amp::Problem2D& problem)
{

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    // Find initial vector to goal
    Eigen::Vector2d r_Gq = problem.q_goal - problem.q_init;; // Distance vector from current position to goal
    Eigen::Vector2d rHat_Gq = r_Gq/r_Gq.norm(); // Distance unit vector to goal

    // Declare candidate point
    Eigen::Vector2d candidatePoint;

    // Loop until we reach the goal
    while(r_Gq.norm() > epsilon)
    {





    }


    path.waypoints.push_back(problem.q_goal);

    return path;
}

