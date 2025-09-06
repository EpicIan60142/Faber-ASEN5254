#include "Bug1.h"
#include "tools/Obstacle.h"
#include "ObstacleChecker.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug1::plan(const amp::Problem2D& problem)
{

    // Your algorithm solves the problem and generates a path.
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    // Find initial vector to goal
    Eigen::Vector2d r_Gq = problem.q_goal - problem.q_init; // Distance vector from current position to goal
    Eigen::Vector2d rHat_Gq = r_Gq/r_Gq.norm(); // Distance unit vector to goal

    // Declare candidate point
    Eigen::Vector2d candidatePoint;

    // Create an obstacle checker object
    ObstacleChecker obsCheck;
    obsCheck.setObstacles(problem.obstacles);

    // Loop until we reach the goal or need to terminate
    double loopCount = 0;
    while(r_Gq.norm() > epsilon && loopCount < loopTimeout)
    {
            // Calculate candidate point
        candidatePoint = path.waypoints.back() + rHat_Gq * dr;

            // Check for collisions with obstacles
        bool collide = obsCheck.collidesWithPoint(candidatePoint);

            // If the candidate point collides with an obstacle, circumnavigate it
        if (!collide)
        {
                // Add candidate point to path
            path.waypoints.push_back(candidatePoint);
        }
        else
        {
                // Keep track of the closest point to the goal on the obstacle
            Eigen::Vector2d closestPoint = path.waypoints.back();

                // Save the start point for circumnavigation - if the closest point is the start point, we need to
                // terminate.
            const Eigen::Vector2d circumStart = path.waypoints.back();

                // Index of path waypoints where we started circumnavigation
            int startCircumNav = path.waypoints.size();

                // Extract which obstacles collided with the point
            std::vector<Collision> collisions = obsCheck.getCollisions();

                // Loop over collisions
            for (int i = 0; i < collisions.size(); i++)
            {
                    // Extract collision
                Collision collision = collisions[i];

                    // Calculate slope of boundary and create tangent and normal vectors to intersection point
                double mBound = (collision.secondBoundVertex[1] - collision.firstBoundVertex[1]) / (collision.secondBoundVertex[0] - collision.firstBoundVertex[0]); // rise / run

                Eigen::Vector2d boundTangent {1, mBound};
                boundTangent = boundTangent/boundTangent.norm();
                Eigen::Vector2d boundNormal {mBound, -1};
                boundNormal = boundNormal/boundNormal.norm();

                    // Propose a new candidate point
                candidatePoint = collision.intersect + dr * boundNormal;
                path.waypoints.push_back(candidatePoint);







            }

        }

            // Calculate new goal heading vector
        r_Gq = problem.q_goal - path.waypoints.back();
        rHat_Gq = r_Gq/r_Gq.norm();

            // Increase number of loops completed
        loopCount++;
    }

    path.waypoints.push_back(problem.q_goal);

    return path;
}

