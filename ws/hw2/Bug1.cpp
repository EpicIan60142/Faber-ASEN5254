#include "Bug1.h"
#include "tools/Obstacle.h"
#include "ObstacleChecker.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug1::plan(const amp::Problem2D& problem)
{
    // Declare empty path
    amp::Path2D path;

    // Initialize with the initial point
    path.waypoints.push_back(problem.q_init);

    // Find initial vector to goal
    Eigen::Vector2d r_Gq = problem.q_goal - problem.q_init; // Distance vector from current position to goal
    Eigen::Vector2d rHat_Gq = r_Gq/r_Gq.norm(); // Distance unit vector to goal

    // Declare candidate point
    Eigen::Vector2d candidatePoint;

    // Create an obstacle checker object
    ObstacleChecker obsCheck;
    obsCheck.setObstacles(problem.obstacles);

    // Initialize loop control variables
    bool keepPlanning = true;
    bool collided = false;
    double loopCount = 0;

    // Run algorithm
    while (keepPlanning && loopCount < loopTimeout)
    {
        // Move toward goal
        while (r_Gq.norm() > epsilon && !collided)
        {
            // Propose a candidate point closer to the goal
            candidatePoint = path.waypoints.back() + rHat_Gq * dr;

            // Check for collisions
            collided = obsCheck.evaluatePrimitives(candidatePoint, leftTurner);

            // If we didn't collide with anything, add the point to the path
            if (!collided)
            {
                path.waypoints.push_back(candidatePoint);
            }
            else
            {
                break;
            }

            // Update the vector to the goal
            r_Gq = problem.q_goal - path.waypoints.back(); // Distance vector from current position to goal
            rHat_Gq = r_Gq/r_Gq.norm(); // Distance unit vector to goal

            // Keep looping until we reach the goal
            if (r_Gq.norm() < epsilon)
            {
                keepPlanning = false;
                // Add final point to path
                path.waypoints.push_back(problem.q_goal);
                break;
            }
        }

        // Circumnavigate
        if (keepPlanning)
        {
            // We collided with something, need to circumnavigate it
            // Pull out calculated collisions
            std::vector<Collision> collisions = obsCheck.getCollisions();

            // Find intersection points closest to the point
            for (int i = 0; i < collisions.size(); i++)
            {
                collisions[i] = obsCheck.calcPointOnBoundary(candidatePoint, collisions[i], leftTurner);
            }

            // Define a new path for circumnavigation
            amp::Path2D pathCircumNav;

            // Add the intersection point as the first point in the circumnavigation path
            pathCircumNav.waypoints.push_back(collisions[0].intersect);

            // Keep track of the first point we encountered
            Eigen::Vector2d q_first = pathCircumNav.waypoints.back();

            // Add a point to the left or right of the point to start
            std::vector<Eigen::Vector2d> vertices;
            if (leftTurner)
            {
                vertices = collisions[0].obstacle.verticesCW();

                // Sort vertices according to where the intersection occurred
                int firstBoundIdx = collisions[0].firstBoundIndex;

                for (int i = 0; i < vertices.size(); i++)
                {
                    vertices[i] = collisions[0].obstacle.verticesCW()[(firstBoundIdx + i)%vertices.size()];
                }

            }
            else
            {
                vertices = collisions[0].obstacle.verticesCCW();

                // Sort vertices according to where the intersection occurred
                int firstBoundIdx = collisions[0].firstBoundIndex;

                for (int i = 0; i < vertices.size(); i++)
                {
                    vertices[i] = collisions[0].obstacle.verticesCCW()[(firstBoundIdx + i)%vertices.size()];
                }
            }

            // Circumnavigate the obstacle until we get back to the first point
            Eigen::Vector2d q_closest = pathCircumNav.waypoints.back();
            unsigned long idxClosest = pathCircumNav.waypoints.size();
            // Loop over each boundary
            for (int i = 0; i < vertices.size(); i++)
            {
                Eigen::Vector2d firstVertex = vertices[i];
                Eigen::Vector2d secondVertex;
                if (i == vertices.size() - 1)
                {
                    secondVertex = vertices[0];
                }
                else
                {
                    secondVertex = vertices[i + 1];
                }

                // Determine boundary tangent vector
                Eigen::Vector2d rHat_B = (secondVertex - firstVertex).normalized();

                // Add points following the boundary until we hit the second vertex
                while ((candidatePoint - secondVertex).norm() > epsilon)
                {
                    candidatePoint = pathCircumNav.waypoints.back() + rHat_B * dr;
                    pathCircumNav.waypoints.push_back(candidatePoint);

                    if ((pathCircumNav.waypoints.back() - problem.q_goal).norm() < (q_closest - problem.q_goal).norm())
                    {
                        q_closest = pathCircumNav.waypoints.back();
                        idxClosest = pathCircumNav.waypoints.size();
                    }
                }
                pathCircumNav.waypoints.push_back(secondVertex);
            }

            // Move along first boundary until we hit the first point
            Eigen::Vector2d firstVertex = vertices[0];
            Eigen::Vector2d secondVertex = vertices[1];

            // Determine boundary tangent vector
            Eigen::Vector2d rHat_B = (secondVertex - firstVertex).normalized();

            // Add points following the boundary until we hit the second vertex
            while ((candidatePoint - q_first).norm() > epsilon)
            {
                candidatePoint = pathCircumNav.waypoints.back() + rHat_B * dr;
                pathCircumNav.waypoints.push_back(candidatePoint);

                if ((pathCircumNav.waypoints.back() - problem.q_goal).norm() < (q_closest - problem.q_goal).norm())
                {
                    q_closest = pathCircumNav.waypoints.back();
                    idxClosest = pathCircumNav.waypoints.size();
                }
            }
            pathCircumNav.waypoints.push_back(q_first);

            // Go back to the closest point if the distance changed
            if ((q_first - q_closest).norm() > epsilon)
            {
                for (int i = pathCircumNav.waypoints.size()-1; i >= idxClosest; i--)
                {
                    pathCircumNav.waypoints.push_back(pathCircumNav.waypoints[i]);
                }
            }
            else
            {
                keepPlanning = false;
                break;
            }

            // Append circumnavigated path to path
            path.waypoints.insert(path.waypoints.end(), pathCircumNav.waypoints.begin(), pathCircumNav.waypoints.end());

            collided = false;
        }

        loopCount++;
    }

    // Add final point to path
    //path.waypoints.push_back(problem.q_goal);

    return path;
}

