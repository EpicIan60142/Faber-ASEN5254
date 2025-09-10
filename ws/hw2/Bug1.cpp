#include "Bug1.h"
#include "tools/Obstacle.h"
#include "ObstacleChecker.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug1::plan(const amp::Problem2D& problem)
{
    /*
    // Report the problem being solved
    std::cout << "Solving the following problem with Bug 1:" << std::endl;
    problem.print();
    */

    // Declare empty path
    amp::Path2D path;

    // Initialize with the initial point
    path.waypoints.push_back(problem.q_init);

    Eigen::Vector2d qH_i = {9e9, 9e9};

    // Find initial vector to goal
    Eigen::Vector2d r_Gq = problem.q_goal - problem.q_init; // Distance vector from current position to goal
    Eigen::Vector2d rHat_Gq = r_Gq/r_Gq.norm(); // Distance unit vector to goal

    // Declare candidate point
    Eigen::Vector2d candidatePoint;

    // Create an obstacle checker object
    ObstacleChecker obsCheck;
    obsCheck.setObstacles(problem.obstacles);

    // Initialize loop control variables
    bool keepPlanning = true; // Overall algorithm loop control
    bool collided = false; // Boolean determining whether we move toward the goal or circumnavigate
    double loopCount = 0; // Loop counter for early termination
    Eigen::Vector2d q_first; // First path point encountered on an obstacle
    Eigen::Vector2d q_closest(9e9, 9e9); // Closest path point to the goal after circumnavigating an obstacle
    bool firstObstacle; // Boolean for resetting q_first
    amp::Path2D pathCircumNav; // Path for circumnavigating obstacles


    // Run algorithm
    while (keepPlanning && loopCount < loopTimeout)
    {
        // Move toward goal
        while (r_Gq.norm() > epsilon && !collided)
        {
            firstObstacle = true;

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
            // Boolean for tracking collisions when circumnavigating
            bool secondCollision = false;

            // We collided with something, need to circumnavigate it
            // Pull out calculated collisions
            obsCheck.evaluatePrimitives(candidatePoint, leftTurner);
            std::vector<Collision> collisions = obsCheck.getCollisions();

            // Find intersection points closest to the point
            obsCheck.evaluatePrimitives(candidatePoint, leftTurner);
            for (int i = 0; i < collisions.size(); i++)
            {
                collisions[i] = obsCheck.calcPointOnBoundary(candidatePoint, path.waypoints.back(), collisions[i], leftTurner);
            }

            // Define a new path for circumnavigation
            amp::Path2D pathCircumNav;

            // Add a point just behind the intersection point as the first point in the circumnavigation path
            Eigen::Vector2d u_intersect = collisions[0].intersect - path.waypoints.back();
            u_intersect = u_intersect/u_intersect.norm();

            candidatePoint = collisions[0].intersect - dr*dr*u_intersect;
            if (!obsCheck.evaluatePrimitives(candidatePoint, leftTurner))
            {
                pathCircumNav.waypoints.push_back(candidatePoint);
            }
            else
            {
                pathCircumNav.waypoints.push_back(path.waypoints.back());
            }

            // Keep track of the first point we encountered
            if (firstObstacle)
            {
                q_first = pathCircumNav.waypoints.back();
                firstObstacle = false;
            }


            // Sort vertices depending on the boundary that was intersected
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

            // Check vertices for duplicates, and remove them if they are duplicate. Already sorted CW or CCW.
            vertices = obsCheck.removeDuplicateVertices(vertices);

            // Circumnavigate the obstacle until we get back to the first point
            // q_closest = pathCircumNav.waypoints.back();
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
                candidatePoint = pathCircumNav.waypoints.back();
                while ((candidatePoint - secondVertex).norm() > epsilon)
                {
                    candidatePoint = pathCircumNav.waypoints.back() + rHat_B * dr;

                    if (obsCheck.evaluatePrimitives(candidatePoint, leftTurner))
                    {
                        secondCollision = true;
                        break;
                    }

                    pathCircumNav.waypoints.push_back(candidatePoint);

                    if ((pathCircumNav.waypoints.back() - problem.q_goal).norm() < (q_closest - problem.q_goal).norm())
                    {
                        q_closest = pathCircumNav.waypoints.back();
                        idxClosest = pathCircumNav.waypoints.size();
                    }
                }

                if (!secondCollision)
                {
                    Eigen::Vector2d u_vertex = secondVertex - collisions[0].centroid;
                    u_vertex = u_vertex/u_vertex.norm();

                    candidatePoint = secondVertex + dr*dr*u_vertex;
                    if (!obsCheck.evaluatePrimitives(candidatePoint, leftTurner))
                    {
                        pathCircumNav.waypoints.push_back(secondVertex + dr*dr*u_vertex);
                    }
                    else
                    {
                        secondCollision = true;
                        break;
                    }
                }
                else
                {
                    break;
                }
            }

            // Finish out traversing the first boundary only if the obstacle isn't a line and we didn't collide again
            if (vertices.size() > 2 && !secondCollision)
            {
                // Move along first boundary until we hit the first point
                Eigen::Vector2d firstVertex = vertices[0];
                Eigen::Vector2d secondVertex = vertices[1];

                // Determine boundary tangent vector
                Eigen::Vector2d rHat_B = (secondVertex - firstVertex).normalized();

                // Add points following the boundary until we hit the first point
                do
                {
                    candidatePoint = pathCircumNav.waypoints.back() + rHat_B * dr;

                    if (!obsCheck.evaluatePrimitives(candidatePoint, leftTurner))
                    {
                        pathCircumNav.waypoints.push_back(candidatePoint);
                    }
                    else
                    {
                        secondCollision = true;
                        break;
                    }

                    if ((pathCircumNav.waypoints.back() - problem.q_goal).norm() < (q_closest - problem.q_goal).norm())
                    {
                        q_closest = pathCircumNav.waypoints.back();
                        idxClosest = pathCircumNav.waypoints.size();
                    }
                } while ((candidatePoint - q_first).norm() > epsilon);

                if ((candidatePoint - q_first).norm() <= epsilon)
                {
                    pathCircumNav.waypoints.push_back(q_first);
                }
            }

            if (!secondCollision)
            {
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

                // Update the vector to the goal
                r_Gq = problem.q_goal - path.waypoints.back(); // Distance vector from current position to goal
                rHat_Gq = r_Gq/r_Gq.norm(); // Distance unit vector to goal

                collided = false;
            }

            // Append circumnavigated path in progress to path
            path.waypoints.insert(path.waypoints.end(), pathCircumNav.waypoints.begin(), pathCircumNav.waypoints.end());
        }

        loopCount++;
    }

    // Add final point to path
    //path.waypoints.push_back(problem.q_goal);

    return path;
}

