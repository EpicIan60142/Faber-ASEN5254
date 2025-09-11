#include "Bug2.h"
#include "tools/Obstacle.h"
#include "ObstacleChecker.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug2::plan(const amp::Problem2D& problem)
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
    double innerCount = 0;

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
            amp::Path2D pathCircumNav;
            Eigen::Vector2d q_first;
            Eigen::Vector2d q_closest = path.waypoints.back();
            bool firstPointAssigned = false;
            bool breakLoop;
            bool atStart = false;
            unsigned long idxClosest = 0;

            // Circumnavigate obstacle until we return to the intersection point
            do
            {
                // Reset loop control variables
                breakLoop = false;

                // Figure out what obstacle we collided with
                obsCheck.evaluatePrimitives(candidatePoint, leftTurner);
                std::vector<Collision> collisions = obsCheck.getCollisions();

                if (collisions.empty())
                {
                    break;
                }

                // Only deal with the first collision for now, find the intersection point closest to the candidate point
                if (!firstPointAssigned) // If first loop, use the constructed path
                {
                    collisions[0] = obsCheck.calcPointOnBoundary(candidatePoint, path.waypoints.back(), collisions[0], leftTurner);
                }
                else // If multiple loops in, use circumnavigated path
                {
                    collisions[0] = obsCheck.calcPointOnBoundary(candidatePoint, pathCircumNav.waypoints.back(), collisions[0], leftTurner);
                }

                // Propose a point just behind the intersection point to start circumnavigation
                Eigen::Vector2d u_intersect;
                if (!firstPointAssigned)
                {
                    u_intersect = (collisions[0].intersect - path.waypoints.back()).normalized();
                }
                else
                {
                    u_intersect = (collisions[0].intersect - pathCircumNav.waypoints.back()).normalized();
                }

                candidatePoint = collisions[0].intersect - dr*dr*u_intersect;

                // Add the point to the circumnavigation path and assign it as the start point if not already done
                if (obsCheck.evaluatePrimitives(candidatePoint, leftTurner))
                {
                    break;
                }

                pathCircumNav.waypoints.push_back(candidatePoint);
                if (!firstPointAssigned)
                {
                    q_first = candidatePoint;
                    firstPointAssigned = true;
                }

                // Sort vertices according to the boundary the candidate point intersected
                std::vector<Eigen::Vector2d> vertices;
                if (leftTurner)
                {
                    // If turning left, want to follow vertices in clockwise direction
                    vertices = collisions[0].obstacle.verticesCW();

                    // Extract what the first vertex of the intersected boundary is
                    int firstVertexIdx = collisions[0].firstBoundIndex;

                    // Sort vertices in order of CW traversal
                    for (int i = 0; i < vertices.size(); i++)
                    {
                        vertices[i] = collisions[0].obstacle.verticesCW()[(firstVertexIdx + i) % vertices.size()];
                    }

                }
                else
                {
                    // If turning right, want to follow vertices in counter clockwise direction
                    vertices = collisions[0].obstacle.verticesCCW();

                    // Extract what the first vertex of the intersected boundary is
                    int firstVertexIdx = collisions[0].firstBoundIndex;

                    // Sort vertices in order of CCW traversal
                    for (int i = 0; i < vertices.size(); i++)
                    {
                        vertices[i] = collisions[0].obstacle.verticesCCW()[(firstVertexIdx + i) % vertices.size()];
                    }
                }

                // Check vertices for duplicates and remove them if they exist
                vertices = obsCheck.removeDuplicateVertices(vertices);

                // Attempt to visit each vertex in order, if we hit another obstacle or q_first then restart the loop
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

                    // Determine boundary unit vector
                    Eigen::Vector2d rHat_B = (secondVertex - firstVertex).normalized();

                    // Propose candidate point following the boundary towards the second vertex
                    candidatePoint = pathCircumNav.waypoints.back();
                    while ((candidatePoint - secondVertex).norm() > epsilon)
                    {
                        // Add a small distance along the boundary unit vector
                        candidatePoint = pathCircumNav.waypoints.back() + rHat_B * dr;

                        if (obsCheck.evaluatePrimitives(candidatePoint, leftTurner))
                        {
                            breakLoop = true;
                            break;
                        }

                        if ((candidatePoint - q_first).norm() < epsilon && pathCircumNav.waypoints.size() > 1)
                        {
                            atStart = true;
                        }

                        pathCircumNav.waypoints.push_back(candidatePoint);

                        if ((pathCircumNav.waypoints.back() - problem.q_goal).norm() < (q_closest - problem.q_goal).norm())
                        {
                            q_closest = pathCircumNav.waypoints.back();
                            idxClosest = pathCircumNav.waypoints.size();
                        }
                    }


                    // Break the for loop if we need to look at a new obstacle or we are at the start point
                    if (breakLoop || atStart)
                    {
                        break;
                    }

                    // Add a buffer point near the second vertex before moving on to the next boundary
                    Eigen::Vector2d u_vertex = (secondVertex - collisions[0].centroid).normalized();

                    candidatePoint = secondVertex + dr*dr*u_vertex;

                    if (obsCheck.evaluatePrimitives(candidatePoint, leftTurner) && (candidatePoint - q_first).norm() > epsilon)
                    {
                        breakLoop = true;
                        break;
                    }

                    pathCircumNav.waypoints.push_back(candidatePoint);
                }

                if (!breakLoop)
                {
                    // Move back towards q_first once all boundaries have been traversed
                    Eigen::Vector2d u_qFirst = (q_first - pathCircumNav.waypoints.back()).normalized();

                    candidatePoint = pathCircumNav.waypoints.back();
                    while ((candidatePoint - q_first).norm() > epsilon)
                    {
                        candidatePoint = pathCircumNav.waypoints.back() + dr*u_qFirst;

                        if (obsCheck.evaluatePrimitives(candidatePoint, leftTurner))
                        {
                            break;
                        }

                        pathCircumNav.waypoints.push_back(candidatePoint);
                    }
                }

                innerCount++;

            } while (innerCount <= 999 && !atStart);

            // Add q_first as the last point in the circumnavigation path
            //pathCircumNav.waypoints.push_back(q_first);

            // Move back towards the closest point if it's different from q_first
            if ((q_first - q_closest).norm() > epsilon && pathCircumNav.waypoints.size() > 1 && q_closest != path.waypoints.back())
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

            // Append circumnavigated path in progress to overall path
            path.waypoints.insert(path.waypoints.end(), pathCircumNav.waypoints.begin(), pathCircumNav.waypoints.end());

            // Reset collided booleans and update pointing vectors
            collided = false;

            r_Gq = problem.q_goal - path.waypoints.back(); // Distance vector from current position to goal
            rHat_Gq = r_Gq.normalized(); // Distance unit vector to goal

        }

        loopCount++;
        //std::cout << "Loop count: " << loopCount << std::endl;
    }

    // Add final point to path
    path.waypoints.push_back(problem.q_goal);

    return path;
}

