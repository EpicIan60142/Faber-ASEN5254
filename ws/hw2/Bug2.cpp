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

    // Calculate the m-line
    Eigen::Vector2d mLine = problem.q_goal - problem.q_init;

    // Find initial vector to goal
    Eigen::Vector2d r_Gq = problem.q_goal - problem.q_init; // Distance vector from current position to goal
    Eigen::Vector2d rHat_Gq = r_Gq/r_Gq.norm(); // Distance unit vector to goal

    // Declare candidate point
    Eigen::Vector2d candidatePoint;

    // Create an obstacle checker object and calculate m-line intersections
    ObstacleChecker obsCheck;
    obsCheck.setObstacles(problem.obstacles);
    obsCheck.calcMLineIntersections(mLine, problem, leftTurner);

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
            bool firstObstacle = true;
            bool breakLoop;
            bool atMLine = false;
            bool firstMLine = true;
            Eigen::Vector2d q_first; // First m-line intersection
            Eigen::Vector2d q_closest(9e9,9e9); // Closest m-line intersection to goal
            unsigned long idxClosest = 0;

            // Circumnavigate obstacle until we return to the intersection point
            do
            {
                // Reset loop control variables
                breakLoop = false;
                atMLine = false;

                // Figure out what obstacle we collided with
                obsCheck.evaluatePrimitives(candidatePoint, leftTurner);
                std::vector<Collision> collisions = obsCheck.getCollisions();

                if (collisions.empty())
                {
                    break;
                }

                int obstacleIdx = collisions[0].obstacleIndex;

                // Only deal with the first collision for now, find the intersection point closest to the candidate point
                if (firstObstacle) // If first loop, use the constructed path
                {
                    collisions[0] = obsCheck.calcPointOnBoundary(candidatePoint, path.waypoints.back(), collisions[0], leftTurner);
                }
                else // If multiple loops in, use circumnavigated path
                {
                    collisions[0] = obsCheck.calcPointOnBoundary(candidatePoint, pathCircumNav.waypoints.back(), collisions[0], leftTurner);
                }

                // Propose a point just behind the intersection point to start circumnavigation
                Eigen::Vector2d u_intersect;
                if (firstObstacle)
                {
                    u_intersect = (collisions[0].intersect - path.waypoints.back()).normalized();
                }
                else
                {
                    u_intersect = (collisions[0].intersect - pathCircumNav.waypoints.back()).normalized();
                }

                firstObstacle = false;

                candidatePoint = collisions[0].intersect - dr*dr*u_intersect;

                if (firstMLine)
                {
                    q_first = collisions[0].intersect;
                    firstMLine = false;
                }

                // Add the point to the circumnavigation path and assign it as the start point if not already done
                if (obsCheck.evaluatePrimitives(candidatePoint, leftTurner))
                {
                    break;
                }

                pathCircumNav.waypoints.push_back(candidatePoint);

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

                // Attempt to visit each vertex in order, if we hit another obstacle then restart the loop. If we hit an
                // m-line intersection, then exit the loop
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
                    while ((candidatePoint - secondVertex).norm() > epsilon && !atMLine)
                    {
                        // Add a small distance along the boundary unit vector
                        candidatePoint = pathCircumNav.waypoints.back() + rHat_B * dr;

                        if (obsCheck.evaluatePrimitives(candidatePoint, leftTurner))
                        {
                            breakLoop = true;
                            break;
                        }

                        // Loop through m-line intersections to see if we hit the m-line
                        mLineIntersection mLineIntersections;
                        mLineIntersections = obsCheck.getMLineIntersections()[obstacleIdx];
                        if (!mLineIntersections.intersections.empty())
                        {
                            for (int ii = 0; ii < mLineIntersections.intersections.size(); ii++)
                            {
                                Eigen::Vector2d q_mLine = mLineIntersections.intersections[ii];

                                rHat_Gq = (problem.q_goal - q_mLine).normalized(); // Distance vector from current position to goal

                                // If the candidate point is close to the intersection and moving toward the goal from the intersection won't collide with an obstacle, we've re-encountered the m-line
                                if ((candidatePoint - q_mLine).norm() < epsilon && !obsCheck.evaluatePrimitives(q_mLine + dr*rHat_Gq, leftTurner) \
                                    && pathCircumNav.waypoints.size() > 1 )
                                {

                                    if ((q_mLine - problem.q_goal).norm() < (q_closest - problem.q_goal).norm())
                                    {
                                        q_closest = q_mLine;
                                    }

                                    if ((candidatePoint - problem.q_goal).norm() < (q_first - problem.q_goal).norm())
                                    {
                                        atMLine = true;
                                    }
                                }
                            }
                        }

                        pathCircumNav.waypoints.push_back(candidatePoint);
                    }


                    // Break the for loop if we need to look at a new obstacle or we are at the start point
                    if (breakLoop || atMLine)
                    {
                        break;
                    }

                    // Add a buffer point near the second vertex before moving on to the next boundary
                    Eigen::Vector2d u_vertex = (secondVertex - collisions[0].centroid).normalized();

                    candidatePoint = secondVertex + dr*dr*u_vertex;

                    if (obsCheck.evaluatePrimitives(candidatePoint, leftTurner))
                    {
                        breakLoop = true;
                        break;
                    }

                    pathCircumNav.waypoints.push_back(candidatePoint);
                }

                innerCount++;

            } while (innerCount <= 999 && !atMLine);

            // Append circumnavigated path in progress to overall path
            path.waypoints.insert(path.waypoints.end(), pathCircumNav.waypoints.begin(), pathCircumNav.waypoints.end());

            if (!pathCircumNav.waypoints.empty())
            {
                // Exit the algorithm if we can't move toward goal after circumnavigating
                r_Gq = problem.q_goal - pathCircumNav.waypoints.back(); // Distance vector from current position to goal
                rHat_Gq = r_Gq.normalized(); // Distance unit vector to goal

                candidatePoint = pathCircumNav.waypoints.back() + rHat_Gq * dr;
                if (obsCheck.evaluatePrimitives(candidatePoint, leftTurner))
                {
                    keepPlanning = false;
                    break;
                }
            }

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

