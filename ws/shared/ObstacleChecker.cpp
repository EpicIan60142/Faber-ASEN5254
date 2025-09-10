//
// Created by ianmf on 9/5/25.
//

#include "Eigen/Dense"

#include "ObstacleChecker.h"

/// @brief Function that evaluates primitives for all obstacles in a workspace for a proposed point
/// @param point Proposed point to calculate primitives with respect to
/// @param leftTurner Whether the robot is a left turner or not
/// @return Boolean indicating whether the point is inside an obstacle
bool ObstacleChecker::evaluatePrimitives(const Eigen::Vector2d &point, bool leftTurner)
{
    // Reset collision list
    collisionList.clear();
    bool collided = false;

    // Loop through all obstacles and evaluate their primitives. A point is inside an obstacle if all its primitives
    // are negative.
    for (int i = 0; i < obstacleList.size(); i++)
    {
        // Extract obstacle of interest
        amp::Obstacle2D obstacle = obstacleList[i];

        // Calculate centroid of obstacle
        Eigen::Vector2d centroid = calcCentroid(obstacle, leftTurner);

        // Extract the right type of vertices
        std::vector<Eigen::Vector2d> vertices;
        if (leftTurner)
        {
            vertices = obstacle.verticesCW();
        }
        else
        {
            vertices = obstacle.verticesCCW();
        }

        // Check vertices for duplicates, and remove them if they are duplicate. Already sorted CW or CCW.
        vertices = removeDuplicateVertices(vertices);

        // Loop through all boundaries and calculate their primitives
        const unsigned long numBoundaries = vertices.size();
        std::vector<double> primitives; // Storage array for primitive results
        for (int ii = 0; ii < numBoundaries; ii++)
        {
            // Extract vertices
            Eigen::Vector2d firstVertex = vertices[ii];
            Eigen::Vector2d secondVertex;
            if (ii == numBoundaries - 1)
            {
                secondVertex = vertices[0];
            }
            else
            {
                secondVertex = vertices[ii + 1];
            }

            // Calculate boundary vector
            Eigen::Vector2d r_B = secondVertex - firstVertex;

            // Project point onto boundary vector and translate to global frame - this is the vector to the intersection
            // point on the boundary
            Eigen::Vector2d r_PF = point - firstVertex;
            Eigen::Vector2d r_IF = (r_PF.dot(r_B)/r_B.squaredNorm()) * r_B;
            Eigen::Vector2d r_I = r_IF + firstVertex;

            // Calculate vector to point from intersection
            Eigen::Vector2d r_PI = point - r_I;
            r_PI = r_PI/r_PI.norm();

            // Project vector to centroid from intersection onto r_PI
            Eigen::Vector2d r_CI = centroid - r_I;
            r_CI = r_CI/r_CI.norm();
            Eigen::Vector2d r_proj = (r_CI.dot(r_PI) / r_PI.squaredNorm()) * r_PI;

            // Evaluate primitive for boundary - if vectors are in the same direction, i.e. the point is inside the
            // obstacle, the dot product will be positive so we force it to be negative to match primitive convention
            double dotProd = r_PI.dot(r_proj);
            primitives.push_back(-dotProd);

            // If any primitive is positive, we haven't collided. Thus, terminate the loop
            if (primitives[ii] > 0)
            {
                collided = false;
                break;
            }

            if (primitives[ii] == 0)
            {
                if (vertices.size() == 2) // Check if the intersection is valid if the obstacle was a line
                {
                    // Sort x and y coordinates in increasing order
                    Eigen::Vector2d xCoords = (firstVertex[0] <= secondVertex[0]) ? Eigen::Vector2d{firstVertex[0], secondVertex[0]} : Eigen::Vector2d{secondVertex[0], firstVertex[0]};
                    Eigen::Vector2d yCoords = (firstVertex[1] <= secondVertex[1]) ? Eigen::Vector2d{firstVertex[1], secondVertex[1]} : Eigen::Vector2d{secondVertex[1], firstVertex[1]};

                    // Intersection point isn't valid if it falls outside the two vertices
                    if (!(r_I[0] >= xCoords[0] && r_I[0] <= xCoords[1] && r_I[1] >= yCoords[0] && r_I[1] <= yCoords[1]))
                    {
                        collided = false;
                        break;
                    }
                }
            }

            collided = true;
        }

        // Append obstacle to collision list if we collided
        if (collided)
        {
            Collision newCollision;

            newCollision.obstacle = obstacle;
            newCollision.centroid = centroid;
            newCollision.obstacleIndex = i;
            newCollision.primitives = primitives;

            collisionList.push_back(newCollision);
        }
    }

    return !collisionList.empty();
}

/*
 * OBSOLETE
/// @brief Function that checks all obstacles in a workspace for collisions with a proposed point
/// @param point proposed point to check collisions with
/// @param all Boolean indicating whether to check all obstacles or just one
/// @param obstacleIndex Index of obstacle to check if all is false
bool ObstacleChecker::collidesWithPoint(const Eigen::Vector2d &point, bool all, int obstacleIndex)
{
    if (all)
    {
        // Reset collisions
        collisionList.clear();

        // Loop through all obstacles and check for collisions. A point collides with an obstacle if its distance vector
        // to the obstacle centroid is less than the distance vector from an obstacle's boundary point to the centroid
        for (int i = 0; i < obstacleList.size(); i++)
        {
            // Extract obstacle of interest
            amp::Obstacle2D obstacle = obstacleList[i];

            // Calculate centroid of obstacle
            double xSum = 0; double ySum = 0;
            for (int ii = 0; ii < obstacle.verticesCCW().size(); ii++)
            {
                xSum += obstacle.verticesCCW()[ii][0];
                ySum += obstacle.verticesCCW()[ii][1];
            }
            auto centroid = Eigen::Vector2d(xSum / obstacle.verticesCCW().size(), ySum / obstacle.verticesCCW().size());

            // Calculate vector from proposed point to centroid. This gives a direction to the inside of the obstacle.
            Eigen::Vector2d r_CP = centroid - point;

            // Calculate slope of the current position vector
            double mPoint = point[1] / point[0];

            // Determine if vector will intersect a boundary
            for (int ii = 0; ii < obstacle.verticesCCW().size()-1; ii++)
            {
                // Extract vertices
                Eigen::Vector2d firstVertex = obstacle.verticesCCW()[ii];
                Eigen::Vector2d secondVertex = obstacle.verticesCCW()[ii + 1];

                // Calculate slope of boundary
                double mBound;
                if (secondVertex[0] - firstVertex[0] == 0) // Boundary is vertical
                {
                    if (secondVertex[1] - firstVertex[1] > 0) // Boundary goes from bottom to top
                    {
                        mBound = 99999;
                    }
                    else
                    {
                        mBound = -99999;
                    }
                }
                else if (secondVertex[1] - firstVertex[1] == 0) // Boundary is horizontal
                {
                    mBound = 0;
                }
                else
                {
                    mBound = (secondVertex[1] - firstVertex[1]) / (secondVertex[0] - firstVertex[0]); // rise / run
                }

                // If lines aren't parallel
                if (mPoint != mBound)
                {
                    // Determine where the vector would intersect the boundary
                    Eigen::Matrix2d A {{mBound, -1},
                                        {mPoint, -1} };
                    Eigen::Vector2d b {mBound*firstVertex[0] - firstVertex[1], mPoint*point[0] - point[1]};

                    Eigen::Vector2d intersectRaw = A.colPivHouseholderQr().solve(b);
                        // Round to nearest 10-thousandths place
                    Eigen::Vector2d intersect {round(intersectRaw[0]*10000.0)/10000.0, round(intersectRaw[1]*10000.0)/10000.0};

                    // Sort vertices by increasing x and y coordinate
                    Eigen::Vector2d xVertices, yVertices;

                    if (firstVertex[0] < secondVertex[0])
                    {
                        xVertices = {firstVertex[0], secondVertex[0]};
                    }
                    else
                    {
                        xVertices = {secondVertex[0], firstVertex[0]};
                    }

                    if (firstVertex[1] < secondVertex[1])
                    {
                        yVertices = {firstVertex[1], secondVertex[1]};
                    }
                    else
                    {
                        yVertices = {secondVertex[1], firstVertex[1]};
                    }

                    // If intersection point is inside the boundary, it is valid
                    if (intersect[0] >= xVertices[0] && intersect[0] <= xVertices[1] && intersect[1] >= yVertices[0] && intersect[1] <= yVertices[1])
                    {
                        // Calculate vector from intersection point to centroid
                        Eigen::Vector2d r_CI = centroid - intersect;

                        // Determine if a collision occurred, i.e. the distance of the point to the centroid is less than
                        // the distance from the intersection to the centroid
                        if (r_CP.norm() < r_CI.norm())
                        {
                            Collision newCollision;

                            newCollision.obstacle = obstacle;
                            newCollision.centroid = centroid;
                            newCollision.intersect = intersect;
                            newCollision.firstBoundVertex = firstVertex;
                            newCollision.secondBoundVertex = secondVertex;
                            newCollision.firstBoundIndex = ii;
                            newCollision.obstacleIndex = i;

                            collisionList.push_back(newCollision);
                        }
                    }
                }
            }
        }

        // If we collided with any obstacles, report it
        return collisionList.size() > 0;
    }
    else
    {
        bool collide = false;

        // Extract obstacle of interest
        amp::Obstacle2D obstacle = obstacleList[obstacleIndex];

        // Calculate centroid of obstacle
        double xSum = 0; double ySum = 0;
        for (int ii = 0; ii < obstacle.verticesCCW().size(); ii++)
        {
            xSum += obstacle.verticesCCW()[ii][0];
            ySum += obstacle.verticesCCW()[ii][1];
        }
        auto centroid = Eigen::Vector2d(xSum / obstacle.verticesCCW().size(), ySum / obstacle.verticesCCW().size());

        // Calculate vector from proposed point to centroid and the slope of the vector
        Eigen::Vector2d r_CP = centroid - point;
        double mPoint = r_CP[1] / r_CP[0];

        // Determine if vector will intersect a boundary
        for (int ii = 0; ii < obstacle.verticesCCW().size()-1; ii++)
        {
            // Extract vertices
            Eigen::Vector2d firstVertex = obstacle.verticesCCW()[ii];
            Eigen::Vector2d secondVertex = obstacle.verticesCCW()[ii + 1];

            // Calculate slope of boundary
            double mBound = (secondVertex[1] - firstVertex[1]) / (secondVertex[0] - firstVertex[0]); // rise / run

            // If lines aren't parallel
            if (mPoint != mBound)
            {
                // Determine where the vector would intersect the boundary
                Eigen::Matrix2d A {{mBound, -1},
                                    {mPoint, -1} };
                Eigen::Vector2d b {mBound*firstVertex[0] - firstVertex[1], mPoint*point[0] - point[1]};

                Eigen::Vector2d intersect = A.colPivHouseholderQr().solve(b);

                // Sort vertices by increasing x and y coordinate
                Eigen::Vector2d xVertices, yVertices;

                if (firstVertex[0] < secondVertex[0])
                {
                    xVertices = {firstVertex[0], secondVertex[0]};
                }
                else
                {
                    xVertices = {secondVertex[0], firstVertex[0]};
                }

                if (firstVertex[1] < secondVertex[1])
                {
                    yVertices = {firstVertex[1], secondVertex[1]};
                }
                else
                {
                    yVertices = {secondVertex[1], firstVertex[1]};
                }

                // If intersection point is inside the boundary, it is valid
                if (intersect[0] >= xVertices[0] && intersect[0] <= xVertices[1] && intersect[1] >= yVertices[0] && intersect[1] <= yVertices[1])
                {
                    // Calculate vector from intersection point to centroid
                    Eigen::Vector2d r_CI = centroid - intersect;

                    // Determine if a collision occurred, i.e. the distance of the point to the centroid is less than
                    // the distance from the intersection to the centroid
                    if (r_CP.norm() < r_CI.norm())
                    {
                        collide = true;
                    }
                }
            }
        }

        return collide;
    }
}
*/

/// @brief function that determines which boundary a point is closest to and its propagated point on that boundary
/// @param point Proposed point to propagate into obstacle
/// @param lastPoint The point before the proposed point in the path sequence
/// @param collision Collision structure containing the obstacle that was collided with
/// @param leftTurner Whether the robot is a left turner or not
/// @return Collision object with intersection point and vertices corresponding to the boundary hit
Collision ObstacleChecker::calcPointOnBoundary(const Eigen::Vector2d& point, const Eigen::Vector2d lastPoint, Collision &collision, bool leftTurner)
{
    // Extract obstacle and centroid
    amp::Obstacle2D obstacle = collision.obstacle;

    // Choose correct vertices
    std::vector<Eigen::Vector2d> vertices;
    if (leftTurner)
    {
        vertices = obstacle.verticesCW();
    }
    else
    {
        vertices = obstacle.verticesCCW();
    }

    // Remove duplicate vertices
    vertices = removeDuplicateVertices(vertices);

    // Loop through all boundaries to find which one is closest to the proposed point
    Eigen::Vector2d closestIntersect = 9999*point; // Arbitrarily large vector
    const unsigned long numBoundaries = vertices.size();
    for (int i = 0; i < numBoundaries; i++)
    {
        // Extract vertices
        Eigen::Vector2d firstVertex = vertices[i];
        Eigen::Vector2d secondVertex;
        if (i == numBoundaries - 1)
        {
            secondVertex = vertices[0];
        }
        else
        {
            secondVertex = vertices[i + 1];
        }

        // Calculate boundary vector
        Eigen::Vector2d r_B = secondVertex - firstVertex;

        // Calculate direction of propagation
        Eigen::Vector2d r_prop = point - lastPoint;

        // Propagate point onto boundary vector and translate to global frame - this is the vector to the intersection
        // point on the boundary
        Eigen::Matrix2d A {{r_prop[0], -r_B[0]}, {r_prop[1], -r_B[1]}};
        Eigen::Vector2d b {firstVertex[0] - lastPoint[0], firstVertex[1] - lastPoint[1]};
        Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);

        Eigen::Vector2d r_I = x[0]*r_prop + lastPoint;

        // Sort x and y coordinates in increasing order
        Eigen::Vector2d xCoords = (firstVertex[0] <= secondVertex[0]) ? Eigen::Vector2d{firstVertex[0], secondVertex[0]} : Eigen::Vector2d{secondVertex[0], firstVertex[0]};
        Eigen::Vector2d yCoords = (firstVertex[1] <= secondVertex[1]) ? Eigen::Vector2d{firstVertex[1], secondVertex[1]} : Eigen::Vector2d{secondVertex[1], firstVertex[1]};

        // Intersection point isn't valid if it falls outside the two vertices
        if (!(r_I[0] >= xCoords[0] && r_I[0] <= xCoords[1] && r_I[1] >= yCoords[0] && r_I[1] <= yCoords[1]))
        {
            continue;
        }

        // Calculate vector from proposed intersect to proposed point
        Eigen::Vector2d r_PI = point - r_I;

        if (r_PI.norm() <= (point - closestIntersect).norm())
        {
            collision.firstBoundVertex = firstVertex;
            collision.secondBoundVertex = secondVertex;
            collision.firstBoundIndex = i;
            collision.intersect = r_I;
            closestIntersect = r_I;
        }
    }

    return collision;
}

/// @brief Function that calculates the centroid of a provided obstacle
/// @param obstacle Obstacle to calculate centroid of
/// @param leftTurner Whether the robot is a left turner or not
/// @return Vector to the centroid of the obstacle
Eigen::Vector2d ObstacleChecker::calcCentroid(const amp::Obstacle2D& obstacle, bool leftTurner)
{
    // Choose correct vertices
    std::vector<Eigen::Vector2d> vertices;
    if (leftTurner)
    {
        vertices = obstacle.verticesCW();
    }
    else
    {
        vertices = obstacle.verticesCCW();
    }

    // Remove duplicate vertices
    vertices = removeDuplicateVertices(vertices);

    // Calculate centroid
    double xSum = 0; double ySum = 0;
    for (int i = 0; i < vertices.size(); i++)
    {
        xSum += vertices[i][0];
        ySum += vertices[i][1];
    }
    auto centroid = Eigen::Vector2d(xSum / vertices.size(), ySum / vertices.size());

    return centroid;
}

std::vector<Eigen::Vector2d> ObstacleChecker::removeDuplicateVertices(const std::vector<Eigen::Vector2d> &vertices)
{
    std::vector<int> duplicateIdx;
    for (int ii = 0; ii < vertices.size()-1; ii++)
    {
        if (vertices[ii] == vertices[ii+1])
        {
            duplicateIdx.push_back(ii+1);
        }
    }

    std::vector<Eigen::Vector2d> newVertices = vertices;
    if (!duplicateIdx.empty() > 0)
    {
        for (unsigned long ii = 0; ii < duplicateIdx.size(); ii++)
        {
            newVertices.erase(newVertices.begin() + duplicateIdx[ii]);
        };
    }

    return newVertices;
}



