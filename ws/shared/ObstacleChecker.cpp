//
// Created by ianmf on 9/5/25.
//

#include "Eigen/Dense"

#include "ObstacleChecker.h"

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

            // Calculate vector from proposed point to centroid and the slope of the vector
            Eigen::Vector2d r_CP = centroid - point;

            // Skip the obstacle if it's too far away
            if (r_CP.norm() > distThreshold)
            {
                continue;
            }

            double mPoint = r_CP[1] / r_CP[0];

            // Determine if vector will intersect a boundary
            for (int ii = 0; ii < obstacle.verticesCCW().size()-1; ii++)
            {
                // Extract vertices
                Eigen::Vector2d firstVertex = obstacle.verticesCCW()[ii];
                Eigen::Vector2d secondVertex = obstacle.verticesCCW()[ii + 1];

                // Calculate slope of boundary
                double mBound;
                if (secondVertex[0] - firstVertex[0] == 0)
                {
                    mBound = 9999;
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

Eigen::Vector2d ObstacleChecker::calcPointOnBoundary(const Eigen::Vector2d& point, const Collision &collision)
{
    // Extract obstacle and centroid
    amp::Obstacle2D obstacle = collision.obstacle;
    Eigen::Vector2d centroid = collision.centroid;

    // Extract vertices
    Eigen::Vector2d firstVertex = collision.firstBoundVertex;
    Eigen::Vector2d secondVertex = collision.secondBoundVertex;

    // Calculate vector from proposed point to centroid and the slope of the vector
    Eigen::Vector2d r_CP = centroid - point;
    double mPoint = r_CP[1] / r_CP[0];

    // Calculate slope of boundary
    double mBound = (secondVertex[1] - firstVertex[1]) / (secondVertex[0] - firstVertex[0]); // rise / run

    // Determine where the vector would intersect the boundary
    const Eigen::Matrix2d A {{mBound, -1},
                        {mPoint, -1} };
    const Eigen::Vector2d b {mBound*firstVertex[0] - firstVertex[1], mPoint*point[0] - point[1]};

    Eigen::Vector2d intersect = A.colPivHouseholderQr().solve(b);

    return intersect;
}

