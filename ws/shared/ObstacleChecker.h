//
// Created by ianmf on 9/5/25.
//

#ifndef AMP_TOOLS_OBSTACLECHECKER_H
#define AMP_TOOLS_OBSTACLECHECKER_H

#include "tools/Obstacle.h"
#include "Collision.h"

// @brief Obstacle checker class for checking whether points collide with a set of obstacles
class ObstacleChecker : amp::Obstacle2D
{
    public:
            // Default constructor
        ObstacleChecker() = default;

            // Collision checking method. Loops through all obstacles and checks if the provided point collides with
            // any of them. Assumes all obstacles in obstacleList are convex.
        bool collidesWithPoint(const Eigen::Vector2d &point, bool all = true, int obstacleIndex = 0);

            // Projects the proposed point onto the obstacle boundary
        Eigen::Vector2d calcPointOnBoundary(const Eigen::Vector2d &point, const Collision &collision);

            // Getter and setter for list of obstacles
        std::vector<amp::Obstacle2D> getObstacles() { return obstacleList; };
        void setObstacles(const std::vector<amp::Obstacle2D> &newObstacles) { obstacleList = newObstacles; };

            // Getter and setter for list of collision indices
        std::vector<Collision> getCollisions() { return collisionList; };

    private:
        std::vector<amp::Obstacle2D> obstacleList; // List of obstacles being checked
        std::vector<Collision> collisionList; // List of indices of collided obstacles
        double distThreshold = 50; // Threshold beyond which to ignore obtstacles
};

#endif //AMP_TOOLS_OBSTACLECHECKER_H