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

            // Primitive evaluation function - better collision checking method. Loops through all obtacles and
            // calculates if all their primitives are negative. If so, we've collided. Assumes all obstacles in
            // obstacleList are convex.
        bool evaluatePrimitives(const Eigen::Vector2d &point, bool leftTurner);

        /*
            // Collision checking method. Loops through all obstacles and checks if the provided point collides with
            // any of them. Assumes all obstacles in obstacleList are convex.
        bool collidesWithPoint(const Eigen::Vector2d &point, bool all = true, int obstacleIndex = 0);
        */

            // Projects the proposed point onto the obstacle's closest boundary
        Collision calcPointOnBoundary(const Eigen::Vector2d &point, Collision &collision, bool leftTurner);

            // Calculates the centroid of an obstacle
        Eigen::Vector2d calcCentroid(const amp::Obstacle2D &obstacle, bool leftTurner);

            // Combines obstacles that share vertices or take up the same space. In other words, combines convex
            // obstacles into nonconvex obstacles as needed
        std::vector<amp::Obstacle2D> combineObstacles(std::vector<amp::Obstacle2D> &obstacleList);

            // Getter and setter for list of obstacles
        std::vector<amp::Obstacle2D> getObstacles() { return obstacleList; };
        void setObstacles(const std::vector<amp::Obstacle2D> &newObstacles) { obstacleList = newObstacles; };

            // Getter and setter for list of collision indices
        std::vector<Collision> getCollisions() { return collisionList; };

    private:
        std::vector<amp::Obstacle2D> obstacleList; // List of obstacles being checked
        std::vector<Collision> collisionList; // List of indices of collided obstacles
};

#endif //AMP_TOOLS_OBSTACLECHECKER_H