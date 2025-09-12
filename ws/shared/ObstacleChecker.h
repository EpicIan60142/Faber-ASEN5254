//
// Created by ianmf on 9/5/25.
//

#ifndef AMP_TOOLS_OBSTACLECHECKER_H
#define AMP_TOOLS_OBSTACLECHECKER_H

#include "tools/Obstacle.h"
#include "Collision.h"
#include "mLineIntersection.h"

// @brief Obstacle checker class for checking whether points collide with a set of obstacles
class ObstacleChecker
{
    public:
            // Default constructor
        ObstacleChecker() = default;

            // Primitive evaluation function - collision checking method. Loops through all obtacles and calculates if
            // all their primitives are negative. If so, we've collided. Assumes all obstacles in obstacleList are
            // convex.
        bool evaluatePrimitives(const Eigen::Vector2d &point, bool leftTurner);

        /*
            // Obsolete collision checking method. Loops through all obstacles and checks if the provided point collides
            // with any of them. Assumes all obstacles in obstacleList are convex.
        bool collidesWithPoint(const Eigen::Vector2d &point, bool all = true, int obstacleIndex = 0);
        */

            // Propagates the proposed point onto the obstacle's closest boundary
        Collision calcPointOnBoundary(const Eigen::Vector2d &point, const Eigen::Vector2d lastPoint, Collision &collision, bool leftTurner);

            // Calculates the centroid of an obstacle
        Eigen::Vector2d calcCentroid(const amp::Obstacle2D &obstacle, bool leftTurner);

            // Removes duplicate vertices from an obstacle
        std::vector<Eigen::Vector2d> removeDuplicateVertices(const std::vector<Eigen::Vector2d> &vertices);

            // Calculates m-line intersections for the obstacles in obstacleList
        std::vector<mLineIntersection> calcMLineIntersections(const Eigen::Vector2d &mLine, const amp::Problem2D &problem, bool leftTurner);

            // Getter and setter for list of obstacles
        std::vector<amp::Obstacle2D> getObstacles() { return obstacleList; };
        void setObstacles(const std::vector<amp::Obstacle2D> &newObstacles) { obstacleList = newObstacles; };

            // Getter for list of collision indices
        std::vector<Collision> getCollisions() { return collisionList; };

            // Getter for list of m-line intersections
        std::vector<mLineIntersection> getMLineIntersections() { return mLineIntersections; };

    private:
        std::vector<amp::Obstacle2D> obstacleList; // List of obstacles being checked
        std::vector<Collision> collisionList; // List of indices of collided obstacles
        std::vector<mLineIntersection> mLineIntersections; // List of m-line intersections for each obstacle
};

#endif //AMP_TOOLS_OBSTACLECHECKER_H