//
// Created by ianmf on 9/5/25.
//

#ifndef AMP_TOOLS_OBSTACLECHECKER_H
#define AMP_TOOLS_OBSTACLECHECKER_H

#include "tools/Obstacle.h"
#include "Collision.h"
#include "mLineIntersection.h"

// Function for SAT projection overlap testing
bool projectionsOverlap(const std::vector<Eigen::Vector2d>& shape1, const std::vector<Eigen::Vector2d>& shape2, const Eigen::Vector2d& axis);

// @brief Obstacle checker class for checking whether points collide with a set of obstacles
class ObstacleChecker
{
    public:
            // Default constructor
        ObstacleChecker() = default;

            // Primitive evaluation function - collision checking method. Loops through all obstacles and calculates if
            // all their primitives are negative. If so, we've collided. Assumes all obstacles in obstacleList are
            // convex.
        bool evaluatePrimitives(const Eigen::Vector2d &point, bool leftTurner);

            // Function that checks whether an obstacle is within some radius of a point
        bool checkPointInRadius(const Eigen::Vector2d &point, const amp::Obstacle2D &obstacle, double radius);

            // Propagates the proposed point onto the obstacle's closest boundary
        Collision calcPointOnBoundary(const Eigen::Vector2d &point, const Eigen::Vector2d lastPoint, Collision &collision, bool leftTurner);

            // Calculates the closest distance to a specified obstacle and the gradient from the closest point to the provided point
        std::pair<double, Eigen::Vector2d> calcClosestDistance(const Eigen::Vector2d &point, int obsIdx) const;

            // Runs the separating axis theorem on a rectangle and convex polygon
        bool checkSeparatingAxisTheorem(const std::vector<Eigen::Vector2d> &rectVertices, std::vector<Eigen::Vector2d> &polyVertices, const std::vector<Eigen::Vector2d> &rectAxes);

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