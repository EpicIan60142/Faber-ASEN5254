//
// Created by ianmf on 9/5/25.
//

#ifndef AMP_TOOLS_COLLISION_H
#define AMP_TOOLS_COLLISION_H

#include "tools/Obstacle.h"
#include <Eigen/Core>

/// @brief Collision structure containing information useful for defining a collision with an object in a workspace
struct Collision
{
    amp::Obstacle2D obstacle; // Obstacle that was collided with
    Eigen::Vector2d centroid; // Centroid of the collided obstacle
    Eigen::Vector2d intersect; // Intersection point
    Eigen::Vector2d firstBoundVertex; // First vertex of the boundary collided with
    Eigen::Vector2d secondBoundVertex; // Second vertex of the boundary collided with
    std::vector<double> primitives; // Array of primitives for the obstacle
    int firstBoundIndex; // Index of first collision boundary vertex
    int obstacleIndex; // Index of the collided obstacle
};

#endif //AMP_TOOLS_COLLISION_H