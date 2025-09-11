//
// Created by ianmf on 9/10/25.
//

#ifndef AMP_TOOLS_MLINEINTERSECTION_H
#define AMP_TOOLS_MLINEINTERSECTION_H
#include "tools/Obstacle.h"

/// @brief Structure for storing information relevant to m-line intersections
struct mLineIntersection
{
    amp::Obstacle2D obstacle; // Obstacle that the m-line intersections belong to
    std::vector<Eigen::Vector2d> intersections; // Vector of obstacle intersection points with the m-line
    int obstacleIndex; // Index of the obstacle that the m-line intersections belong to
};

#endif //AMP_TOOLS_MLINEINTERSECTION_H