//
// Created by ianmf on 9/5/25.
//

#include "Eigen/Dense"
#include "tools/Environment.h"

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

bool ObstacleChecker::checkPointInRadius(const Eigen::Vector2d &point, const amp::Obstacle2D &obstacle, double radius)
{
    // Extract the obstacle vertices
    std::vector<Eigen::Vector2d> vertices;
    vertices = obstacle.verticesCCW();

    // Check vertices for duplicates, and remove them if they are duplicate. Already sorted CW or CCW.
    vertices = removeDuplicateVertices(vertices);

    // Start obstacle min distance to robot as unbounded
    double minDist = std::numeric_limits<double>::max();

    // Loop through all boundaries and calculate their primitives
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

        // Project point onto boundary vector and translate to global frame - this is the vector to the intersection
        // point on the boundary
        Eigen::Vector2d r_PF = point - firstVertex;
        double t = r_PF.dot(r_B)/r_B.squaredNorm();
        t = std::max(0.0, std::min(1.0,t)); // Ensure intersection point is always on the boundary (between vertices or on a vertex)
        Eigen::Vector2d r_IF = t * r_B;
        Eigen::Vector2d r_I = r_IF + firstVertex;

        // Calculate vector to point from intersection
        Eigen::Vector2d r_PI = point - r_I;

        // Calculate distance to the robot center
        double dist = r_PI.norm();
        minDist = std::min(minDist, dist);
    }
    return minDist <= radius;
}

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

std::pair<double, Eigen::Vector2d> ObstacleChecker::calcClosestDistance(const Eigen::Vector2d& point, const int obsIdx) const
{
        // Extract the obstacle of interest
    amp::Obstacle2D obstacle = obstacleList[obsIdx];

        // Loop over boundaries, project point onto boundary, and check resulting distance. Save the minimum distance.
    double minDistance = 9e9;
    Eigen::Vector2d closestIntersect;
    const std::vector<Eigen::Vector2d> &vertices = obstacle.verticesCCW();
    const int numBoundaries = vertices.size();
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

        // Project point onto boundary vector and translate to global frame - this is the vector to the intersection
        // point on the boundary
        Eigen::Vector2d r_PF = point - firstVertex;

        // Calculate projection parameter to find intersection point along the boundary
        double t = r_PF.dot(r_B)/r_B.squaredNorm();
        t = std::clamp(t, 0.0, 1.0); // Ensure we always get a point - if projection is off the boundary, then it is at a vertex

        // Calculate intersection vector
        Eigen::Vector2d r_I = firstVertex + r_B*t;

        // Calculate vector to point from intersection
        Eigen::Vector2d r_PI = point - r_I;

        // Save minimum distance if it's smaller than the currently saved value
        if (r_PI.norm() < minDistance)
        {
            minDistance = r_PI.norm();
            closestIntersect = r_PI;
        }
    }

    if (minDistance < 1e-3)
    {
        minDistance = 1e-3;
        //return {minDistance, Eigen::Vector2d::Zero()};
    }

    Eigen::Vector2d minGradient = closestIntersect/minDistance;

    return {minDistance, minGradient};
}

/// @brief Function that checks the separating axis theorem between a rectangle and convex polygon
/// @param rectVertices Vector of rectangle vertices
/// @param polyVertices Vector of polygon vertices to check against
/// @param rectAxes Vector of rectangle axes in the length and width directions
/// @return Boolean indicating whether the rectangle and polygon intersect or not
bool ObstacleChecker::checkSeparatingAxisTheorem(const std::vector<Eigen::Vector2d> &rectVertices, std::vector<Eigen::Vector2d> &polyVertices, const std::vector<Eigen::Vector2d> &rectAxes)
{
    // Test rectangle axes
    for (const auto& axis : rectAxes) {
        if (!projectionsOverlap(rectVertices, polyVertices, axis)) {
            return false; // Separating axis found, no collision
        }
    }

    // Test polygon axes (each edge normal)
    for (size_t i = 0; i < polyVertices.size(); ++i) {
        size_t next = (i + 1) % polyVertices.size();
        Eigen::Vector2d edge = polyVertices[next] - polyVertices[i];
        Eigen::Vector2d normal(-edge.y(), edge.x()); // Perpendicular to edge
        normal.normalize();

        if (!projectionsOverlap(rectVertices, polyVertices, normal)) {
            return false; // Separating axis found, no collision
        }
    }

    return true; // No separating axis found, collision detected
}

    // Implementation for SAT projection overlap checking
bool projectionsOverlap(const std::vector<Eigen::Vector2d>& shape1,
                       const std::vector<Eigen::Vector2d>& shape2,
                       const Eigen::Vector2d& axis) {

    // Project shape1 onto axis
    double min1 = std::numeric_limits<double>::max();
    double max1 = std::numeric_limits<double>::lowest();
    for (const auto& vertex : shape1) {
        double projection = vertex.dot(axis);
        min1 = std::min(min1, projection);
        max1 = std::max(max1, projection);
    }

    // Project shape2 onto axis
    double min2 = std::numeric_limits<double>::max();
    double max2 = std::numeric_limits<double>::lowest();
    for (const auto& vertex : shape2) {
        double projection = vertex.dot(axis);
        min2 = std::min(min2, projection);
        max2 = std::max(max2, projection);
    }

    // Check if projections overlap
    return !(max1 < min2 || max2 < min1);
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

std::vector<mLineIntersection> ObstacleChecker::calcMLineIntersections(const Eigen::Vector2d &mLine, const amp::Problem2D &problem, bool leftTurner)
{
    // Loop over all obstacles
    for (int i = 0; i < this->obstacleList.size(); i++)
    {
        // Choose correct vertices and remove duplicates
        std::vector<Eigen::Vector2d> vertices;
        if (leftTurner)
        {
            vertices = this->obstacleList[i].verticesCW();
        }
        else
        {
            vertices = this->obstacleList[i].verticesCCW();
        }

        vertices = removeDuplicateVertices(vertices);

        // Declare mLineIntersection structure for this obstacle
        mLineIntersection obstacleIntersection;
        obstacleIntersection.obstacleIndex = i;
        obstacleIntersection.obstacle = this->obstacleList[i];

        // Loop over all boundaries and append intersections to the structure
        for (int ii = 0; ii < vertices.size()-1; ii++)
        {
            // Pull out first and second vertex for this boundary
            Eigen::Vector2d firstVertex = this->obstacleList[i].verticesCCW()[ii];
            Eigen::Vector2d secondVertex = this->obstacleList[i].verticesCCW()[ii + 1];

            // Calculate boundary vector
            Eigen::Vector2d r_B = secondVertex - firstVertex;

            // Calculate intersections for this boundary
            // Propagate m-line onto boundary vector and translate to global frame - this is the vector to the
            // intersection point on the boundary
            Eigen::Matrix2d A {{mLine[0], -r_B[0]}, {mLine[1], -r_B[1]}};
            Eigen::Vector2d b {firstVertex[0] - problem.q_init[0], firstVertex[1] - problem.q_init[1]};
            Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);

            Eigen::Vector2d r_I = x[0]*mLine + problem.q_init;

            // Sort x and y coordinates in increasing order
            Eigen::Vector2d xCoords = (firstVertex[0] <= secondVertex[0]) ? Eigen::Vector2d{firstVertex[0], secondVertex[0]} : Eigen::Vector2d{secondVertex[0], firstVertex[0]};
            Eigen::Vector2d yCoords = (firstVertex[1] <= secondVertex[1]) ? Eigen::Vector2d{firstVertex[1], secondVertex[1]} : Eigen::Vector2d{secondVertex[1], firstVertex[1]};

            // Intersection point isn't valid if it falls outside the two vertices
            if (r_I[0] >= 0.99*xCoords[0] && r_I[0] <= 1.01*xCoords[1] && r_I[1] >= 0.99*yCoords[0] && r_I[1] <= 1.01*yCoords[1])
            {
                obstacleIntersection.intersections.push_back(r_I);
            }
        }

        mLineIntersections.push_back(obstacleIntersection);
    }

    return mLineIntersections;
}



