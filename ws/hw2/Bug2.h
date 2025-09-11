#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class Bug2 : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        amp::Path2D plan(const amp::Problem2D& problem) override;

        // Getter and setter for incremental distance
        double getDr() const { return dr; };
        void setDr(const double newDr) { dr = newDr; epsilon = newDr; };

        // Getter and setter for left/right turner
        double getLeftTurner() const {return leftTurner; };
        void setLeftTurner(const bool newTurner) { leftTurner = newTurner; };

    private:
        // Add any member variables here...
        double dr = 0.01; // [m] Incremental distance for propagating bug path
        double epsilon = dr; // [m] Epsilon for determining when we're close to the goal
        bool leftTurner = true; // Whether the robot is a left or right turner
        double loopTimeout = 1e2; // Number of main loops before the algorithm times out
};