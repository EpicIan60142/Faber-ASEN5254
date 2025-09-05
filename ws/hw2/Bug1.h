#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class Bug1 : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
        void setDr(double dr) { dr = dr; } // Setter for incremental distance
        double getDr() { return dr; } // Getter for incremental distance

    private:
        // Add any member variables here...
        double dr = 0.01; // [m] Incremental distance for propagating bug path
        double epsilon = dr; // [m] Epsilon for determining when we're close to the goal
};