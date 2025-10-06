#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW4.h"
#include "hw/HW6.h"

class MyWaveFrontAlgorithm : public amp::WaveFrontAlgorithm {
    public:
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) override;
    private:
        int maxLoopCount = 1e5;
};

class MyWaveFrontGridSpace : public amp::ConfigurationSpace2D, public amp::DenseArray2D<int>
{
    public:
        MyWaveFrontGridSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
                : amp::ConfigurationSpace2D(x0_min, x0_max, x1_min, x1_max),
                  amp::DenseArray2D<int>(x0_cells, x1_cells)
        {}

        virtual bool inCollision(double x0, double x1) const override {
            return false;
        }
};
