//
// Created by ianmf on 9/24/25.
//

#ifndef AMP_TOOLS_CSPACE_H
#define AMP_TOOLS_CSPACE_H

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"
#include "hw/HW6.h"

// Derive the amp::GridCSpace2D class and override the missing method
class MyGridCSpace2D : public amp::GridCSpace2D {
    public:
        MyGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) // Call base class constructor
        {}

        // Override this method for determining which cell a continuous point belongs to
        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;

        // Function that converts a cell to a continuous point, i.e. the midpoint of that cell
        Eigen::Vector2d getPointFromCell(std::pair<std::size_t, std::size_t> cell) const;

};

// Adapter class to make GridCSpace2D compatible with ConfigurationSpace
class GridCSpaceAdapter : public amp::ConfigurationSpace {
    public:
        GridCSpaceAdapter(const amp::GridCSpace2D& cspace)
            : amp::ConfigurationSpace(Eigen::Vector2d(cspace.x0Bounds().first, cspace.x1Bounds().first),
                                     Eigen::Vector2d(cspace.x0Bounds().second, cspace.x1Bounds().second)),
              grid_cspace(cspace) {}

        virtual bool inCollision(const Eigen::VectorXd& cspace_state) const override {
            if (cspace_state.size() != 2) {
                throw std::invalid_argument("Expected 2D configuration space state");
            }
            return grid_cspace.inCollision(cspace_state[0], cspace_state[1]);
        }

    private:
            const amp::GridCSpace2D& grid_cspace;
};


// Derive the HW4 ManipulatorCSConstructor class and override the missing method
class MyManipulatorCSConstructor : public amp::ManipulatorCSConstructor {
    public:
        // To make things easy, add the number of cells as a ctor param so you can easily play around with it
        MyManipulatorCSConstructor(std::size_t cells_per_dim) : m_cells_per_dim(cells_per_dim) {}

        // Override this method for computing all of the boolean collision values for each cell in the cspace
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override;

    private:
        std::size_t m_cells_per_dim;
};

// Derive the PointAgentCSConstructor class and override the missing method
class MyPointAgentCSConstructor : public amp::PointAgentCSConstructor {
public:
    // To make things easy, add the number of cells as a ctor param so you can easily play around with it
    MyPointAgentCSConstructor(std::size_t x1_cells, std::size_t x2_cells) : m_x1_cells(x1_cells), m_x2_cells(x2_cells) {}

    // Override this method for computing all of the boolean collision values for each cell in the cspace
    virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::Environment2D& env) override;

private:
    std::size_t m_x1_cells;
    std::size_t m_x2_cells;
};

    // Function for checking if a point is within the bounds of an N-D Cspace
bool isWithinBounds(const Eigen::VectorXd &point, const amp::ConfigurationSpace &cspace);

#endif //AMP_TOOLS_CSPACE_H