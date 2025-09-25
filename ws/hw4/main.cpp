// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpace.h"
#include "Manipulator2D.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

        // Problem 2a demonstration
    Manipulator2D manipulator2a({0.5, 1, 0.5});

    amp::ManipulatorState test_state(manipulator2a.nLinks());
    test_state << M_PI/6, M_PI/3, (7*M_PI)/4;
    std::cout << "Joint Positions for FK given state (" << test_state[0] << "," << test_state[1] << "," << test_state[2] << "):" << std::endl;
    for (int i = 0; i <= manipulator2a.nLinks(); i++)
    {
        Eigen::Vector2d position = manipulator2a.getJointLocation(test_state, i);
        std::cout << "(" << position[0] << "," << position[1] << ")" << std::endl;
    }

    // The visualizer uses your implementation of forward kinematics to show the joint positions so you can use that to test your FK algorithm
    Visualizer::makeFigure(manipulator2a, test_state);

        // Problem 2b demonstration
    Manipulator2D manipulator2b({1, 0.5, 1});

    test_state.setZero();
    Eigen::Vector2d point = {2,0};
    test_state = manipulator2b.getConfigurationFromIK(point);

    std::cout << std::endl << "Joint Positions from IK for given point (" << point[0] << "," << point[1] << "):" << std::endl;
    for (int i = 0; i <= manipulator2b.nLinks(); i++)
    {
        Eigen::Vector2d position = manipulator2b.getJointLocation(test_state, i);
        std::cout << "(" << position[0] << "," << position[1] << ")" << std::endl;
    }

    Visualizer::makeFigure(manipulator2b, test_state);

    // Create the collision space constructor
    std::size_t n_cells = 100;
    MyManipulatorCSConstructor cspace_constructor(n_cells);

    Manipulator2D manipulator3({1.0, 1.0});

    // Create the collision space using a given manipulator and environment
    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator3, HW4::getEx3Workspace1());

    // You can visualize your cspace 
    Visualizer::makeFigure(*cspace);

    Visualizer::saveFigures(true, "hw4_figs");

    // Grade method
    amp::HW4::grade<Manipulator2D>(cspace_constructor, "Ian.Faber@colorado.edu", argc, argv);
    return 0;
}