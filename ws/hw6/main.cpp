#include "AMPCore.h"

#include "hw/HW2.h"
#include "hw/HW6.h"

#include "MyAStar.h"
#include "MyWaveFrontAlgorithm.h"
#include "CSpace.h"
#include "Manipulator2D.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // You will need your 2-link manipulator from HW4
    Manipulator2D manipulator;
    Problem2D point_problemWS1 = HW2::getWorkspace1();
    Problem2D point_problemWS2 = HW2::getWorkspace2();
    Problem2D manip_problem = HW6::getHW4Problem2();
    
    // Construct point-agent and manipulator cspace instances.
        // Grid size (grid square side lengths) specified for exercise 1
    double grid_size = 0.25;
        // Make cells with side lengths of 0.25 for HW 2 WS 1
    std::size_t n_x1_cellsWS1 = (point_problemWS1.x_max - point_problemWS1.x_min) / (grid_size);
    std::size_t n_x2_cellsWS1 = (point_problemWS1.y_max - point_problemWS1.y_min) / (grid_size);
        // Make 50 cells in theta1 and theta2 for HW 4 workspaces
    std::size_t n_manip_cells = 50;
    std::shared_ptr<MyPointAgentCSConstructor> point_agent_ctorWS1 = std::make_shared<MyPointAgentCSConstructor>(n_x1_cellsWS1, n_x2_cellsWS1);
    std::shared_ptr<MyManipulatorCSConstructor> manipulator_ctor = std::make_shared<MyManipulatorCSConstructor>(n_manip_cells);
    std::shared_ptr<WaveFrontAlgorithm> wf_algo = std::make_shared<MyWaveFrontAlgorithm>();
    
    // Combine your wavefront planner with a cspace object (you do not need to modify these classes).
    PointWaveFrontAlgorithm point_algoWS1(wf_algo, point_agent_ctorWS1);
    ManipulatorWaveFrontAlgorithm manip_algo(wf_algo, manipulator_ctor);

    // Return a path for the point-agent and manipulator using c-space planning.
    Path2D pathWS1 = point_algoWS1.plan(point_problemWS1);
    Visualizer::makeFigure(point_problemWS1, pathWS1); // Visualize path in workspace
    Visualizer::makeFigure(*point_algoWS1.getCSpace(), pathWS1); // Visualize path in cspace

    ManipulatorTrajectory2Link trajectory = manip_algo.plan(manipulator, manip_problem);
    Visualizer::makeFigure(manip_problem, manipulator, trajectory);
    Visualizer::makeFigure(*manip_algo.getCSpace(), trajectory);

    // For Exercise 3, you will need to implement the A* algorithm.
    ShortestPathProblem problem = HW6::getEx3SPP();
    LookupSearchHeuristic heuristic = HW6::getEx3Heuristic();
    MyAStarAlgo algo;
    MyAStarAlgo::GraphSearchResult result = algo.search(problem, heuristic);

    Visualizer::saveFigures(true, "hw6_figs");

    amp::HW6::grade<PointWaveFrontAlgorithm, ManipulatorWaveFrontAlgorithm, MyAStarAlgo>("Ian.Faber@colorado.edu", argc, argv, std::make_tuple(wf_algo, point_agent_ctorWS1), std::make_tuple(wf_algo, manipulator_ctor), std::make_tuple());
    return 0;
}