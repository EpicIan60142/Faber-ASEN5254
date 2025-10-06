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
    Problem2D manip_problemWS1 = HW6::getHW4Problem1();
    Problem2D manip_problemWS2 = HW6::getHW4Problem2();
    Problem2D manip_problemWS3 = HW6::getHW4Problem3();

    // Construct point-agent and manipulator cspace instances.
        // Grid size (grid square side lengths) specified for exercise 1
    double grid_size = 0.25;
        // Make cells with side lengths of 0.25 for HW 2 WS 1
    std::size_t n_x1_cellsWS1 = (point_problemWS1.x_max - point_problemWS1.x_min) / (grid_size);
    std::size_t n_x2_cellsWS1 = (point_problemWS1.y_max - point_problemWS1.y_min) / (grid_size);
        // Make cells with side lengths of 0.25 for HW 2 WS 2
    std::size_t n_x1_cellsWS2 = (point_problemWS2.x_max - point_problemWS2.x_min) / (grid_size);
    std::size_t n_x2_cellsWS2 = (point_problemWS2.y_max - point_problemWS2.y_min) / (grid_size);
        // Make 50 cells in theta1 and theta2 for HW 4 workspaces
    std::size_t n_manip_cells = 50;

        // Create constructor pointers
    std::shared_ptr<MyPointAgentCSConstructor> point_agent_ctorWS1 = std::make_shared<MyPointAgentCSConstructor>(n_x1_cellsWS1, n_x2_cellsWS1);
    std::shared_ptr<MyPointAgentCSConstructor> point_agent_ctorWS2 = std::make_shared<MyPointAgentCSConstructor>(n_x1_cellsWS2, n_x2_cellsWS2);
    std::shared_ptr<MyManipulatorCSConstructor> manipulator_ctorWS1 = std::make_shared<MyManipulatorCSConstructor>(n_manip_cells);
    std::shared_ptr<MyManipulatorCSConstructor> manipulator_ctorWS2 = std::make_shared<MyManipulatorCSConstructor>(n_manip_cells);
    std::shared_ptr<MyManipulatorCSConstructor> manipulator_ctorWS3 = std::make_shared<MyManipulatorCSConstructor>(n_manip_cells);
    std::shared_ptr<WaveFrontAlgorithm> wf_algo = std::make_shared<MyWaveFrontAlgorithm>();
    
    // Combine your wavefront planner with a cspace object (you do not need to modify these classes).
    PointWaveFrontAlgorithm point_algoWS1(wf_algo, point_agent_ctorWS1);
    PointWaveFrontAlgorithm point_algoWS2(wf_algo, point_agent_ctorWS2);
    ManipulatorWaveFrontAlgorithm manip_algoWS1(wf_algo, manipulator_ctorWS3);
    ManipulatorWaveFrontAlgorithm manip_algoWS2(wf_algo, manipulator_ctorWS2);
    ManipulatorWaveFrontAlgorithm manip_algoWS3(wf_algo, manipulator_ctorWS3);

    // Return a path for the point-agent and manipulator using c-space planning.
        // HW 2 WS 1
    Path2D pathWS1 = point_algoWS1.plan(point_problemWS1);
    Visualizer::makeFigure(point_problemWS1, pathWS1); // Visualize path in workspace
    Visualizer::makeFigure(*point_algoWS1.getCSpace(), pathWS1); // Visualize path in cspace

        // HW 2 WS 2
    Path2D pathWS2 = point_algoWS2.plan(point_problemWS2);
    Visualizer::makeFigure(point_problemWS2, pathWS2); // Visualize path in workspace
    Visualizer::makeFigure(*point_algoWS2.getCSpace(), pathWS2); // Visualize path in cspace

        // HW 4 WS 1
    ManipulatorTrajectory2Link trajectoryWS1 = manip_algoWS1.plan(manipulator, manip_problemWS1);
    Visualizer::makeFigure(manip_problemWS1, manipulator, trajectoryWS1);
    Visualizer::makeFigure(*manip_algoWS1.getCSpace(), trajectoryWS1);

        // HW 4 WS 2
    ManipulatorTrajectory2Link trajectoryWS2 = manip_algoWS2.plan(manipulator, manip_problemWS2);
    Visualizer::makeFigure(manip_problemWS2, manipulator, trajectoryWS2);
    Visualizer::makeFigure(*manip_algoWS2.getCSpace(), trajectoryWS2);

        // HW 4 WS 3
    ManipulatorTrajectory2Link trajectoryWS3 = manip_algoWS3.plan(manipulator, manip_problemWS3);
    Visualizer::makeFigure(manip_problemWS3, manipulator, trajectoryWS3);
    Visualizer::makeFigure(*manip_algoWS3.getCSpace(), trajectoryWS3);

        // Random point WS
    std::vector<Eigen::Vector2d> collision_points;
    std::shared_ptr<MyPointAgentCSConstructor> point_agent_ctorRWS = std::make_shared<MyPointAgentCSConstructor>(100, 100);
    PointWaveFrontAlgorithm point_algoRWS(wf_algo, point_agent_ctorRWS);
    amp::Problem2D point_problemRWS;
    amp::Path2D pathRWS;
    bool randPointSuccess = HW6::generateAndCheck(point_algoRWS, pathRWS, point_problemRWS, collision_points);
    Visualizer::makeFigure(point_problemRWS, pathRWS);
    Visualizer::makeFigure(*point_algoRWS.getCSpace(), pathRWS);

        // Random manipulator WS
    collision_points.clear();
    std::shared_ptr<MyManipulatorCSConstructor> manipulator_ctorRWS = std::make_shared<MyManipulatorCSConstructor>(50);
    ManipulatorWaveFrontAlgorithm manip_algoRWS(wf_algo, manipulator_ctorRWS);
    amp::Problem2D manip_problemRWS;
    amp::ManipulatorTrajectory2Link trajectoryRWS;
    bool randManipSuccess = HW6::generateAndCheck(manip_algoRWS, manipulator, trajectoryRWS, manip_problemRWS, collision_points);
    Visualizer::makeFigure(manip_problemRWS, manipulator, trajectoryRWS);
    Visualizer::makeFigure(*manip_algoRWS.getCSpace(), trajectoryRWS);

    // For Exercise 3, you will need to implement the A* algorithm.
    ShortestPathProblem problem = HW6::getEx3SPP();
    LookupSearchHeuristic heuristic = HW6::getEx3Heuristic();
    MyAStarAlgo algo;
    MyAStarAlgo::GraphSearchResult result = algo.search(problem, heuristic);

    Visualizer::saveFigures(true, "hw6_figs");

    amp::HW6::grade<PointWaveFrontAlgorithm, ManipulatorWaveFrontAlgorithm, MyAStarAlgo>("Ian.Faber@colorado.edu", argc, argv, std::make_tuple(wf_algo, point_agent_ctorWS1), std::make_tuple(wf_algo, manipulator_ctorWS2), std::make_tuple());
    return 0;
}