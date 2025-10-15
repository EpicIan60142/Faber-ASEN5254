// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"

using namespace amp;

int main(int argc, char** argv) {
    // Set random seed
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Test PRM on WS 2a of HW 5
        // Get problem environment
    Problem2D problem_HW5WS2a = HW5::getWorkspace1();
            // Make problem bounds match the problem statement
    problem_HW5WS2a.x_max = 11; problem_HW5WS2a.x_min = -1;
    problem_HW5WS2a.y_max = 3; problem_HW5WS2a.y_min = -3;
        // Make PRM algo
    MyPRM prm_HW5WS2a;
    prm_HW5WS2a.setParameters(200, 1);
        // Make path
    amp::Path2D pathPRM_HW5WS2a = prm_HW5WS2a.plan(problem_HW5WS2a);
    std::cout << "Path length for HW 5 WS 2a: " << pathPRM_HW5WS2a.length() << std::endl;
        // Extract graph
    std::shared_ptr<amp::Graph<double>> graphPRM_HW5WS2a = prm_HW5WS2a.getGraph();
        // Extract nodes
    std::map<amp::Node, Eigen::Vector2d> nodes;
    nodes = prm_HW5WS2a.getNodes();
        // Make figure
    Visualizer::makeFigure(problem_HW5WS2a, pathPRM_HW5WS2a, *graphPRM_HW5WS2a, nodes);

    // Test PRM on Workspace 1 of HW2
    Problem2D problem_HW2WS1 = HW2::getWorkspace1();
    MyPRM prm_HW2WS1;
    prm_HW2WS1.setParameters(200, 2);
    amp::Path2D pathPRM_HW2WS1 = prm_HW2WS1.plan(problem_HW2WS1);
    std::cout << "Path length for HW 2 WS 1: " << pathPRM_HW2WS1.length() << std::endl;
    std::shared_ptr<amp::Graph<double>> graphPRM_HW2WS1 = prm_HW2WS1.getGraph();
    nodes = prm_HW2WS1.getNodes();
    Visualizer::makeFigure(problem_HW2WS1, pathPRM_HW2WS1, *graphPRM_HW2WS1, nodes);

    // Test PRM on Workspace 2 of HW 2
    Problem2D problem_HW2WS2 = HW2::getWorkspace2();
    MyPRM prm_HW2WS2;
    prm_HW2WS2.setParameters(200, 2);
    amp::Path2D pathPRM_HW2WS2 = prm_HW2WS2.plan(problem_HW2WS2);
    std::cout << "Path length for HW 2 WS 2: " << pathPRM_HW2WS2.length() << std::endl;
    std::shared_ptr<amp::Graph<double>> graphPRM_HW2WS2 = prm_HW2WS2.getGraph();
    nodes = prm_HW2WS2.getNodes();
    Visualizer::makeFigure(problem_HW2WS2, pathPRM_HW2WS2, *graphPRM_HW2WS2, nodes);

    // Test PRM on random workspace
    Problem2D problem_rand;
    MyPRM prm_rand;
    prm_rand.setParameters(500, 3);
    amp::Path2D pathPRM_rand;
    HW7::generateAndCheck(prm_rand, pathPRM_rand, problem_rand);
    std::shared_ptr<amp::Graph<double>> graphPRM_rand = prm_rand.getGraph();
    nodes = prm_rand.getNodes();
    Visualizer::makeFigure(problem_rand, pathPRM_rand, *graphPRM_rand, nodes);

    // Benchmark PRM for 2a of HW 5
        // Make vectors for results
    std::vector<double> validSols;
    std::list<std::vector<double>> times;
    std::list<std::vector<double>> lengths;
        // Make vector of test parameters
    std::vector<std::pair<int, double>> testSuite = {{200, 0.5}, {200, 1}, {200, 1.5}, {200, 2}, {500, 0.5}, {500, 1}, {500, 1.5}, {500, 2}};
        // Loop over test parameters and benchmark each set
    for (int i = 0; i < testSuite.size(); i++)
    {
        std::cout << "Benchmarking PRM with parameters: n = " << testSuite[i].first << ", r = " << testSuite[i].second << std::endl;
        int nRuns = 100;
        int validCount = 0;
        std::vector<double> timeCol;
        std::vector<double> lengthCol;
        for (int j = 0; j < nRuns; j++)
        {
            MyPRM prm_benchmark;
            prm_benchmark.setParameters(testSuite[i].first, testSuite[i].second);
            amp::Timer PRMTimer("PRMTimer");
            amp::Path2D pathPRM_benchmark = prm_benchmark.plan(problem_HW5WS2a);
            double time = PRMTimer.now(TimeUnit::ms);
            if (abs(time) < 10000) // Sometimes the timer messes up and we get massive times, want to avoid that
            {
                timeCol.push_back(time);
            }

            if (pathPRM_benchmark.valid)
            {
                validCount++;
                lengthCol.push_back(pathPRM_benchmark.length());
            }
            else
            {
                lengthCol.push_back(0);
            }
        }
        times.push_back(timeCol);
        lengths.push_back(lengthCol);
        validSols.push_back(validCount);
    }

        // Make box plots
            // Time benchmark
    std::list<std::vector<double>> dataSet = times;
    std::vector<std::string> labels = {"Case 1", "Case 2", "Case 3", "Case 4", "Case 5", "Case 6", "Case 7", "Case 8"};
    std::string title = "PRM Time Benchmark";
    std::string xlabel = " ";
    std::string ylabel = "Elapsed Time [ms]";
    Visualizer::makeBoxPlot(dataSet, labels, title, xlabel, ylabel);

            // Path length benchmark
    dataSet = lengths;
    labels = {"Case 1", "Case 2", "Case 3", "Case 4", "Case 5", "Case 6", "Case 7", "Case 8"};
    title = "PRM Length Benchmark";
    xlabel = " ";
    ylabel = "Path Length";
    Visualizer::makeBoxPlot(dataSet, labels, title, xlabel, ylabel);

            // Validity benchmark
    std::vector<double> dataSet2 = validSols;
    labels = {"Case 1", "Case 2", "Case 3", "Case 4", "Case 5", "Case 6", "Case 7", "Case 8"};
    title = "PRM Validity Benchmark";
    xlabel = " ";
    ylabel = "Number of Valid Solutions";
    Visualizer::makeBarGraph(dataSet2, labels, title, xlabel, ylabel);

    // Generate a random problem and test RRT
    Problem2D problem;
    MyRRT rrt;
    Path2D path;
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    nodes.clear();
    HW7::generateAndCheck(rrt, path, problem);
    Visualizer::makeFigure(problem, path, *graphPtr, nodes);
    Visualizer::saveFigures(true, "hw7_figs");

    // Grade method
    HW7::grade<MyPRM, MyRRT>("Ian.Faber@colorado.edu", argc, argv, std::make_tuple(500,3), std::make_tuple());
    return 0;
}