// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"

using namespace amp;

int main(int argc, char** argv) {
    // Set random seed
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Enable/disable smoothing and benchmarking
    bool PRMSmooth = true;
    bool benchmarkPRM = false;
    bool benchmarkRRT = false;

    // Test PRM on WS 2a of HW 5
        // Get problem environment
    Problem2D problem_HW5WS2a = HW5::getWorkspace1();
            // Make problem bounds match the problem statement
    problem_HW5WS2a.x_max = 11; problem_HW5WS2a.x_min = -1;
    problem_HW5WS2a.y_max = 3; problem_HW5WS2a.y_min = -3;
        // Make PRM algo
    MyPRM prm_HW5WS2a;
    prm_HW5WS2a.setParameters(200, 1, PRMSmooth);
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
    prm_HW2WS1.setParameters(200, 2, PRMSmooth);
    amp::Path2D pathPRM_HW2WS1 = prm_HW2WS1.plan(problem_HW2WS1);
    std::cout << "Path length for HW 2 WS 1: " << pathPRM_HW2WS1.length() << std::endl;
    std::shared_ptr<amp::Graph<double>> graphPRM_HW2WS1 = prm_HW2WS1.getGraph();
    nodes = prm_HW2WS1.getNodes();
    Visualizer::makeFigure(problem_HW2WS1, pathPRM_HW2WS1, *graphPRM_HW2WS1, nodes);

    // Test PRM on Workspace 2 of HW 2
    Problem2D problem_HW2WS2 = HW2::getWorkspace2();
    MyPRM prm_HW2WS2;
    prm_HW2WS2.setParameters(200, 2, PRMSmooth);
    amp::Path2D pathPRM_HW2WS2 = prm_HW2WS2.plan(problem_HW2WS2);
    std::cout << "Path length for HW 2 WS 2: " << pathPRM_HW2WS2.length() << std::endl;
    std::shared_ptr<amp::Graph<double>> graphPRM_HW2WS2 = prm_HW2WS2.getGraph();
    nodes = prm_HW2WS2.getNodes();
    Visualizer::makeFigure(problem_HW2WS2, pathPRM_HW2WS2, *graphPRM_HW2WS2, nodes);

    // Test PRM on random workspace
    Problem2D problem_rand;
    MyPRM prm_rand;
    prm_rand.setParameters(500, 3, PRMSmooth);
    amp::Path2D pathPRM_rand;
    HW7::generateAndCheck(prm_rand, pathPRM_rand, problem_rand);
    std::shared_ptr<amp::Graph<double>> graphPRM_rand = prm_rand.getGraph();
    nodes = prm_rand.getNodes();
    Visualizer::makeFigure(problem_rand, pathPRM_rand, *graphPRM_rand, nodes);

    if (benchmarkPRM)
    {
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
                std::cout << "HW 5 2a Suite " << i+1 << ", Run " << j+1 << std::endl;
                MyPRM prm_benchmark;
                prm_benchmark.setParameters(testSuite[i].first, testSuite[i].second, PRMSmooth);
                amp::Timer PRMTimer("PRMTimer");
                amp::Path2D pathPRM_benchmark = prm_benchmark.plan(problem_HW5WS2a);
                double time = PRMTimer.now(TimeUnit::ms);
                if (time >= 0) // Sometimes the timer messes up and we get negative times, want to avoid that
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
        std::string title = "PRM Time Benchmark - Exercise 2a of HW 5";
        std::string xlabel = " ";
        std::string ylabel = "Elapsed Time [ms]";
        Visualizer::makeBoxPlot(dataSet, labels, title, xlabel, ylabel);

        // Path length benchmark
        dataSet = lengths;
        labels = {"Case 1", "Case 2", "Case 3", "Case 4", "Case 5", "Case 6", "Case 7", "Case 8"};
        title = "PRM Length Benchmark - Exercise 2a of HW 5";
        xlabel = " ";
        ylabel = "Path Length";
        Visualizer::makeBoxPlot(dataSet, labels, title, xlabel, ylabel);

        // Validity benchmark
        std::vector<double> dataSet2 = validSols;
        labels = {"Case 1", "Case 2", "Case 3", "Case 4", "Case 5", "Case 6", "Case 7", "Case 8"};
        title = "PRM Validity Benchmark - Exercise 2a of HW 5";
        xlabel = " ";
        ylabel = "Number of Valid Solutions";
        Visualizer::makeBarGraph(dataSet2, labels, title, xlabel, ylabel);

        // Benchmark PRM for HW 2
        for (int k = 0; k < 2; k++)
        {
            // Choose workspace to benchmark
            amp::Problem2D problem;
            switch (k)
            {
            case 0:
                problem = problem_HW2WS1;
            case 1:
                problem = problem_HW2WS2;
            default:
                problem = problem_HW2WS1;
            }

            // Make vectors for results
            validSols.clear();
            times.clear();
            lengths.clear();
            // Make vector of test parameters
            testSuite = {{200, 1}, {200, 2}, {500, 1}, {500, 2}, {1000, 1}, {1000, 2}};
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
                    std::cout << "HW 2 Ex 2 problem " << k+1 << ", Suite " << i+1 << ", Run " << j+1 << std::endl;
                    MyPRM prm_benchmark;
                    prm_benchmark.setParameters(testSuite[i].first, testSuite[i].second, PRMSmooth);
                    amp::Timer PRMTimer("PRMTimer");
                    amp::Path2D pathPRM_benchmark = prm_benchmark.plan(problem);
                    double time = PRMTimer.now(TimeUnit::ms);
                    if (time >= 0) // Sometimes the timer messes up and we get negative times, want to avoid that
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
            dataSet = times;
            labels = {"Case 1", "Case 2", "Case 3", "Case 4", "Case 5", "Case 6"};
            if (k == 0)
            {
                title = "PRM Time Benchmark - W1 of HW 2";
            }
            else
            {
                title = "PRM Time Benchmark - W2 of HW 2";
            }
            xlabel = " ";
            ylabel = "Elapsed Time [ms]";
            Visualizer::makeBoxPlot(dataSet, labels, title, xlabel, ylabel);

            // Path length benchmark
            dataSet = lengths;
            labels = {"Case 1", "Case 2", "Case 3", "Case 4", "Case 5", "Case 6"};
            if (k == 0)
            {
                title = "PRM Length Benchmark - W1 of HW 2";
            }
            else
            {
                title = "PRM Length Benchmark - W2 of HW 2";
            }
            xlabel = " ";
            ylabel = "Path Length";
            Visualizer::makeBoxPlot(dataSet, labels, title, xlabel, ylabel);

            // Validity benchmark
            dataSet2 = validSols;
            labels = {"Case 1", "Case 2", "Case 3", "Case 4", "Case 5", "Case 6"};
            if (k == 0)
            {
                title = "PRM Validity Benchmark - W1 of HW 2";
            }
            else
            {
                title = "PRM Validity Benchmark - W2 of HW 2";
            }
            xlabel = " ";
            ylabel = "Number of Valid Solutions";
            Visualizer::makeBarGraph(dataSet2, labels, title, xlabel, ylabel);
        }
    }

    // Generate a random problem and test RRT
    Problem2D problem;
    MyRRT rrt;
    Path2D path;
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    nodes.clear();
    HW7::generateAndCheck(rrt, path, problem);
    Visualizer::makeFigure(problem, path, *graphPtr, nodes);

    // Save figures
    if (PRMSmooth && benchmarkPRM)
    {
        Visualizer::saveFigures(true, "hw7_figs_PRMBenchmarkSmoothed");
    }
    else if (!PRMSmooth && benchmarkPRM)
    {
        Visualizer::saveFigures(true, "hw7_figs_PRMBenchmarkUnsmoothed");
    }
    else
    {
        Visualizer::saveFigures(true, "hw7_figs");
    }


    // Grade method
    HW7::grade<MyPRM, MyRRT>("Ian.Faber@colorado.edu", argc, argv, std::make_tuple(500,3), std::make_tuple());
    return 0;
}