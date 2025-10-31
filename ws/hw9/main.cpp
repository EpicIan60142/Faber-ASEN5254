// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW9.h"
#include "hw/HW2.h"
#include "MyKinoRRT.h"

using namespace amp;

// Load problems and map agent for quick testing
std::vector<KinodynamicProblem2D> problems = {HW9::getStateIntProblemWS1(), HW9::getStateIntProblemWS2(), HW9::getFOUniProblemWS1(), HW9::getFOUniProblemWS2(), HW9::getSOUniProblemWS1(), HW9::getSOUniProblemWS2(), HW9::getCarProblemWS1(), HW9::getParkingProblem()};
std::unordered_map<AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
    {AgentType::SingleIntegrator, []() { return std::make_shared<MySingleIntegrator>(); }},
    {AgentType::FirstOrderUnicycle, []() { return std::make_shared<MyFirstOrderUnicycle>(); }},
    {AgentType::SecondOrderUnicycle, []() { return std::make_shared<MySecondOrderUnicycle>(); }},
    {AgentType::SimpleCar, []() { return std::make_shared<MySimpleCar>(); }}
};

int main(int argc, char** argv) {
    // Enable benchmarking
    bool benchmarkKinoRRT = false;
    bool benchmarkParking = false;

    // Select problem, plan, check, and visualize
    std::vector<int> probIdx = {0, 2, 4, 7};
    for (int i = 0; i < probIdx.size(); i++)
    {
        std::cout << "Planning for problem " << probIdx[i] << std::endl;
        KinodynamicProblem2D prob = problems[probIdx[i]];
        MyKinoRRT kino_planner;
        KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
        HW9::check(path, prob);
        if (true)
        {
            amp::Path2D path2d;
            for (int j = 0; j < path.waypoints.size(); j++)
            {
                double x = path.waypoints[j][0];
                double y = path.waypoints[j][1];
                path2d.waypoints.push_back({x,y});
            }
            std::cout << "Path length for problem " << probIdx[i] << ": " << path2d.length() << std::endl;
            Visualizer::makeFigure(prob, path, false); // Set to 'true' to render animation
            if (i == probIdx.size()-1)
            {
                Visualizer::makeFigure(prob, path, true); // Set to 'true' to render animation

                // Save control inputs to file
                std::ofstream outfile("../../data/controlData.csv"); // Saves to the cmake-debug-build/bin directory
                if (outfile.is_open())
                {
                    outfile << "time,velocity,steeringAngle\n";
                    double cumTime = 0;
                    for (size_t i = 0; i < path.controls.size(); i++)
                    {
                        cumTime += path.durations[i];
                        outfile << cumTime << "," << path.controls[i][0] << "," << path.controls[i][1] << "\n";
                    }
                    outfile.close();
                }
            }
        }
    }

    // Save figures
    Visualizer::saveFigures(true, "hw9_figs");

    // Benchmark KinoRRT if enabled
    if (benchmarkKinoRRT)
    {
        // Define test suite, problems to test, and number of runs per suite per problem
        std::vector<std::pair<int, int>> testSuite = {{50000, 1}, {50000,5}, {50000, 10}, {50000,15}};
        std::vector<KinodynamicProblem2D> testProblems = {HW9::getStateIntProblemWS1(), HW9::getFOUniProblemWS1(), HW9::getSOUniProblemWS1()};
        int nRuns = 50;

        // Make vectors for results
        std::vector<std::vector<double>> validSols; // Dimensions nProblems x nTestSuites
        std::vector<std::list<std::vector<double>>> times; // Dimensions nProblems x nTestSuites x nRuns
        std::vector<std::list<std::vector<double>>> lengths; // Dimensions nProblems x nTestSuites x nRuns

        // Loop over each problem and test suite
        for (int i = 0; i < testProblems.size(); i++)
        {
            // Initialize tracking variables
            std::vector<double> validProb;
            std::list<std::vector<double>> timeProb;
            std::list<std::vector<double>> lengthProb;

            // Pull out the correct problem
            KinodynamicProblem2D &problem = testProblems[i];
            std::cout << "Benchmarking problem " << i+1 << std::endl;

            // Run test suite and save results
            for (int j = 0; j < testSuite.size(); j++)
            {
                // Initialize tracking variables
                double validCol = 0;
                std::vector<double> timeCol;
                std::vector<double> lengthCol;

                // Pull out the correct test
                std::pair<int, int> test = testSuite[j];
                std::cout << "\tBenchmarking problem " << i+1 << " with parameters n = " << test.first << ", uSample = " << test.second << std::endl;

                // Run nRuns test cases
                for (int k = 0; k < nRuns; k++)
                {
                    std::cout << "\t\tProblem " << i+1 << ", Suite " << j+1 << ", Run " << k+1 << std::endl;
                    // Make planner with the test suite parameters
                    double pGoal = 0.05;
                    MyKinoRRT kinoPlanner_benchmark(test.first, test.second, pGoal);

                    // Make timer and solve problem
                    amp::Timer timer("KinoRRT Timer");
                    amp::KinoPath pathKino_benchmark = kinoPlanner_benchmark.plan(problem, *agentFactory[problem.agent_type]());
                    double time = timer.now(TimeUnit::ms);

                    // Collate data
                    if (time >= 0) // Sometimes the timer messes up and we get negative times, want to avoid that
                    {
                        timeCol.push_back(time);
                    }

                    if (pathKino_benchmark.valid)
                    {
                        validCol++;

                        // Convert KinoPath to 2D
                        amp::Path2D path2d;
                        for (int l = 0; l < pathKino_benchmark.waypoints.size(); l++)
                        {
                            double x = pathKino_benchmark.waypoints[l][0];
                            double y = pathKino_benchmark.waypoints[l][1];
                            path2d.waypoints.push_back({x,y});
                        }
                        lengthCol.push_back(path2d.length());
                    }
                    else
                    {
                        lengthCol.push_back(0);
                    }
                }

                // Append data for this test suite to data for this problem
                validProb.push_back(validCol);
                timeProb.push_back(timeCol);
                lengthProb.push_back(lengthCol);
            }

            // Append data for this problem to overall benchmarking data
            validSols.push_back(validProb);
            times.push_back(timeProb);
            lengths.push_back(lengthProb);
        }

        // Make plots
            // Time benchmark
        for (int i = 0; i < times.size(); i++)
        {
            std::list<std::vector<double>> &dataSet = {times[i]};
            std::vector<std::string> labels = {"Case 1", "Case 2", "Case 3", "Case 4"};
            std::string title = "KinoRRT Time Benchmark - Problem " + std::to_string(i+1);
            std::string xlabel = " ";
            std::string ylabel = "Time (ms)";
            Visualizer::makeBoxPlot(dataSet, labels, title, xlabel, ylabel);
        }

            // Length benchmark
        for (int i = 0; i < lengths.size(); i++)
        {
            std::list<std::vector<double>> &dataSet = {lengths[i]};
            std::vector<std::string> labels = {"Case 1", "Case 2", "Case 3", "Case 4"};
            std::string title = "KinoRRT Length Benchmark - Problem " + std::to_string(i+1);
            std::string xlabel = " ";
            std::string ylabel = "Path Length";
            Visualizer::makeBoxPlot(dataSet, labels, title, xlabel, ylabel);
        }

            // Validity benchmark
        for (int i = 0; i < validSols.size(); i++)
        {
            std::vector<double> &dataSet = validSols[i];
            std::vector<std::string> labels = {"Case 1", "Case 2", "Case 3", "Case 4"};
            std::string title = "KinoRRT Validity Benchmark - " + std::to_string(i+1);
            std::string xlabel = " ";
            std::string ylabel = "Number of Valid Solutions";
            Visualizer::makeBarGraph(dataSet, labels, title, xlabel, ylabel);
        }

        // Save figures
        Visualizer::saveFigures(true, "hw9_figs_benchmarkKinoRRT");
    }

    // Benchmark parallel parking if enabled
    if (benchmarkParking)
    {
        // Define test suite, problem, and number of runs
        std::vector<int> testSuite = {1, 5, 10, 15};
        KinodynamicProblem2D problem = HW9::getParkingProblem();
        int nRuns = 100;

        // Make vectors for results
        std::vector<double> validSols;
        std::list<std::vector<double>> times;
        std::list<std::vector<double>> lengths;

        // Loop over test suite and save results
        for (int i = 0; i < testSuite.size(); i++)
        {
            // Initialize tracking variables
            double validCol = 0;
            std::vector<double> timeCol;
            std::vector<double> lengthCol;

            std::cout << "Benchmarking parking problem with uSample = " << testSuite[i] << std::endl;

            for (int j = 0; j < nRuns; j++)
            {
                std::cout << "\tuSample = " << testSuite[i] << ", Run " << j+1 << std::endl;
                // Make planner with the test suite parameters
                int nSample = 50000;
                double pGoal = 0.05;
                MyKinoRRT parkingPlanner_benchmark(nSample, testSuite[i], pGoal);

                // Make timer and solve problem
                amp::Timer timer("Parking Timer");
                amp::KinoPath pathPark_benchmark = parkingPlanner_benchmark.plan(problem, *agentFactory[problem.agent_type]());
                double time = timer.now(TimeUnit::ms);

                // Collate data
                if (time >= 0) // Sometimes the timer messes up and we get negative times, want to avoid that
                {
                    timeCol.push_back(time);
                }

                if (pathPark_benchmark.valid)
                {
                    validCol++;

                    // Convert KinoPath to 2D
                    amp::Path2D path2d;
                    for (int l = 0; l < pathPark_benchmark.waypoints.size(); l++)
                    {
                        double x = pathPark_benchmark.waypoints[l][0];
                        double y = pathPark_benchmark.waypoints[l][1];
                        path2d.waypoints.push_back({x,y});
                    }
                    lengthCol.push_back(path2d.length());
                }
                else
                {
                    lengthCol.push_back(0);
                }
            }

            // Append data for this test suite to data for this problem
            validSols.push_back(validCol);
            times.push_back(timeCol);
            lengths.push_back(lengthCol);
        }

        // Make plots
            // Time benchmark
        std::list<std::vector<double>> &dataSet = times;
        std::vector<std::string> labels = {"Case 1", "Case 2", "Case 3", "Case 4"};
        std::string title = "Parking Time Benchmark";
        std::string xlabel = " ";
        std::string ylabel = "Time (ms)";
        Visualizer::makeBoxPlot(dataSet, labels, title, xlabel, ylabel);

            // Length benchmark
        dataSet = lengths;
        labels = {"Case 1", "Case 2", "Case 3", "Case 4"};
        title = "Parking Length Benchmark";
        xlabel = " ";
        ylabel = "Path Length";
        Visualizer::makeBoxPlot(dataSet, labels, title, xlabel, ylabel);

            // Validity benchmark
        std::vector<double> &dataSet2 = validSols;
        labels = {"Case 1", "Case 2", "Case 3", "Case 4"};
        title = "Parking Validity Benchmark";
        xlabel = " ";
        ylabel = "Number of Valid Solutions";
        Visualizer::makeBarGraph(dataSet2, labels, title, xlabel, ylabel);

        // Save figures
        Visualizer::saveFigures(true, "hw9_figs_benchmarkParking");
    }

    // Grade code
    //HW9::grade<MyKinoRRT, MySingleIntegrator, MyFirstOrderUnicycle, MySecondOrderUnicycle, MySimpleCar>("Ian.Faber@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple());

    return 0;
}
