// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"

using namespace amp;

    // Declare vector mean helper function
double mean(std::vector<double> vec);

    // Main function
int main(int argc, char** argv) {
    // Initialize random seed
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Control booleans
    bool runCentral = false;
    bool benchmarkCentral = false;
    bool runDecoupled = true;
    bool benchmarkDecouple = false;

    // Centralized planning
    if (runCentral)
    {
        // 2 agent problem
                // Define path object
        MultiAgentPath2D path2agent;
                // Define problem for 2 agents
        MultiAgentProblem2D problem2agent = HW8::getWorkspace1(2);
                // Define empty vector for collisions
        std::vector<std::vector<Eigen::Vector2d>> collision_states;
                // Make a centralized planner
        MyCentralPlanner central_planner2agent;
                // Set parameters
        central_planner2agent.setParameters(7500, 0.5, 0.05, 0.25);
                // Find a path and check it
        std::cout << "Solving centralized 2 agent problem" << std::endl;
        path2agent = central_planner2agent.plan(problem2agent);
        bool isValid = HW8::check(path2agent, problem2agent, collision_states);
                // Make a figure
        //Visualizer::makeFigure(problem2agent, path2agent, collision_states);

            // 3 agent problem
        MultiAgentPath2D path3agent;
        MultiAgentProblem2D problem3agent = HW8::getWorkspace1(3);
        collision_states.clear();
        MyCentralPlanner central_planner3agent;
        central_planner3agent.setParameters(7500, 0.5, 0.05, 0.25);
        std::cout << "Solving centralized 3 agent problem" << std::endl;
        path3agent = central_planner3agent.plan(problem3agent);
        isValid = HW8::check(path3agent, problem3agent, collision_states);
        //Visualizer::makeFigure(problem3agent, path3agent, collision_states);

            // 4 agent problem
        MultiAgentPath2D path4agent;
        MultiAgentProblem2D problem4agent = HW8::getWorkspace1(4);
        collision_states.clear();
        MyCentralPlanner central_planner4agent;
        central_planner4agent.setParameters(7500, 0.5, 0.05, 0.25);
        std::cout << "Solving centralized 4 agent problem" << std::endl;
        path4agent = central_planner4agent.plan(problem4agent);
        isValid = HW8::check(path4agent, problem4agent, collision_states);
        //Visualizer::makeFigure(problem4agent, path4agent, collision_states);

            // 5 agent problem
        MultiAgentPath2D path5agent;
        MultiAgentProblem2D problem5agent = HW8::getWorkspace1(5);
        collision_states.clear();
        MyCentralPlanner central_planner5agent;
        central_planner5agent.setParameters(7500, 0.5, 0.05, 0.25);
        std::cout << "Solving centralized 5 agent problem" << std::endl;
        path5agent = central_planner5agent.plan(problem5agent);
        isValid = HW8::check(path5agent, problem5agent, collision_states);
        //Visualizer::makeFigure(problem5agent, path5agent, collision_states);

            // 6 agent problem
        MultiAgentPath2D path6agent;
        MultiAgentProblem2D problem6agent = HW8::getWorkspace1(6);
        collision_states.clear();
        MyCentralPlanner central_planner6agent;
        central_planner6agent.setParameters(7500, 0.5, 0.05, 0.25);
        std::cout << "Solving centralized 6 agent problem" << std::endl;
        path6agent = central_planner6agent.plan(problem6agent);
        isValid = HW8::check(path6agent, problem6agent, collision_states);
        //Visualizer::makeFigure(problem6agent, path6agent, collision_states);

            // Random problem
        MultiAgentPath2D pathRandom;
        MultiAgentProblem2D problemRandom;
        collision_states.clear();
        MyCentralPlanner central_plannerRandom;
        central_plannerRandom.setParameters(7500, 0.5, 0.05, 0.25);
        std::cout << "Solving centralized random multi agent problem" << std::endl;
        HW8::generateAndCheck(central_plannerRandom, pathRandom, problemRandom, collision_states);
        //Visualizer::makeFigure(problemRandom, pathRandom, collision_states);

        // Centralized benchmarking
        if (benchmarkCentral)
        {
            // Preallocate data arrays for box plots
            std::list<std::vector<double>> computeTimes;
            std::list<std::vector<double>> treeSizes;
            // Preallocate compute time vectors
            std::vector<double> agentTimes2;
            std::vector<double> agentTimes3;
            std::vector<double> agentTimes4;
            std::vector<double> agentTimes5;
            std::vector<double> agentTimes6;
            // Preallocate tree size vectors
            std::vector<double> agentSize2;
            std::vector<double> agentSize3;
            std::vector<double> agentSize4;
            std::vector<double> agentSize5;
            std::vector<double> agentSize6;
            // Run the planning problem multiple times across varying numbers of agents
            int nRuns = 100;
            for (int i = 0; i < nRuns; i++)
            {
                // Loop over different numbers of agents
                for (int j = 2; j <= 6; j++)
                {
                    std::cout << "Benchmarking centralized planning with " << j << " agents, run " << i << std::endl;
                    // Define path, problem, and planner to use
                    MultiAgentPath2D pathBenchmark;
                    MultiAgentProblem2D problemBenchmark = HW8::getWorkspace1(j);
                    MyCentralPlanner central_plannerBenchmark;
                    // Set planner parameters
                    central_plannerBenchmark.setParameters(7500, 0.5, 0.05, 0.25);
                    // Run planner and benchmark compute time
                    amp::Timer benchmarkTimer("BenchmarkTimer");
                    pathBenchmark = central_plannerBenchmark.plan(problemBenchmark);
                    double time = benchmarkTimer.now(TimeUnit::ms);
                    // Pull out tree and get its size
                    auto tree = central_plannerBenchmark.getGraph();
                    double treeSize = tree->nodes().size();
                    // Assign values to the correct data vector
                    switch (j)
                    {
                        case 2:
                            if (time >=0) // Only append compute time if it is valid
                            {
                                agentTimes2.push_back(time);
                            }
                            agentSize2.push_back(treeSize);
                            break;
                        case 3:
                            if (time >=0)
                            {
                                agentTimes3.push_back(time);
                            }
                            agentSize3.push_back(treeSize);
                            break;
                        case 4:
                            if (time >=0)
                            {
                                agentTimes4.push_back(time);
                            }
                            agentSize4.push_back(treeSize);
                            break;
                        case 5:
                            if (time >=0)
                            {
                                agentTimes5.push_back(time);
                            }
                            agentSize5.push_back(treeSize);
                            break;
                        case 6:
                            if (time >=0)
                            {
                                agentTimes6.push_back(time);
                            }
                            agentSize6.push_back(treeSize);
                            break;
                        default:
                            break;
                    }
                }
            }

            // Compile results together, find averages, and create labels
            std::vector<double> averageTimes = {mean(agentTimes2), mean(agentTimes3), mean(agentTimes4), mean(agentTimes5), mean(agentTimes6)};
            computeTimes = {agentTimes2, agentTimes3, agentTimes4, agentTimes5, agentTimes6};
            std::vector<double> averageSizes = {mean(agentSize2), mean(agentSize3), mean(agentSize4), mean(agentSize5), mean(agentSize6)};
            treeSizes = {agentSize2, agentSize3, agentSize4, agentSize5, agentSize6};
            std::vector<std::string> labels = {"2 Agents", "3 Agents", "4 Agents", "5 Agents", "6 Agents"};

            std::cout << "Average times: ";
            for (double averageTime : averageTimes)
            {
                std::cout << averageTime << ", ";
            }
            std::cout << std::endl;

            std::cout << "Average sizes: ";
            for (double averageSize : averageSizes)
            {
                std::cout << averageSize << ", ";
            }
            std::cout << std::endl;

            // Create boxplots
            //Visualizer::makeBoxPlot(computeTimes, labels, "Centralized Planning Time Benchmark", "Number of Agents", "Elapsed Time [ms]");
            //Visualizer::makeBoxPlot(treeSizes, labels, "Centralized Planning Tree Size Benchmark", "Number of Agents", "Tree Size");
        }

        // Visualize centralized planner
        Visualizer::saveFigures(true, "hw8_figs_Centralized");
    }

    // Decoupled planning
    if (runDecoupled)
    {
            // 2 agent problem
        MultiAgentPath2D path2agent;
        MultiAgentProblem2D problem2agent = HW8::getWorkspace1(2);
        std::vector<std::vector<Eigen::Vector2d>> collision_states;
        MyDecentralPlanner decentral_planner2agent;
        decentral_planner2agent.setParameters(7500, 0.5, 0.05, 0.25);
        std::cout << "Solving decoupled 2 agent problem" << std::endl;
        path2agent = decentral_planner2agent.plan(problem2agent);
        bool isValid = HW8::check(path2agent, problem2agent, collision_states);
        //Visualizer::makeFigure(problem2agent, path2agent, collision_states);

            // 3 agent problem
        MultiAgentPath2D path3agent;
        MultiAgentProblem2D problem3agent = HW8::getWorkspace1(3);
        collision_states.clear();
        MyDecentralPlanner decentral_planner3agent;
        decentral_planner3agent.setParameters(7500, 0.5, 0.05, 0.25);
        std::cout << "Solving decoupled 3 agent problem" << std::endl;
        path3agent = decentral_planner3agent.plan(problem3agent);
        isValid = HW8::check(path3agent, problem3agent, collision_states);
        //Visualizer::makeFigure(problem3agent, path3agent, collision_states);

            // 4 agent problem
        MultiAgentPath2D path4agent;
        MultiAgentProblem2D problem4agent = HW8::getWorkspace1(4);
        collision_states.clear();
        MyDecentralPlanner decentral_planner4agent;
        decentral_planner4agent.setParameters(7500, 0.5, 0.05, 0.25);
        std::cout << "Solving decoupled 4 agent problem" << std::endl;
        path4agent = decentral_planner4agent.plan(problem4agent);
        isValid = HW8::check(path4agent, problem4agent, collision_states);
        //Visualizer::makeFigure(problem4agent, path4agent, collision_states);

            // 5 agent problem
        MultiAgentPath2D path5agent;
        MultiAgentProblem2D problem5agent = HW8::getWorkspace1(5);
        collision_states.clear();
        MyDecentralPlanner decentral_planner5agent;
        decentral_planner5agent.setParameters(7500, 0.5, 0.05, 0.25);
        std::cout << "Solving decoupled 5 agent problem" << std::endl;
        path5agent = decentral_planner5agent.plan(problem5agent);
        isValid = HW8::check(path5agent, problem5agent, collision_states);
        //Visualizer::makeFigure(problem5agent, path5agent, collision_states);

            // 6 agent problem
        MultiAgentPath2D path6agent;
        MultiAgentProblem2D problem6agent = HW8::getWorkspace1(6);
        collision_states.clear();
        MyDecentralPlanner decentral_planner6agent;
        decentral_planner6agent.setParameters(7500, 0.5, 0.05, 0.25);
        std::cout << "Solving decoupled 6 agent problem" << std::endl;
        path6agent = decentral_planner6agent.plan(problem6agent);
        isValid = HW8::check(path6agent, problem6agent, collision_states);
        //Visualizer::makeFigure(problem6agent, path6agent, collision_states);

            // Random problem
        MultiAgentPath2D path;
        MultiAgentProblem2D problem = HW8::getWorkspace1(3);
        MyDecentralPlanner decentral_planner;
        collision_states = {{}};
        std::cout << "Solving decoupled random multi agent problem" << std::endl;
        HW8::generateAndCheck(decentral_planner, path, problem, collision_states);
        //Visualizer::makeFigure(problem, path, collision_states);

        // Decoupled benchmarking
        if (benchmarkDecouple)
        {

        }

        // Visualize decoupled planner
        Visualizer::saveFigures(true, "hw8_figs_Decoupled");
    }

    // Grade planners
    HW8::grade<MyCentralPlanner, MyDecentralPlanner>("Ian.Faber@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());

    return 0;
}

double mean(std::vector<double> vec)
{
    double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
    return sum / vec.size();
}