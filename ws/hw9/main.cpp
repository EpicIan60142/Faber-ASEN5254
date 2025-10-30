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
    // Select problem, plan, check, and visualize
    std::vector<int> probIdx = {0, 2, 4, 6, 7};
    for (int i = 0; i < probIdx.size(); i++)
    {
        std::cout << "Planning for problem " << probIdx[i] << std::endl;
        KinodynamicProblem2D prob = problems[probIdx[i]];
        MyKinoRRT kino_planner;
        KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
        HW9::check(path, prob);
        if (path.valid)
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
            //Visualizer::makeFigure(prob, path, true); // Set to 'true' to render animation
        }
    }
    Visualizer::saveFigures(true, "hw9_figs");
    HW9::grade<MyKinoRRT, MySingleIntegrator, MyFirstOrderUnicycle, MySecondOrderUnicycle, MySimpleCar>("Ian.Faber@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple());
    return 0;
}
