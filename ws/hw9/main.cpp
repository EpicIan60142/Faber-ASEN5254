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
    int select = 2;
    KinodynamicProblem2D prob = problems[select];
    MyKinoRRT kino_planner;
    KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());

    /*
    for (size_t i = 0; i < path.waypoints.size(); i++) {
        std::cout << "Waypoint " << i << ": ";
        for (int j = 0; j < path.waypoints[i].size(); j++) {
            std::cout << path.waypoints[i][j] << " ";
        }
        std::cout << std::endl;
    }

    for (size_t i = 0; i < path.controls.size(); i++) {
        std::cout << "Control " << i << ": ";
        for (int j = 0; j < path.controls[i].size(); j++) {
            std::cout << path.controls[i][j] << " ";
        }
        std::cout << std::endl;
    }

    for (size_t i = 0; i < path.controls.size(); i++) {
        std::cout << "Duration " << i << ": ";
        std::cout << path.durations[i] << " ";
        std::cout << std::endl;
    }
    */

    HW9::check(path, prob);
    if (path.valid)
        Visualizer::makeFigure(prob, path, true); // Set to 'true' to render animation
    Visualizer::saveFigures(true, "hw9_figs");
    HW9::grade<MyKinoRRT, MySingleIntegrator, MyFirstOrderUnicycle, MySecondOrderUnicycle, MySimpleCar>("Ian.Faber@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple());
    return 0;
}