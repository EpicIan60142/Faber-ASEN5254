//
//  Main source file for the ASEN 5254 Final Project
//
// Created by ianmf on 12/2/25.
//

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include "World.h"
#include "StateSpaceDatabase.h"
#include "ControlSpaceDatabase.h"
#include "StateValidityCheckerDatabase.h"
#include "StatePropagatorDatabase.h"
#include "GoalRegionDatabase.h"
#include "OptimizationObjectiveDatabase.h"
#include "PostProcessing.h"

namespace fs = std::filesystem;
namespace ob = ompl::base;
namespace oc = ompl::control;

// this function sets-up an ompl planning problem for an arbtrary number of agents
oc::SimpleSetupPtr controlSimpleSetUp(const World *w)
{
    // Grab the agent -- assume only one for demo purposes
    Cubesat *c = w->getCubesats()[0];

    // Create state and control spaces
    ob::StateSpacePtr space = createBoundedCubesatStateSpace(w->getWorldDimensions()[0], w->getWorldDimensions()[1], w->getWorldDimensions()[2]);
    oc::ControlSpacePtr cspace = createUniform3DRealVectorControlSpace(space, c);

    // Define a simple setup class from the state and control spaces
    auto ss = std::make_shared<oc::SimpleSetup>(cspace);

    // Set state validity checker (includes collision checking)
    ss->setStateValidityChecker(std::make_shared<isStateValid_3D>(ss->getSpaceInformation(), w, c));

    // Use the ODESolver to propagate the system.
    auto odeFunction = [w](const oc::ODESolver::StateType &q, const oc::Control *control, oc::ODESolver::StateType &qdot)
    {
        CubesatCWHODE(q, control, qdot, w->getMeanMotion());
    };
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), odeFunction));
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));//, &SecondOrderCarODEPostIntegration));

    // Set up optimization objectives
    ss->setOptimizationObjective(getMinimizeCostObjective(ss->getSpaceInformation(), w->getRings()[1]));

    // Create start state
    ob::ScopedState<> start(space);
    start[0] = c->getStartLocation()[0];
    start[1] = c->getStartLocation()[1];
    start[2] = c->getStartLocation()[2];
    start[3] = c->getStartLocation()[3];
    start[4] = c->getStartLocation()[4];
    start[5] = c->getStartLocation()[5];

    // Create goal region (defined in GoalRegionDatabase.h)
    ob::GoalPtr goal(new GoalRegionCubesat(ss->getSpaceInformation(), c->getGoalLocation(), w->getRings()[1]));

    // Set propagation step size and min/max number of steps
    ss->getSpaceInformation()->setPropagationStepSize(0.05);
    ss->getSpaceInformation()->setMinMaxControlDuration(5, 20);

    // Add start and goal to problem setup
    ss->setStartState(start);
    ss->setGoal(goal);

    OMPL_INFORM("Successfully Setup the problem instance");
    return ss;
}

// Sequential planning function between rings
std::vector<oc::PathControl> planSequential(const std::string plannerString, const std::string problemFile, const int cubesatIdx)
{
    // Extract the overall world
    World *w = yaml2world("../problems/" + problemFile + ".yml");
    auto worldDims = w->getWorldDimensions();

    // Extract the rings and cubesat of interest
    auto rings = w->getRings();
    auto cubesat = w->getCubesats()[cubesatIdx];

    std::vector<oc::PathControl> segments;

    OMPL_INFORM("Starting sequential planning through %zu rings", rings.size());

    // Current state starts at cubesat start location
    Eigen::VectorXd current_state = cubesat->getStartLocation();

    for (size_t ring_idx = 1; ring_idx < rings.size(); ring_idx++)
    {
        OMPL_INFORM("Planning segment %zu: Ring %zu -> Ring %zu",
                    ring_idx, ring_idx-1, ring_idx);

        // Create a simplified world for this segment
        World segmentWorld = *w;  // Copy world

        double buffer = 50; // m, buffer in each direction in addition to existing coordinate differences
        auto centerMean = (rings[ring_idx].center_ + rings[ring_idx-1].center_) / 2.0;
        auto xDiff = std::abs(rings[ring_idx].center_[0] - rings[ring_idx-1].center_[0]);
        auto yDiff = std::abs(rings[ring_idx].center_[1] - rings[ring_idx-1].center_[1]);
        auto zDiff = std::abs(rings[ring_idx].center_[2] - rings[ring_idx-1].center_[2]);
        segmentWorld.setWorldDimensions({centerMean[0] - xDiff - buffer, centerMean[0] + xDiff + buffer}, {centerMean[1] - yDiff - buffer, centerMean[1] + yDiff + buffer}, {centerMean[2] - zDiff - buffer, centerMean[2] + zDiff + buffer});

        // Set goal to current ring
        Eigen::VectorXd ringGoal(6);
        ringGoal << rings[ring_idx].center_[0], rings[ring_idx].center_[1], rings[ring_idx].center_[2],
                     rings[ring_idx].normal_[0], rings[ring_idx].normal_[1], rings[ring_idx].normal_[2];
        cubesat->setGoalLocation(ringGoal);

        // Set up planning problem for this segment
        oc::SimpleSetupPtr ss = controlSimpleSetUp(&segmentWorld);

        // Configure planner with more aggressive parameters for shorter segments
        if (plannerString == "SST")
        {
            auto planner = std::make_shared<oc::SST>(ss->getSpaceInformation());

            // More aggressive parameters for shorter segments
            planner->setGoalBias(0.6);  // Higher bias since we have clear intermediate goal
            planner->setSelectionRadius(5.0);  // Can be smaller for focused search
            planner->setPruningRadius(2.0);

            ss->setPlanner(planner);

            OMPL_INFORM("SST configured for segment %zu:", ring_idx);
            OMPL_INFORM("  Goal bias: 0.6, Selection: 5.0, Pruning: 2.0");
        }

        ss->setup();

        // Solve this segment with shorter timeout
        double segment_timeout = 30.0;  // Shorter timeout per segment
        bool solved = ss->solve(segment_timeout);

        if (solved)
        {
            auto segment_path = ss->getSolutionPath();
            segments.push_back(segment_path);

            OMPL_INFORM("Segment %zu solved! Path length: %.2f",
                        ring_idx, segment_path.length());

            // Update current_state to the final state of this segment
            auto final_state = segment_path.getState(segment_path.getStateCount() - 1);
            const double *state_vals = final_state->as<ob::CompoundStateSpace::StateType>()
                                      ->as<ob::RealVectorStateSpace::StateType>(0)->values;

            for (int i = 0; i < 6; i++) {
                current_state[i] = state_vals[i];
            }

            OMPL_INFORM("Updated start state for next segment: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                        current_state[0], current_state[1], current_state[2],
                        current_state[3], current_state[4], current_state[5]);

            // Update cubesat start to current state
            cubesat->setStartLocation(current_state);

        } else {
            OMPL_ERROR("Failed to solve segment %zu", ring_idx);
            break;  // Stop if any segment fails
        }
    }

    return segments;

}

// Function to combine segment paths into a single solution
oc::PathControl concatenatePaths(const std::vector<oc::PathControl>& segments,
                                 const ob::SpaceInformationPtr& si) {

    if (segments.empty()) {
        throw std::runtime_error("No segments to concatenate");
    }

    // Start with first segment
    oc::PathControl combined_path = segments[0];

    // Append remaining segments
    for (size_t i = 1; i < segments.size(); i++) {
        // Get states and controls from current segment
        for (size_t j = 1; j < segments[i].getStateCount(); j++) {  // Skip first state to avoid duplication
            combined_path.append(segments[i].getState(j),
                               segments[i].getControl(j-1),
                               segments[i].getControlDuration(j-1));
        }
    }

    OMPL_INFORM("Combined path has %u states, total length: %.2f",
                combined_path.getStateCount(), combined_path.length());

    return combined_path;
}


// main planning function -- uses simple setup
void planControl(std::string planner_string, std::string problem_file) {
    OMPL_INFORM("Starting sequential ring-by-ring planning");

    // Plan segments
    std::vector<oc::PathControl> segments = planSequential(planner_string, problem_file, 0);

    if (segments.empty()) {
        OMPL_ERROR("Sequential planning failed - no segments completed");
        return;
    }

    if (segments.size() < 2) {  // Adjust based on expected number of rings
        OMPL_WARN("Only completed %zu segments - incomplete solution", segments.size());
    }

    // Create a dummy SimpleSetup for output formatting
    World *w = yaml2world("../problems/" + problem_file + ".yml");
    oc::SimpleSetupPtr ss = controlSimpleSetUp(w);

    // Combine segments into single path
    try {
        oc::PathControl combined_path = concatenatePaths(segments, ss->getSpaceInformation());

        // Set the combined path as the solution
        ss->getProblemDefinition()->clearSolutionPaths();
        ss->getProblemDefinition()->addSolutionPath(std::make_shared<oc::PathControl>(combined_path));

        OMPL_INFORM("Sequential planning SUCCESS!");
        OMPL_INFORM("Total segments: %zu", segments.size());
        OMPL_INFORM("Combined path length: %.2f", combined_path.length());

        // Write solution
        write2sys(ss, w->getCubesats(), problem_file);

    } catch (const std::exception& e) {
        OMPL_ERROR("Failed to concatenate paths: %s", e.what());
    }

}

int main(int argc, char ** argv) {
    std::string plannerName = "SST";
    std::string problem = "RaceCourse";
    OMPL_INFORM("Running Race Course Problem with %s", plannerName.c_str());
    planControl(plannerName, problem);
}



