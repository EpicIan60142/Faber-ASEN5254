#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW8.h"

// This is file is loaded from the shared/ directory
// Overwrite with your MySamplingBasedPlanners.h and MySamplingBasedPlanners.cpp from hw7
#include "MySamplingBasedPlanners.h" 


class MyCentralPlanner : public amp::CentralizedMultiAgentRRT {
    public:
            // Constructor
        MyCentralPlanner() {};
        MyCentralPlanner(int nSample, double rConnect, double pGoal, double epsilon)
            : nSample(nSample), rConnect(rConnect), pGoal(pGoal), epsilon(epsilon) {};

            // Planning method
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

            // Getters and setters
        void setParameters(int newNSample, double newRConnect, double newPGoal, double newEpsilon)
        {
            if (newNSample > 0) { nSample = newNSample; }
            if (newRConnect > 0) { rConnect = newRConnect; }
            if (newPGoal > 0) { pGoal = newPGoal; }
            if (newEpsilon > 0) { epsilon = newEpsilon; }
        }

        std::shared_ptr<amp::Graph<double>> getGraph()
        {
            return graphPtr;
        }

    private:
        int nSample = 7500; // Number of tree nodes to sample
        double rConnect = 0.5; // Step size for extending tree nodes
        double pGoal = 0.05; // Goal bias probability
        double epsilon = 0.25; // Threshold for hitting the goal
        std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>(); // Graph
        std::map<amp::Node, Eigen::VectorXd> nodes; // Nodes
};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT {
    public:
            // Constructor
        MyDecentralPlanner() {};
        MyDecentralPlanner(int nSample, double rConnect, double pGoal, double epsilon)
            : nSample(nSample), rConnect(rConnect), pGoal(pGoal), epsilon(epsilon) {};

            // Planning method
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

            // Getters and setters
        void setParameters(int newNSample, double newRConnect, double newPGoal, double newEpsilon)
        {
            if (newNSample > 0) { nSample = newNSample; }
            if (newRConnect > 0) { rConnect = newRConnect; }
            if (newPGoal > 0) { pGoal = newPGoal; }
            if (newEpsilon > 0) { epsilon = newEpsilon; }
        }

        std::shared_ptr<amp::Graph<double>> getGraph()
        {
            return graphPtr;
        }

    private:
        int nSample = 7500; // Number of tree nodes to sample
        double rConnect = 0.5; // Step size for extending tree nodes
        double pGoal = 0.05; // Goal bias probability
        double epsilon = 0.25; // Threshold for hitting the goal
        int maxTries = 100; // Number of priority orders to attempt
        std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>(); // Graph
        std::map<amp::Node, Eigen::VectorXd> nodes; // Nodes
};