#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"

// Generic PRM planner as per homework hint
class GenericPRM
{
    public:
        amp::Path plan(const Eigen::VectorXd &init_state, const Eigen::VectorXd &goal_state, const amp::ConfigurationSpace &cspace, int nSample, double rConnect, std::shared_ptr<amp::Graph<double>> &graph, std::map<amp::Node, Eigen::Vector2d> &nodes);
};

// Specific 2D planners
class MyPRM : public amp::PRM2D, public GenericPRM {
    public:
            // Constructors
        MyPRM(){};
        MyPRM(int nSample, double rConnect) : nSample(nSample), rConnect(rConnect) {}

            // Planning method
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

            // Setters
        void setParameters(const int newNSample, const double newRConnect)
        {
            if (nSample > 0) {
                this->nSample = newNSample;
            }
            if (rConnect > 0) {
                this->rConnect = newRConnect;
            }
        }

            // Getters
        std::shared_ptr<amp::Graph<double>> getGraph()
        {
            return graphPtr;
        }

        std::map<amp::Node, Eigen::Vector2d> getNodes()
        {
            return nodes;
        }

    private:
        int nSample = 200; // Number of random samples to take
        double rConnect = 1; // Radius to define connecting valid configurations
        double gridSize = 0.25; // CSpace grid size
        std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>(); // Graph
        std::map<amp::Node, Eigen::Vector2d> nodes; // Nodes
};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
};
