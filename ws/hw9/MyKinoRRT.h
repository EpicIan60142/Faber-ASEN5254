#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"

#include "CSpace.h"

class MyKinoRRT : public amp::KinodynamicRRT {
    public:
        // Constructor
        MyKinoRRT() {};

        // Planning method
        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;

        // Node distance method
        double calculateDistance(const Eigen::VectorXd &state1, const Eigen::VectorXd &state2, const amp::KinodynamicProblem2D &problem);

    private:
        int nSample = 50000; // Number of random node samples to take
        int uSample = 15; // How many random controls to sample per node
        double rConnect = 0.5; // Radius to define connecting valid configurations
        double pGoal = 0.05; // Probability of selecting the goal point as a configuration
        std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>(); // Graph
        std::map<amp::Node, Eigen::VectorXd> nodes; // Configuration nodes
        std::map<amp::Node, Eigen::VectorXd> controls; // Control nodes
        std::map<amp::Node, double> durations; // Duration nodes
};  

class MySingleIntegrator : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
        Eigen::VectorXd dynamics(Eigen::VectorXd &state, const Eigen::VectorXd &control);
};

class MyFirstOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, const double dt) override;
        Eigen::VectorXd dynamics(Eigen::VectorXd &state, const Eigen::VectorXd &control);
};

class MySecondOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, const double dt) override;
        Eigen::VectorXd dynamics(Eigen::VectorXd &state, const Eigen::VectorXd &control);
};

class MySimpleCar : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, const double dt) override;
        Eigen::VectorXd dynamics(Eigen::VectorXd &state, const Eigen::VectorXd &control);
};