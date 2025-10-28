#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"

class MyKinoRRT : public amp::KinodynamicRRT {
    public:
        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;

    private:
        int nSample = 50000; // Number of random node samples to take
        int uSample = 1; // How many random controls to sample per node
        double rConnect = 1; // Radius to define connecting valid configurations
        double dt = 0.1; // Step size for propagating dynamics in seconds
        std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>(); // Graph
        std::map<amp::Node, Eigen::VectorXd> nodes; // Nodes
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
    private:
        double radius = 0.25; // Unicycle wheel radius
};

class MySecondOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, const double dt) override;
        Eigen::VectorXd dynamics(Eigen::VectorXd &state, const Eigen::VectorXd &control);
    private:
        double radius = 0.25; // Unicycle wheel radius
};

class MySimpleCar : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, const double dt) override;
        Eigen::VectorXd dynamics(Eigen::VectorXd &state, const Eigen::VectorXd &control);
    private:
        double length = 5;
        double width = 2;
};