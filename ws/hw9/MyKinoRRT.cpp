#include "MyKinoRRT.h"

// Runge-Kutta 4th order integrator template
template<typename AgentType>
Eigen::VectorXd RK4(AgentType &agent, Eigen::VectorXd& state, const Eigen::VectorXd& control, const double dt)
{
    // Intermediate states
    Eigen::VectorXd w1 = agent.dynamics(state, control);
    Eigen::VectorXd tempState = state + (0.5 * dt * w1);
    Eigen::VectorXd w2 = agent.dynamics(tempState, control);
    tempState = state + (0.5 * dt * w2);
    Eigen::VectorXd w3 = agent.dynamics(tempState, control);
    tempState = state + (dt * w3);
    Eigen::VectorXd w4 = agent.dynamics(tempState, control);

    // Propagate
    return state + ((dt/6.0) * (w1 + 2*w2 + 2*w3 + w4));
}

// Kinodynamic RRT
amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    amp::KinoPath path;
    Eigen::VectorXd state = problem.q_init;
    path.waypoints.push_back(state);
    for (int i = 0; i < 10; i++) {
        Eigen::VectorXd control = Eigen::VectorXd::Random(problem.q_init.size());
        agent.propagate(state, control, this->dt);
        path.waypoints.push_back(state);
        path.controls.push_back(control);
        path.durations.push_back(this->dt);
    }
    path.valid = true;
    return path;
}

// Single Integrator
void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, const double dt)
{
    state = RK4(*this, state, control, dt);
};

Eigen::VectorXd MySingleIntegrator::dynamics(Eigen::VectorXd &state, const Eigen::VectorXd &control)
{
    return control;
}

// First order unicycle
void MyFirstOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, const double dt)
{
    state = RK4(*this, state, control, dt);
}

Eigen::VectorXd MyFirstOrderUnicycle::dynamics(Eigen::VectorXd &state, const Eigen::VectorXd &control)
{
    // Extract states used in dynamics
    double theta = state[2];

    // Calculate rates of change
    double xDot = control[0]*this->radius*std::cos(theta);
    double yDot = control[0]*this->radius*std::sin(theta);
    double thetaDot = control[1];

    // Assign output
    Eigen::VectorXd dX = Eigen::VectorXd(state.size());
    dX << xDot, yDot, thetaDot;

    return dX;
}

// Second order unicycle
void MySecondOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, const double dt)
{
    state = RK4(*this, state, control, dt);
}

Eigen::VectorXd MySecondOrderUnicycle::dynamics(Eigen::VectorXd &state, const Eigen::VectorXd &control)
{
    // Extract states used in dynamics
    double theta = state[2];
    double sigma = state[3];
    double omega = state[4];

    // Calculate rates of change
    double xDot = sigma*this->radius*std::cos(theta);
    double yDot = sigma*this->radius*std::sin(theta);
    double thetaDot = omega;
    double sigmaDot = control[0];
    double omegaDot = control[1];

    // Assign output
    Eigen::VectorXd dX = Eigen::VectorXd(state.size());
    dX << xDot, yDot, thetaDot, sigmaDot, omegaDot;

    return dX;
}

// Simple car
void MySimpleCar::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, const double dt)
{
    state = RK4(*this, state, control, dt);
}

Eigen::VectorXd MySimpleCar::dynamics(Eigen::VectorXd &state, const Eigen::VectorXd &control)
{
    // Extract states used in dynamics
    double theta = state[2];
    double v = state[3];
    double phi = state[4];

    // Calculate rates of change
    double xDot = v*std::cos(theta);
    double yDot = v*std::sin(theta);
    double thetaDot = (v/this->length)*std::tan(phi);
    double vDot = control[0];
    double phiDot = control[1];

    // Assign output
    Eigen::VectorXd dX = Eigen::VectorXd(state.size());
    dX << xDot, yDot, thetaDot, vDot, phiDot;

    return dX;
}
