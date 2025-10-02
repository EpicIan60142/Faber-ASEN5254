#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

// Include obstacle checker class
#include "ObstacleChecker.h"

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double d_star, double zeta, std::vector<double> &Q_star, double eta, double epsilon) :
			d_star(d_star),
			zeta(zeta),
			Q_star(Q_star),
			eta(eta),
			epsilon(epsilon) {}

		// Setter functions
		void addQstar(double Qstar) {if (Qstar > 0.0){ Q_star.push_back(Qstar); }};

		// Getter functions
		double getdStar() const {return d_star;}
		double getzeta() const {return zeta;}
		std::vector<double> getQstar() const {return Q_star;}
		double getEta() const {return eta;}
		double getEpsilon() const {return epsilon;}

		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;
	private:
		double d_star, zeta, eta, epsilon;
		std::vector<double> Q_star;
		// Add additional member variables here...
		double alpha = 0.01; // Gradient step size
		int maxLoopCount = 1e5; // Maximum number of loops to avoid infinite looping
};

class MyPotentialFunction : public amp::PotentialFunction2D {
	public:
			// Constructor
		MyPotentialFunction(const MyGDAlgorithm &algorithm, const amp::Problem2D &problem) :
			algo(algorithm),
			prob(problem) {}

			// Returns the potential function value (height) for a given 2D point.
		virtual double operator()(const Eigen::Vector2d& q) const override {
			return U_att(q) + U_rep(q);
		}

			// Returns gradient vector
		virtual Eigen::Vector2d getGradient(const Eigen::Vector2d& q) const override {
            return gradU_att(q) + gradU_rep(q);
        }

			// Attractive potential function
		double U_att(const Eigen::Vector2d& q) const
		{
				// Pull out constants
			const double dStar = algo.getdStar();
			const Eigen::Vector2d q_goal = prob.q_goal;
			const double zeta = algo.getzeta();

				// Compute distance to goal
			const double d = (q - q_goal).norm();

				// Compute potential function value depending on d vs. dStar
			if (d <= dStar)
			{
				return 0.5*zeta*d*d;
			}

			return dStar*zeta*d - 0.5*zeta*dStar*dStar;

		}

			// Repulsive potential function
		double U_rep(const Eigen::Vector2d& q) const
		{
				// Pull out constants
			const std::vector<double> Q_star = algo.getQstar();
			const double eta = algo.getEta();

				// Create obstacle checker object
			ObstacleChecker obsCheck;
			obsCheck.setObstacles(prob.obstacles);

			// Loop through obstacles and build potential
			double U_rep = 0;
			for (int i = 0; i < prob.obstacles.size(); i++)
			{
				std::pair<double, Eigen::Vector2d> result = obsCheck.calcClosestDistance(q, i);

				double d = result.first;

				if (d <= 0.5*algo.getEpsilon())
				{
					d = 0.5*algo.getEpsilon();
				}

				if (d <= Q_star[i])
				{
					U_rep += 0.5*eta*pow((1/d) - (1/Q_star[i]), 2);
				}
			}

			return U_rep;
		}

			// Attractive gradient
		Eigen::Vector2d gradU_att(const Eigen::Vector2d &q) const
		{
				// Pull out constants
			const double dStar = algo.getdStar();
			const Eigen::Vector2d q_goal = prob.q_goal;
			const double zeta = algo.getzeta();

				// Compute distance to goal
			const double d = (q_goal - q).norm();

				// Compute gradient
			if (d <= dStar)
			{
				return zeta*(q-q_goal); //{zeta*(q[0] - q_goal[0]), zeta*(q[1] - q_goal[1]) };
			}

			return (dStar*zeta*(q-q_goal))/d; //{(dStar*zeta*(q[0]-q_goal[0]))/d, (dStar*zeta*(q[1]-q_goal[1]))/d};

		}

			// Repulsive potential function
		Eigen::Vector2d gradU_rep(const Eigen::Vector2d &q) const
		{
			// Pull out constants
			const std::vector<double> Q_star = algo.getQstar();
			const double eta = algo.getEta();

			// Create obstacle checker object
			ObstacleChecker obsCheck;
			obsCheck.setObstacles(prob.obstacles);

			// Loop through obstacles and build gradient
			Eigen::Vector2d gradU_rep = {0,0};
			for (int i = 0; i < prob.obstacles.size(); i++)
			{
				std::pair<double, Eigen::Vector2d> result = obsCheck.calcClosestDistance(q, i);

				double d = result.first;
				Eigen::Vector2d grad = result.second;

				if (d <= 0.5*algo.getEpsilon())
				{
					d = 0.5*algo.getEpsilon();
				}

				if (d <= Q_star[i])
				{
					const Eigen::Vector2d component = eta*((1/Q_star[i]) - (1/d))*(grad/pow(d,2)); //{eta*((1/Q_star[i]) - (1/d))*(grad[0]/pow(d,2)), eta*((1/Q_star[i]) - (1/d))*(grad[1]/pow(d,2))};
					gradU_rep += component;
				}
			}

			return gradU_rep;

		}

	private:
		MyGDAlgorithm algo;
		amp::Problem2D prob;
};