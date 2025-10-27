//
// Created by ianmf on 9/24/25.
//

#ifndef AMP_TOOLS_CSPACE_H
#define AMP_TOOLS_CSPACE_H

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "ObstacleChecker.h"
#include "hw/HW4.h"
#include "hw/HW6.h"

// Function for checking if a point is within the bounds of an N-D Cspace
bool isWithinBounds(const Eigen::VectorXd &point, const amp::ConfigurationSpace &cspace);

// Derive the amp::GridCSpace2D class and override the missing method
class MyGridCSpace2D : public amp::GridCSpace2D {
    public:
        MyGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) // Call base class constructor
        {}

        // Override this method for determining which cell a continuous point belongs to
        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;

        // Function that converts a cell to a continuous point, i.e. the midpoint of that cell
        Eigen::Vector2d getPointFromCell(std::pair<std::size_t, std::size_t> cell) const;

};

// Adapter class to make GridCSpace2D compatible with ConfigurationSpace
class GridCSpaceAdapter : public amp::ConfigurationSpace {
    public:
        GridCSpaceAdapter(const amp::GridCSpace2D& cspace)
            : amp::ConfigurationSpace(Eigen::Vector2d(cspace.x0Bounds().first, cspace.x1Bounds().first),
                                     Eigen::Vector2d(cspace.x0Bounds().second, cspace.x1Bounds().second)),
              grid_cspace(cspace) {}

        virtual bool inCollision(const Eigen::VectorXd& cspace_state) const override {
            if (cspace_state.size() != 2) {
                throw std::invalid_argument("Expected 2D configuration space state");
            }
            return grid_cspace.inCollision(cspace_state[0], cspace_state[1]);
        }

    private:
            const amp::GridCSpace2D& grid_cspace;
};


// Derive the HW4 ManipulatorCSConstructor class and override the missing method
class MyManipulatorCSConstructor : public amp::ManipulatorCSConstructor {
    public:
        // To make things easy, add the number of cells as a ctor param so you can easily play around with it
        MyManipulatorCSConstructor(std::size_t cells_per_dim) : m_cells_per_dim(cells_per_dim) {}

        // Override this method for computing all of the boolean collision values for each cell in the cspace
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override;

    private:
        std::size_t m_cells_per_dim;
};

// Derive the PointAgentCSConstructor class and override the missing method
class MyPointAgentCSConstructor : public amp::PointAgentCSConstructor {
public:
    // To make things easy, add the number of cells as a ctor param so you can easily play around with it
    MyPointAgentCSConstructor(std::size_t x1_cells, std::size_t x2_cells) : m_x1_cells(x1_cells), m_x2_cells(x2_cells) {}

    // Override this method for computing all of the boolean collision values for each cell in the cspace
    virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::Environment2D& env) override;

private:
    std::size_t m_x1_cells;
    std::size_t m_x2_cells;
};

// Class for a Multi-agent C-space
class MultiAgentCSpace : public amp::ConfigurationSpace
{
    public:
        // Constructor
        MultiAgentCSpace(const std::vector<amp::CircularAgentProperties> &agent_properties,
                         const amp::Environment2D &env)
            :  amp::ConfigurationSpace(computeLowerBounds(agent_properties, env), computeUpperBounds(agent_properties, env)),
               agentProps(agent_properties),
               env(env)
        {
            agentSize = agentProps[0].q_init.size();
            numAgents = agentProps.size();
            obsCheck.setObstacles(env.obstacles);
        }

        // Collision checker
        bool inCollision(const Eigen::VectorXd& metaState) const override
        {
            return !isWithinBounds(metaState, *this) || agentAgentCollision(metaState) || agentEnvCollision(metaState);
        }

        // Collision helper functions
        bool agentEnvCollision(const Eigen::VectorXd& metaState) const;
        bool agentAgentCollision(const Eigen::VectorXd& metaState) const;

    private:
        // Private members
        std::vector<amp::CircularAgentProperties> agentProps; // Properties for each agent
        amp::Environment2D env; // Environment with obstacles
        int agentSize; // Dimensions of each agents' state
        int numAgents; // Number of agents in meta state
        ObstacleChecker obsCheck; // Obstacle checker object

        // Private methods
            // Calculate lower Cspace bounds
        static Eigen::VectorXd computeLowerBounds(const std::vector<amp::CircularAgentProperties> &agent_properties, const amp::Environment2D &env)
        {
            // Get size of meta state
            int metaSize = 0;
            for (const auto& agent : agent_properties)
            {
                metaSize += agent.q_init.size();
            }

            // Calculate lower bounds for each agent
            Eigen::VectorXd lowerBounds(metaSize);
            int currentIndex = 0;
            for (const auto& agent : agent_properties)
            {
                // Assign individual agent lower bounds
                int agentSize = agent.q_init.size();
                for (int i = 0; i < agentSize; ++i)
                {
                    // We are assuming a 2D environment here!
                    if (i == 0)
                    {
                        lowerBounds[currentIndex + i] = env.x_min; // x coordinate
                    }
                    else
                    {
                        lowerBounds[currentIndex + i] = env.y_min; // y coordinate
                    }
                }
                currentIndex += agentSize;
            }
            return lowerBounds;
        }

            // Calculate upper Cspace bounds
        static Eigen::VectorXd computeUpperBounds(const std::vector<amp::CircularAgentProperties> &agent_properties, const amp::Environment2D &env)
        {
            // Determine meta state size
            int metaSize = 0;
            for (const auto& agent : agent_properties)
            {
                metaSize += agent.q_init.size();
            }

            // Calculate upper bounds for each agent
            Eigen::VectorXd upperBounds(metaSize);
            int currentIndex = 0;
            for (const auto& agent : agent_properties)
            {
                // Assign an individual agent's bounds
                int agentSize = agent.q_init.size();
                for (int i = 0; i < agentSize; ++i)
                {
                    // Assuming 2D environment!
                    if (i == 0)
                    {
                        upperBounds[currentIndex + i] = env.x_max; // x coordinate
                    }
                    else
                    {
                        upperBounds[currentIndex + i] = env.y_max; // y coordinate
                    }
                }
                currentIndex += agentSize;
            }
            return upperBounds;
        }
};

// Class for a decoupled multi-agent C-space
class DecoupledAgentCSpace : public amp::ConfigurationSpace
{
    public:
        // Constructor
        DecoupledAgentCSpace(const std::vector<amp::CircularAgentProperties> &agent_properties,
                             const amp::Environment2D &env,
                             const std::vector<amp::Path> &agent_paths,
                             const int &agent_index)
            : amp::ConfigurationSpace(computeLowerBounds(agent_properties, agent_index, env), computeUpperBounds(agent_properties, agent_index, env)),
              agentProps(agent_properties),
              env(env),
              agentPaths(agent_paths),
              agentIdx(agent_index)
        {
            obsCheck.setObstacles(env.obstacles);
            time = 0;
        }

        // Collision checkers
        bool inCollision(const Eigen::VectorXd &q) const override // Override the default collision checker
        {
            return !isWithinBounds(q, *this) || agentAgentCollision(q, time) || agentEnvCollision(q);
        }

        // Time setter
        void setTime(const int &newTime)
        {
            if (newTime > 0)
            {
                time = newTime;
            }
        }

        // Collision helper functions
        bool agentEnvCollision(const Eigen::VectorXd &q) const;
        bool agentAgentCollision(const Eigen::VectorXd &q, int time) const;

    private:
        // Private members
        std::vector<amp::CircularAgentProperties> agentProps; // Properties for each agent
        amp::Environment2D env; // Environment with obstacles
        const std::vector<amp::Path> agentPaths; // Previously computed agent paths
        const int agentIdx; // Index of the agent we're making the Cspace for
        mutable int time; // Current time
        ObstacleChecker obsCheck; // Obstacle checker object

        // Private methods
            // Calculate lower Cspace bounds
        static Eigen::VectorXd computeLowerBounds(const std::vector<amp::CircularAgentProperties> &agent_properties, const int agentIdx, const amp::Environment2D &env)
        {
            // Determine agent state size and assign bound size
            int agentSize = agent_properties[agentIdx].q_init.size();
            Eigen::VectorXd lowerBounds(agentSize);

            // Assign agent lower bounds
            for (int i = 0; i < agentSize; ++i)
            {
                // We are assuming a 2D environment here!
                if (i == 0)
                {
                    lowerBounds[i] = env.x_min; // x coordinate
                }
                else
                {
                    lowerBounds[i] = env.y_min; // y coordinate
                }
            }

            return lowerBounds;
        }

            // Calculate upper Cspace bounds
        static Eigen::VectorXd computeUpperBounds(const std::vector<amp::CircularAgentProperties> &agent_properties, const int agentIdx, const amp::Environment2D &env)
        {
            // Determine agent state size
            int agentSize = agent_properties[agentIdx].q_init.size();
            Eigen::VectorXd upperBounds(agentSize);

            // Assign agent upper bounds
            for (int i = 0; i < agentSize; ++i)
            {
                // Assuming 2D environment!
                if (i == 0)
                {
                    upperBounds[i] = env.x_max; // x coordinate
                }
                else
                {
                    upperBounds[i] = env.y_max; // y coordinate
                }
            }

            return upperBounds;
        }
};

#endif //AMP_TOOLS_CSPACE_H