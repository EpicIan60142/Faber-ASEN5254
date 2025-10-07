#include "MyWaveFrontAlgorithm.h"

#include "Manipulator2D.h"
#include "CSpace.h"

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator)
{
    // Implement your WaveFront algorithm here
    // Make wavefront grid and utility grid by copying the passed in cspace grid
    std::pair<std::size_t, std::size_t> n_cells = grid_cspace.size();
    std::pair<double, double> x0Bounds = grid_cspace.x0Bounds();
    std::pair<double, double> x1Bounds = grid_cspace.x1Bounds();

    MyWaveFrontGridSpace waveFrontGrid(n_cells.first, n_cells.second, x0Bounds.first, x0Bounds.second, x1Bounds.first, x1Bounds.second);

    MyGridCSpace2D utilityGrid(n_cells.first, n_cells.second, x0Bounds.first, x0Bounds.second, x1Bounds.first, x1Bounds.second);

    // Initialize tracking vectors
    std::vector<std::pair<std::size_t, std::size_t>> wavefront; // Grid indices to be checked for validity, then get assigned wave levels
    std::vector<std::pair<std::size_t, std::size_t>> checked; // Grid indices that have been assigned already

    // Make sure we start with a valid goal configuration
    Eigen::Vector2d q_goal_copy = q_goal;
    if (isManipulator)
    {
        for (int i = 0; i < q_goal_copy.size(); i++)
        {
            if (q_goal_copy[i] < 0)
            {
                q_goal_copy[i] += 2*M_PI;
            }
        }
    }

    // Get cell index of goal point
    std::pair<std::size_t, std::size_t> goalIdx = grid_cspace.getCellFromPoint(q_goal_copy[0], q_goal_copy[1]);

    // Assign distance values for obstacles and goal to grid
    for (int i = 0; i < n_cells.first; i++)
    {
        for (int j = 0; j < n_cells.second; j++)
        {
            // Assign obstacle distances as 1
            if (grid_cspace(i,j) == true)
            {
                waveFrontGrid(i,j) = 1;
            }
            // Assign goal cell as a distance of 2
            else if (std::pair<std::size_t, std::size_t> {i, j} == goalIdx)
            {
                waveFrontGrid(i,j) = 2;
            }
        }
    }

    // Initialize the wavefront and wave level
    wavefront.push_back(goalIdx);
    std::size_t wavefrontSize = 0;
    double waveLevel = 2;
    bool stopWave = false;

    // Construct wavefront
    while (!stopWave)
    {
        // Assign previous wavefront to checked vector
        for (int i = 0; i < wavefront.size(); i++)
        {
            checked.push_back(wavefront[i]);
        }

        // Document how many indices were part of the last wavefront
        wavefrontSize = wavefront.size();

        // Clear the wavefront and indicate that the wave level hasn't increased yet
        bool increasedLevel = false;
        wavefront.clear();

        // Construct a candidate wavefront
        std::vector<std::pair<std::size_t, std::size_t>> candidateWavefront;
        for (std::size_t i = checked.size()-wavefrontSize; i < checked.size(); i++)
        {
            candidateWavefront.push_back(checked[i]);
        }

        // Loop through elements in the candidate wavefront vector
        for (int i = 0; i < candidateWavefront.size(); i++)
        {
            // Calculate candidate cells
            std::vector<std::pair<std::size_t, std::size_t>> candidateCells;
            if (isManipulator) // Account for angle wrapping
            {
                // Wrapping in theta2 direction
                if (candidateWavefront[i].second + 1 >= n_cells.second)
                {
                    candidateCells.push_back({candidateWavefront[i].first, 0}); // "Above cell"
                }
                else
                {
                    candidateCells.push_back({candidateWavefront[i].first, candidateWavefront[i].second+1}); // Above cell
                }

                if (candidateWavefront[i].second - 1 >= n_cells.second)
                {
                    candidateCells.push_back({candidateWavefront[i].first, n_cells.second-1}); // "Below cell"
                }
                else
                {
                    candidateCells.push_back({candidateWavefront[i].first, candidateWavefront[i].second-1}); // Below cell
                }

                // Wrapping in theta1 direction
                if (candidateWavefront[i].first + 1 >= n_cells.first)
                {
                    candidateCells.push_back({0, candidateWavefront[i].second}); // "Right cell"
                }
                else
                {
                    candidateCells.push_back({candidateWavefront[i].first+1, candidateWavefront[i].second}); // Right cell
                }

                if (candidateWavefront[i].first - 1 >= n_cells.first)
                {
                    candidateCells.push_back({n_cells.first-1, candidateWavefront[i].second}); // "Left cell"
                }
                else
                {
                    candidateCells.push_back({candidateWavefront[i].first-1, candidateWavefront[i].second}); // Left cell
                }
            }
            else
            {
                candidateCells.push_back({candidateWavefront[i].first, candidateWavefront[i].second+1}); // Above cell
                candidateCells.push_back({candidateWavefront[i].first, candidateWavefront[i].second-1}); // Below cell
                candidateCells.push_back({candidateWavefront[i].first+1, candidateWavefront[i].second}); // Right cell
                candidateCells.push_back({candidateWavefront[i].first-1, candidateWavefront[i].second}); // Left cell
            }

            // Loop over candidate cells
            for (int j = 0; j < candidateCells.size(); j++)
            {
                // Pull out cell
                std::pair<std::size_t, std::size_t> cell = candidateCells[j];

                // Check cell bounds, if outside bounds then skip
                if (cell.first < 0 || cell.first >= n_cells.first || cell.second < 0 || cell.second >= n_cells.second)
                {
                    continue;
                }

                // Check for existing assignment, only cells with values of 0 can be assigned
                if (waveFrontGrid(cell.first, cell.second) == 0)
                {
                    // Increase the wave level for all candidate cells
                    if (!increasedLevel)
                    {
                        waveLevel++;
                        increasedLevel = true;
                    }

                    // Assign level and add cell to valid wavefront
                    waveFrontGrid(cell.first, cell.second) = waveLevel;
                    wavefront.push_back(cell);
                }
            }
        }

        if (!increasedLevel)
        {
            stopWave = true;
        }
    }

    // Make path object
    amp::Path2D path;

    // Add starting point to path
        // Make sure we start with a valid configuration
    Eigen::Vector2d q_init_copy = q_init;
    if (isManipulator)
    {
        for (int i = 0; i < q_init_copy.size(); i++)
        {
            if (q_init_copy[i] < 0)
            {
                q_init_copy[i] += 2*M_PI;
            }
        }
    }
    path.waypoints.push_back(q_init_copy);

    // Unwrap initial configuration if this is a manipulator
    Eigen::Vector2d bounds0;
    Eigen::Vector2d bounds1;

    // Get starting cell
    std::pair<std::size_t, std::size_t> cell = grid_cspace.getCellFromPoint(path.waypoints.back()[0], path.waypoints.back()[1]);

    // Bound checking
    if (cell.first > n_cells.first)
    {
        cell.first = 0;
    }

    if (cell.second > n_cells.second)
    {
        cell.second = 0;
    }

        // Loop through the wavefront grid until we reach the goal point
    int loopCount = 0;
    while (waveFrontGrid(cell.first, cell.second) > 2 && loopCount < this->maxLoopCount)
    {
            // Check cells above, below, left, and right for decreasing wave level
        int waveLevel = waveFrontGrid(cell.first, cell.second);
        std::vector<std::pair<std::size_t, std::size_t>> candidateCells;

        if (isManipulator) // Account for angle wrapping
        {
                // Wrapping in theta2 direction
            if (cell.second + 1 >= n_cells.second)
            {
                candidateCells.push_back({cell.first, 0}); // "Above cell"
            }
            else
            {
                candidateCells.push_back({cell.first, cell.second+1}); // Above cell
            }

            if (cell.second - 1 >= n_cells.second)
            {
                candidateCells.push_back({cell.first, n_cells.second-1}); // "Below cell"
            }
            else
            {
                candidateCells.push_back({cell.first, cell.second-1}); // Below cell
            }

            // Wrapping in theta1 direction
            if (cell.first + 1 >= n_cells.first)
            {
                candidateCells.push_back({0, cell.second}); // "Right cell"
            }
            else
            {
                candidateCells.push_back({cell.first+1, cell.second}); // Right cell
            }

            if (cell.first - 1 >= n_cells.first)
            {
                candidateCells.push_back({n_cells.second-1, cell.second}); // "Left cell"
            }
            else
            {
                candidateCells.push_back({cell.first-1, cell.second}); // Left cell
            }
        }
        else
        {
            candidateCells.push_back({cell.first, cell.second+1}); // Above cell
            candidateCells.push_back({cell.first, cell.second-1}); // Below cell
            candidateCells.push_back({cell.first+1, cell.second}); // Right cell
            candidateCells.push_back({cell.first-1, cell.second}); // Left cell
        }

        for (int i = 0; i < candidateCells.size(); i++)
        {
                // Check that the cell is valid
            if (candidateCells[i].first < 0 || candidateCells[i].first > n_cells.first || candidateCells[i].second < 0 || candidateCells[i].second > n_cells.second)
            {
                continue;
            }

                // If the distance is decreasing but not an obstacle, assign the cell to the path and break out of the loop
            if (waveFrontGrid(candidateCells[i].first, candidateCells[i].second) < waveLevel && waveFrontGrid(candidateCells[i].first, candidateCells[i].second) != 1)
            {
                cell = candidateCells[i];
                path.waypoints.push_back(utilityGrid.getPointFromCell(cell));
                break; // break out of inner loop
            }
        }
        loopCount++;
    }

    path.waypoints.push_back(q_goal_copy);

        // Unwrap path angles if this is a manipulator
    if (isManipulator)
    {
            // Set valid bounds for CSpace path
        bounds0 = Eigen::Vector2d(0.0, 0.0);
        bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);

        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }


    return path;
}
