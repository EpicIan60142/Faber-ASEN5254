#include "MyWaveFrontAlgorithm.h"

#include "CSpace.h"

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    // Implement your WaveFront algorithm here
        // Make wavefront grid space by copying the passed in cspace grid
    std::pair<std::size_t, std::size_t> n_cells = grid_cspace.size();
    std::pair<double, double> x0Bounds = grid_cspace.x0Bounds();
    std::pair<double, double> x1Bounds = grid_cspace.x1Bounds();

    MyWaveFrontGridSpace waveFrontGrid(n_cells.first, n_cells.second, x0Bounds.first, x0Bounds.second, x1Bounds.first, x1Bounds.second);

        // Initialize tracking vectors
    std::vector<std::pair<std::size_t, std::size_t>> wavefront; // Grid indices to be checked for validity, then get assigned wave levels
    std::vector<std::pair<std::size_t, std::size_t>> checked; // Grid indices that have been assigned already

        // Get cell index of goal point
    std::pair<std::size_t, std::size_t> goalIdx = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);

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
            candidateCells.push_back({candidateWavefront[i].first, candidateWavefront[i].second+1}); // Above cell
            candidateCells.push_back({candidateWavefront[i].first, candidateWavefront[i].second-1}); // Below cell
            candidateCells.push_back({candidateWavefront[i].first+1, candidateWavefront[i].second}); // Right cell
            candidateCells.push_back({candidateWavefront[i].first-1, candidateWavefront[i].second}); // Left cell

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

        // Make path
    amp::Path2D path;
    path.waypoints.push_back(q_init);
    path.waypoints.push_back(q_goal);

        // Check path validity
    Eigen::Vector2d bounds0;
    Eigen::Vector2d bounds1;
    if (isManipulator)
    {
        bounds0 = Eigen::Vector2d(0.0, 0.0);
        bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
    }
    else
    {
        bounds0 = Eigen::Vector2d(x0Bounds.first, x1Bounds.first);
        bounds1 = Eigen::Vector2d(x0Bounds.second, x1Bounds.second);
    }
    amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);

    return path;
}
