#include "MyAStar.h"

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;

        // Initialize open and closed lists, which consist of nodes and priorities
    std::vector<std::pair<amp::Node, double>> open_list;
    std::vector<amp::Node> closed_list;

        // Start open list with children of the initial node and their priorities
    open_list.push_back({problem.init_node, heuristic(problem.init_node)});

        // Loop until open list is empty or we find a solution
    while (!open_list.empty())
    {
            // Extract lowest priority node, assuming they're sorted by priority in ascending order
        amp::Node lowestPriorityNode = open_list[0].first;

            // Add lowest priority node to the closed list
        closed_list.push_back(lowestPriorityNode);

            // If lowest priority node was the goal, exit the loop
        if (lowestPriorityNode == problem.goal_node)
        {
            break;
        }

            // Erase lowest priority node from the open list
        open_list.erase(open_list.begin());

            // Loop through children of the lowest priority node and add them to the open list if they aren't in it already
        std::vector<amp::Node> children = problem.graph->children(lowestPriorityNode);
        for (int i = 0; i < children.size(); i++)
        {
            amp::Node child = children[i];

            bool partOfOpenList = false;
            for (int j = 0; j < open_list.size(); j++)
            {
                    // If child node exists in the open list, ignore it
                if (open_list[j].first == child)
                {
                    partOfOpenList = true;
                    break;
                }
            }

            if (!partOfOpenList)
            {
                open_list.push_back({child, heuristic(child)});
            }
        }
    }

    bool exitedEarly = open_list.empty();

    GraphSearchResult result = {false, {}, 0.0}; // initialize the results object
    result.node_path.push_back(problem.init_node);
    result.node_path.push_back(problem.goal_node);
    result.path_cost += 1.0;

    result.print();
    return result;
}
