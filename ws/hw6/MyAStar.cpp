#include "MyAStar.h"

#include <queue> // Provides access to priority_queue

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;

        // Define a priority queue object that automatically sorts by priority - this comes from the queue built-in library
            // Comparison lambda function that'll sort by priority, not node
    auto cmp = [](const std::pair<amp::Node, double>& a, const std::pair<amp::Node, double>& b)
    {
        return a.second > b.second;
    };
            // Priority queue that uses cmp to sort a vector of pairs (Node, priority) by priority
    std::priority_queue<std::pair<amp::Node, double>,
                        std::vector<std::pair<amp::Node, double>>,
                        decltype(cmp)> openList(cmp);

        // Define a set for the closed list
    std::set<amp::Node> closedList;

        // Define maps for keeping track of the smallest path lengths for each node and for updating parent pointers
    std::map<amp::Node, double> pathLength;
    std::map<amp::Node, amp::Node> parentPtr;

        // Initialize A* with the starting node
            // Initialize path length mapping
    pathLength[problem.init_node] = 0.0;
            // Calculate priority
    double priority = pathLength[problem.init_node] + heuristic(problem.init_node);
            // Push starting node onto the priority queue
    openList.push({problem.init_node, priority});

        // Begin A* algorithm
    int iterations = 0;
    while (!openList.empty())
    {
            // Extract node with lowest priority, which is at the top of the priority queue
        amp::Node lowestPriorityNode = openList.top().first;

            // Remove from the open list and append to the closed list
        openList.pop();
        closedList.insert(lowestPriorityNode);

            // Check if we've reached the goal
        if (lowestPriorityNode == problem.goal_node)
        {
                // Initialize the result object as a successful path
            GraphSearchResult result = {true, {}, pathLength[lowestPriorityNode]};

                // Reconstruct the path by following parent pointers all the way to the init_node
            amp::Node currentNode = lowestPriorityNode;
            while (currentNode != problem.init_node)
            {
                    // Add current node to the result path
                result.node_path.push_back(currentNode);
                    // Update current node to be the parent node
                currentNode = parentPtr[currentNode];
            }
                // Append initial node
            result.node_path.push_back(problem.init_node);

                // Reverse order of the path, which was constructed from goal to initial node
            std::reverse(result.node_path.begin(), result.node_path.end());

                // Print and return result
            iterations++;
            std::cout << "A* Graph Search: Found path in " << iterations << " iterations" << std::endl;
            result.print();
            return result;
        }

            // Expand the lowest priority node
        std::vector<amp::Node> children = problem.graph->children(lowestPriorityNode);
        std::vector<double> edges = problem.graph->outgoingEdges(lowestPriorityNode);

            // Add children to open list and update path lengths/parent pointers
        for (size_t i = 0; i < children.size(); i++)
        {
                // Extract current child
            amp::Node child = children[i];

                // If current child is already in the closed list, skip it
            if (closedList.contains(child))
            {
                continue;
            }

                // Calculate path length from parent to child
            double pathLengthToChild = pathLength[lowestPriorityNode] + edges[i];

                // Assign/update path length and parent pointers, and append child to the open list
            if (!pathLength.contains(child) || pathLengthToChild < pathLength[child])
            {
                pathLength[child] = pathLengthToChild;
                parentPtr[child] = lowestPriorityNode;
                double priority = pathLength[child] + heuristic(child);
                openList.push({child, priority});
            }
        }
        iterations++;
    }

    std::cout << "A* Graph Search: No path found for " << problem.init_node << " --> " << problem.goal_node << " after " << iterations << " iterations" << std::endl;
    GraphSearchResult result = {false, {}, 0.0};
    result.print();
    return result;
}
