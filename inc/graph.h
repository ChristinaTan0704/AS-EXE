#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <stack>
#include <algorithm>  // for std::remove, std::find
#include <fstream>

class DirectedGraph {
public:
    DirectedGraph() = default;

    DirectedGraph(const DirectedGraph& other) {
        // Deep copy of adjacency lists
        adjList = other.adjList;
        toFrom = other.toFrom;
    }

    // Adjacency list to represent the graph (from -> to)
    std::unordered_map<int, std::vector<int>> adjList;

    // Reverse adjacency list to represent incoming edges (to -> from)
    std::unordered_map<int, std::vector<int>> toFrom;

    // Add an edge from 'from' to 'to'
    void addEdge(int from, int to) {
        adjList[from].push_back(to);
        toFrom[to].push_back(from);
    }

    // Remove a node by its ID
    bool removeNode(int node) {
        // Check if the node exists
        if (adjList.find(node) == adjList.end() && toFrom.find(node) == toFrom.end()) {
            return false;
        }

        // Remove all outgoing edges from the node
        if (adjList.find(node) != adjList.end()) {
            for (int to : adjList[node]) {
                // Remove the corresponding incoming edge in toFrom
                auto& fromList = toFrom[to];
                fromList.erase(std::remove(fromList.begin(), fromList.end(), node), fromList.end());
            }
            adjList.erase(node);
        }

        // Remove all incoming edges to the node
        if (toFrom.find(node) != toFrom.end()) {
            for (int from : toFrom[node]) {
                // Remove the corresponding outgoing edge in adjList
                auto& toList = adjList[from];
                toList.erase(std::remove(toList.begin(), toList.end(), node), toList.end());
            }
            toFrom.erase(node);
        }

        return true;
    }

    // Remove an edge from 'from' to 'to'
    bool removeEdge(int from, int to) {
        // Check if the edge exists in adjList
        if (adjList.find(from) == adjList.end()) {
            return false;
        }

        // Remove the edge from adjList
        auto& toList = adjList[from];
        auto it = std::find(toList.begin(), toList.end(), to);
        if (it != toList.end()) {
            toList.erase(it);

            // Remove the corresponding entry in toFrom
            auto& fromList = toFrom[to];
            fromList.erase(std::remove(fromList.begin(), fromList.end(), from), fromList.end());

            return true;
        }

        return false;
    }

    // Detect if there is a cycle in the graph
    bool hasCycle() {
        std::unordered_set<int> visited;
        std::unordered_set<int> inStack;
        for (const auto& pair : adjList) {
            if (hasCycleUtil(pair.first, visited, inStack)) {
                return true;
            }
        }
        return false;
    }

    // Check if there is a cycle starting from a given node
    bool hasCycleFromNode(int start) {
        std::unordered_set<int> visited;
        std::unordered_set<int> inStack;
        return hasCycleUtil(start, visited, inStack);
    }

    // Function to export the graph as a DOT file
    void exportToDOT(const std::string& filename) {
        std::ofstream file(filename);
        if (!file) {
            std::cerr << "Error opening file!\n";
            return;
        }

        file << "digraph G {\n";
        for (const auto& pair : adjList) {
            int from = pair.first;
            for (int to : pair.second) {
                file << "    " << from << " -> " << to << ";\n";
            }
        }
        file << "}\n";

        file.close();
        std::cout << "DOT file generated: " << filename << std::endl;
    }

private:
    // Utility function to check for cycles using DFS
    bool hasCycleUtil(int node, std::unordered_set<int>& visited, std::unordered_set<int>& inStack) {
        // If the node is currently in the recursion stack, we have a cycle
        if (inStack.find(node) != inStack.end()) {
            return true;
        }

        // If already visited, no need to visit again
        if (visited.find(node) != visited.end()) {
            return false;
        }

        // Mark the node as visited and add to the recursion stack
        visited.insert(node);
        inStack.insert(node);

        // Visit all the neighbors of the node
        if (adjList.find(node) != adjList.end()) {
            for (int neighbor : adjList[node]) {
                if (hasCycleUtil(neighbor, visited, inStack)) {
                    return true;
                }
            }
        }

        // Remove the node from the recursion stack once done
        inStack.erase(node);
        return false;
    }
};