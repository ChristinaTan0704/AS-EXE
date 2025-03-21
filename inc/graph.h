#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <stack>
#include <algorithm>  // for std::remove, std::find
#include <fstream>
#include "common.h"
// using namespace std;

class DirectedGraph {
public:
    DirectedGraph() = default;

    DirectedGraph(const DirectedGraph& other) {
        // Deep copy of adjacency lists
        fromTo = other.fromTo;
        toFrom = other.toFrom;
    }

    // Adjacency list to represent the graph (from -> to)
    std::unordered_map<int, std::unordered_set<int>> fromTo;

    // Reverse adjacency list to represent incoming edges (to -> from)
    std::unordered_map<int, std::unordered_set<int>> toFrom;

    // Add an edge from 'from' to 'to'
    void addEdge(int from, int to) {
        cout << "addEdge " << from << " --> " << to  << endl; // debug TODO del
        fromTo[from].insert(to);
        toFrom[to].insert(from);
    }


    bool NodeInGraph(int node) {
        return fromTo.find(node) != fromTo.end() || toFrom.find(node) != toFrom.end();
    }

    bool EdgeInGraph(int from, int to) {
        return fromTo.find(from) != fromTo.end() && fromTo[from].find(to) != fromTo[from].end() ; // don't need to check toFrom
    }

    // Remove a node by its ID
    bool removeNode(int node) {
        // Check if the node exists
        if (fromTo.find(node) == fromTo.end() && toFrom.find(node) == toFrom.end()) {
            cout << "removeNode Error: " << node << " not in the graph fromTo.find(node) == fromTo.end() && toFrom.find(node) == toFrom.end()" << endl;
            return false;
        }

        // Remove all outgoing edges from the node
        if (fromTo.find(node) != fromTo.end()) {
            for (int to : fromTo[node]) {
                // Remove the corresponding incoming edge in toFrom
                // auto& fromList = toFrom[to];
                // std::cout << "remove " << node << " from " << to << " in toFrom[] " << std::endl;
                // fromList.erase(node);
                toFrom[to].erase(node);
                cout << "remove " << node << " in toFrom[to], to node " << to << " in toFrom[] " << std::endl;
            }

            fromTo.erase(node);
        }

        if (toFrom.find(node) != toFrom.end()){
            for (int from : toFrom[node]){
                // Remove the corresponding outgoing edge in fromTo
                // auto& toList = fromTo[from];
                // std::cout << "remove " << node << " from " << from << " in fromTo[] " << std::endl;
                // toList.erase(node);
                fromTo[from].erase(node);
                cout << "remove " << node << " in fromTo[from], from node " << from << " in fromTo[] " << std::endl;
            }
            toFrom.erase(node);
        }


        return true;
    }

    void updateIndegree(vector<int> & in_degree){
        for (const auto& pair : toFrom){
            in_degree[pair.first] = pair.second.size();
        }
        return;
    }

    // Remove an edge from 'from' to 'to'
    void removeEdge(int from, int to) {
        // Check if the edge exists in fromTo
        if (fromTo.find(from) == fromTo.end()) {
            cout << "removeEdge Error: fromTo.find(from) == fromTo.end()" << endl;
            return;
        }

        // Remove the edge from fromTo
        fromTo[from].erase(to);
        toFrom[to].erase(from);

        cout << "removeEdge " << from << " --> " << to  << endl;
        cout << "after removal : fromTo[from].size() " << fromTo[from].size() << " toFrom[from].size() " << toFrom[from].size() << endl;

        return;
    }

    // Detect if there is a cycle in the graph
    bool hasCycle() {
        std::unordered_set<int> visited;
        std::unordered_set<int> inStack;
        for (const auto& pair : fromTo) {
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
        for (const auto& pair : fromTo) {
            int from = pair.first;
            for (int to : pair.second) {
                file << "    " << from << " -> " << to << ";\n";
            }
        }
        file << "}\n";

        file.close();
        std::cout << "DOT file generated: " << filename << std::endl;
    }

    void check_graph(std::vector<int> &in_degree){
       return ;
        for (const auto& pair : toFrom){
            if (pair.second.size() != in_degree[pair.first]){
                std::cout << "Error: " << pair.first << " in_degree " << in_degree[pair.first] << " toFrom size " << pair.second.size() << std::endl;
            }

            for (int one_from : pair.second){
                //std::find(fromTo.begin(), fromTo.end(), one_from) == fromTo.end() 
                if (fromTo.find(one_from) == fromTo.end()){
                    std::cout << "Error: Node " << one_from  << " --> " << pair.first << " in toFrom not in fromTo List (fromNode) " << pair.first << endl;
                    std::cout << std::endl;
                }
                else if (std::find(fromTo[one_from].begin(), fromTo[one_from].end(), pair.first) == fromTo[one_from].end()){
                    std::cout << "Error: Edge " << pair.first  << " --> " << one_from << " in toFrom not in fromTo one_from = " << one_from << "Current fromTo[one_from] List :" ;
                    for (int one_to : fromTo[one_from]){
                        std::cout << one_to << " ";
                    }
                    std::cout << std::endl;}
            }
        }

        for (const auto& pair : fromTo){

            for (int one_to : pair.second){
                if (toFrom.find(one_to) == toFrom.end()){
                    std::cout << "Error: Node " << pair.first  << " --> " << one_to << " in fromTo not in toFrom List (toNode) " << pair.first << endl;
                    std::cout << std::endl;
                }
                else if (std::find(toFrom[one_to].begin(), toFrom[one_to].end(), pair.first) == toFrom[one_to].end()){
                    std::cout << "Error: Edge " << one_to  << " --> " << pair.first << " in fromTo not in toFrom one_to = " << one_to << "Current toFrom[one_to] List :" ;
                    for (int one_from : toFrom[one_to]){
                        std::cout << one_from << " ";
                    }
                    std::cout << std::endl;}
            }
        }
        cout << "check_graph() : check done" << endl;
        return;
        
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
        if (fromTo.find(node) != fromTo.end()) {
            for (int neighbor : fromTo[node]) {
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