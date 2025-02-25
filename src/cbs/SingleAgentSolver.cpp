#include "SingleAgentSolver.h"

list<int> SingleAgentSolver::getNextLocations(int curr) const // including itself and its neighbors
{
  list<int> rst = instance.getNeighbors(curr);
  rst.emplace_back(curr);
  return rst;
}

void SingleAgentSolver::compute_heuristics()
{
  struct Node
  {
    int location;
    int value;

    Node() = default;
    Node(int location, int value) : location(location), value(value) {}

    // the following is used to compare nodes in the OPEN list
    struct compare_node
    {
      // returns true if n1 > n2 (note -- this gives us *min*-heap).
      bool operator()(const Node &n1, const Node &n2) const
      {
        return n1.value >= n2.value;
      }
    }; // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
  };

  my_heuristic.resize(goal_location.size()); // the number of goal_locations for the agent
  heuristic_landmark.resize(goal_location.size(), 0);

  for (int i = 0; i < goal_location.size(); i++)
  {
    my_heuristic[i].resize(instance.map_size, MAX_TIMESTEP);

    // generate a heap that can save nodes (and a open_handle)
    boost::heap::pairing_heap<Node, boost::heap::compare<Node::compare_node>> heap;

    Node root(goal_location[i], 0);
    my_heuristic[i][goal_location[i]] = 0;
    heap.push(root); // add root to heap
    while (!heap.empty())
    {
      Node curr = heap.top();
      heap.pop();
      for (int next_location : instance.getNeighbors(curr.location))
      {
        if (my_heuristic[i][next_location] > curr.value + 1)
        {
          my_heuristic[i][next_location] = curr.value + 1;
          Node next(next_location, curr.value + 1);
          heap.push(next);
        }
      }
    }
  }

  for (int i = goal_location.size() - 2; i >= 0; i--)
  {
    heuristic_landmark[i] = heuristic_landmark[i + 1] + my_heuristic[i + 1][goal_location[i]];
  }
  // e.g. 3 goals; heuristic_landmark[2] = 0; heuristic_landmark[1] = distance between goal 2 and 1
  // heuristic_landmark[1] = distance between goal 2 and 1 + distance between goal 1 and 0

  // if is ddmapd instance, compute shortest distance to the safe parking location
  if (ddmapd_path_planning){
    parking_heuristic.resize(instance.map_size, MAX_TIMESTEP);
    auto safe_parking_loc = instance.start_locations[agent_idx];
    parking_heuristic[safe_parking_loc] = 0;

    // generate a heap that can save nodes (and a open_handle)
    boost::heap::pairing_heap<Node, boost::heap::compare<Node::compare_node>> heap;

    Node root(safe_parking_loc, 0);
    parking_heuristic[safe_parking_loc] = 0;
    heap.push(root); // add root to heap
    while (!heap.empty())
    {
      Node curr = heap.top();
      heap.pop();
      for (int next_location : instance.getNeighbors(curr.location))
      {
        if (parking_heuristic[next_location] > curr.value + 1)
        {
          parking_heuristic[next_location] = curr.value + 1;
          Node next(next_location, curr.value + 1);
          heap.push(next);
        }
      }
    }

  }


}
