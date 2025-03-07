#include "PBS.h"
#include "SpaceTimeAStar.h"
#include <stack>
typedef pairing_heap<CBSNode *, compare<CBSNode::compare_node>> dfs_stack_t;
#include <algorithm>
#include <random>


// Comparator function that compares based on the values in segment_value map
bool compareBySegmentValue(const int& a, const int& b, const std::map<int, int>& segment_value) {
    return segment_value.at(a) < segment_value.at(b);
}


void PBS::printPaths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		cout << "Agent " << i << " : ";
    for (int loc_idx = 0; loc_idx < final_paths[i].path.size(); loc_idx++)
    {
      if (final_paths[i].path[loc_idx].segmentID!=-1){
        cout << final_paths[i].path[loc_idx].location << " @ " << "(" << search_engines[0]->instance.getRowCoordinate(final_paths[i].path[loc_idx].location) << "," << search_engines[0]->instance.getColCoordinate(final_paths[i].path[loc_idx].location) << ") " << " [ t " << loc_idx << " sID " << final_paths[i].path[loc_idx].segmentID << " taskID " << segments[final_paths[i].path[loc_idx].segmentID].taskID << " ] --> ";
      }
      else{
        cout << final_paths[i].path[loc_idx].location << " @ " << "(" << search_engines[0]->instance.getRowCoordinate(final_paths[i].path[loc_idx].location) << "," << search_engines[0]->instance.getColCoordinate(final_paths[i].path[loc_idx].location) << ") " << " [ t " << loc_idx << " sID " << final_paths[i].path[loc_idx].segmentID << " ] --> ";
      }
    }
    cout << endl;
	}
}

//PBS::PBS(const Instance &instance, int screen): CBS(instance, screen)
//{
//  clock_t t = clock();
//
//  this->screen = screen;
//  num_of_tasks = instance.segments.size();
//  num_of_agents = instance.getDefaultNumberOfAgents();
//  ddmapd_instance = instance.ddmapd_instance;
//  vertices = instance.vertices;
//  agent_parkLoc = instance.agent_parkLoc;
//
//  // build graph
//  for (auto vertex : vertices)
//  {
//    for (int type1_id : vertex.type1_idList)
//    {
//      initial_graph.addEdge(vertex.id, type1_id);
//    }
//    for (int type2_id : vertex.type2_idList)
//    {
//      initial_graph.addEdge(vertex.id, type2_id);
//    }
//  }
//
//  if (initial_graph.hasCycle()){
//    cout << "Initial graph has cycle" << endl;
//    exit(1);
//  }
//  else{
//    cout << "Good, initial graph has no cycle" << endl;
//  }
//
//
//}



//PBS::PBS(const Instance &instance, int screen): CBS(instance, screen)
//{
//  clock_t t = clock();
//
//  this->screen = screen;
//  num_of_tasks = instance.segments.size();
//  num_of_agents = instance.getDefaultNumberOfAgents();
//  ddmapd_instance = instance.ddmapd_instance;
//  vertices = instance.vertices;
//  agent_parkLoc = instance.agent_parkLoc;
//
//  // build graph
//  for (auto vertex : vertices)
//  {
//    for (int type1_id : vertex.type1_idList)
//    {
//      initial_graph.addEdge(vertex.id, type1_id);
//    }
//    for (int type2_id : vertex.type2_idList)
//    {
//      initial_graph.addEdge(vertex.id, type2_id);
//    }
//  }
//
//  if (initial_graph.hasCycle()){
//    cout << "Initial graph has cycle" << endl;
//    exit(1);
//  }
//  else{
//    cout << "Good, initial graph has no cycle" << endl;
//  }
//
//
//}

bool PBS::valid_connected_segment(int curr_segment_ID, int next_segment_ID, int current_agent)
{
  bool continuous = false;
  bool released = false;

  // check if the segment trajectory is continuous
  int pre_segment_end = segments[curr_segment_ID].trajectory.back();
  int curr_segment_start = segments[next_segment_ID].trajectory.front();
  
  // debug TODO del
  // cout << "pre_segment_end " << pre_segment_end << " curr_segment_start " << curr_segment_start << endl;
  if (pre_segment_end == curr_segment_start){
    continuous = true;
  }

  // debug TODO del
  // cout << "curr_segment_ID " << curr_segment_ID << " next_segment_ID " << next_segment_ID << " earlist_start_time[next_segment_ID] " << earlist_start_time[next_segment_ID] << " completion_time[curr_segment_ID] " << completion_time[curr_segment_ID] << endl;
  // debug TODO del

  // check if the segment is released
  if (earlist_start_time[next_segment_ID] <= completion_time[curr_segment_ID])
  {
    released = true;
  }
  // cout << "valid_connected_segment " << continuous << released << endl; // debug TODO del
  return continuous and released;
}

std::tuple<std::vector<int>, std::vector<int>> PBS::get_combined_segments(int curr_segment_id)
{

  int current_agent = segments[curr_segment_id].agent;
  int current_pos = segments[curr_segment_id].seq_pos;
  int curr_task = segments[curr_segment_id].taskID;
  vector<int> combined_segments = {curr_segment_id};
  vector<int> future_connected_segments = {};
  for (int pos = current_pos + 1; pos < goal_segmentIDs[current_agent].size(); pos++)
  {
    int next_segment_id = goal_segmentIDs[current_agent][pos];
    // debug TODO del
    // cout << "next_segment_id " << next_segment_id << " curr_inDegree " << curr_inDegree[next_segment_id] << endl;
    if (curr_inDegree[next_segment_id] == 0 and valid_connected_segment(curr_segment_id, next_segment_id, current_agent))
    {
      if (segments[next_segment_id].taskID == curr_task)
      {
        combined_segments.push_back(next_segment_id);
        curr_inDegree[next_segment_id] = -1; // mark as visited
        for (int dep : segments[next_segment_id].parents)
        {
          curr_inDegree[dep]--;
        }
      }
      else
      {
        future_connected_segments.push_back(next_segment_id);
      }
    }
    curr_segment_id = next_segment_id;
  }
  return std::make_tuple(combined_segments, future_connected_segments);
}

bool PBS::all_planned()
{
  for (int degree : curr_inDegree)
  {
    if (degree != -1)
    {
      return false;
    }
  }
  cout << "all " << curr_inDegree.size() << " segments are planned " << endl;
  return true;
}

void PBS::build_constrain_table(ConstraintTable & curr_ct_table, int agent_id){

  // curr_ct_table.copy(agent_constraint_table[agent_id]);
  for (int i = 0; i < num_of_agents; i++){
    if (i != agent_id and !agent_dummy_paths[i].empty() and agent_dummy_paths[i].size() > 0){
      curr_ct_table.addPath(agent_dummy_paths[i], true);
    }
  }
  
  for (int i = 0 ; i < final_paths.size(); i++){
    if (final_paths[i].size() > 0 and segments[i].agent != agent_id){
      curr_ct_table.addPath(final_paths[i], false);
    }
  }

  // add agent_future_paths
  for (int i = 0; i < num_of_agents; i++){
    if (i != agent_id and !agent_future_paths[i].empty() and agent_future_paths[i].size() > 0){
      curr_ct_table.addPath(agent_future_paths[i], false);
    }
  }
  agent_constraint_table[agent_id].ct.clear();
  agent_constraint_table[agent_id].copy(curr_ct_table);

}

bool PBS::solve(double time_limit, int cost_lowerbound, int cost_upperbound)
{


}

void PBS::Update_state(Path planned_path, int dummy_path_length, int agent_id, vector<int> future_connected_segments){

  // update path 
  int pre_segment_ID = planned_path.path[0].segmentID;
  Path temp;
  int curr_time = planned_path.begin_time;
  temp.begin_time = curr_time;
  for (int idx = 0; idx < planned_path.path.size(); idx++){
    int curr_segmentID = planned_path.path[idx].segmentID;
    if (pre_segment_ID != curr_segmentID){
      // update path for pre_segment_ID
      final_paths[pre_segment_ID] = temp;

      temp.path.clear();
      temp.path.push_back(final_paths[pre_segment_ID].path.back()); // add the start loc
      temp.begin_time = curr_time - 1;
      pre_segment_ID = curr_segmentID;
    }
    temp.path.push_back(planned_path.path[idx]);
    curr_time ++ ; 
  }

  // get dummy path 
  int start_idx = planned_path.size() - dummy_path_length; 
  Path dummy_path;
  dummy_path.path.resize(dummy_path_length);
  for (int i = start_idx; i < planned_path.path.size(); i++)
  {
    dummy_path.path[i - start_idx] = planned_path.path[i];
  }
  dummy_path.begin_time = planned_path.begin_time + start_idx;
  agent_dummy_paths[agent_id].path = dummy_path.path;
  agent_dummy_paths[agent_id].begin_time = dummy_path.begin_time;

  // get path excluding dummy path
  Path pure_path;
  for (int i = 0; i < planned_path.size() - dummy_path_length; i++)
  {
    pure_path.path.push_back(planned_path.path[i]);
  }
  pure_path.begin_time = planned_path.begin_time;
  agent_start_time[agent_id] = pure_path.end_time();

  // compose path for future_connected_segments
  Path future_path;
  if (!future_connected_segments.empty()){
    for (int segmentID : future_connected_segments){
      future_path.path.insert(future_path.path.end(), final_paths[segmentID].path.begin() + 1, final_paths[segmentID].path.end());
    }
    future_path.begin_time = pure_path.end_time() + 1; 
  }


  // check if future_path is constrained by agent's path OR dummy path other than the current agent (agent_id)
  bool future_constrained = false;
  if (future_path.size() > 0){
    for (int agent = 0; agent < num_of_agents; agent++){
      if (agent != agent_id ){
        if (agent_constraint_table[agent].constrained(future_path)){
          future_path.path.clear();
          future_constrained = true;
          break;
        }
      }
    }
  }

  // update agent_future_paths
  if (!future_constrained){
    agent_future_paths[agent_id].path = future_path.path;
    agent_future_paths[agent_id].begin_time = future_path.begin_time;
  }


  // debug TODO del

  cout << "pure path : ";
  cout << "begin time " << pure_path.begin_time << " end time " << pure_path.end_time() << endl;
  for (int i = 0; i < pure_path.path.size(); i++)
  {
    cout << pure_path.path[i].location << " @ " << "(" << search_engines[0]->instance.getRowCoordinate(pure_path.path[i].location) << "," << search_engines[0]->instance.getColCoordinate(pure_path.path[i].location) << ") " << " [ t " << i + pure_path.begin_time << " ] --> ";
  }

  cout << endl;

  if (!future_connected_segments.empty()){
    cout << "future path : ";
    cout << "begin time " << future_path.begin_time << " end time " << future_path.end_time() << endl;
    for (int i = 0; i < future_path.path.size(); i++)
    {
      cout << future_path.path[i].location << " @ " << "(" << search_engines[0]->instance.getRowCoordinate(future_path.path[i].location) << "," << search_engines[0]->instance.getColCoordinate(future_path.path[i].location) << ") " << " [ t " << i + future_path.begin_time << " ] --> ";
    }

    cout << endl;
  }

  cout << "dummy path : ";
  cout << "begin time " << dummy_path.begin_time << " end time " << dummy_path.end_time() << endl;
  for (int i = 0; i < dummy_path.path.size(); i++)
  {
    cout << dummy_path.path[i].location << " @ " << "(" << search_engines[0]->instance.getRowCoordinate(dummy_path.path[i].location) << "," << search_engines[0]->instance.getColCoordinate(dummy_path.path[i].location) << ") " << " [ t " << i + dummy_path.begin_time << " ] --> ";
  }
  cout << endl;

  // debug TODO del



}

void PBS::join_paths()
{
  // TODO double check on this 
  cout << "join path" << endl;
  joined_paths.resize(num_of_agents);
  for (int i = 0; i < num_of_agents; i++)
  {
    for (int j = 0; j < goal_segmentIDs[i].size(); j++)
    {
      int task_id = goal_segmentIDs[i][j];
      if (j == 0)
      {
        joined_paths[i].path.push_back(final_paths[task_id].front());
      }

      assert(joined_paths[i].size() - 1 == final_paths[task_id].begin_time);
      for (int k = 1; k < final_paths[task_id].size(); k++)
      {
        joined_paths[i].path.push_back(final_paths[task_id].at(k));
      }
    }
  }
  final_paths.resize(num_of_agents);

  for (int i = 0; i < num_of_agents; i++)
  {
    final_paths[i] = joined_paths[i];
    // add dummy path 
    final_paths[i].path.insert(final_paths[i].path.end(), agent_dummy_paths[i].path.begin(), agent_dummy_paths[i].path.end());
  }

}

PBS::~PBS()
{
  releaseNodes();
  mdd_helper.clear();
}

int PBS::get_dis_between_task(int start_taskID, int end_taskID) // ID is the global ID
{
  int start_end = segments[start_taskID].trajectory.back();
  int end_start = segments[end_taskID].trajectory.front();
  return search_engines[0]->instance.getManhattanDistance(start_end, end_start);
  // int start_agent, start_task, end_agent, end_task;
  // tie(start_agent, start_task) = id2task[start_taskID]; // get the agent and task local ID
  // tie(end_agent, end_task) = id2task[end_taskID];       // get the agent and task local ID
  // int start_goal_loc = segments[start_taskID].trajectory.front();
  // int end_goal_loc = segments[end_taskID].trajectory.front();

  // if (ddmapd_instance)
  // {
  //   int traj_end = segments[start_taskID].trajectory.back();
  //   return segments[start_taskID].trajectory.size() - 1 + search_engines[start_agent]->instance.getManhattanDistance(traj_end, end_goal_loc);
  // }
  // else
  // {
  //   return search_engines[start_agent]->instance.getManhattanDistance(start_goal_loc, end_goal_loc);
  // }
}

/*
Update:
1. earlist_start_time
2. completion_time (for segment combination)
3. est_makespan
4. task_locVal 
*/
void PBS::UpdateTaskEst()
{
  vector<int> earliest_completion_time;
  earliest_completion_time.resize(num_of_tasks, 0);
  for (int task_id = 0; task_id < num_of_tasks; task_id++)
  {
    if (!final_paths[task_id].path.empty())
    {
      earliest_completion_time.push_back(final_paths[task_id].end_time()); // the time at the traj_end 
    }
  }

  // Step 1: Initialize the queue for topological sorting
  std::queue<int> q;
  completion_time.clear();
  earlist_start_time.clear();
  completion_time.resize(num_of_tasks, 0);    // Completion time for each task
  earlist_start_time.resize(num_of_tasks, 0); // Completion time for each task

  // Step 2: Enqueue tasks with no dependencies (in-degree == 0)
  for (int i = 0; i < num_of_tasks; ++i)
  {
    if (init_inDegree[i] == 0)
    {
      q.push(i);
      // agent_parkLocations
      completion_time[i] = max(earliest_completion_time[i],
                              search_engines[0]->instance.getManhattanDistance(agent_parkLocations[segments[i].agent], segments[i].trajectory.front()) + segments[i].traj_len - 1); // agent start loc + task traj_len

    }
  }

  int visited_count = 0; // To detect cycles
  auto in_degree = init_inDegree;

  // Step 3: Process the tasks in topological order
  while (!q.empty())
  {
    int current = q.front();
    q.pop();
    visited_count++;

    // Traverse all tasks dependent on the current task
    for (int next : segments[current].parents)
    { 
      completion_time[next] = max({
          completion_time[next],                                                                     // Current completion time of `next`
          completion_time[current] + get_dis_between_task(current, next) + segments[next].traj_len - 1, // Completion time of `current` + distance
          earliest_completion_time[next]                                                             // Earliest start time of `next`
      });

      // Decrement the in-degree and enqueue if it becomes 0
      if (--in_degree[next] == 0)
      {
        q.push(next);
      }
    }
  }

  // Detect cycle: If not all tasks are visited
  if (visited_count != num_of_tasks)
  {
    cout << "Cycle detected in task dependencies!" << endl;
  }

  // Step 4: Find the maximum completion time
  est_makespan = 0;
  for (int i = 0; i < num_of_tasks; ++i)
  {
    est_makespan = max(est_makespan, completion_time[i]);
    earlist_start_time[i] = completion_time[i] - (segments[i].traj_len - 1);
    earlist_start_time[i] = max(0, earlist_start_time[i]); // this is not very accurate for the completed task but accurate for the uncompleted task
  }
  

}

void PBS::Update_task_locVal()
{
  for (int i = 0; i < num_of_tasks; i++){
    task_locVal[i].clear();
    task_locVal[i].resize(map_size, 0);
  }
  
  std::unordered_map<int, std::vector<int>> estStart_task;

  for (int i = 0; i < num_of_tasks; ++i)
  {
    estStart_task[earlist_start_time[i]].push_back(i);
  }

  // Step 4: get the completion time from completion_time and trajectory length for dd-mapd instance
  std::unordered_map<int, std::vector<int>> estStart_locs;
  for (auto one_task_info : estStart_task)
  {
    int est_start_time = one_task_info.first;
    for (auto task_id : one_task_info.second)
    {
      int offset_t = 0;
      for (int loc : segments[task_id].trajectory)
      {
        estStart_locs[est_start_time + offset_t].push_back(loc);
        offset_t++;
      }
    }
  }

  for (int i = 0; i < num_of_tasks; i++)
  {
    if (!final_paths[i].path.empty()) // the trajectory is already added in space-time obstacles
    {
      continue;
    }

    int task_start = earlist_start_time[i];
    int start_time = max(0, task_start - task_gap_threshold);
    int agent, task;
    tie(agent, task) = id2task[i];
    int end_time = task_start + search_engines[agent]->agent_segments[task].trajectory.size();
    end_time = min(end_time, task_start + task_gap_threshold);
    double relevant_index = 1;
    for (int t = start_time; t < end_time; t++)
    {
      if (estStart_locs.find(t) == estStart_locs.end()) // if not trajectory on this time
      {
        continue;
      }
      for (int loc : estStart_locs[t])
      {
        double localVal;
        if (t == task_start)
        {
          localVal = locVal_offset;
        }
        else
        {
          localVal = locVal_offset / (double)abs(t - task_start);
        }
        task_locVal[i][loc] += localVal;
      }
    }
  }
}

vector<int> PBS::get_independent_segment()
{

  vector<int> independent_segments;
  // Collect segments with in-degree 0
  for (int segment_ID = 0; segment_ID < curr_inDegree.size(); ++segment_ID)
  {
    if (curr_inDegree[segment_ID] == 0)
    {
      independent_segments.push_back(segment_ID);
    }
  }

  // mark the in-degree of dependent segments as visited
  for (int segment_ID : independent_segments)
  {
    curr_inDegree[segment_ID] = -1;
    // update the in-degree of the dependent segments
    for (int next_segmentID : segments[segment_ID].parents)
    {
      curr_inDegree[next_segmentID]--;
    }
  }


      

  return independent_segments;
}

bool PBS::validateSolution()
{
  for (int a1 = 0; a1 < num_of_agents; a1++)
  {
    for (int a2 = 0; a2 < num_of_agents; a2++)
    {
      if (a1 == a2){continue;}

      size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
      for (size_t timestep = 0; timestep < min_path_length; timestep++)
      {
        int loc1 = paths[a1]->at(timestep).location;
        int loc2 = paths[a2]->at(timestep).location;
        if (loc1 == loc2)
        {
          cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << endl;
          return false;
        }
        else if (timestep < min_path_length - 1
             && loc1 == paths[a2]->at(timestep + 1).location
             && loc2 == paths[a1]->at(timestep + 1).location)
        {
          cout << "Agents " << a1 << " and " << a2 << " collides at (" <<
             loc1 << "-->" << loc2 << ") at timestep " << timestep << endl;
          return false;
        }
      }
      if (paths[a1]->size() != paths[a2]->size())
      {
        int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
        int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
        int loc1 = paths[a1_]->back().location;
        for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
        {
          int loc2 = paths[a2_]->at(timestep).location;
          if (loc1 == loc2)
          {
            cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << endl;
            return false; // It's at least a semi conflict
          }
        }
      }
    }
  }
  cout << "Success: No conflicts found in the solution." << endl;
  return true;
}


string PBS::getSolverName() const
{
  return "PBS";
}