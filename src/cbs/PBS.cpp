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

PBS::PBS(const Instance &instance, int screen) : CBS(instance, false, heuristics_type::ZERO, screen)
{
  clock_t t = clock();

  this->screen = screen;
  num_of_tasks = instance.segments.size();
  num_of_agents = instance.getDefaultNumberOfAgents();
  ddmapd_instance = instance.ddmapd_instance;
  focal_w = 1;

  dependency_graph = instance.dependency_graph;
  segments = instance.segments;
  est_makespan = 0;
  completion_time.resize(num_of_tasks, 0);
  earlist_start_time.resize(num_of_tasks, 0);
  agent_start_time.resize(num_of_agents, 0);
  map_size = instance.map_size;
  task_locVal.resize(num_of_tasks);
  agent_future_paths.resize(num_of_agents);
  for (int i = 0; i < num_of_tasks; i++){
    task_locVal[i].resize(map_size, 0);
  }
  id2task.resize(num_of_tasks);
  agent_constraint_table.resize(num_of_agents);
  agent_dummy_paths.resize(num_of_agents);
  final_paths.resize(num_of_tasks);
  goal_segmentIDs = instance.goal_segmentIDs;
  agent_parkLocations = instance.start_locations;
  // initilize the id2task
  for (auto one_segment : instance.segments)
  {
    id2task[one_segment.id] = make_pair(one_segment.agent, one_segment.seq_pos);
  }

  // Calculate in-degrees
  curr_inDegree.resize(dependency_graph.size(), 0); // dependency_graph[to_segment] = {from_segment}
  for (int segment_id = 0; segment_id < num_of_tasks; segment_id++)
  {
    curr_inDegree[segment_id] = dependency_graph[segment_id].size();

  }

  // debug TODO del
  // cout << "inDegree : ";
  // for (int i = 0; i < num_of_tasks; i++)
  // {
  //   cout << "## segmentID " << i << " inDegree " << curr_inDegree[i] << endl;
  //   if (curr_inDegree[i] == 0){
  //     for (auto one_parent : segments[i].parents){
  //       cout << "parent " << one_parent << " " << " inDegree " << curr_inDegree[one_parent] << endl;
  //     }
  //   }
  // }
  // debug TODO del

  init_inDegree = curr_inDegree;
  search_engines.resize(num_of_agents);

  // idbase.resize(num_of_agents, 0);
  // initilize the search engines for each agent
  for (int i = 0; i < num_of_agents; i++)
  {
    search_engines[i] = new MultiLabelSpaceTimeAStar(instance, i);
//        if (i != 0)
    // {
    //   idbase[i] = idbase[i - 1] + search_engines[i - 1]->goal_location.size();
    // }
  }

  runtime_preprocessing = (double)(clock() - t) / CLOCKS_PER_SEC;

  if (screen >= 2) // print start and goals
  {
    instance.printAgents();
  }

}

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

  double update_time = 0;
  double build_ct_time = 0;
  double est_time = 0;
  double a_start_time = 0;
  auto sys_start_time = clock();


  vector<int> agent_current_loc;
  vector<int> segment_traj;
  vector<int> trajEnds;
  vector<int> segmentIDs;
  std::map<int, int> agent_pathLen;
  agent_current_loc.resize(num_of_agents);

  auto est_start_time = clock();
  UpdateTaskEst();
  if (dummy_avoid){
    Update_task_locVal();
  }

  int makespan_lower_bound = 0;
  int cost_lower_bound = 0;

  for (int i = 0; i < goal_segmentIDs.size(); i++)
  {
    makespan_lower_bound = max(makespan_lower_bound, completion_time[goal_segmentIDs[i].back()]);
    cost_lower_bound += completion_time[goal_segmentIDs[i].back()];
  }
  
  cout << "makespan_lower_bound " << makespan_lower_bound << " cost_lower_bound " << cost_lower_bound << endl;

  // exit(0); // TODO del
  est_time += (double)(clock() - est_start_time) / CLOCKS_PER_SEC;


  for (int i = 0; i < num_of_agents; i++)
  {
    agent_current_loc[i] = search_engines[i]->start_location;
  }
  
  while (!all_planned())
  {
    vector<int> independent_segments;
    independent_segments = get_independent_segment();

    if (independent_segments.size() > num_of_agents){
      cout << "independent segments size " << independent_segments.size() << " num of agents " << num_of_agents << endl;
      return false;
    }

    if (independent_segments.empty())
    {
      cout << "no independent segments but planning not finished" << endl;
      return false;
    }
		// ("heuristic", po::value<int>()->default_value(1), "(1) random (2) cost (3) makespan (4) number of dependencies")
    if (heuristic == 1) // random priority
    {
      std::shuffle(independent_segments.begin(), independent_segments.end(), std::default_random_engine(std::random_device()()));
    }
    else{
      std::map<int, int> segment_value;
      if (heuristic == 2) // cost, prioritize larger cost
      {
        for (int segment_id : independent_segments)
        {
          segment_value[segment_id] = agent_pathLen[segments[segment_id].agent];
        }
      }
      if (heuristic == 3) // makespan, prioritize larger makespan
      {
        for (int segment_id : independent_segments)
        {
          int last_segment_id = goal_segmentIDs[segments[segment_id].agent].back();
          segment_value[segment_id] = completion_time[last_segment_id];
        }
      }
      if (heuristic == 4) // number of dependencies, prioritize larger number of dependencies
      {
        for (int segment_id : independent_segments)
        {
          segment_value[segment_id] = segments[segment_id].parents.size();
        }
      }
      std::sort(independent_segments.begin(), independent_segments.end(),
        [&segment_value](int a, int b) {
            return compareBySegmentValue(a, b, segment_value);
        });
    }

    for (int segment_id : independent_segments)
    {
      // ConstraintTable ct;
      int curr_agent = segments[segment_id].agent;
      int curr_pos = segments[segment_id].seq_pos;
      auto [combined_segments, future_connected_segments] = get_combined_segments(segment_id);
      int curr_len = 0;
      for (int index = 0; index < combined_segments.size(); index++)
      {
        int one_segment_id = combined_segments[index];
        if (index==0){
          segment_traj.insert(segment_traj.end(), segments[one_segment_id].trajectory.begin(), segments[one_segment_id].trajectory.end());
          curr_len += segments[one_segment_id].trajectory.size();
        }
        else{
          // exclude the first location of the segment trajectory
          segment_traj.insert(segment_traj.end(), segments[one_segment_id].trajectory.begin() + 1, segments[one_segment_id].trajectory.end());
          curr_len += segments[one_segment_id].trajectory.size() - 1;
        }
        trajEnds.push_back(curr_len - 1);
        segmentIDs.push_back(one_segment_id);
      }

      
      ConstraintTable* ct = new ConstraintTable();
      auto build_ct_start_time = clock();
      build_constrain_table(*ct, curr_agent); 
      build_ct_time += (double)(clock() - build_ct_start_time) / CLOCKS_PER_SEC;
      // cout << "build ct time " << build_ct_time << endl; // debug TODO del

      // if (curr_agent == 6){ // debug TODO del
      //   search_engines[curr_agent]->debug_agent = 6;
      // }
      // else{
      //   search_engines[curr_agent]->debug_agent = -1;
      // }
      auto a_start_time = clock();
      Path planned_path = search_engines[curr_agent]->findPathSegmentToParkWithTrajAvoid(*ct, agent_start_time[curr_agent], earlist_start_time[segment_id], agent_current_loc[curr_agent], search_engines[curr_agent]->start_location, segment_traj, trajEnds, segmentIDs, task_locVal[segment_id]);
      a_start_time = (double)(clock() - a_start_time) / CLOCKS_PER_SEC;
      // cout << "A* time " << a_start_time << endl; // debug TODO del
      delete ct;  // Properly destroys the object and frees memory

      if (planned_path.empty())
      {
        cout << "no solution for segment " << segment_id << endl;
        return false;
      }


      //  debug TODO del
      cout << "##### planning for agent " << curr_agent << " path start time " << agent_start_time[curr_agent] << " segment " << segment_id << " segment_earlist_start " << earlist_start_time[segment_id]  << " ##### " << endl;
      cout << "combined_segments : ";
      for (int seg : combined_segments)
      {
        cout << seg << "  ";
      }
      cout << endl;


      cout << "segment_traj : ";
      for (int i = 0; i < segment_traj.size(); i++)
      {
        // cout << segment_traj[i] << " @(" << search_engines[0]->instance.getRowCoordinate(segment_traj[i]) << "," << search_engines[0]->instance.getColCoordinate(segment_traj[i]) << ") t " << segment_traj.begin() + i << " --> ";
        cout << segment_traj[i] << " @(" << search_engines[0]->instance.getRowCoordinate(segment_traj[i]) << "," << search_engines[0]->instance.getColCoordinate(segment_traj[i])  << " --> ";
      }
      cout << endl;

      cout << "trajEnds : ";
      for (int loc : trajEnds)
      {
        cout << loc << " @(" << search_engines[0]->instance.getRowCoordinate(segment_traj[loc]) << "," << search_engines[0]->instance.getColCoordinate(segment_traj[loc]) << ") --> ";
      }
      cout << endl;


      cout << "planned_path : ";
      for (int i = 0; i < planned_path.size(); i++)
      {
        cout << planned_path[i].location << " @(" << search_engines[0]->instance.getRowCoordinate(planned_path[i].location) << "," << search_engines[0]->instance.getColCoordinate(planned_path[i].location) << ") t " << planned_path.begin_time + i << " --> ";
      }
      cout << endl;



      agent_current_loc[curr_agent] = segment_traj.back();

      auto update_start_time = clock();
      Update_state(planned_path, search_engines[curr_agent]->dummy_path_len, curr_agent, future_connected_segments);
      update_time += (double)(clock() - update_start_time) / CLOCKS_PER_SEC;
      // cout << "update time " << update_time << endl; // debug TODO del

      est_start_time = clock();
      UpdateTaskEst();
      if (dummy_avoid){
        Update_task_locVal();
      }
      est_time += (double)(clock() - est_start_time) / CLOCKS_PER_SEC;
      // cout << "est time " << est_time << endl; // debug TODO del

      segment_traj.clear();
      trajEnds.clear();
      segmentIDs.clear();
      agent_pathLen[curr_agent] = planned_path.size() - search_engines[curr_agent]->dummy_path_len - 1; // -1 minus the previous start location

        if (curr_agent == 5){
            int debug = 1;
        }
    }
    independent_segments.clear();
  }

  join_paths();
  printPaths();

  int makespan = 0;
  int sum_of_cost = 0;
  paths.resize(final_paths.size(), nullptr);
  for (int i= 0 ; i < num_of_agents; i++){
    int path_len = final_paths[i].path.size() - agent_dummy_paths[i].path.size();
    cout << "agent " << i << " path_length " << path_len << " " ;
    makespan = max(makespan, path_len);
    sum_of_cost += path_len;
    paths[i] = &final_paths[i];
  }
  cout << endl;
  cout << "makespan " << makespan << " sum_of_cost " << sum_of_cost << endl;

  cout << "runtime " << (double)(clock() - sys_start_time) / CLOCKS_PER_SEC << " a_start_time " << a_start_time << " update_time " << update_time << " build_ct_time " << build_ct_time << " est_time " << est_time << endl;

  if (validateSolution()){
    
    return true;
  }
  else{
    return false;
  }

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