#include "PBS.h"
#include "SpaceTimeAStar.h"
#include <stack>
typedef pairing_heap<CBSNode*, compare<CBSNode::compare_node>> dfs_stack_t;
void PBS::printResults() const
{

  int makespan = 0;
  int total_path_len = 0;
  for (int i = 0; i < num_of_agents; i++)
  {
    makespan = max(makespan, int(paths[i]->size() - 1));
    total_path_len += paths[i]->size() - 1;
  }


  for (int task_global_ID = 0; task_global_ID < num_of_tasks; task_global_ID++)
  {
    int agent, task;
    tie(agent, task) = id2task[task_global_ID];
    cout << "Dummy Path : " << task_global_ID << " agent " << agent << " task " << task << " : ";
    for (int i = 0; i < curr_dummy_paths[task_global_ID]->size(); i++)
    {
      cout << "(" << search_engines[agent]->instance.getRowCoordinate(curr_dummy_paths[task_global_ID]->at(i).location) << ", " << search_engines[agent]->instance.getColCoordinate(curr_dummy_paths[task_global_ID]->at(i).location) << " time " << curr_dummy_paths[task_global_ID]->begin_time + i << " )@" << " --> ";
    }
    cout << endl;
  }

  if (solution_cost >= 0) // solved
    cout << "Solved,";
  else if (solution_cost == -1) // time_out
    cout << "Timeout,";
  else if (solution_cost == -2) // no solution
    cout << "No solutions,";
  else if (solution_cost == -3) // nodes out
    cout << "Nodesout,";

  cout << " makespan : " << makespan << " ,cost : " << total_path_len << " ,runtime : " << runtime << " ,HL_expanded : " << num_HL_expanded << " ,LL_expanded : " << num_LL_expanded << " ,min_f_val : " << min_f_val << " ,dummy_start_g_val : " << dummy_start->g_val << " ,dummy_start_f_val : " << dummy_start->g_val + dummy_start->h_val << endl;
  cout << "topology_sort_time : " << topology_sort_time << " sum_a_star_times: " << sum_a_star_times << " a_star runtime: " << a_star_runtime << endl;
  cout << "part_a time : " << part_a_time << " part_b time : " << part_b_time << " part_c time : " << part_c_time << " part_d time : " << part_d_time << " part_e time : " << part_e_time << " part_f time : " << part_f_time << " part_g time : " << part_g_time << " part_h time : " << part_h_time << " part_i time : " << part_i_time << endl;
  // cout << solution_cost << "," << runtime << "," <<
  //   num_HL_expanded << "," << num_LL_expanded << "," << // HL_num_generated << "," << LL_num_generated << "," <<
  //   min_f_val << "," << dummy_start->g_val << "," << dummy_start->g_val + dummy_start->h_val << "," <<
  //   endl;
}

void PBS::printPaths() const
{
  const Instance *instance = &search_engines[0]->instance;
  for (int i = 0; i < num_of_agents; i++)
  {
    cout << "Agent " << i << " (cost =  " << paths[i]->size() - 1 << "): ";

    for (int t = 0; t < paths[i]->size(); t++)
    {
      cout << "(" << instance->getRowCoordinate(paths[i]->at(t).location) << ", " << instance->getColCoordinate(paths[i]->at(t).location)  << " task " << paths[i]->at(t).task << " )@" << t;
      if (paths[i]->at(t).is_goal)
      {
        cout << "*";
      }
      cout << "->";
    }
    cout << endl;
    for (int j = 0; j < paths[i]->timestamps.size(); j++)
    {
      cout << "(" << instance->getRowCoordinate(search_engines[i]->goal_location[j]) << ", " << instance->getColCoordinate(search_engines[i]->goal_location[j]) << " task " << paths[i]->at(j).task << " )@" << paths[i]->timestamps[j];
      cout << "->";
    }
    cout << endl;
  }
}

inline bool PBS::is_task_a_final_one(int task)
{
  int agent, i;
  tie(agent, i) = id2task[task];
  return i == search_engines[agent]->goal_location.size() - 1;
}

inline void PBS::updatePaths(CBSNode *curr)
{
  for (int i = 0; i < num_of_tasks; i++)
    paths[i] = &paths_found_initially[i];
  vector<bool> updated(num_of_tasks, false); // initialized for false

  while (curr != nullptr)
  {
    for (auto it = curr->paths.begin(); it != curr->paths.end(); ++it)
    {
      if (!updated[it->first])
      {
        paths[it->first] = &(it->second);
        updated[it->first] = true;
      }
    }
    curr = curr->parent;
  }
}

// get the planned paths and dummy paths so far, add the dummy path to the end of each agent's planned path
inline void PBS::updatePathsWithDummyPaths(CBSNode *curr)
{


  for (int i = 0; i < num_of_tasks; i++)
  {
    paths[i] = &paths_found_initially[i];
    curr_dummy_paths[i] = &dummy_paths_found_initially[i];
  }

  // // debug TODO del debug
  // vector<int> debug_id = {326, 229};
  // if (ddmapd_instance)
  // {
  //   cout << "##### generateing child #####" << endl;
  //   cout << "----------------- updatePathsWithDummyPaths init -----------------" << endl;
  //   for (int i = 0; i < num_of_agents; i++)
  //   {
  //     for (int j = 0; j < search_engines[i]->goal_location.size(); j++)
  //     {
  //       if (debug_id.size() != 2 or (task2id({i, j}) != debug_id[0] and task2id({i, j}) != debug_id[1]))
  //       {
  //         continue;
  //       }
  //       if (paths[task2id({i, j})]->path.size() == 0 and curr_dummy_paths[task2id({i, j})]->path.size() == 0)
  //       {
  //         continue;
  //       }
  //       cout << "agent " << i << " task " << j << " task global ID : " << task2id({i, j}) << " path size : " << paths[task2id({i, j})]->path.size() << " dummy path size : " << curr_dummy_paths[task2id({i, j})]->path.size() << endl;
  //       cout << "path : " << endl;
  //       for (auto p : paths[task2id({i, j})]->path)
  //       {
  //         cout << p.location << " --> ";
  //       }
  //       cout << endl;
  //       cout << "dummy path : " << endl;
  //       for (auto p : curr_dummy_paths[task2id({i, j})]->path)
  //       {
  //         cout << p.location << " --> ";
  //       }
  //       cout << endl;
  //     }
  //   }
  // }
  // // debug TODO del debug


  vector<bool> updated(num_of_tasks, false); // initialized for false
  vector<bool> dummy_updated(num_of_tasks, false); // initialized for false
  while (curr != nullptr)
  {
    for (auto it = curr->paths.begin(); it != curr->paths.end(); ++it)
    {
      if (!updated[it->first])
      {
        paths[it->first] = &(it->second);
        updated[it->first] = true;
      }
    }
    for (auto it = curr->planned_dummy_paths.begin(); it != curr->planned_dummy_paths.end(); ++it)
    {
      if (!dummy_updated[it->first])
      {
        curr_dummy_paths[it->first] = &(it->second);
        dummy_updated[it->first] = true;
      }
    }
    curr = curr->parent;
  }


  // // debug TODO del debug
  // if (ddmapd_instance)
  // {
  //   vector<int> debug_id = {326, 229};
  //   cout << "##### generateing child #####" << endl;
  //   cout << "----------------- updatePathsWithDummyPaths result -----------------" << endl;
  //   for (int i = 0; i < num_of_agents; i++)
  //   {
  //     for (int j = 0; j < search_engines[i]->goal_location.size(); j++)
  //     {
  //       if (debug_id.size() != 2 or (task2id({i, j}) != debug_id[0] and task2id({i, j}) != debug_id[1]))
  //       {
  //         continue;
  //       }
  //       if (paths[task2id({i, j})]->path.size() == 0 and curr_dummy_paths[task2id({i, j})]->path.size() == 0)
  //       {
  //         continue;
  //       }
  //       cout << "agent " << i << " task " << j << " task global ID : " << task2id({i, j}) << " path size : " << paths[task2id({i, j})]->path.size() << " dummy path size : " << curr_dummy_paths[task2id({i, j})]->path.size() << endl;
  //       cout << "path : " << endl;
  //       for (auto p : paths[task2id({i, j})]->path)
  //       {
  //         cout << p.location << " --> ";
  //       }
  //       cout << endl;
  //       cout << "dummy path : " << endl;
  //       for (auto p : curr_dummy_paths[task2id({i, j})]->path)
  //       {
  //         cout << p.location << " --> ";
  //       }
  //       cout << endl;
  //     }
  //   }
  // }
  // // debug TODO del debug
  
}

void PBS::AddDummyPathToAllLastTask(vector<Path*> & raw_paths) // update the paths based on curr_dummy_paths
{
  // add dummy to the end of the planned path of the last task
  for (int agent_id = 0; agent_id < num_of_agents; agent_id++)
  {
    for (int task_id = 0; task_id < search_engines[agent_id]->goal_location.size(); task_id++)
    {
      int curr_task_global_id = task2id({agent_id, task_id});
      int next_task_global_id = get_next_taskID(curr_task_global_id);
      if (raw_paths[curr_task_global_id]->path.size() != 0 and (next_task_global_id == -1 or raw_paths[next_task_global_id]->path.size() == 0))
      {
        raw_paths[curr_task_global_id]->path.insert(raw_paths[curr_task_global_id]->path.end(), curr_dummy_paths[curr_task_global_id]->path.begin(), curr_dummy_paths[curr_task_global_id]->path.end());
        // debug TODO debug delete this
        // cout << "########## preprocess :  add dummy path to " << curr_task_global_id << endl;
        // for (auto p : raw_paths[curr_task_global_id]->path)
        // {
        //   cout << p.location << " --> ";
        // }
        // cout << endl;
        // debug TODO debug delete this

      }
    }
  }
}

PBS::PBS(const Instance &instance, int screen) : CBS(instance, false, heuristics_type::ZERO, screen)
{
  this->ddmapd_instance = instance.ddmapd_instance;
  this->screen = screen;
  this->focal_w = 1;
  this->num_of_agents = instance.getDefaultNumberOfAgents();
  // mdd_helper(initial_constraints, search_engines),
  clock_t t = clock();
  
  search_engines.resize(num_of_agents);
  idbase.resize(num_of_agents, 0);

  for (int i = 0; i < num_of_agents; i++)
  {
    search_engines[i] = new MultiLabelSpaceTimeAStar(instance, i);
    for (int j = 0; j < search_engines[i]->goal_location.size(); j++)
    {
      id2task.push_back({i, j});
    }
    if (i != 0)
    {
      idbase[i] = idbase[i - 1] + search_engines[i - 1]->goal_location.size();
    }
  }

  // initialize priorities
  for (int i = 0; i < num_of_agents; i++)
  {
    for (int j = 1; j < search_engines[i]->goal_location.size(); j++)
    {
      initial_priorities.push_back({task2id({i, j - 1}), task2id({i, j}), -1, -1, constraint_type::GPRIORITY});
    }
  }
  for (int i = 0; i < instance.temporal_cons.size(); i++)
  {
    if (instance.temporal_cons[i].empty())
    {
      continue;
    }
    int from_agent = i / num_of_agents;
    int to_agent = i % num_of_agents;
    for (auto landmarks : instance.temporal_cons[i])
    {
      int from_landmark = landmarks.first;
      int to_landmark = landmarks.second;
      cout << "add " << from_agent << "(" << from_landmark << ") " << to_agent << "(" << to_landmark << ")" << endl;
      initial_priorities.push_back({task2id({from_agent, from_landmark}), task2id({to_agent, to_landmark}), -1, -1, constraint_type::GPRIORITY});
    }
  }

  num_of_tasks = id2task.size();

  runtime_preprocessing = (double)(clock() - t) / CLOCKS_PER_SEC;

  if (screen >= 2) // print start and goals
  {
    instance.printAgents();
  }

  task_locVal.resize(num_of_tasks);
  map_size = instance.map_size;

  
}

void PBS::get_adj_list(CBSNode *node, vector<vector<int>> &adj_list)
{
  adj_list.clear();
  adj_list.resize(num_of_tasks);
  while (node != nullptr)
  {
    for (auto con : node->constraints)
    {
      int a, x, y, t;
      constraint_type type;
      tie(a, x, y, t, type) = con;
      adj_list[a].push_back(x);
    }

    node = node->parent;
  }
}

void PBS::get_adj_list(CBSNode *node, vector<vector<int>> &adj_list, vector<vector<int>> &adj_list_r)
{
  adj_list.clear();
  adj_list.resize(num_of_tasks);
  adj_list_r.clear();
  adj_list_r.resize(num_of_tasks);
  while (node != nullptr)
  {
    for (auto con : node->constraints)
    {
      int a, x, y, t;
      constraint_type type;
      tie(a, x, y, t, type) = con;
      adj_list[a].push_back(x);
      adj_list_r[x].push_back(a);
    }

    node = node->parent;
  }
}

bool PBS::topological_sort(vector<vector<int>> &adj_list, vector<int> &planning_order)
{
  clock_t topo_start_time = clock();
  planning_order.clear();
  vector<bool> closed(num_of_tasks, false);
  vector<bool> expanded(num_of_tasks, false);
  for (int i = 0; i < num_of_tasks; i++)
  {
    if (closed[i])
    {
      continue;
    }
    std::stack<int> dfs_stack;
    dfs_stack.push(i);
    while (!dfs_stack.empty())
    {
      auto task = dfs_stack.top();
      dfs_stack.pop();
      if (closed[task])
      {
        continue;
      }
      if (expanded[task])
      {
        closed[task] = true;
        planning_order.push_back(task);
      }
      else
      {
        expanded[task] = true;
        dfs_stack.push(task);
        for (auto ch : adj_list[task])
        {
          if (closed[ch])
          {
            continue;
          }
          if (expanded[ch])
          {
            cout << "detect cycle";
            return false;
          }
          dfs_stack.push(ch);
        }
      }
    }
  }
  std::reverse(planning_order.begin(), planning_order.end());

  // toposort checker
  unordered_set<int> high;
  for (auto i : planning_order)
  {
    for (auto j : adj_list[i])
    {
      if (high.find(j) != high.end())
      {
        cout << "REVERSE EDGES!" << endl;
        assert(false);
      }
    }
    high.insert(i);
  }

  assert(planning_order.size() == num_of_tasks);
  topology_sort_time += ((double)(clock() - topo_start_time) / CLOCKS_PER_SEC);
  cout << "one topological sort time: " << ((double)(clock() - topo_start_time) / CLOCKS_PER_SEC) << endl;
  
  return true;
}

unordered_set<int> reachable_set(int source, vector<vector<int>> adj_list)
{
  unordered_set<int> res;
  std::stack<int> q({source});
  while (!q.empty())
  {
    int curr = q.top();
    q.pop();
    if (res.find(curr) != res.end())
    {
      continue;
    }
    res.insert(curr);
    for (auto ch : adj_list[curr])
    {
      if (res.find(ch) == res.end())
      {
        q.push(ch);
      }
    }
  }
  return res;
}

// void PBS::build_ct_remove_dummy_path(vector<Path*> & curr_paths, ConstraintTable &ct, int task_id, vector<vector<int>> adj_list_r)
// {
//   int agent, task;
//   tie(agent, task) = id2task[task_id];

//   // if it's not the first goal, remove the dummy path of it's previous goal
//   if (task >= 1)
//   {
//     int pre_task_id = task2id({agent, task - 1});
//     int pre_task_length = curr_dummy_paths[pre_task_id]->path.size();
//     for (int i = 0; i < pre_task_length; ++i)
//     {
//       curr_paths[pre_task_id]->path.pop_back();
//     }

//     // debug TODO debug delete this
//     cout << "########## preprocess :  delete dummy path : " << endl;
//     if (ddmapd_instance)
//     {
//       cout << "dummy task global ID : " << pre_task_id << " agent " << agent << " task " << task - 1 << " length: " << pre_task_length << "  previous task end : " << search_engines[agent]->goal_location[task] << " segment_end : " << search_engines[agent]->agent_segments[task].trajectory.back() << " parking loc : " << search_engines[agent]->instance.start_locations[agent] << endl;
//       cout << "dummy segment traj: ";
//       for (auto loc : search_engines[agent]->agent_segments[task - 1].trajectory)
//       {
//         cout << loc << " ";
//       }
//       cout << endl;
//       cout << "after deleting dummy path: ";
//       for (auto p : paths_found_initially[pre_task_id].path)
//       {
//         cout << p.location << " --> ";
//       }
//       cout << endl;
//     }
//   }

//   ct.goal_location = search_engines[agent]->goal_location[task];

//   auto high_prio_agents = reachable_set(task_id, adj_list_r);
//   high_prio_agents.erase(task_id);

//   cout << "Higher-priority tasks: ";
//   for (int i = 0; i < num_of_tasks; i++)
//   {
//     if (high_prio_agents.find(i) != high_prio_agents.end())
//     {
//       // int agent, task;
//       tie(agent, task) = id2task[i];
//       // in the dummy path situation wait_at_goal does not make any difference because the other agents' parking locations are always unreachable for the current agent
//       bool wait_at_goal = task == search_engines[agent]->goal_location.size() - 1;
//       ct.addPath(*curr_paths[i], wait_at_goal);

//       cout << "(" << agent << ", " << task << ") ";
//     }
//   }
//   cout << endl;

//   for (auto precedent : temporal_adj_list_r[task_id])
//   {
//     assert(!curr_paths[precedent]->empty());
//     ct.length_min = max(ct.length_min, curr_paths[precedent]->end_time() + 1);
//   }
//   ct.latest_timestep = max(ct.latest_timestep, ct.length_min);
// }



void PBS::build_ct_with_dummypath(ConstraintTable &ct, int task_id, vector<vector<int>> adj_list_r, vector<int>planned_tasks)
{

  int curr_agent, curr_task;
  tie(curr_agent, curr_task) = id2task[task_id];
  ct.goal_location = search_engines[curr_agent]->goal_location[curr_task];

  auto high_prio_agents = reachable_set(task_id, adj_list_r);
  high_prio_agents.erase(task_id);

  cout << "Higher-priority tasks: ";
  int agent, task;
  for (int i = 0; i < num_of_tasks; i++)
  {
    if (high_prio_agents.find(i) != high_prio_agents.end()) // avoid collision with high priority agents
    {
      // int agent, task;
      tie(agent, task) = id2task[i];
      bool wait_at_goal = task == search_engines[agent]->goal_location.size() - 1;
      if (agent == curr_agent){
        // exclude the last location of the path
        auto temp = *paths[i];
        if (!temp.path.empty()){ 
          temp.path.pop_back();
        }
        ct.addPath(temp, wait_at_goal);
      }
      else{
        auto full_path = *paths[i];
        full_path.path.insert(full_path.path.end(), curr_dummy_paths[i]->path.begin(), curr_dummy_paths[i]->path.end());
        ct.addPath(full_path, wait_at_goal);
      }

      cout << "(" << agent << ", " << task << ") ";
    }
    else if (std::find(planned_tasks.begin(), planned_tasks.end(), i) != planned_tasks.end()) // avoid collision with planned dummy path
    {
      /* code */
      ct.addPath(*curr_dummy_paths[i], true);
    }
    
  }
  cout << endl;
  // cout << "soft cons: ";
  // for (int i = 0; i < num_of_tasks; i++){
  //   if (high_prio_agents.find(i) == high_prio_agents.end() && paths[i] != nullptr && !paths[i]->empty()){
  //     auto task = id2task[i];
  //     cout << "(" << task.first << ", " << task.second << ") ";
  //   }
  // }
  // cout << endl;

  // temporal cons
  for (auto precedent : temporal_adj_list_r[task_id])
  {
    assert(!paths[precedent]->empty());
    tie(agent, task) = id2task[precedent];
    if (agent == curr_agent){
      ct.length_min = max(ct.length_min, paths[precedent]->end_time());
    }else{
      ct.length_min = max(ct.length_min, paths[precedent]->end_time());
    }
    cout << "precedent " << precedent << " agent " << agent << " task " << task << " ( curr_agent " <<  curr_agent << " curr_task " << curr_task << ") end time " << paths[precedent]->end_time() << " length_min " << ct.length_min << endl;
  }
  ct.latest_timestep = max(ct.latest_timestep, ct.length_min);
}



void PBS::build_ct(ConstraintTable &ct, int task_id, vector<vector<int>> adj_list_r)
{

  int agent, task;
  tie(agent, task) = id2task[task_id];
  ct.goal_location = search_engines[agent]->goal_location[task];

  auto high_prio_agents = reachable_set(task_id, adj_list_r);
  high_prio_agents.erase(task_id);

  cout << "Higher-priority tasks: ";
  for (int i = 0; i < num_of_tasks; i++)
  {
    if (high_prio_agents.find(i) != high_prio_agents.end())
    {
      // int agent, task;
      tie(agent, task) = id2task[i];
      bool wait_at_goal = task == search_engines[agent]->goal_location.size() - 1;
      ct.addPath(*paths[i], wait_at_goal);

      cout << "(" << agent << ", " << task << ") ";
    }
  }
  cout << endl;
  // cout << "soft cons: ";
  // for (int i = 0; i < num_of_tasks; i++){
  //   if (high_prio_agents.find(i) == high_prio_agents.end() && paths[i] != nullptr && !paths[i]->empty()){
  //     auto task = id2task[i];
  //     cout << "(" << task.first << ", " << task.second << ") ";
  //   }
  // }
  // cout << endl;

  // temporal cons
  for (auto precedent : temporal_adj_list_r[task_id])
  {
    assert(!paths[precedent]->empty());
    ct.length_min = max(ct.length_min, paths[precedent]->end_time() + 1);
  }
  ct.latest_timestep = max(ct.latest_timestep, ct.length_min);
}

bool PBS::findOneConflict(int task1, int task2)
{
  assert(paths[task1] != nullptr && !paths[task1]->empty());
  assert(paths[task2] != nullptr && !paths[task2]->empty());
  if (paths[task1]->end_time() > paths[task2]->end_time())
  {
    std::swap(task1, task2);
  }
  for (int t = max(paths[task1]->begin_time, paths[task2]->begin_time) + 1; t < paths[task1]->end_time() + 1; t++)
  {
    int i1 = t - paths[task1]->begin_time;
    int i2 = t - paths[task2]->begin_time;
    if (paths[task1]->at(i1).location == paths[task2]->at(i2).location)
    {
      cout << "vertex conf! << " << task1 << " and " << task2 << " at " << "( " << paths[task1]->at(i1).location << " t " << i1 << " ) " << " and " << "( " << paths[task2]->at(i2).location << " t " << i2 << " ) " << endl;
      return true;
    }
    if (paths[task1]->at(i1 - 1).location == paths[task2]->at(i2).location && paths[task1]->at(i1).location == paths[task2]->at(i2 - 1).location)
    {
      cout << "edge conf! << " << task1 << " and " << task2 << " at " << "( " << paths[task1]->at(i1 - 1).location << " t " << i1 - 1 << " ) " << " and " << "( " << paths[task2]->at(i2).location << " t " << i2 << " ) " << " and " << "( " << paths[task1]->at(i1).location << " t " << i1 << " ) " << " and " << "( " << paths[task2]->at(i2 - 1).location << " t " << i2 - 1 << " ) " << endl;
      cout << "edge conf! " << i1 << ", " << i2 << endl;
      return true;
    }
  }
  if (is_task_a_final_one(task1))
  {
    int goal_loc_1 = paths[task1]->back().location;
    for (int t = max(paths[task1]->end_time(), paths[task2]->begin_time + 1); t < paths[task2]->end_time(); t++)
    {
      int i2 = t - paths[task2]->begin_time;
      if (goal_loc_1 == paths[task2]->at(i2).location)
      {
        cout << "goal vertex conf!" << endl;
        return true;
      }
    }
  }

  return false;
}



bool PBS::findOneConflictWithDummyPath(int task1, int task2)
{
  assert(paths[task1] != nullptr && !paths[task1]->empty());
  assert(paths[task2] != nullptr && !paths[task2]->empty());
  
  int task1_agent = id2task[task1].first;
  int task2_agent = id2task[task2].first;
  if (task1_agent != task2_agent)
  {
    AddDummyPathToLastTask(*paths[task1], *curr_dummy_paths[task1]);
    AddDummyPathToLastTask(*paths[task2], *curr_dummy_paths[task2]);
  }

  if (paths[task1]->end_time() > paths[task2]->end_time())
  {
    std::swap(task1, task2); // let task1 be the one first ends
  }

  // if (ddmapd_instance){
  //   // debug TODO del debug
  //   cout << "added dummy path to task" << endl;
  //   cout << "task 1 : " << task1 << " agent : " << task1_agent << " task 2 : " << task2 << " agent : " << task2_agent << endl;
  //   cout << "task 1 path size : " << paths[task1]->path.size() << " dummy path size : " << curr_dummy_paths[task1]->path.size() << endl;
  //   cout << "task 1 path : ";
  //   for (auto p : paths[task1]->path)
  //   {
  //     cout << p.location << " --> ";
  //   }
  //   cout << endl;
  //   cout << "task 1 dummy path : ";
  //   for (auto p : curr_dummy_paths[task1]->path)
  //   {
  //     cout << p.location << " --> ";
  //   }
  //   cout << endl;
  //   cout << "task 2 path size : " << paths[task2]->path.size() << " dummy path size : " << curr_dummy_paths[task2]->path.size() << endl;
  //   cout << "task 2 path : ";
  //   for (auto p : paths[task2]->path)
  //   {
  //     cout << p.location << " --> ";
  //   }
  //   cout << endl;
  //   cout << "task 2 dummy path : ";
  //   for (auto p : curr_dummy_paths[task2]->path)
  //   {
  //     cout << p.location << " --> ";
  //   }
  //   cout << endl;
  //   // debug TODO del debug
  // }

  

  // if ((task1 == 326 and task2 == 229) or (task1 == 229 and task2 == 326)){
  //   int debug = 1;
  // }

  // debug TODO del debug
  // cout << "paths[task1]->begin_time " << paths[task1]->begin_time << " paths[task2]->begin_time " << paths[task2]->begin_time << " paths[task1]->end_time() " << paths[task1]->end_time() <<  " paths[task2]->end_time() " << paths[task2]->end_time() << endl;
  for (int t = max(paths[task1]->begin_time, paths[task2]->begin_time) + 1; t < paths[task1]->end_time() + 1; t++)
  {
    int i1 = t - paths[task1]->begin_time;
    int i2 = t - paths[task2]->begin_time;
    // debug TODO del debug
    // cout << " t " << t << " i1 " << i1  << " paths[task1]->at(i1).location " << paths[task1]->at(i1).location << " i2 " << i2 << " paths[task2]->at(i2).location " << paths[task2]->at(i2).location << endl;
    if (paths[task1]->at(i1).location == paths[task2]->at(i2).location)
    {
      cout << "vertex conf! << " << task1 << " and " << task2 << " at " << "( " << paths[task1]->at(i1).location << " t " << i1 << " ) " << " and " << "( " << paths[task2]->at(i2).location << " t " << i2 << " ) " << endl;
      if (task1_agent != task2_agent)
      {
        remove_dummy_path(curr_dummy_paths[task1]->path.size(), *paths[task1]);
        remove_dummy_path(curr_dummy_paths[task2]->path.size(), *paths[task2]);
      }
      return true;
    }
    if (paths[task1]->at(i1 - 1).location == paths[task2]->at(i2).location && paths[task1]->at(i1).location == paths[task2]->at(i2 - 1).location)
    {
      cout << "edge conf! << " << task1 << " and " << task2 << " at " << "( " << paths[task1]->at(i1 - 1).location << " t " << i1 - 1 << " ) " << " and " << "( " << paths[task2]->at(i2).location << " t " << i2 << " ) " << " and " << "( " << paths[task1]->at(i1).location << " t " << i1 << " ) " << " and " << "( " << paths[task2]->at(i2 - 1).location << " t " << i2 - 1 << " ) " << endl;
      cout << "edge conf! " << i1 << ", " << i2 << endl;
      if (task1_agent != task2_agent)
      {
        remove_dummy_path(curr_dummy_paths[task1]->path.size(), *paths[task1]);
        remove_dummy_path(curr_dummy_paths[task2]->path.size(), *paths[task2]);
      }
      return true;
    }
  }
  if (is_task_a_final_one(task1))
  {
    int goal_loc_1 = paths[task1]->back().location;
    for (int t = max(paths[task1]->end_time(), paths[task2]->begin_time + 1); t < paths[task2]->end_time(); t++)
    {
      int i2 = t - paths[task2]->begin_time;
      if (goal_loc_1 == paths[task2]->at(i2).location)
      {
        cout << "goal vertex conf!" << endl;
        if (task1_agent != task2_agent)
        {
          remove_dummy_path(curr_dummy_paths[task1]->path.size(), *paths[task1]);
          remove_dummy_path(curr_dummy_paths[task2]->path.size(), *paths[task2]);
        }
        return true;
      }
    }
  }
  if (task1_agent != task2_agent)
  {
    remove_dummy_path(curr_dummy_paths[task1]->path.size(), *paths[task1]);
    remove_dummy_path(curr_dummy_paths[task2]->path.size(), *paths[task2]);
  }
  return false;
}


bool PBS::generateChild(CBSNode *node, CBSNode *parent)
{

  // debug TODO del debug
  vector<int> debug_id = {};
  if (ddmapd_instance)
  {
    cout << "##### generateing child #####" << endl;
    cout << "----------------- initial paths -----------------" << endl;
    for (int i = 0; i < num_of_agents; i++)
    {
      for (int j = 0; j < search_engines[i]->goal_location.size(); j++)
      {
        if (paths[task2id({i, j})]->path.size() == 0 and curr_dummy_paths[task2id({i, j})]->path.size() == 0)
        {
          continue;
        }
        cout << "agent " << i << " task " << j << " task global ID : " << task2id({i, j}) << " path size : " << paths[task2id({i, j})]->path.size() << " dummy path size : " << curr_dummy_paths[task2id({i, j})]->path.size() << endl;
        cout << "path : " << endl;
        for (auto p : paths[task2id({i, j})]->path)
        {
          cout << p.location << " --> ";
        }
        cout << endl;
        cout << "dummy path : " << endl;
        for (auto p : curr_dummy_paths[task2id({i, j})]->path)
        {
          cout << p.location << " --> ";
        }
        cout << endl;
      }
    }
  }
  // debug TODO del debug
  branch_a_star_times = 0;
  vector<int> planned_tasks;
  clock_t part_b_start_time = clock();
  clock_t t1 = clock();
  node->parent = parent;
  vector<Path *> copy(paths);
  vector<Path *> dummy_copy(curr_dummy_paths);
  vector<vector<int>> adj_list, adj_list_r;
  clock_t part_c_start = clock();
  get_adj_list(node, adj_list, adj_list_r);
  cout << "part_c get_adj_list time: " << ((double)(clock() - part_c_start) / CLOCKS_PER_SEC) << endl;
  part_c_time += ((double)(clock() - part_c_start) / CLOCKS_PER_SEC);

  // remove paths that are affected;
  clock_t part_d_start = clock();
  auto affected_tasks = reachable_set(std::get<1>(node->constraints.front()), adj_list);
  for (auto task : affected_tasks)
  {
    node->paths.emplace_back(task, Path());
    node->planned_dummy_paths.emplace_back(task, Path());
    paths[task] = &node->paths.back().second;
    curr_dummy_paths[task] = &node->planned_dummy_paths.back().second;
    // cout << "affected task globalID : " << task << " paths[task] size : " << paths[task]->path.size() << " curr_dummy_paths[task] size : " << curr_dummy_paths[task]->path.size() << endl; // debug TODO del debug
  }
  // initialize unaffected tasks to be in planned_tasks
  for( int i = 0; i < num_of_tasks; i++){
    if (affected_tasks.find(i) == affected_tasks.end() && paths[i] != nullptr && !paths[i]->empty()){
      planned_tasks.push_back(i);
    }
  }
  cout << "part_d remove paths time: " << ((double)(clock() - part_d_start) / CLOCKS_PER_SEC) << endl;
  part_d_time += ((double)(clock() - part_d_start) / CLOCKS_PER_SEC);
  // if (ddmapd_instance)
  // {
  //   AddDummyPathToAllLastTask(paths);
  // }

  // debug TODO del debug
  // if (ddmapd_instance)
  // {
  //   cout << "----------------- preprocess : after adding paths -----------------" << endl;
  //   for (int i = 0; i < num_of_agents; i++)
  //   {
  //     for (int j = 0; j < search_engines[i]->goal_location.size(); j++)
  //     {

  //       if (debug_id.size() != 2 or (task2id({i, j}) != debug_id[0] and task2id({i, j}) != debug_id[1]))
  //       {
  //         continue;
  //       }
  //       if (paths[task2id({i, j})]->path.size() == 0 and curr_dummy_paths[task2id({i, j})]->path.size() == 0)
  //       {
  //         continue;
  //       }
  //       cout << "agent " << i << " task " << j << " task global ID : " << task2id({i, j}) << " path size : " << paths[task2id({i, j})]->path.size() << " dummy path size : " << curr_dummy_paths[task2id({i, j})]->path.size() << endl;
  //       cout << "path : " << endl;
  //       for (auto p : paths[task2id({i, j})]->path)
  //       {
  //         cout << p.location << " --> ";
  //       }
  //       cout << endl;
  //       cout << "dummy path : " << endl;
  //       for (auto p : curr_dummy_paths[task2id({i, j})]->path)
  //       {
  //         cout << p.location << " --> ";
  //       }
  //       cout << endl;
  //     }
  //   }
  // }
  // debug TODO del debug

  vector<int> planning_order;
  bool has_cycle = !topological_sort(adj_list, planning_order);
  if (has_cycle)
  {
    cout << "cycle in priority graph" << endl;
    return false;
  }

  node->is_solution = true;

  // replanning
  for (auto i : planning_order)
  {

    int agent, task;
    ConstraintTable ct;
    tie(agent, task) = id2task[i];
    if (paths[i]->empty()) // only replan the affected tasks
    { 
      int start_time = 0;

      if (task != 0)
      {
        assert(!paths[task2id({agent, task - 1})]->empty());
        start_time = paths[task2id({agent, task - 1})]->end_time();
      }

      cout << "############################  plan for " << agent << "(" << task << ")" <<  " global ID : " << i ;
      if (ddmapd_instance){
        cout <<  " segment start " << search_engines[agent]->goal_location[task] << " segment end " << search_engines[agent]->agent_segments[task].trajectory.back() << " parking loc " << search_engines[agent]->instance.start_locations[agent];
      }
      cout << endl;

      // debug TODO del
      if (i == 243 or i == 244){
        search_engines[agent]->debug_agent = i;
        cout << "---- task_locVal ------" << endl;
        for (int p = 0; p < task_locVal[i].size(); p++){
          cout << "loc " << p << " val " << task_locVal[i][p] << endl;
        }
      }else{
        search_engines[agent]->debug_agent = -1;
      }

      clock_t part_e_start = clock();
      if (ddmapd_instance)
      {
        build_ct_with_dummypath(ct, i, adj_list_r, planned_tasks);
      }
      else
      {
        build_ct(ct, i, adj_list_r);
      }
      cout << " earlist start time (ct.length_min) : " << ct.length_min << endl;
      cout << "part_e build_ct time: " << ((double)(clock() - part_e_start) / CLOCKS_PER_SEC) << endl;
      part_e_time += ((double)(clock() - part_e_start) / CLOCKS_PER_SEC);

      
      bool reuse_old_path = false;
      AddDummyPathToLastTask(*copy[i], *dummy_copy[i]);
      if (copy[i]->begin_time == start_time && copy[i]->end_time() >= ct.length_min)
      {
        reuse_old_path = true;
        for (int j = 0; j + 1 < copy[i]->size(); j++)
        {
          int actual_t = j + copy[i]->begin_time;
          int loc = copy[i]->at(j).location;
          int next_loc = copy[i]->at(j + 1).location;
          if (ct.constrained(loc, actual_t))
          {
            reuse_old_path = false;
            break;
          }
          if (ct.constrained(loc, next_loc, actual_t + 1))
          {
            reuse_old_path = false;
            break;
          }
        }
        if (is_task_a_final_one(i))
        {
          if (copy[i]->end_time() < ct.getHoldingTime())
          {
            reuse_old_path = false;
          }
        }
        else
        {
          if (ct.constrained(ct.goal_location, copy[i]->end_time()))
          {
            reuse_old_path = false;
          }
        }
      }
      remove_dummy_path(dummy_copy[i]->path.size(), *copy[i]);
      


      if (reuse_old_path)
      {
        cout << "reuse old path" << endl;
        *paths[i] = *copy[i]; 
        *curr_dummy_paths[i] = *dummy_copy[i];
      }
      else
      {
        clock_t part_f_start = clock();
        CalculateTaskStartTime(node->constraints);
        cout << "part_f CalculateTaskStartTime time: " << ((double)(clock() - part_f_start) / CLOCKS_PER_SEC) << endl;
        part_f_time += ((double)(clock() - part_f_start) / CLOCKS_PER_SEC);
        if (ddmapd_instance)
        {
          if (dummy_avoid){
            *paths[i] = search_engines[agent]->findPathSegmentToParkWithTrajAvoid(ct, start_time, task, 0, task_locVal[i]);
            if (search_engines[agent]->timeout){
               *paths[i] = search_engines[agent]->findPathSegmentToPark(ct, start_time, task, 0); 
            }
          }else{
            *paths[i] = search_engines[agent]->findPathSegmentToPark(ct, start_time, task, 0);
          }

          num_LL_expanded += search_engines[agent]->num_expanded;
          a_star_runtime += search_engines[agent]->findPathSegmentToPark_time;
          branch_a_star_times += 1;
          cout << "one a_star runtime " << search_engines[agent]->findPathSegmentToPark_time << endl;
          if (!paths[i]->empty()){
            updateDummyPath(*paths[i], *curr_dummy_paths[i], search_engines[agent]->dummy_path_len);
            remove_dummy_path(curr_dummy_paths[i]->path.size(), *paths[i]);
          }
          
        }
        else
        {
          *paths[i] = search_engines[agent]->findPathSegment(ct, start_time, task, 0);
        }
      }
      // debug TODO del debug
      if (ddmapd_instance)
      {
        cout << "generated child node path: " << endl;
        cout << "task global ID : " << i << " previous task end : " << search_engines[agent]->goal_location[task] << " segment_end : " << search_engines[agent]->agent_segments[task].trajectory.back() << " parking loc : " << search_engines[agent]->instance.start_locations[agent] << endl;
        cout << "segment traj: ";
        for (auto loc : search_engines[agent]->agent_segments[task].trajectory)
        {
          cout << loc << " ";
        }
        cout << endl;
        cout << "path: ";
        for (auto p : paths[i]->path)
        {
          cout << p.location << " --> ";
        }
        cout << endl;
        cout << "dummy path: ";
        for (auto p : curr_dummy_paths[i]->path)
        {
          cout << p.location << " --> ";
        }
        cout << endl;
      }
      // debug TODO del debug

      if (paths[i]->empty())
      {
        cout << "No path exists for agent " << agent << "(" << task << ")" << endl;
        return false;
      }
      planned_tasks.push_back(i);
    }

    clock_t part_g_start = clock();
    auto high_prio_agents = reachable_set(i, adj_list_r);
    high_prio_agents.erase(i);
    // for (auto j : high_prio_agents) // should not have conflict with higher-priority tasks
    // {
    //   bool is_conf = (ddmapd_instance) ? findOneConflictWithDummyPath(i, j) : findOneConflict(i, j);
    //   if (is_conf)
    //   {
    //     cout << "between " << i << " and " << j << endl;
    //     // debug TODO del debug
    //     // if (ddmapd_instance){
    //     //   cout << "path for task globalID : " << i << " path size : " << paths[i]->path.size() << " dummy path size : " << curr_dummy_paths[i]->path.size() << endl;
    //     //   cout << "path : " << endl;
    //     //   for (auto p : paths[i]->path)
    //     //   {
    //     //     cout << p.location << " --> ";
    //     //   }
    //     //   cout << endl;
    //     //   cout << "dummy path : " << endl;
    //     //   for (auto p : curr_dummy_paths[i]->path)
    //     //   {
    //     //     cout << p.location << " --> ";
    //     //   }
    //     //   cout << endl;

          
    //     //   cout << "path for task globalID : " << j << " path size : " << paths[j]->path.size() << " dummy path size : " << curr_dummy_paths[j]->path.size() << endl;
    //     //   cout << "path : " << endl;
    //     //   for (auto p : paths[j]->path)
    //     //   {
    //     //     cout << p.location << " --> ";
    //     //   }
    //     //   cout << endl;
    //     //   cout << "dummy path : " << endl;
    //     //   for (auto p : curr_dummy_paths[j]->path)
    //     //   {
    //     //     cout << p.location << " --> ";
    //     //   }
    //     //   cout << endl;
    //     // }
    //     // debug TODO del debug


    //   }
    //   assert(!is_conf);
    // }
    
    
    
    // cout << "part_g check higher-priority conflict time: " << ((double)(clock() - part_g_start) / CLOCKS_PER_SEC) << endl;
    part_g_time += ((double)(clock() - part_g_start) / CLOCKS_PER_SEC);

    bool conflict_found = false;

    clock_t part_h_start = clock();
    for (auto j : planning_order) // check if there is conflict with same-priority tasks
    {
      if (i == j)
      {
        break;
      }
      if (high_prio_agents.find(j) == high_prio_agents.end())
      {
        bool is_conf = (ddmapd_instance) ? findOneConflictWithDummyPath(i, j) : findOneConflict(i, j);
        if (is_conf)
        {
          cout << "Conflict between " << i << " and " << j << endl;
          shared_ptr<Conflict> conflict(new Conflict());
          conflict->priorityConflict(i, j);
          node->conflict = conflict;
          node->is_solution = false;
          conflict_found = true;
          break;
        }
      }
    }
    // cout << "part_h check same-priority conflict time: " << ((double)(clock() - part_h_start) / CLOCKS_PER_SEC) << endl;
    part_h_time += ((double)(clock() - part_h_start) / CLOCKS_PER_SEC);

    
    num_LL_generated += search_engines[agent]->num_generated;

    if (conflict_found)
    {
      break;
    }
  }

  // // remove the dummy path from paths
  // for (int i = 0; i < num_of_tasks; i++)
  // {
  //   int next_taskID = get_next_taskID(i);
  //   // only the last task will have a dummy path
  //   if (paths[i]->path.size() > 0 && (next_taskID == -1 or paths[next_taskID]->path.size() == 0))
  //   {
  //     // debug TODO del debug
  //     cout << "--- generate child POST PROCESS : remove dummy path len : " << curr_dummy_paths[i]->path.size() << " before remove path size : " << paths[i]->path.size() << endl;
  //     if (curr_dummy_paths[i]->path.size() >= paths[i]->path.size())
  //     {
  //       int debug = 1;
  //     }

  //     // debug TODO del debug
  //     remove_dummy_path(curr_dummy_paths[i]->path.size(), *paths[i]);
  //   }
  // }

  // // debug TODO del debug
  // if (ddmapd_instance)
  // {
  //   cout << "----------------- updated paths -----------------" << endl;
  //   for (int i = 0; i < num_of_agents; i++)
  //   {
  //     for (int j = 0; j < search_engines[i]->goal_location.size(); j++)
  //     {
  //       if (debug_id.size() != 2 or (task2id({i, j}) != debug_id[0] and task2id({i, j}) != debug_id[1]))
  //       {
  //         continue;
  //       }
  //       if (paths[task2id({i, j})]->path.empty() and curr_dummy_paths[task2id({i, j})]->path.empty())
  //       {
  //         continue;
  //       }
  //       cout << "agent " << i << " task " << j << " task global ID : " << task2id({i, j}) << " path size : " << paths[task2id({i, j})]->path.size() << " dummy path size : " << curr_dummy_paths[task2id({i, j})]->path.size() << endl;
  //       cout << "path : " << endl;
  //       for (auto p : paths[task2id({i, j})]->path)
  //       {
  //         cout << p.location << " --> ";
  //       }
  //       cout << endl;
  //       cout << "dummy path : " << endl;
  //       for (auto p : curr_dummy_paths[task2id({i, j})]->path)
  //       {
  //         cout << p.location << " --> ";
  //       }
  //       cout << endl;
  //     }
  //   }
  // }
  // // debug TODO del debug

  clock_t part_i_start = clock();
  if (pbs_heuristic == 1)
  { 
    node->g_val = 0;
    for (int i = 0; i < num_of_agents; i++)
    {
      int g_i = search_engines[i]->heuristic_landmark[0];
      for (int j = 0; j < search_engines[i]->goal_location.size(); j++)
      {
        // here will only use the last task's planned path + heuristic
        int task_id = task2id({i, j});
        if (paths[task_id]->empty())
        {
          break;
        }
        g_i = paths[task_id]->end_time();
        if (j + 1 < search_engines[i]->goal_location.size())
        {
          // g_i += search_engines[i]->heuristic_landmark[j + 1];
          if (ddmapd_instance)
          {
            g_i = g_i + search_engines[i]->heuristic_landmark[j + 1] + search_engines[i]->agent_segments[j+1].trajectory.size() - 1;
          }
          else
          {
            g_i += search_engines[i]->heuristic_landmark[j + 1];
          }
        }
      }
      node->g_val += g_i;
    }
  }
  else if (pbs_heuristic == 2)
  {
    int num_tasks = id2task.size();

    // initialize priorities
    vector<pair<int, int>> priority; // {a, x} a should be done before x
    for (auto con : node->constraints)
    {
      int a, x, y, t;
      constraint_type type;
      tie(a, x, y, t, type) = con;
      if (type == constraint_type::GPRIORITY)
      {
        priority.push_back({a, x}); // <id_from, id_to, -1, -1, GPRIORITY>: used for PBS
      }
    }

    // initialize earliest_start_time
    vector<int> earliest_start_time;
    for (int task_id = 0; task_id < num_tasks; task_id++)
    {
      if (paths[task_id]->empty())
      {
        earliest_start_time.push_back(0);
      }
      else
      {
        earliest_start_time.push_back(paths[task_id]->end_time());
      }
    }

    node->g_val = earlistCompletionTime(num_tasks, priority, earliest_start_time);
  }
  // compute g
  cout << "part_i compute child node g time: " << ((double)(clock() - part_i_start) / CLOCKS_PER_SEC) << endl;
  part_i_time += ((double)(clock() - part_i_start) / CLOCKS_PER_SEC);
  cout << "part_b generate child time: " << ((double)(clock() - part_b_start_time) / CLOCKS_PER_SEC) << endl;
  part_b_time += ((double)(clock() - part_b_start_time) / CLOCKS_PER_SEC);

  cout << "new child g_val: " << node->g_val << endl;
  runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
  return true;
}

bool PBS::generateRoot()
{
  
  branch_a_star_times = 0;
  vector<int>planned_tasks;
  dummy_start = new CBSNode();
  dummy_start->g_val = 0;
  paths.resize(num_of_tasks, nullptr);
  curr_dummy_paths.resize(num_of_tasks, nullptr);
  for (auto con : initial_priorities)
  {
    dummy_start->constraints.push_back(con);
  }
  CalculateTaskStartTime(dummy_start->constraints);
  vector<vector<int>> adj_list, adj_list_r;
  get_adj_list(dummy_start, adj_list, adj_list_r);
  get_adj_list(dummy_start, temporal_adj_list, temporal_adj_list_r);

  vector<int> planning_order;

  topological_sort(adj_list, planning_order);

  // cout << "after sorting" << endl;
  // for (auto task_id: planning_order){
  //   auto task = id2task[task_id];
  //   cout << "(" << task.first << ", " << task.second << ") ";
  // }
  // cout << endl;

  // initialize paths_found_initially
  paths_found_initially.resize(num_of_tasks, Path());
  dummy_paths_found_initially.resize(num_of_tasks, Path());

  dummy_start->is_solution = true;
  for (auto i : planning_order)
  {
    // CAT cat(dummy_start->makespan + 1);  // initialized to false
    // updateReservationTable(cat, i, *dummy_start);
    int agent, task;
    tie(agent, task) = id2task[i];
    int start_time = 0;
    if (task != 0)
    {
      assert(!paths_found_initially[task2id({agent, task - 1})].empty());
      start_time = paths_found_initially[task2id({agent, task - 1})].end_time();
    }

    ConstraintTable ct;


    if (ddmapd_instance)
    { 
    build_ct_with_dummypath(ct, i, adj_list_r, planned_tasks); 
    }
    else
    {
     build_ct(ct, i, adj_list_r);
    }

    if (ddmapd_instance)
    {
      if (dummy_avoid){
        paths_found_initially[i] = search_engines[agent]->findPathSegmentToParkWithTrajAvoid(ct, start_time, task, 0, task_locVal[i]);
        if (search_engines[agent]->timeout){
          paths_found_initially[i] = search_engines[agent]->findPathSegmentToPark(ct, start_time, task, 0);
        }
      }else{
        paths_found_initially[i] = search_engines[agent]->findPathSegmentToPark(ct, start_time, task, 0);
      }
      num_LL_expanded += search_engines[agent]->num_expanded;
      branch_a_star_times += 1;
      a_star_runtime += search_engines[agent]->findPathSegmentToPark_time;
      if (!paths_found_initially[i].empty()){
        updateDummyPath(paths_found_initially[i], dummy_paths_found_initially[i], search_engines[agent]->dummy_path_len);
        remove_dummy_path(dummy_paths_found_initially[i].path.size(), paths_found_initially[i]);
      }else{
        int debug = 1; // TODO del
      }
      // debug TODO del debug
      if (i==743){
          int debug = 1;

      }

      if (ddmapd_instance)
      {
        cout << "###### generate root" << endl;
        cout << "agent: " << agent << " task: " << task << " task global ID : " << i << " previous task end : " << search_engines[agent]->goal_location[task] << " segment_end : " << search_engines[agent]->agent_segments[task].trajectory.back() << " parking loc : " << search_engines[0]->instance.start_locations[agent] ;
        cout << " earlist start time : " << ct.getHoldingTime() << endl;
        cout << "segment traj: ";
        for (auto loc : search_engines[agent]->agent_segments[task].trajectory)
        {
          cout << loc << " ";
        }
        cout << endl;
        cout << "paths_found_initially: " << "length: " << paths_found_initially[i].size() << endl;
        for (auto p : paths_found_initially[i].path)
        {
          cout << p.location << " --> ";
        }
        cout << endl;

        cout << "dummy_paths_found_initially :" << "length: " << dummy_paths_found_initially[i].size() << endl;
        for (int j = 0; j < dummy_paths_found_initially[i].path.size(); j++)
        {
          cout << dummy_paths_found_initially[i].path[j].location << " --> ";
        }
        cout << endl;
      }

      // debug TODO debug and print the path
    }
    else
    {
      paths_found_initially[i] = search_engines[agent]->findPathSegment(ct, start_time, task, 0);
    }
    if (paths_found_initially[i].empty())
    {
      cout << "No path exists for agent " << agent << "(" << task << ")" << endl;
      return false;
    }
    // udpate paths here for findOneConflict and build_ct
    paths[i] = &paths_found_initially[i];
    curr_dummy_paths[i] = &dummy_paths_found_initially[i]; 

    auto high_prio_agents = reachable_set(i, adj_list_r);
    high_prio_agents.erase(i);

    for (auto j : high_prio_agents){
      bool is_conf = (ddmapd_instance) ? findOneConflictWithDummyPath(i, j) : findOneConflict(i, j);
      assert(!is_conf);
    }

    bool conflict_found = false;

    // check if there's any conflict with same-priority tasks
    for (auto j : planning_order)
    {
      if (i == j)
      {
        break;
      }
      if (high_prio_agents.find(j) == high_prio_agents.end())
      {
        bool is_conf = (ddmapd_instance) ? findOneConflictWithDummyPath(i, j) : findOneConflict(i, j);
        if (is_conf)
        {
          cout << "Conflict between " << i << " and " << j << endl;
          shared_ptr<Conflict> conflict(new Conflict());
          conflict->priorityConflict(i, j);
          dummy_start->conflict = conflict;
          dummy_start->is_solution = false;
          conflict_found = true;
        }
      }
    }

    // dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);
    // dummy_start->g_val += (int) paths_found_initially[i].size() - 1;
    
    num_LL_generated += search_engines[agent]->num_generated;

    if (conflict_found)
    {
      break;
    }
    planned_tasks.push_back(i);
  }

  for (int i = 0; i < num_of_tasks; i++)
  {
    paths[i] = &paths_found_initially[i];
    curr_dummy_paths[i] = &dummy_paths_found_initially[i];
  }

  // // debug TODO del debug
  // cout << "------------------------------- Generate Root Result Paths : -------------------------------" << endl;
  // if (ddmapd_instance)
  // {
  //   for (int i = 0; i < num_of_tasks; i++)
  //   {
  //     if (paths[i]->empty())
  //     {
  //       if (!curr_dummy_paths[i]->empty())
  //       {
  //         cout << "ATTENTION paths[i]->empty() but curr_dummy_paths[i] is not empty" << endl;
  //         cout << "task " << i << " path size: " << curr_dummy_paths[i]->size() << " path end time: " << curr_dummy_paths[i]->end_time() << endl;
  //         for (auto p : curr_dummy_paths[i]->path)
  //         {
  //           cout << p.location << " --> ";
  //         }
  //         cout << endl;
  //       }
  //     }
  //     else
  //     {
  //       if (paths[i]->path.size() > curr_dummy_paths[i]->path.size())
  //       {
  //         cout << "ATTENTION paths[i]->path.size() > curr_dummy_paths[i]->path.size()" << " path size: " << paths[i]->size() << " dummy path size: " << curr_dummy_paths[i]->size() << endl;
  //       }
  //       cout << "ROOT paths" << endl;
  //       for (int j = 0; j < paths[i]->path.size(); j++)
  //       {
  //         cout << paths[i]->path[j].location << " --> ";
  //       }
  //       cout << endl;
  //       cout << "ROOT dummy paths" << endl;
  //       for (int j = 0; j < curr_dummy_paths[i]->path.size(); j++)
  //       {
  //         cout << curr_dummy_paths[i]->path[j].location << " --> ";
  //       }
  //       cout << endl;
  //     }
  //   }
  // }
  // // debug TODO del debug

  // generate dummy start and update data structures
  dummy_start->h_val = 0;
  dummy_start->depth = 0;
  // dummy_start->open_handle = open_list.push(dummy_start);
  // dummy_start->focal_handle = focal_list.push(dummy_start);

  num_HL_generated++;
  dummy_start->time_generated = num_HL_generated;

  // we don't need to use findConflicts in PBS?
  // findConflicts(*dummy_start);

  // We didn't compute the node-selection tie-breaking value for the root node
  // since it does not need it.
  // min_f_val = max(min_f_val, (double) dummy_start->g_val);
  // focal_list_threshold = min_f_val * focal_w;

  // if (screen >= 2) // print start and goals
  // {
  // printPaths();
  // }
  if (pbs_heuristic == 1)
  {
    for (int i = 0; i < num_of_agents; i++)
    {
      int g_i = search_engines[i]->heuristic_landmark[0];
      for (int j = 0; j < search_engines[i]->goal_location.size(); j++)
      {
        int task_id = task2id({i, j});
        if (paths[task_id]->empty())
        {
          break;
        }
        g_i = paths[task_id]->end_time();
        if (j + 1 < search_engines[i]->goal_location.size())
        {
          g_i += search_engines[i]->heuristic_landmark[j + 1];
        }
      }
      dummy_start->g_val += g_i;
    }
  }
  else if (pbs_heuristic == 2)
  {
    int num_tasks = id2task.size();

    // initialize priorities
    vector<pair<int, int>> priority; // {a, x} a should be done before x
    for (auto con : dummy_start->constraints)
    {
      int a, x, y, t;
      constraint_type type;
      tie(a, x, y, t, type) = con;
      if (type == constraint_type::GPRIORITY)
      {
        priority.push_back({a, x}); // <id_from, id_to, -1, -1, GPRIORITY>: used for PBS
      }
    }

    // initialize earliest_start_time
    vector<int> earliest_start_time;
    for (int task_id = 0; task_id < num_tasks; task_id++)
    {
      if (paths[task_id]->empty())
      {
        earliest_start_time.push_back(0);
      }
      else
      {
        earliest_start_time.push_back(paths[task_id]->end_time());
      }
    }

    dummy_start->g_val = earlistCompletionTime(num_tasks, priority, earliest_start_time);
  }
  allNodes_table.push_back(dummy_start);
  // compute g
  return true;
}

bool PBS::solve(double time_limit, int cost_lowerbound, int cost_upperbound)
{
  this->min_f_val = cost_lowerbound;
  this->cost_upperbound = cost_upperbound;
  this->time_limit = time_limit;

  if (screen > 0) // 1 or 2
  {
    string name = getSolverName();
    name.resize(35, ' ');
    cout << name << ": ";
  }
  // set timer
  start = clock();

  // print constraints;
  cout << "temporal cons:" << endl;
  for (auto cons : initial_priorities)
  {
    int a, x, y, t;
    constraint_type type;
    tie(a, x, y, t, type) = cons;
    auto task_from = id2task[a];
    auto task_to = id2task[x];
    cout << "precedence from " << task_from.first << "(" << task_from.second << ") to " << task_to.first << "(" << task_to.second << ")" << endl;
  }

  generateRoot();
  sum_a_star_times += branch_a_star_times;
  cout << "Root " << " branch_a_star_times " << branch_a_star_times << endl;
  if (dummy_start->is_solution)
  {
    solution_found = true;
    join_paths();
    solution_cost = dummy_start->g_val;
    goal_node = dummy_start;
  }

  

  // dfs_stack_t dfs_stack;
  std::stack<CBSNode *> dfs_stack;
  dfs_stack.push(dummy_start);

  while (!dfs_stack.empty() && !solution_found)
  {
    // updateFocalList();
    // if (min_f_val >= cost_upperbound)
    // {
    // 	solution_cost = (int) min_f_val;
    // 	solution_found = false;
    // 	break;
    // }
    runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
    if (runtime > time_limit || num_HL_expanded > node_limit)
    { // time/node out
      solution_cost = -1;
      solution_found = false;
      break;
    }

    // debug TODO del
    auto temp_stack = dfs_stack;
    cout << "-------------- dfs_stack ----------- " << endl;

    // Print the elements of the stack
    while (!temp_stack.empty()) {
        CBSNode* node = temp_stack.top();
        std::cout << " stack node h-val " << node->h_val << " g-val " << node->g_val << std::endl;
        temp_stack.pop();
    }
    cout << "-------------------------------------------- " << endl;
    // debug TODO del

    CBSNode *curr = dfs_stack.top();
    cout << "pop h-val " << curr->h_val << " g-val " << curr->g_val << endl;
    dfs_stack.pop();
    
    // open_list.erase(curr->open_handle);
    // takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
    if (ddmapd_instance)
    {
      updatePathsWithDummyPaths(curr); // generate paths and curr_dummy_paths for child node
    }
    else
    {
      updatePaths(curr);
    }

    if (screen > 1)
      cout << endl
           << "Pop " << *curr << endl;

    // Expand the node
    num_HL_expanded++;
    curr->time_expanded = num_HL_expanded;
    CBSNode *child[2] = {new CBSNode(), new CBSNode()};

    // curr->conflict = chooseConflict(*curr);

    child[0]->constraints = curr->conflict->constraint1;
    child[1]->constraints = curr->conflict->constraint2;

    if (screen > 1)
      cout << "	Expand " << *curr << endl
           << "	on " << *(curr->conflict) << endl;

    bool solved[2] = {false, false};
    vector<Path *> copy(paths);
    vector<Path *> dummy_copy(curr_dummy_paths);
    
    
    for (int i = 0; i < 2; i++)
    {
      if (i > 0)
      {
        paths = copy;
        curr_dummy_paths = dummy_copy;
      }
      // debug TODO delete later
      // vector<int> debug_id = {326, 229};
      // if (ddmapd_instance)
      // {
      //   cout << "generate child " << i << endl; 
      //   cout << "----------------- copy paths -----------------" << endl;
      //   for (int i = 0; i < num_of_agents; i++)
      //   {
      //     for (int j = 0; j < search_engines[i]->goal_location.size(); j++)
      //     {
      //       if (debug_id.size() != 2 or (task2id({i, j}) != debug_id[0] and task2id({i, j}) != debug_id[1]))
      //       {
      //         continue;
      //       }
      //       if (paths[task2id({i, j})]->path.size() == 0 and curr_dummy_paths[task2id({i, j})]->path.size() == 0)
      //       {
      //         continue;
      //       }
      //       cout << "agent " << i << " task " << j << " task global ID : " << task2id({i, j}) << " path size : " << paths[task2id({i, j})]->path.size() << " dummy path size : " << curr_dummy_paths[task2id({i, j})]->path.size() << endl;
      //       cout << "path : " << endl;
      //       for (auto p : paths[task2id({i, j})]->path)
      //       {
      //         cout << p.location << " --> ";
      //       }
      //       cout << endl;
      //       cout << "dummy path : " << endl;
      //       for (auto p : curr_dummy_paths[task2id({i, j})]->path)
      //       {
      //         cout << p.location << " --> ";
      //       }
      //       cout << endl;
      //     }
      //   }
      // }
      // // debug TODO del debug
      
      solved[i] = generateChild(child[i], curr);
      cout << "child "<< " g-val " << child[i]->g_val << " h-val " << child[i]->h_val << " branch_a_star_times " << branch_a_star_times << endl;
      sum_a_star_times += branch_a_star_times;
      if (!solved[i])
      {
        cout << "gen child " << i << " failed" << endl;
        delete child[i];
        continue;
      }
      clock_t part_a_start = clock();
      if (child[i]->is_solution) // no conflicts
      {                          // found a solution (and finish the while look)
        solution_found = true;
        solution_cost = child[i]->g_val;
        goal_node = child[i];
        if (ddmapd_instance)
        {
          updatePathsWithDummyPaths(child[i]); // update the current solution
          // AddDummyPathToAllLastTask(paths); // TODO add back
        }
        else
        {
          updatePaths(child[i]);
        }
        join_paths();
        break;
      }
      cout << "part_a path update " << (double)(clock() - part_a_start) / CLOCKS_PER_SEC << endl;
      part_a_time += (double)(clock() - part_a_start) / CLOCKS_PER_SEC;
    }

    for (int i = 0; i < 2; i++)
    {
      if (solved[i])
      {
        // pushNode(child[i]);
        num_HL_generated++;
        child[i]->time_generated = num_HL_generated;
        dfs_stack.push(child[i]);
        if (screen > 1)
        {
          cout << "		Generate " << *child[i] << endl;
        }
      }
    }
    curr->clear();
  } // end of while loop

  runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
  if (solution_found && !validateSolution())
  {
    cout << "Solution invalid!!!" << endl;
    printPaths();
    exit(-1);
  }

  if (screen > 0)
  { // 1 or 2
    if (solution_found)
    {
      printPaths();
    }
  }
  printResults();
  return solution_found;
}

string PBS::getSolverName() const
{
  return "PBS";
}

void PBS::join_paths()
{
  cout << "join path" << endl;
  joined_paths.resize(num_of_agents);
  for (int i = 0; i < num_of_agents; i++)
  {
    for (int j = 0; j < search_engines[i]->goal_location.size(); j++)
    {
      int task_id = task2id({i, j});
      if (j == 0)
      {
        joined_paths[i].path.push_back(paths[task_id]->front());
      }

      assert(joined_paths[i].size() - 1 == paths[task_id]->begin_time);
      for (int k = 1; k < paths[task_id]->size(); k++)
      {
        joined_paths[i].path.push_back(paths[task_id]->at(k));
      }
      joined_paths[i].timestamps.push_back(joined_paths[i].size() - 1);
    }
  }
  paths.resize(num_of_agents);
  for (int i = 0; i < num_of_agents; i++)
  {
    paths[i] = &joined_paths[i];
  }
}

PBS::~PBS()
{
  releaseNodes();
  mdd_helper.clear();
}

int PBS::get_task_distance(int start_taskID, int end_taskID) // ID is the global ID
{
  int start_agent, start_task, end_agent, end_task;
  tie(start_agent, start_task) = id2task[start_taskID]; // get the agent and task local ID
  tie(end_agent, end_task) = id2task[end_taskID];       // get the agent and task local ID
  int start_goal_loc = search_engines[start_agent]->goal_location[start_task];
  int end_goal_loc = search_engines[end_agent]->goal_location[end_task];

  if (ddmapd_instance)
  {
    int traj_end = search_engines[start_agent]->agent_segments[start_task].trajectory.back();
    return search_engines[start_agent]->agent_segments[start_task].trajectory.size() - 1 + search_engines[start_agent]->instance.getManhattanDistance(traj_end, end_goal_loc);
  }
  else
  {
    return search_engines[start_agent]->instance.getManhattanDistance(start_goal_loc, end_goal_loc);
  }
}

// earliest_start_time is the earliest start time baed on current planned path, if unplanned, it is 0
int PBS::earlistCompletionTime(int num_tasks, vector<pair<int, int>> &priority, vector<int> &earliest_start_time)
{
  // Step 1: Build the graph and calculate in-degrees for topological sort
  vector<vector<int>> graph(num_tasks); // Adjacency list for the graph
  vector<int> in_degree(num_tasks, 0);  // In-degree (number of incoming edges) for each task

  for (const auto &p : priority) // priority is the global ID
  {
    int task_a = p.first;
    int task_b = p.second;
    graph[task_a].push_back(task_b); // Add edge from task_a to task_b
    in_degree[task_b]++;             // Increment in-degree of task_b
  }

  // Step 2: Initialize the queue for topological sorting
  std::queue<int> q;
  vector<int> completion_time(num_tasks, 0); // Completion time for each task

  // Enqueue tasks with no dependencies (in-degree == 0)
  for (int i = 0; i < num_tasks; ++i)
  {
    if (in_degree[i] == 0)
    {
      q.push(i);
     completion_time[i] = earliest_start_time[i]; // Start at its earliest start time
    }
  }

  int visited_count = 0; // To detect cycles

  // Step 3: Process the tasks in topological order
  while (!q.empty())
  {
    int current = q.front();
    q.pop();
    visited_count++;

    // Traverse all tasks dependent on the current task
    for (int next : graph[current])
    {
      // Calculate the earliest time `next` can start based on `current`

     completion_time[next] = max({
         completion_time[next],                                       // Current completion time of `next`
         completion_time[current] + get_task_distance(current, next), // Completion time of `current` + distance
          earliest_start_time[next]                                    // Earliest start time of `next`
      });

      // Decrement the in-degree and enqueue if it becomes 0
      if (--in_degree[next] == 0)
      {
        q.push(next);
      }
    }
  }

  // Detect cycle: If not all tasks are visited
  if (visited_count != num_tasks)
  {
    cout << "Cycle detected in task dependencies!" << endl;
  }

  // Step 4: get the completion time from completion_time and trajectory length for dd-mapd instance
  if (ddmapd_instance)
  {
    for (int i = 0; i < num_tasks; ++i)
    {
      int agent, task;
      tie(agent, task) = id2task[i];
      completion_time[i] += search_engines[agent]->agent_segments[task].trajectory.size() - 1;
    }
  }

  // Step 5: Find the maximum completion time
  int total_time = 0;
  for (int i = 0; i < num_tasks; ++i)
  {
    cout << i << " completion time: " << completion_time[i] << endl;
    total_time = max(total_time, completion_time[i]);
  }


  

  return total_time;
}



void PBS::CalculateTaskStartTime(list<Constraint>  & constraints)
{
 
  
  int num_tasks = id2task.size();

  for (int i = 0; i < num_of_tasks; i++){
    task_locVal[i].resize(map_size, 0);
  }

  // initialize priorities
  vector<pair<int, int>> priority; // {a, x} a should be done before x
  for (auto con : constraints)
  {
    int a, x, y, t;
    constraint_type type;
    tie(a, x, y, t, type) = con;
    if (type == constraint_type::GPRIORITY)
    {
      priority.push_back({a, x}); // <id_from, id_to, -1, -1, GPRIORITY>: used for PBS
    }
  }

  // initialize earliest_start_time
  vector<int> earliest_start_time;
  for (int task_id = 0; task_id < num_tasks; task_id++)
  {
    if (paths[task_id]!=nullptr and !paths[task_id]->empty())
    {
      earliest_start_time.push_back(paths[task_id]->end_time());
    }
    else
    {
      earliest_start_time.push_back(0);
    }
  }

  std::unordered_map<int, std::vector<int>> estStart_task;
  // Step 1: Build the graph and calculate in-degrees for topological sort
  vector<vector<int>> graph(num_tasks); // Adjacency list for the graph
  vector<int> in_degree(num_tasks, 0);  // In-degree (number of incoming edges) for each task

  for (const auto &p : priority) // priority is the global ID
  {
    int task_a = p.first;
    int task_b = p.second;
    graph[task_a].push_back(task_b); // Add edge from task_a to task_b
    in_degree[task_b]++;             // Increment in-degree of task_b
  }

  // Step 2: Initialize the queue for topological sorting
  std::queue<int> q;
  vector<int> start_time(num_tasks, 0); // Completion time for each task

  // Enqueue tasks with no dependencies (in-degree == 0)
  for (int i = 0; i < num_tasks; ++i)
  {
    if (in_degree[i] == 0)
    {
      q.push(i);
     start_time[i] = earliest_start_time[i]; // Start at its earliest start time
    }
  }

  int visited_count = 0; // To detect cycles

  // Step 3: Process the tasks in topological order
  while (!q.empty())
  {
    int current = q.front();
    q.pop();
    visited_count++;

    // Traverse all tasks dependent on the current task
    for (int next : graph[current])
    {
      // Calculate the earliest time `next` can start based on `current`

     start_time[next] = max({
         start_time[next],                                       // Current completion time of `next`
         start_time[current] + get_task_distance(current, next), // Completion time of `current` + distance
          earliest_start_time[next]                                    // Earliest start time of `next`
      });

      // Decrement the in-degree and enqueue if it becomes 0
      if (--in_degree[next] == 0)
      {
        q.push(next);
      }
    }
  }

  // Detect cycle: If not all tasks are visited
  if (visited_count != num_tasks)
  {
    cout << "Cycle detected in task dependencies!" << endl;
  }

  for (int i = 0; i < num_tasks; ++i)
  {
    estStart_task[start_time[i]].push_back(i);
  }

  // Step 4: get the completion time from completion_time and trajectory length for dd-mapd instance
  std::unordered_map<int, std::vector<int>> estStart_locs;
  for (auto one_task_info : estStart_task)
  {
    int est_start_time = one_task_info.first;
    for (auto task_id : one_task_info.second)
    {
      int agent, task;
      tie(agent, task) = id2task[task_id];
      int offset_t = 0;
      for (int loc : search_engines[agent]->agent_segments[task].trajectory)
      {
        estStart_locs[est_start_time+offset_t].push_back(loc);
        offset_t++;
      }
    }
  }
  // TODO can be improved to excude the segment of itself 
  for (int i = 0; i < num_tasks; i++)
  {
    int task_start = start_time[i];
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
        // task_locVal[i][loc] = max(task_locVal[i][loc], (int)localVal);
      }
    }
    
  }

}


void PBS::updateDummyPath(Path &curr_path, Path &dummy_path, int &dummy_path_length)
{
  
  int start_idx = curr_path.size() - dummy_path_length; 
  dummy_path.path.clear();
  dummy_path.path.resize(dummy_path_length);

  for (int i = start_idx; i < curr_path.path.size(); i++)
  {
    dummy_path.path[i - start_idx] = curr_path.path[i];
  }
  dummy_path.begin_time = curr_path.begin_time + start_idx;
  
}

int PBS::get_previous_taskID(int current_global_ID)
{
  int agent, task;
  tie(agent, task) = id2task[current_global_ID];
  if (task == 0)
  {
    return -1;
  }
  return task2id({agent, task - 1});
}

int PBS::get_next_taskID(int current_global_ID)
{
  int agent, task;
  tie(agent, task) = id2task[current_global_ID];
  if (task == search_engines[agent]->goal_location.size() - 1)
  {
    return -1;
  }
  return task2id({agent, task + 1});
}

void PBS::remove_dummy_path(int dummy_length, Path &curr_path)
{
  for (int i = 0; i < dummy_length; ++i)
  {
    curr_path.path.pop_back();
  }

}



void PBS::AddDummyPathToLastTask(Path &path, Path &dummy_path)
{
  for (int i = 0; i < dummy_path.size(); i++)
  {
    path.path.push_back(dummy_path[i]);
  }

}