#include "SWI.h"
#include <stack>
#include <algorithm>
#include <random>
#include "Hungarian.h"


SWI::~SWI(){
    vertices.clear();
    agent_parkLoc.clear();
    cout << "SWI destructor" << endl;
}

void SWI::Print_one_agent_path(int agent_id){
  cout << "############# AGENT PATHS ###############" << endl;
  int sum_of_cost = 0;
  int makespan = 0;
  int min_make_span = 100000;
  set<pair<int, int>> agent_taskID_step;
  int timestep = 0;
  int last_pos_timestep = 0;
  int switch_cost = 0;
  cout << "agent " << agent_id << " " ;
  for (auto loc_info : agent_paths[agent_id]){
    int loc = std::get<0>(loc_info);
    int vertexID = std::get<3>(loc_info);
    int shelf_step = std::get<2>(loc_info);
    // location, taskID/shelfID, shelf_step, vertexID
    if (std::get<1>(loc_info) == -1){
      cout << -1 << "(" << "[" << -1 << "," << -1 << "]" << "," << -1 << "," << -1 << "," << loc << ")@" << timestep << " --> ";
    }
    else{
      agent_taskID_step.insert(std::make_pair(std::get<1>(loc_info), std::get<2>(loc_info)));
      cout << std::get<0>(loc_info) << "(" << "[" << instance.getCoordinate(loc).first << "," << instance.getCoordinate(loc).second << "]" << "," << std::get<1>(loc_info) << "," << std::get<2>(loc_info) << "," << std::get<3>(loc_info)  << " init_inDegree " << init_in_degree[std::get<3>(loc_info)] << ")@" << timestep << " --> ";
    }

    if (shelf_step > 0){
      int pre_shelf = std::get<1>(agent_paths[agent_id][timestep - 1]);
      if (pre_shelf != std::get<1>(loc_info)){
        switch_cost += 2;
      }
    }



    if (vertexID != -1){
      last_pos_timestep = timestep;
    }
    timestep ++;
  }
  cout << "agent " << agent_id << " last_pos_timestep " << last_pos_timestep << endl;
  makespan = std::max(makespan, last_pos_timestep);
  min_make_span = std::min(min_make_span, timestep);
  sum_of_cost += last_pos_timestep;
  cout << endl;
  cout << "sum_of_cost " << sum_of_cost << " switch_cost " << switch_cost << " all " << sum_of_cost + switch_cost << " makespan " << makespan << " (min_make_span " << min_make_span << ")" << " traj_len "  << agent_taskID_step.size() << endl;
}

void SWI::Print_agent_paths(){
  cout << "############# AGENT PATHS ###############" << endl;
  int sum_of_cost = 0;
  int makespan = 0;
  int min_make_span = 100000;
  int switch_cost = 0;
  set<pair<int, int>> agent_taskID_step;
  for (int i = 0; i < num_of_agents; i++){
    int timestep = 0;
    int last_pos_timestep = 0;
    cout << "agent " << i << " " ;
    for (auto loc_info : agent_paths[i]){
      int loc = std::get<0>(loc_info);
      int vertexID = std::get<3>(loc_info);
      // location, taskID/shelfID, shelf_step, vertexID
      if (std::get<1>(loc_info) == -2){
        cout << -1 << "(" << "[" << -1 << "," << -1 << "]" << "," << -1 << "," << -1 << "," << loc << ")@" << timestep << " --> ";
      }
      else{
        if (std::get<2>(loc_info) >= 0){
          agent_taskID_step.insert(std::make_pair(std::get<1>(loc_info), std::get<2>(loc_info)));
        }
        cout << std::get<0>(loc_info) << "(" << "[" << instance.getCoordinate(loc).first << "," << instance.getCoordinate(loc).second << "]" << "," << std::get<1>(loc_info) << "," << std::get<2>(loc_info) << "," << std::get<3>(loc_info)  << " init_inDegree " << init_in_degree[std::get<3>(loc_info)] << ")@" << timestep << " --> ";
      }
      if (vertexID != -1){
        last_pos_timestep = timestep;
      }

      if (vertexID > 0){
        int pre_shelf = std::get<1>(agent_paths[i][timestep - 1]);
        if (pre_shelf != std::get<1>(loc_info)){
          switch_cost += 2;
        }
      }

      timestep ++;

    }
    makespan = std::max(makespan, last_pos_timestep);
    min_make_span = std::min(min_make_span, last_pos_timestep);
    sum_of_cost += last_pos_timestep;
    cout << endl;
  }
  cout << "sum_of_cost " << sum_of_cost << " switch_cost " << switch_cost << " all " << sum_of_cost + switch_cost << " makespan " << makespan << " (min_make_span " << min_make_span << ")" << " traj_len "  << agent_taskID_step.size() << endl;
  if (Pass_all_vertex()){ // debug 
  // if return std::none_of(curr_in_degree.begin(), curr_in_degree.end(), [](int degree) { return degree > 0; });
   bool all_finished = std::all_of(curr_in_degree.begin(), curr_in_degree.end(), [](int x) {
        return x == 0;
  });
  if (all_finished){
    cout << "### SUCC Finished all tasks ###" << endl;
  }
  else{
    cout << "### FAILED not finished all tasks ###" << endl;
    cout << " curr_in_degree.size() " <<  curr_in_degree.size() << endl;
    for (int i = 0; i < curr_in_degree.size(); i++){
        if (curr_in_degree[i] != 0){
          cout << "vertex " << i << " in_degree " << curr_in_degree[i] << " depedent vertices: ";
          for (int next_vertex : curr_graph.toFrom[i]){
            cout << next_vertex << " ";
        }
        cout << endl;
      }
    }
  }
  //   cout << "############# check agent_taskID_step #############" << endl;
  //   for (auto one_task_step : agent_taskID_step){
  //     cout << "taskID " << one_task_step.first << " step " << one_task_step.second << " finished" << endl;
  //   }
  //   cout << " ############# check passed vertex #############" << endl;
  //   for (auto vertex : vertices){
  //     pair<int, int> taskID_step = std::make_pair(vertex.taskID, vertex.traj_step);
  //     if (agent_taskID_step.find(taskID_step) == agent_taskID_step.end()){
  //       cout << "taskID " << vertex.taskID << " step " << vertex.traj_step << " not in agent_taskID_step" << endl;
  //     }
  //   }
    
  }
}

bool SWI::Run_main(){
    while (not Pass_all_vertex()){
      sys_timestep ++;
      cout << "### Start a new round ###" << endl;
      cout << "################## sys_timestep " << sys_timestep << " ##################" << endl;
      Assign_and_pick_shelves();
      Move_all_agents_onestep();
      Print_agent_paths();  

      cout << "node finished " << std::count(passed.begin(), passed.end(), true) << " / " << curr_in_degree.size() << endl; // debug TODO del
    }
    cout << "### SUCC Finished all tasks ###" << endl;
    Print_agent_paths();  
    return true;
}
  
vector<int> SWI::Get_independent_vertex(DirectedGraph input_graph){
  vector<int> independent_vertices_simu;
  vector<int> independent_vertices_curr;
  // Collect segments with in-degree 0
  for (const auto& pair : input_graph.fromTo)
  {
    int vertex_id = pair.first;

    if (curr_in_degree[vertex_id] == 0 && vertices[vertex_id].traj_step < vertices[vertex_id].traj_len - 1) // if in_degree is 0 and not the last step of the task
    { 
      int taskID = vertices[vertex_id].taskID;
      int traj_step = vertices[vertex_id].traj_step;
      int next_ID = get_next_node(vertex_id);
      cout << "vertex " << vertex_id <<" in_degree " << curr_in_degree[vertex_id] << " next_ID " << next_ID << " next_ID in_degree " << curr_in_degree[next_ID] << " taskID " << taskID << " traj_step " << traj_step << " next_ID taskID " << vertices[next_ID].taskID << " next_ID traj_step " << vertices[next_ID].traj_step << endl; // debug TODO del

      // skip the shelf that's carrying by other agent
      if (find(agent_carryingShelf.begin(), agent_carryingShelf.end(), taskID) != agent_carryingShelf.end()){
        continue;
      }

      if (curr_in_degree[next_ID] == 1){
          int type2_sat_time = get_type2_sat_time(vertex_id);
          if (type2_sat_time <= sys_timestep){
            independent_vertices_curr.push_back(vertex_id);
          }
          else{
            independent_vertices_simu.push_back(vertex_id);
          }
        
      }
    }
  }


  if (independent_vertices_curr.size() == 0){
    return independent_vertices_simu;
  }
  else if (not SIMULATE){
    return independent_vertices_curr;
  }
  else if (SIMULATE){
    for (auto curr_vertex : independent_vertices_curr){
      independent_vertices_simu.push_back(curr_vertex);
    }
    return independent_vertices_simu;
  }


} 

SWI::SWI(const Instance &instance, int screen)
{
  clock_t t = clock();

  this->screen = screen;
  num_of_tasks = instance.segments.size();
  num_of_agents = instance.getDefaultNumberOfAgents();
  ddmapd_instance = instance.ddmapd_instance;
  vertices = instance.vertices;
  agent_parkLoc = instance.agent_parkLoc;
  taskStep2id = instance.taskStep2id;
  num_of_vertices = vertices.size();
  curr_in_degree.resize(num_of_vertices, 0);
  this->instance = instance;
  visited.resize(num_of_vertices, false);
  passed.resize(num_of_vertices, false);
  agent_free.resize(num_of_agents, true); // at first all agents are free
  agent_currPathStep.resize(num_of_agents, 0);
  agent_currPath.resize(num_of_agents);
  agent_continueVertex.resize(num_of_agents, -1);
  agent_carryingShelf.resize(num_of_agents, -1);
  
  agent_paths.resize(num_of_agents);
  for (int agent_id = 0; agent_id < num_of_agents; agent_id++)
  {
    agent_paths[agent_id].push_back(std::make_tuple(agent_parkLoc[agent_id], -1, -1, -1));
  }


  // build graph
  vertex_type2Time.resize(num_of_vertices);
  for (auto vertex : vertices)
  {
    if (vertices[vertex.id].traj_step == 0)
    {
      visited[vertex.id] = true;
    }
    for (int type1_id : vertex.type1_idList)
    {
      curr_graph.addEdge(type1_id, vertex.id);
    }
    for (int type2_id : vertex.type2_idList)
    {
      curr_graph.addEdge(type2_id, vertex.id);
      vertex_type2Time[vertex.id][type2_id] = 0;
    }
  }
  curr_graph.updateIndegree(curr_in_degree);
  init_in_degree = curr_in_degree;
  

  curr_graph.check_graph(curr_in_degree); // debug TODO del

  if (curr_graph.hasCycle()){
    cout << "Initial graph has cycle" << endl;
    exit(1);
  }
  else{
    cout << "Good, initial graph has no cycle" << endl;
  }
}

// check if the edge is switchable, and update the graph if switchable
bool SWI::Switchable(int vertex_ID){

    if (not SWITCH){
      return false;
    }

    // debug TODO del  
    cout << "Switchable() : vertex_ID " << vertex_ID << " inital in_degree " << curr_in_degree[vertex_ID] << " depedent vertices: ";
    for (int next_vertex : curr_graph.toFrom[vertex_ID]){
      cout << next_vertex << " ";
    }
    cout << endl;
    
    // return false; // debug TODO del
    int vertex_step = vertices[vertex_ID].traj_step;
    int vertex_task = vertices[vertex_ID].taskID;
    
    bool is_switchable = true;
    // last step of the task nothing to switch

    DirectedGraph copiedGraph = curr_graph;  // Uses the copy constructor
    vector<int> copied_in_degree = curr_in_degree;
    auto precedent_list = curr_graph.toFrom[vertex_ID];
    
    for (int precedent_id : precedent_list){
        // type 1 edge
        if (vertices[precedent_id].taskID == vertex_task){
            continue;
        }

        if (visited[precedent_id]){
          is_switchable = false;
          break;
        }

        // if precedent_id not in the graph, means the precedent_id is already resolve, then no need to switch
        if (copiedGraph.fromTo.find(precedent_id) == copiedGraph.fromTo.end()){
            continue;
        }
        int precedent_step = vertices[precedent_id].traj_step;
        int precedent_task = vertices[precedent_id].taskID;

        // if (precedent_step - 1  >= 0 && vertex_step + 1 < vertices[vertex_ID].traj_len){
        int new_fromID = taskStep2id[std::make_tuple(vertex_task, vertex_step + 1)];
        int new_toID = taskStep2id[std::make_tuple(precedent_task, precedent_step - 1)];

        if (visited[new_toID]){
          is_switchable = false;
          break;
        }


        if (precedent_step - 1  < 0 || vertex_step + 1 >= vertices[vertex_ID].traj_len){
          is_switchable = false;
          break;
        }

        // if new_toID not in the graph, then no need to switch
        if (not copiedGraph.NodeInGraph(new_toID) or not copiedGraph.NodeInGraph(new_fromID)){
          is_switchable = false;
          break;
        }
        copiedGraph.check_graph(copied_in_degree); // debug TODO del
        copiedGraph.removeEdge(precedent_id, vertex_ID);
        vertex_type2Time[vertex_ID].erase(precedent_id);
        copied_in_degree[vertex_ID]--; 
        copiedGraph.check_graph(copied_in_degree); // debug TODO del
        copiedGraph.addEdge(new_fromID, new_toID);
        copied_in_degree[new_toID]++;
        vertex_type2Time[new_toID][new_fromID] = 0;
        cout << "vertex " << new_fromID << " sat_time " << vertex_type2Time[new_toID][new_fromID] << endl; // debug TODO del
        copiedGraph.check_graph(copied_in_degree); // debug TODO del
        cout << "Switchable() : trying switch " << precedent_id << "-->" << vertex_ID << " to " << new_fromID << "-->" << new_toID << endl; // debug TODO del 

        if (copiedGraph.hasCycleFromNode(new_fromID)){
          is_switchable = false;
          break;
        }
        cout << "Switchable() : succ switch " << precedent_id << "-->" << vertex_ID << " to " << new_fromID << "-->" << new_toID  << " new degree " << new_fromID << ":" << copied_in_degree[new_fromID] << " " << new_toID << ":" << copied_in_degree[new_toID] << endl; // debug TODO del
        
    }

    if (is_switchable){
        cout << "succ switched " << vertex_ID << " (" << vertices[vertex_ID].taskID << "," << vertices[vertex_ID].traj_step << ") " << endl;
        curr_graph = copiedGraph;
        curr_in_degree = copied_in_degree;
        
    }
    
    return is_switchable;
    
}

void SWI::Assign_and_pick_shelves(){
  // ---- get free agent and independent vertices ----
  vector<int> avail_agents;
  vector<pair<int, int>> agent_locs;
  for (int agent_id = 0; agent_id < num_of_agents; agent_id++){
      if (agent_free[agent_id]){
          avail_agents.push_back(agent_id);
          int agent_curr_loc = std::get<0>(agent_paths[agent_id].back());
          cout << "agent " << agent_id << " is available at " << "[" << instance.getCoordinate(agent_curr_loc).first << "," << instance.getCoordinate(agent_curr_loc).second << "]" << endl;
          agent_locs.push_back(instance.getCoordinate(agent_curr_loc));
          agent_currPath[agent_id].clear();
          agent_currPathStep[agent_id] = 0;
      }
  }
  if (avail_agents.size() == 0){ // this might be able to remove 
      cout << "No available agents" << endl;
      return ;
  }
  vector<int> independent_vertices = Get_independent_vertex(curr_graph);
  vector<pair<int, int>> vertex_locs;
  for (int vertex_id : independent_vertices){
    cout << "independent vertex " << vertex_id << " (" << vertices[vertex_id].taskID << "," << vertices[vertex_id].traj_step << ") [" << instance.getCoordinate(vertices[vertex_id].loc).first << "," << instance.getCoordinate(vertices[vertex_id].loc).second << "]" << endl;
    vertex_locs.push_back(instance.getCoordinate(vertices[vertex_id].loc));
  }

  if (independent_vertices.size() == 0){
    cout << "No independent vertices" << endl;
    // at the very end, the non-carrying agent just repeat the last step 
    for (int agent_id : avail_agents){
        agent_currPath[agent_id].push_back(std::make_tuple(std::get<0>(agent_paths[agent_id].back()), -1, -1, -1));
        cout << "agent " << agent_id << " wait at " << "[" << instance.getCoordinate(std::get<0>(agent_paths[agent_id].back())).first << "," << instance.getCoordinate(std::get<0>(agent_paths[agent_id].back())).second << "]" << endl;
    }

    return;
  }


  // ---- HungarianAlgorithm assignment ----
  vector<vector<double>> costMatrix;
  for (int i = 0; i < avail_agents.size(); i++){
      vector<double> row;
      int agent_id = avail_agents[i];
      for (int j = 0; j < independent_vertices.size(); j++){
          int overhead = 0;
          if (shelf_overhead){
            auto pre_info = agent_paths[agent_id].back();
            if (std::get<3>(pre_info) != independent_vertices[j]){
              overhead = 1; // additonal timestep for pick up and release
            }
          }
          int earlist_pickup_time = get_type2_sat_time(independent_vertices[j]); 
          int agent_cur_loc = std::get<0>(agent_paths[agent_id].back());
          int shortest_dis = instance.getManhattanDistance(agent_locs[i], vertex_locs[j]);
          int pre_shelf = std::get<1>(agent_paths[agent_id].back());
          int vertex_id = independent_vertices[j];
          if (shelf_overhead && pre_shelf != vertices[vertex_id].taskID){ // "pickup" overhead 
            shortest_dis ++; 
          }
          // dis : the timestep that the shelf can arrive the goal vertex (don't need to consider the relase time)
          int dis = std::max(earlist_pickup_time - sys_timestep, shortest_dis);
          cout << "agent " << agent_id << " vertex " << vertex_id  << " @[" << vertex_locs[j].first << "," << vertex_locs[j].second << "] agent_loc " << agent_locs[i].first << "," << agent_locs[i].second << " dis " << dis << " overhead " << overhead << " earlist_pickup_time " << earlist_pickup_time << " shortest_dis " << shortest_dis << " pre_shelf " << pre_shelf << " vertex_shelf " << vertices[vertex_id].taskID << endl; //debug TODO del
          row.push_back(dis);
      }
      costMatrix.push_back(row);
  }
  HungarianAlgorithm HungAlgo;
  vector<int> assignment;
	double cost = HungAlgo.Solve(costMatrix, assignment);

  // --- plan path for agents based on current assignment ---
  int curr_vertex = -1;
  int next_vertex = -1 ;
  map<int, int> agent_assignVertex;
  for (int idx = 0; idx < avail_agents.size(); idx++){
    // agent idx assign to independent_vertices[assignment[idx]]
    if (assignment[idx] == -1){
      // number of agent more than shelves; assign to the last vertex
      int agent_id = avail_agents[idx];
      int last_loc = std::get<0>(agent_paths[agent_id].back());
      agent_currPath[agent_id].push_back(std::make_tuple(last_loc, -1, -1, -1));
      cout << "agent " << agent_id << " stop at location " << last_loc << endl;
      continue;
    }
    agent_assignVertex[avail_agents[idx]] = independent_vertices[assignment[idx]];
    cout << "agnet " << avail_agents[idx] << " assign to vertex " << independent_vertices[assignment[idx]] <<  " taskID " << vertices[independent_vertices[assignment[idx]]].taskID << " at [" << instance.getCoordinate(vertices[independent_vertices[assignment[idx]]].loc).first << "," << instance.getCoordinate(vertices[independent_vertices[assignment[idx]]].loc).second << "]" << endl;
  }

  for (int agent_id = 0; agent_id < num_of_agents; agent_id++){

    if (agent_continueVertex[agent_id] != -1){
      agent_assignVertex[agent_id] = agent_continueVertex[agent_id];
      cout << "agent " << agent_id << " assign continue on vertex " << agent_assignVertex[agent_id] <<  " taskID " << vertices[agent_assignVertex[agent_id]].taskID << " at [" << instance.getCoordinate(vertices[agent_assignVertex[agent_id]].loc).first << "," << instance.getCoordinate(vertices[agent_assignVertex[agent_id]].loc).second << "]" << endl;
    }
  }

	for (auto one_assign : agent_assignVertex){
    int agent_id = one_assign.first;
    
    vector<int> longest_traj;
    curr_vertex = one_assign.second;
    longest_traj.push_back(curr_vertex);
    curr_vertex = get_next_node(curr_vertex);
    longest_traj.push_back(curr_vertex); 
    // keep exploring the follwing vertex until the next vertex is hard-constratined 
    next_vertex = get_next_node(curr_vertex);
    while (vertices[next_vertex].traj_step <= vertices[next_vertex].traj_len - 1 && next_vertex != -1 && curr_in_degree[next_vertex] == 1 && curr_in_degree[curr_vertex] == 1){
      
      longest_traj.push_back(next_vertex);
      curr_vertex = next_vertex;
      next_vertex = get_next_node(curr_vertex);
    }
    for (int one_vertex : longest_traj){
      visited[one_vertex] = true;
    }
    Plan_path_to_loc(agent_id, longest_traj);
    // debug TODO del 
    
    cout << "agent " << agent_id << " loc " << std::get<0>(agent_paths[agent_id].back()) <<  " @ [" << instance.getCoordinate(std::get<0>(agent_paths[agent_id].back())).first << "," << instance.getCoordinate(std::get<0>(agent_paths[agent_id].back())).second << "] " << " plan path to vertex " << one_assign.second << " (" << vertices[one_assign.second].taskID << " at [" << instance.getCoordinate(vertices[one_assign.second].loc).first << "," << instance.getCoordinate(vertices[one_assign.second].loc).second << "]" << endl;
    cout << "longest_traj : ";
    for (int k = 0; k < longest_traj.size(); k++){
      cout <<  longest_traj[k] << "[" << instance.getCoordinate(vertices[longest_traj[k]].loc).first << "," << instance.getCoordinate(vertices[longest_traj[k]].loc).second << "]" << " (" << vertices[longest_traj[k]].taskID << "," << vertices[longest_traj[k]].traj_step << ") --> " ;
    }
    cout << endl;
    cout << "agent " << agent_id << " agent_currPath : ";
    for (int k = 0; k < agent_currPath[agent_id].size(); k++){
      cout  << "[" << instance.getCoordinate(std::get<0>(agent_currPath[agent_id][k])).first << "," << instance.getCoordinate(std::get<0>(agent_currPath[agent_id][k])).second << "] " << " (" << std::get<1>(agent_currPath[agent_id][k]) << "," << std::get<2>(agent_currPath[agent_id][k]) << "," << std::get<3>(agent_currPath[agent_id][k]) << ") --> ";
    }
    cout << endl;


   }

      

}

// plan the path from curr_loc to target_loc; NOTE! update agent_paths WITHOUT adding curr_loc as the FIRST STEP
// trajectory should be agent from curr_loc to target_loc
void SWI::Plan_path_to_loc(int agent, vector<int> longest_traj){
  // --- init ---

  bool continue_on_pre_shelf = false;
  auto agent_info = agent_paths[agent].back();
  int agent_curr_loc = std::get<0>(agent_info);
  int pickup_loc = vertices[longest_traj[0]].loc;
  int curr_shelf = vertices[longest_traj[0]].taskID;
  agent_currPath[agent].clear();
  agent_currPathStep[agent] = 0;


  int pre_vertex = std::get<3>(agent_info);
  if (pre_vertex == longest_traj[0]){
    continue_on_pre_shelf = true;
  }

  int earlist_arrive_time = 0; // earlist start time to reach the pickup location; assume the agent is moving along the trajectory without stop
  for (int traj_step = 0; traj_step < longest_traj.size(); traj_step++){
    int one_vertex = longest_traj[traj_step];
    int loc_start = get_type2_sat_time(one_vertex);
    if (loc_start - traj_step > earlist_arrive_time){
      earlist_arrive_time = loc_start - traj_step;
    }
  }


  // shortest path estimate 
  vector<int> est_path_to_pickup;
  if (path_planner == 0){
    // the least amount of possible time to reach the target location when it's available
    int shortest_dis = instance.getManhattanDistance(agent_curr_loc, vertices[longest_traj[0]].loc);
    //  (earlist_arrive_time - 1 - sys_timestep) is the minimal travel time to the second last vertex; (shortest_dis - 2) is the minimal estimate travel time excluding the start and goal location (agent already at the start location)
    int min_gap_time = (earlist_arrive_time - 1 - sys_timestep) - (shortest_dis - 2) ;
    if (min_gap_time > 0){
      for (int i = 0; i < min_gap_time; i++){
        agent_currPath[agent].push_back(std::make_tuple(agent_curr_loc, -1, -1, -1));
      }
    }
    est_path_to_pickup = instance.getManhattanPath(agent_curr_loc, vertices[longest_traj[0]].loc);
    // debug TODO del
    cout << "agent " << agent << " shortest_dis " << shortest_dis << " earlist_arrive_time " << earlist_arrive_time << " min_gap_time " << min_gap_time << " est_path_to_pickup : ";
    for (int i = 0; i < est_path_to_pickup.size(); i++){
      cout << "[ " << instance.getCoordinate(est_path_to_pickup[i]).first << "," << instance.getCoordinate(est_path_to_pickup[i]).second << "] --> ";
    }
    cout << endl;
    if (est_path_to_pickup.size() > 0){
      est_path_to_pickup.pop_back(); // the last location is the shelf carrying location
      for (int i = 0; i < est_path_to_pickup.size(); i++){ 
        // location, taskID/shelfID, shelf_step, vertexID
        agent_currPath[agent].push_back(std::make_tuple(est_path_to_pickup[i], -1, -1, -1));
      }
    }

    if (continue_on_pre_shelf){
      longest_traj.erase(longest_traj.begin());
    }


  }

  // add the shelf carrying trajectory
  for (int traj_step = 0; traj_step < longest_traj.size(); traj_step++){
    int one_traj_vID = longest_traj[traj_step];
    agent_currPath[agent].push_back(std::make_tuple(vertices[one_traj_vID].loc, vertices[one_traj_vID].taskID, vertices[one_traj_vID].traj_step, one_traj_vID));

  }


}


void SWI::Move_all_agents_onestep(){
  for (int agent_id = 0; agent_id < num_of_agents; agent_id++){
    agent_free[agent_id] = true;
    agent_continueVertex[agent_id] = -1;
    agent_carryingShelf[agent_id] = -1;
  }

  int release_overhead = 0;
  if (shelf_overhead){
    release_overhead = 1;
  }



  for (int agent_id = 0; agent_id < num_of_agents; agent_id++){
    cout << "moving agent " << agent_id << " at " << "[" << instance.getCoordinate(std::get<0>(agent_paths[agent_id].back())).first << "," << instance.getCoordinate(std::get<0>(agent_paths[agent_id].back())).second << "]" << "progress step " << agent_currPathStep[agent_id] << " / " << agent_currPath[agent_id].size()- 1 << endl;
    int shelf_step = std::get<2>(agent_currPath[agent_id][agent_currPathStep[agent_id]]);
    int curr_shelf = std::get<1>(agent_currPath[agent_id][agent_currPathStep[agent_id]]);
    int curr_vertex = std::get<3>(agent_currPath[agent_id][agent_currPathStep[agent_id]]);
    bool just_start_carrying = false;
    if (shelf_step != -1 && agent_currPathStep[agent_id] == 0){
      just_start_carrying = true;
    }
    else if (shelf_step != -1 && agent_currPathStep[agent_id] >= 1 ){
      if (std::get<2>(agent_currPath[agent_id][agent_currPathStep[agent_id] - 1]) == -1){
        just_start_carrying = true;
      }
    }
    if (just_start_carrying){
      cout << "agent " << agent_id << " just start carrying shelf " << std::get<1>(agent_currPath[agent_id][agent_currPathStep[agent_id]]) << " at " << "[" << instance.getCoordinate(std::get<0>(agent_currPath[agent_id][agent_currPathStep[agent_id]])).first << "," << instance.getCoordinate(std::get<0>(agent_currPath[agent_id][agent_currPathStep[agent_id]])).second << "] vertex " << std::get<3>(agent_currPath[agent_id][agent_currPathStep[agent_id]]) << " (" << vertices[std::get<3>(agent_currPath[agent_id][agent_currPathStep[agent_id]])].taskID << "," << vertices[std::get<3>(agent_currPath[agent_id][agent_currPathStep[agent_id]])].traj_step << ")" << endl;
      // update type2 edge sat_time
      for (int traj_step = agent_currPathStep[agent_id]; traj_step < agent_currPath[agent_id].size(); traj_step++){
        int one_traj_vID = std::get<3>(agent_currPath[agent_id][traj_step]);
        auto dependents = curr_graph.fromTo[one_traj_vID];
        for (auto one_dependent : dependents){
          if (vertices[one_dependent].taskID == curr_shelf){ // current shelf, type 1 edge
            continue; 
          }
          curr_graph.check_graph(curr_in_degree); // debug TODO del
          curr_graph.removeEdge(one_traj_vID, one_dependent); // remove type 2 edge
          curr_in_degree[one_dependent] --;
          curr_graph.check_graph(curr_in_degree); // debug TODO del
          vertex_type2Time[one_dependent][one_traj_vID] = sys_timestep + traj_step;
          cout << "one_dependent " << one_dependent << " type2 parent " << one_traj_vID << " sat_time " << vertex_type2Time[one_dependent][one_traj_vID] << endl; // debug TODO del
        }
        int pre_vertex = std::get<3>(agent_paths[agent_id].back());
        if (traj_step == 0){ // get the pre_vertex from previous path and check the curr_shelf
          int pre_shelf = std::get<1>(agent_paths[agent_id].back());
          if (curr_shelf != pre_shelf){
            pre_vertex = -1; 
          }
        }else{
          pre_vertex = std::get<3>(agent_currPath[agent_id][traj_step - 1]);
        }
        if (pre_vertex != -1){
          curr_graph.check_graph(curr_in_degree); // debug TODO del
          curr_graph.removeEdge(pre_vertex, one_traj_vID); // remove type 1 edge
          curr_in_degree[one_traj_vID] --;
          curr_graph.check_graph(curr_in_degree); // debug TODO del
          curr_graph.removeNode(pre_vertex); // remove the previous location
          curr_graph.check_graph(curr_in_degree); // debug TODO del
        }
      }

    }

    if (agent_currPathStep[agent_id] < agent_currPath[agent_id].size() - 1 - release_overhead){
      if (shelf_step != -1){
        agent_free[agent_id] = false;
        agent_carryingShelf[agent_id] = curr_shelf;
      }
    }
    // reset agent_currPath if reach the end
    if (agent_currPathStep[agent_id] == agent_currPath[agent_id].size() - 1 - release_overhead && curr_vertex != -1){
      int next_vertex = get_next_node(curr_vertex);
      if (next_vertex != -1 && (curr_in_degree[next_vertex] <= 1 || Switchable(next_vertex))){
        visited[curr_vertex] = true;
        visited[next_vertex] = true;
        agent_free[agent_id] = false;
        agent_carryingShelf[agent_id] = curr_shelf;
        agent_continueVertex[agent_id] = curr_vertex;
        
        cout << "agent " << agent_id << " reach the end of the trajectory, continue on the same shelf " << curr_vertex << " (" << vertices[curr_vertex].taskID << "," << vertices[curr_vertex].traj_step << ") [ " << instance.getCoordinate(vertices[curr_vertex].loc).first << "," << instance.getCoordinate(vertices[curr_vertex].loc).second << "]" << endl;
      }
      else{
        cout << "agent " << agent_id << " reach the end of the trajectory, break on " << curr_vertex << " next " << next_vertex << " next_vertex in_degree " << curr_in_degree[next_vertex] << endl; 
      }

      if (vertices[curr_vertex].traj_step == vertices[curr_vertex].traj_len - 1){
        curr_graph.removeNode(curr_vertex);
        curr_graph.check_graph(curr_in_degree); // debug TODO del
        cout << "finished vertex " << curr_vertex << " ( shelf & step " << vertices[curr_vertex].taskID << "," << vertices[curr_vertex].traj_step << ") [ " << instance.getCoordinate(vertices[curr_vertex].loc).first << "," << instance.getCoordinate(vertices[curr_vertex].loc).second << "]" << endl;
      }
    }
    if (curr_vertex != -1){
      passed[curr_vertex] = true;
    }
    agent_paths[agent_id].push_back(agent_currPath[agent_id][agent_currPathStep[agent_id]]);
    agent_currPathStep[agent_id] ++;
  }
}
