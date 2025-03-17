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
  cout << "agent " << agent_id << " " ;
  for (auto loc_info : agent_paths[agent_id]){
    int loc = std::get<0>(loc_info);
    // location, taskID/shelfID, shelf_step, vertexID
    if (std::get<1>(loc_info) == -2){
      cout << -1 << "(" << "[" << -1 << "," << -1 << "]" << "," << -1 << "," << -1 << "," << loc << ")@" << timestep << " --> ";
    }
    else{
      agent_taskID_step.insert(std::make_pair(std::get<1>(loc_info), std::get<2>(loc_info)));
      cout << std::get<0>(loc_info) << "(" << "[" << instance.getCoordinate(loc).first << "," << instance.getCoordinate(loc).second << "]" << "," << std::get<1>(loc_info) << "," << std::get<2>(loc_info) << "," << std::get<3>(loc_info)  << " init_inDegree " << init_in_degree[std::get<3>(loc_info)] << ")@" << timestep << " --> ";
    }
    timestep ++;
  }
  makespan = std::max(makespan, timestep);
  min_make_span = std::min(min_make_span, timestep);
  sum_of_cost += timestep;
  cout << endl;
  cout << "sum_of_cost " << sum_of_cost << " switch_cost " << switch_cost << " all " << sum_of_cost + switch_cost << " makespan " << makespan << " (min_make_span " << min_make_span << ")" << " traj_len "  << agent_taskID_step.size() << endl;
}

void SWI::Print_agent_paths(){
  cout << "############# AGENT PATHS ###############" << endl;
  int sum_of_cost = 0;
  int makespan = 0;
  int min_make_span = 100000;
  set<pair<int, int>> agent_taskID_step;
  for (int i = 0; i < num_of_agents; i++){
    int timestep = 0;
    cout << "agent " << i << " " ;
    for (auto loc_info : agent_paths[i]){
      int loc = std::get<0>(loc_info);
      // location, taskID/shelfID, shelf_step, vertexID
      if (std::get<1>(loc_info) == -2){
        cout << -1 << "(" << "[" << -1 << "," << -1 << "]" << "," << -1 << "," << -1 << "," << loc << ")@" << timestep << " --> ";
      }
      else{
        agent_taskID_step.insert(std::make_pair(std::get<1>(loc_info), std::get<2>(loc_info)));
        cout << std::get<0>(loc_info) << "(" << "[" << instance.getCoordinate(loc).first << "," << instance.getCoordinate(loc).second << "]" << "," << std::get<1>(loc_info) << "," << std::get<2>(loc_info) << "," << std::get<3>(loc_info)  << " init_inDegree " << init_in_degree[std::get<3>(loc_info)] << ")@" << timestep << " --> ";
      }
      timestep ++;
    }
    makespan = std::max(makespan, timestep);
    min_make_span = std::min(min_make_span, timestep);
    sum_of_cost += timestep;
    cout << endl;
  }
  cout << "sum_of_cost " << sum_of_cost << " switch_cost " << switch_cost << " all " << sum_of_cost + switch_cost << " makespan " << makespan << " (min_make_span " << min_make_span << ")" << " traj_len "  << agent_taskID_step.size() << endl;
}

bool SWI::Run_main(){
    while (not Pass_all_vertex()){
       cout << "### Start a new round ###" << endl;
       Assign_and_pick_shelves();
       Move_all_agents();
    }
    cout << "### SUCC Finished all tasks ###" << endl;
    Print_agent_paths();  
    return true;
}
  
vector<int> SWI::Get_independent_vertex(DirectedGraph input_graph){
  vector<int> independent_vertices;
  // Collect segments with in-degree 0
  for (const auto& pair : input_graph.fromTo)
  {
    int vertex_id = pair.first;
    // debug TODO del
    cout << "vertex " << vertex_id << " in_degree " << curr_in_degree[vertex_id] << " depedent vertices: ";
    for (int next_vertex : input_graph.toFrom[vertex_id]){
      cout << next_vertex << " ";
    }
    cout << endl;
    // debug TODO del
    if (curr_in_degree[vertex_id] == 0 && vertices[vertex_id].traj_step < vertices[vertex_id].traj_len - 1) // if in_degree is 0 and not the last step of the task
    {
      int taskID = vertices[vertex_id].taskID;
      int traj_step = vertices[vertex_id].traj_step;
      int next_ID = get_next_node(vertex_id);
      // debug TODO del
      cout << "vertex " << vertex_id << " in_degree " << curr_in_degree[vertex_id] << " taskID " << taskID << " step " << traj_step << " next_ID " << next_ID << " next_in_degree " << curr_in_degree[next_ID];
      cout << " depedent vertices: ";
      for (int next_vertex : input_graph.toFrom[next_ID]){
        cout << next_vertex << " ";
      }
      cout << endl;

      if (curr_in_degree[next_ID] == 1){
        independent_vertices.push_back(vertex_id);
      }
    }
  }
  return independent_vertices;

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
  agent_currLoc = agent_parkLoc;
  this->instance = instance;
  agent_preVertex.resize(num_of_agents, -1);
  agent_currVertex.resize(num_of_agents, -1);
  agent_nextVertex.resize(num_of_agents, -1);
  passed.resize(num_of_vertices, false);
  

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
      passed[vertex.id] = true;
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


        // if any precedent_id is in agent_currVertex, then it's not is_switchable
        if (std::find(agent_currVertex.begin(), agent_currVertex.end(), precedent_id) != agent_currVertex.end()){
            is_switchable = false;
            break;
        }
        // if any precedent_id is in agent_nextVertex, then it's not is_switchable
        if (std::find(agent_nextVertex.begin(), agent_nextVertex.end(), precedent_id) != agent_nextVertex.end()){
            is_switchable = false;
            break;
        }
        // if any precedent_id is in agent_preVertex, then don't switch it as this point cloud be continue to carry on next time 
        if (std::find(agent_preVertex.begin(), agent_preVertex.end(), precedent_id) != agent_preVertex.end()){
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

        if (passed[new_toID]){
          is_switchable = false;
          break;
        }

        // if any new_toID is in agent_currVertex, then it's not is_switchable
        if (std::find(agent_currVertex.begin(), agent_currVertex.end(), new_toID) != agent_currVertex.end()){
            is_switchable = false;
            break;
        }
        // if any new_toID is in agent_nextVertex, then it's not is_switchable
        if (std::find(agent_nextVertex.begin(), agent_nextVertex.end(), new_toID) != agent_nextVertex.end()){
            is_switchable = false;
            break;
        }
        // if any new_toID is in agent_preVertex, then don't switch it as this point cloud be continue to carry on next time 
        if (std::find(agent_preVertex.begin(), agent_preVertex.end(), new_toID) != agent_preVertex.end()){
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
        // copiedGraph.check_graph(copied_in_degree); // debug TODO del
        copiedGraph.removeEdge(precedent_id, vertex_ID);
        vertex_type2Time[vertex_ID].erase(precedent_id);
        copied_in_degree[vertex_ID]--; 
        // copiedGraph.check_graph(copied_in_degree); // debug TODO del
        copiedGraph.addEdge(new_fromID, new_toID);
        copied_in_degree[new_toID]++;
        vertex_type2Time[new_toID][new_fromID] = 0;
        cout << "vertex " << new_fromID << " sat_time " << vertex_type2Time[new_toID][new_fromID] << endl; // debug TODO del
        // copiedGraph.check_graph(copied_in_degree); // debug TODO del
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
    // initilze available agents
    vector<int> avail_agents;
    vector<pair<int, int>> agent_locs;
    for (int i = 0; i < num_of_agents; i++){
        
        if (agent_currVertex[i] == -1){
            avail_agents.push_back(i);
            cout << "agent " << i << " is available at " << "[" << instance.getCoordinate(agent_currLoc[i]).first << "," << instance.getCoordinate(agent_currLoc[i]).second << "]" << endl;
            agent_locs.push_back(instance.getCoordinate(agent_currLoc[i]));
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
    exit(1);
    return;
  }


  // assignment
  vector<vector<double>> costMatrix;
  for (int i = 0; i < avail_agents.size(); i++){
      vector<double> row;
      for (int j = 0; j < independent_vertices.size(); j++){
          int overhead = 0;
          if (shelf_overhead){
            if (agent_preVertex[i] != independent_vertices[j]){
              overhead = 1; // additonal timestep for pick up and release
            }
          }
          int earlist_pickup_time = get_type2_sat_time(independent_vertices[j]); 
          int earlist_goal_time = get_type2_sat_time(get_next_node(independent_vertices[j]));
          int curr_time = agent_paths[i].size() - 1;
          // agent_currLoc
          int est_earlist_arriveTime = curr_time + instance.getManhattanDistance(agent_locs[i], vertex_locs[j]) + overhead; // + 1 to the goal vertex
          // int est_earlist_arriveTime = curr_time + instance.getManhattanDistance(agent_currLoc[i], vertex_locs[j]) + overhead  + 1; // + 1 to the goal vertex
          // dis : the timestep that the shelf can arrive the goal vertex (don't need to consider the relase time)
          int dis = std::max(std::max(est_earlist_arriveTime, earlist_pickup_time + 1), earlist_goal_time) - curr_time;
          // cout << "est_earlist_arriveTime " << est_earlist_arriveTime << " earlist_pickup_time " << earlist_pickup_time << " earlist_goal_time " << earlist_goal_time << " max " << std::max(std::max(est_earlist_arriveTime, earlist_pickup_time + 1), earlist_goal_time) << " curr_time " << curr_time << " dis " << dis << " overhead " << overhead << endl;
          // int dis = instance.getManhattanDistance(agent_locs[i], vertex_locs[j]);
          row.push_back(dis);
      }
      costMatrix.push_back(row);
  }
  HungarianAlgorithm HungAlgo;
  vector<int> assignment;
	double cost = HungAlgo.Solve(costMatrix, assignment);

  // update agent_currVertex
	for (unsigned int x = 0; x < costMatrix.size(); x++){
    // agent x assign to independent_vertices[assignment[x]]
    if (assignment[x] == -1){
      continue;
    }
    // move to the assigned vertex (vetex with in_degree 0 and next in_degree 1)
    switch_cost += 2;
    agent_currVertex[x] = independent_vertices[assignment[x]];
    // update agent_nextVertex
    if (vertices[agent_currVertex[x]].traj_step + 1 < vertices[agent_currVertex[x]].traj_len){
     agent_nextVertex[x] = get_next_node(agent_currVertex[x]);
    }
    else{
      // this should not happen; i
      agent_nextVertex[x] = -1;
      cout << "Error : assigned the goal vertex " << agent_currVertex[x] << " (" << vertices[agent_currVertex[x]].taskID << "," << vertices[agent_currVertex[x]].traj_step << ") " << endl;
    }

    cout << "agent " << x << "@ [" << instance.getCoordinate(agent_currLoc[x]).first << "," << instance.getCoordinate(agent_currLoc[x]).second << "]"  << " assigned to vertex " << independent_vertices[assignment[x]] << " ( taskID  " << vertices[independent_vertices[assignment[x]]].taskID << ", traj_step " << vertices[independent_vertices[assignment[x]]].traj_step << ") [" << instance.getCoordinate(vertices[independent_vertices[assignment[x]]].loc).first << "," << instance.getCoordinate(vertices[independent_vertices[assignment[x]]].loc).second << "]" << " goal vertex_ID --> " << agent_nextVertex[x]  << "[" << instance.getCoordinate(vertices[agent_nextVertex[x]].loc).first << "," << instance.getCoordinate(vertices[agent_nextVertex[x]].loc).second << "]" <<  endl;


   }

}

// plan the path from curr_loc to target_loc; NOTE! update agent_paths WITHOUT adding curr_loc as the FIRST STEP
// trajectory should be agent from curr_loc to target_loc
void SWI::Plan_path_to_loc(int agent, int agent_curr_loc, int pickup_vertex, int goal_vertex){
  bool continue_on_pre_shelf = false;
  int curr_task = vertices[pickup_vertex].taskID;
  int curr_step = vertices[pickup_vertex].traj_step;
  int pre_node = get_pre_node(pickup_vertex);
  int pickup_earlist_time = get_type2_sat_time(pickup_vertex);
  int goal_earlist_time = get_type2_sat_time(goal_vertex);
  int curr_time = agent_paths[agent].size() - 1;

  if (pickup_vertex == agent_preVertex[agent]){
    continue_on_pre_shelf = true;
  }
  if (path_planner == 0){

    if (shelf_overhead){
      pickup_earlist_time -- ; 
    }

    // if (goal_vertex == 835){
    //   int debug = 1;
    //   Print_one_agent_path(agent);
    // }

    // the least amount of possible time to reach the target location when it's available
    int min_gap_time = std::max(pickup_earlist_time - curr_time, goal_earlist_time - 1 - curr_time);
    int shortest_dis = instance.getManhattanDistance(agent_curr_loc, vertices[pickup_vertex].loc); 

    if (min_gap_time > shortest_dis){
      for (int i = 0; i < min_gap_time - shortest_dis; i++){
        agent_paths[agent].push_back(std::make_tuple(shortest_dis, -2, -1, -1)); // shortest distance, -2 means it's moving to the traget location (using est cost)
      }
    }

    // if (goal_vertex == 835){
    //   int debug = 1;
    //   Print_one_agent_path(agent);
    // }

    // shortest_dis-1 to exclude the start location 
    for (int i = 0; i < shortest_dis-1; i++){
      agent_paths[agent].push_back(std::make_tuple(shortest_dis, -2, -1, -1)); // shortest distance, -2 means it's moving to the traget location (using est cost)
    }

    // if (goal_vertex == 835){
    //   int debug = 1;
    //   Print_one_agent_path(agent);
    // }
    if (not continue_on_pre_shelf){
      agent_paths[agent].push_back(std::make_tuple(vertices[pickup_vertex].loc, vertices[pickup_vertex].taskID, vertices[pickup_vertex].traj_step, pickup_vertex));
    }
    if (shelf_overhead && not continue_on_pre_shelf){ // add the "pick up" and "release" move for uncontinue shelf
      agent_paths[agent].push_back(std::make_tuple(vertices[pickup_vertex].loc, vertices[pickup_vertex].taskID, vertices[pickup_vertex].traj_step, pickup_vertex));
      agent_paths[agent].push_back(std::make_tuple(vertices[goal_vertex].loc, vertices[goal_vertex].taskID, vertices[goal_vertex].traj_step, goal_vertex));
    }
    // location, taskID/shelfID, shelf_step, vertexID
    agent_paths[agent].push_back(std::make_tuple(vertices[goal_vertex].loc, vertices[goal_vertex].taskID, vertices[goal_vertex].traj_step, goal_vertex));
  }
  // update type 2 edge and type 1 edge 
  int pickup_arrive_time;
  int goal_arrive_time;
  curr_time = agent_paths[agent].size() - 1;
  if (shelf_overhead){
    goal_arrive_time = curr_time - 1; // exclude the shelf release time; the shelf location is avaible once the shelf arrived, no need to be relased
  }
  else{
    goal_arrive_time = curr_time;
  }
  
  if (shelf_overhead){
    pickup_arrive_time = goal_arrive_time - 2; // minus the last move to the goal vertex and the pickup overhead
  }
  else{
    pickup_arrive_time = goal_arrive_time - 1; // minus the last move to the goal vertex
  }
  // update type 2 edge for pickup node
  auto sat_type2_idList_pickup = curr_graph.fromTo[pickup_vertex];
  for (int type2_id : sat_type2_idList_pickup){
    // this only happen for the first pickup of a shelf
    if (vertices[type2_id].taskID == curr_task){ // type 1 edge
      continue;
    }
    cout <<"pickup_vertex " << pickup_vertex  << " type2_id : " << type2_id << " sat_time : " << pickup_arrive_time << endl;
    // vertex_type2Time[type2_id][pickup_vertex] = pickup_arrive_time;
    // curr_graph.check_graph(curr_in_degree); // debug TODO del
    curr_graph.removeEdge(pickup_vertex, type2_id);
    curr_in_degree[type2_id]--;
    // curr_graph.check_graph(curr_in_degree); // debug TODO del

  }
  
  // update type 2 edge for goal node
  auto sat_type2_idList_goal = curr_graph.fromTo[goal_vertex];
  for (int type2_id : sat_type2_idList_goal){
    if (vertices[type2_id].taskID == curr_task){ // type 1 edge
      continue;
    }
    cout <<"goal_vertex " << goal_vertex  << " type2_id : " << type2_id << " sat_time : " << goal_arrive_time << endl;
    vertex_type2Time[type2_id][goal_vertex] = goal_arrive_time;
    // curr_graph.check_graph(curr_in_degree); // debug TODO del
    curr_graph.removeEdge(goal_vertex, type2_id);
    curr_in_degree[type2_id]--;
    // curr_graph.check_graph(curr_in_degree); // debug TODO del


  }
  // remove type 1 edge 
  // curr_graph.check_graph(curr_in_degree); // debug TODO del
  curr_graph.removeEdge(pickup_vertex, goal_vertex);
  curr_in_degree[goal_vertex]--;
  // curr_graph.check_graph(curr_in_degree); // debug TODO del

  curr_graph.removeNode(pickup_vertex);

  if (vertices[goal_vertex].traj_step == vertices[goal_vertex].traj_len - 1){
    cout << "finished on vertex " << goal_vertex << " (" << vertices[goal_vertex].taskID << "," << vertices[goal_vertex].traj_step << ") " << ' curr out_degree ' << curr_graph.fromTo[goal_vertex].size() << endl;
    // curr_graph.removeNode(goal_vertex);
    agent_preVertex[agent] = -1;
  }

  // Update agent state information 
  agent_currLoc[agent] = vertices[goal_vertex].loc;
  agent_preVertex[agent] = goal_vertex;
  agent_currVertex[agent] = -1;
  agent_nextVertex[agent] = -1;
  passed[pickup_vertex] = true;
  passed[goal_vertex] = true;

}

bool SWI::Any_movable(){
  // if any agent_currVertex is -1 
  for (int i = 0; i < num_of_agents; i++){
    if (agent_currVertex[i] != -1){
      return true;
    }
  }
  return false;
}

void SWI::Move_all_agents(){
  while(Any_movable()){
    // sort agents by agent_paths
    std::vector<int> sorted_agent(agent_paths.size());
    for (int i = 0; i < agent_paths.size(); ++i) {
        sorted_agent[i] = i;
    }

    // Sort the agent IDs based on the size of agent_paths[id]
    std::sort(sorted_agent.begin(), sorted_agent.end(), [&](int id1, int id2) {
        return agent_paths[id1].size() < agent_paths[id2].size();
    });


    // move agent along the shelf trajectory for one timestep
    for (int main_agent = 0; main_agent < num_of_agents; main_agent++){
      cout << "### agent " << main_agent << " start moving " << endl;
      if (agent_currVertex[main_agent] != -1){
        while (agent_currVertex[main_agent] != -1){
          Plan_path_to_loc(main_agent, agent_currLoc[main_agent], agent_currVertex[main_agent], agent_nextVertex[main_agent]);
          // Print_agent_paths(); // TODO del
          Update_agent_movable(main_agent);
        }
        cout << "### update all agent states " << endl;
        for (int j = 0; j < num_of_agents; j++){
          Update_agent_movable(j);
        }
      }
      
    }
  }
}

// update agent_currVertex with next vertex if movable 
void SWI::Update_agent_movable(int agent_id){

    if (agent_preVertex[agent_id] == -1 || vertices[agent_preVertex[agent_id]].traj_step >= vertices[agent_preVertex[agent_id]].traj_len - 1){
      cout << "agent " << agent_id << " finish the task " << agent_preVertex[agent_id] << " (" << vertices[agent_preVertex[agent_id]].taskID << "," << vertices[agent_preVertex[agent_id]].traj_step << ") " << endl;
      return;
    }
    int next_vertex = get_next_node(agent_preVertex[agent_id]);

    // next_vertex not reachable by the next timestep 
    int est_earlist_arriveTime = get_type2_sat_time(next_vertex);
    if (agent_paths[agent_id].size() + 1 < est_earlist_arriveTime){
      cout << "agent " << agent_id << " can't move to next vertex (not reachable)" << next_vertex << " (" << vertices[next_vertex].taskID << "," << vertices[next_vertex].traj_step << ") " << " est_earlist_arriveTime " << est_earlist_arriveTime << " agent_paths[agent_id].size() " << agent_paths[agent_id].size() << endl;
      return;
    }

    // check if next location available
    if (curr_in_degree[next_vertex] == 1){ // only type 1 constraint 
      if (agent_currLoc[agent_id] == vertices[agent_preVertex[agent_id]].loc){
        agent_nextVertex[agent_id] = next_vertex;
        agent_currVertex[agent_id] = agent_preVertex[agent_id];
        cout << "agent " << agent_id << " continue on vertex " << agent_currVertex[agent_id] << "[" << instance.getCoordinate(vertices[agent_currVertex[agent_id]].loc).first << "," << instance.getCoordinate(vertices[agent_currVertex[agent_id]].loc).second << "]" << " ( taskID/shelfID " << vertices[agent_currVertex[agent_id]].taskID << ", step " << vertices[agent_currVertex[agent_id]].traj_step << ") " << endl;
      }
    }else if (curr_in_degree[next_vertex] > 1){
      if (Switchable(next_vertex)){
        agent_nextVertex[agent_id] = next_vertex;
        agent_currVertex[agent_id] = agent_preVertex[agent_id];
        cout << "Switch succ! agent " << agent_id << " continue on vertex " << agent_currVertex[agent_id] <<  "[" << instance.getCoordinate(vertices[agent_currVertex[agent_id]].loc).first << "," << instance.getCoordinate(vertices[agent_currVertex[agent_id]].loc).second << "]" << " ( taskID/shelfID " << vertices[agent_currVertex[agent_id]].taskID << ", step " << vertices[agent_currVertex[agent_id]].traj_step << ") " << " in_degree " << curr_in_degree[next_vertex] << endl;
      }
      else{
        cout << "Switch fail! agent " << agent_id << " break on vertex " << agent_currVertex[agent_id] <<  "[" << instance.getCoordinate(vertices[agent_currVertex[agent_id]].loc).first << "," << instance.getCoordinate(vertices[agent_currVertex[agent_id]].loc).second << "]" << " ( taskID/shelfID " << vertices[agent_currVertex[agent_id]].taskID << ", step " << vertices[agent_currVertex[agent_id]].traj_step << ") " << " in_degree " << curr_in_degree[next_vertex] << endl;
      }
    }
}


