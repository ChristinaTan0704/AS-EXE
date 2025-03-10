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
        cout << std::get<0>(loc_info) << "(" << "[" << instance.getCoordinate(loc).first << "," << instance.getCoordinate(loc).second << "]" << "," << std::get<1>(loc_info) << "," << std::get<2>(loc_info) << "," << std::get<3>(loc_info)  << ")@" << timestep << " --> ";
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
    // any curr_in_degree > 0 
    // cout << "init graph check" << endl;
    // curr_graph.check_graph(curr_in_degree); // debug TODO del
    while (not Pass_all_vertex()){
      // debug TODO del
      // cout << "degree : ";
      // int degree_num = 0;
      // for (auto degree : curr_in_degree){
      //   cout  << degree << " ";
      //   if (degree > 0){
      //     degree_num++;
      //   }

      // }
      // cout << endl;
      // cout << "degree_num " << degree_num << endl;


       cout << "### Start a new round ###" << endl;
       Assign_and_pick_shelves();
       Move_all_agents();
       for (int i = 0; i < num_of_agents; i++){
         agent_preVertex[i] = -1; // reset preVertex
       }
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
    if (curr_in_degree[vertex_id] == 0 && vertices[vertex_id].traj_step < vertices[vertex_id].traj_len - 1) // if in_degree is 0 and not the last step of the task
    {
      int taskID = vertices[vertex_id].taskID;
      int traj_step = vertices[vertex_id].traj_step;
      int next_ID = taskStep2id[std::make_tuple(taskID, traj_step + 1)];
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

  agent_paths.resize(num_of_agents);
  for (int agent_id = 0; agent_id < num_of_agents; agent_id++)
  {
    agent_paths[agent_id].push_back(std::make_tuple(agent_parkLoc[agent_id], -1, -1, -1));
  }


  // build graph
  for (auto vertex : vertices)
  {
    for (int type1_id : vertex.type1_idList)
    {
      curr_graph.addEdge(type1_id, vertex.id);
    }
    for (int type2_id : vertex.type2_idList)
    {
      curr_graph.addEdge(type2_id, vertex.id);
    }
  }
  curr_graph.updateIndegree(curr_in_degree);

  // curr_graph.check_graph(curr_in_degree); // debug TODO del


  if (curr_graph.hasCycle()){
    cout << "Initial graph has cycle" << endl;
    exit(1);
  }
  else{
    cout << "Good, initial graph has no cycle" << endl;
  }

  // curr_graph.exportToDOT("initial_graph.dot"); // graph vis
  // init agent_currVertex

  // agent_currVertex.resize(num_of_agents, 0); agent_currVertex[0] = -1; // debug TODO del
  // Assign_and_pick_shelves(); 
  // Print_agent_paths(); // debug TODO del
  // int debug = 1;
  // // debug TODO del; check if number of Switchable make sense
  // int num_switchable = 0;
  // int num_unswitchable = 0;
  // for (int i = 0; i < vertices.size(); i++){
  //   if (vertices[i].type2_idList.size()==0){
  //       continue;
  //   }
  //   if (Switchable(i)){
  //     num_switchable++;
  //   cout << "vertex " << i << "(" << vertices[i].taskID << "," << vertices[i].traj_step << ") " << " true " << endl;
  //   }
  //   else{
  //     num_unswitchable++;
  //   cout << "vertex " << i << "(" << vertices[i].taskID << "," << vertices[i].traj_step << ") " << " false " << endl;
  //   }
  // }
  // cout << "num_switchable " << num_switchable << " num_unswitchable " << num_unswitchable << endl;


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
    for (int precedent_id : vertices[vertex_ID].type2_idList){
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
        // if any precedent_id is in agent_preVertex, then it's not is_switchable
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

        if (precedent_step - 1  < 0 || vertex_step + 1 >= vertices[vertex_ID].traj_len){
          is_switchable = false;
          break;
        }

        // if new_toID not in the graph, then no need to switch
        if (not copiedGraph.inGraph(new_toID) or not copiedGraph.inGraph(new_fromID)){
          is_switchable = false;
          break;
        }
        // cout << "check before remove edge " << endl; // debug TODO del
        // copiedGraph.check_graph(copied_in_degree); // debug TODO del
        copiedGraph.removeEdge(precedent_id, vertex_ID);
        copied_in_degree[vertex_ID]--; 
        // cout << "check 1 " << endl; // debug TODO del
        // copiedGraph.check_graph(copied_in_degree); // debug TODO del
        copiedGraph.addEdge(new_fromID, new_toID);
        copied_in_degree[new_toID]++;
        // cout << "check 2 " << endl; // debug TODO del
        // copiedGraph.check_graph(copied_in_degree); // debug TODO del
        cout << "Switchable() : trying switch " << precedent_id << "-->" << vertex_ID << " to " << new_fromID << "-->" << new_toID << endl; // debug TODO del

        if (copiedGraph.hasCycleFromNode(new_fromID)){
          is_switchable = false;
          break;
        }
        // if (copiedGraph.hasCycle()){
        //   is_switchable = false;
        //   break;
        // }
        cout << "Switchable() : succ switch " << precedent_id << "-->" << vertex_ID << " to " << new_fromID << "-->" << new_toID << endl; // debug TODO del
        // }
        // else{
        //     is_switchable = false;
        //     break;
        // }
        
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
      vertex_locs.push_back(instance.getCoordinate(vertices[vertex_id].loc));
    }

    // TODO debug del
    if (independent_vertices.size() == 0 and not Pass_all_vertex()){
        for (int i = 0; i < curr_in_degree.size(); i++){
          cout << "vertex " << i << " in_degree " << curr_in_degree[i] << " curr_dependent vertices: ";
          for (int fromID : curr_graph.toFrom[i]){
            cout << fromID << " ";
          }
          cout << endl;
        }
        // curr_graph.check_graph(curr_in_degree); // TODO del
        cout << "No independen/t vertices" << endl;

        exit(1) ;
    }
  
  // debug todo del
  //   cout << "toFrom check  " << endl; // debug TODO del
  // for (int i = 0; i < curr_in_degree.size(); i++){
  //     cout << "vertex " << i << " in_degree " << curr_in_degree[i] << " curr_dependent vertices: ";
  //     for (int fromID : curr_graph.toFrom[i]){
  //       cout << fromID << " ";
  //     }
  //     cout << endl;
  //   }
  //   cout << "fromTo check  " << endl; // debug TODO del
  //   for (int i = 0; i < curr_in_degree.size(); i++){
  //     cout << "vertex " << i  << " curr_outgoing : ";
  //     for (int fromID : curr_graph.fromTo[i]){
  //       cout << fromID << " ";
  //     }
  //     cout << endl;
  //   }


  // assignment
  vector<vector<double>> costMatrix;
  for (int i = 0; i < avail_agents.size(); i++){
      vector<double> row;
      for (int j = 0; j < independent_vertices.size(); j++){
          int dis = instance.getManhattanDistance(agent_locs[i], vertex_locs[j]);
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
    Plan_path_to_loc(x, agent_currLoc[x], vertices[agent_currVertex[x]].loc, agent_currVertex[x]);
    // update agent_nextVertex
    if (vertices[agent_currVertex[x]].traj_step + 1 < vertices[agent_currVertex[x]].traj_len){
     agent_nextVertex[x] = taskStep2id[std::make_tuple(vertices[agent_currVertex[x]].taskID, vertices[agent_currVertex[x]].traj_step + 1)];
    }
    else{
      // this should not happen; i
      agent_nextVertex[x] = -1;
      cout << "Error : assigned the goal vertex " << agent_currVertex[x] << " (" << vertices[agent_currVertex[x]].taskID << "," << vertices[agent_currVertex[x]].traj_step << ") " << endl;
    }
    // remove the type 2 (outgoing) vertex of the assigned vertex
    auto type2_idList = curr_graph.fromTo[agent_currVertex[x]];
    for (int toID : type2_idList){
      if (vertices[toID].taskID != vertices[agent_currVertex[x]].taskID){
        curr_graph.removeEdge(agent_currVertex[x], toID);
        curr_in_degree[toID]--;
      }
    }

    // curr_graph.removeNode(agent_currVertex[x]);
	  // cout << "assign " <<  x << " at " << agent_currLoc[x] << " [" << instance.getCoordinate(agent_currLoc[x]).first << "," << instance.getCoordinate(agent_currLoc[x]).second << "] " << " to " << independent_vertices[assignment[x]] << " (taskID/shelfID " << vertices[independent_vertices[assignment[x]]].taskID << " loc " << vertices[independent_vertices[assignment[x]]].loc << " [" << instance.getCoordinate(vertices[independent_vertices[assignment[x]]].loc).first << "," << instance.getCoordinate(vertices[independent_vertices[assignment[x]]].loc).second << "] )" << endl;
  }

}

// plan the path from curr_loc to target_loc; NOTE! update agent_paths WITHOUT adding curr_loc as the FIRST STEP
void SWI::Plan_path_to_loc(int agent, int curr_loc, int target_loc, int vertex_ID){
  if (path_planner == 0){
    int shortest_dis = instance.getManhattanDistance(curr_loc, target_loc);
    for (int i = 0; i < shortest_dis-1; i++){
      agent_paths[agent].push_back(std::make_tuple(shortest_dis, -2, -1, -1)); // shortest distance, -2 means it's moving to the traget location (using est cost)
    }
    agent_paths[agent].push_back(std::make_tuple(target_loc, vertices[vertex_ID].taskID, vertices[vertex_ID].traj_step, vertex_ID));
  }

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
    // move agent along the shelf trajectory for one timestep
    for (int i = 0; i < num_of_agents; i++){
      if (agent_currVertex[i] != -1){
        // update all agent and graph variables
        int curr_task = vertices[agent_currVertex[i]].taskID;
        int curr_step = vertices[agent_currVertex[i]].traj_step;
        int next_vertex_ID = taskStep2id[std::make_tuple(curr_task, curr_step + 1)];
        agent_currLoc[i] = vertices[next_vertex_ID].loc;
        agent_paths[i].push_back(std::make_tuple(agent_currLoc[i], vertices[next_vertex_ID].taskID, vertices[next_vertex_ID].traj_step, next_vertex_ID));
        cout << "### agent " << i << " move from " << agent_currVertex[i]  << " to " <<  next_vertex_ID << "@" << agent_currLoc[i] << " [" << instance.getCoordinate(agent_currLoc[i]).first << "," << instance.getCoordinate(agent_currLoc[i]).second << "] " << " ( taskID/shelfID " << vertices[next_vertex_ID].taskID << ", step " << vertices[next_vertex_ID].traj_step << ") " << endl;
        // update all the indegree 
        cout << "removeNode before check indegree " << curr_in_degree[agent_currVertex[i]] << endl;
        curr_graph.removeEdge(agent_currVertex[i], next_vertex_ID); // this includes removing the type 1 edges
        curr_in_degree[next_vertex_ID]--;
        curr_graph.removeNode(agent_currVertex[i]); // remove the vertex becuase the shelf has been moved away
        cout << "removeNode after check" << endl; // debug TODO del
        // curr_graph.check_graph(curr_in_degree); // TODO del
        agent_currVertex[i] = -1;
        agent_nextVertex[i] = -1;


        // cout << "fromTo check  " << endl; // debug TODO del
        // for (int i = 0; i < curr_in_degree.size(); i++){
        //   cout << "vertex " << i << " curr_outgoing : ";
        //   for (int fromID : curr_graph.fromTo[i]){
        //     cout << fromID << " ";
        //   }
        //   cout << endl;
        // }

        // remove the type 2 edges for next_vertex_ID
        auto type2_idList = curr_graph.fromTo[next_vertex_ID];
        for (int toID : type2_idList){
          // curr_task should be type 1 edge
          if (vertices[toID].taskID != curr_task){
            curr_graph.removeEdge(next_vertex_ID, toID);
            curr_in_degree[toID]--;
          }
        }
        // cout << "removeEdge type 2 after check" << endl; // debug TODO del
        // curr_graph.check_graph(curr_in_degree); // TODO del

        if (curr_step + 1 == vertices[agent_currVertex[i]].traj_len - 1){
          agent_preVertex[i] = -1; // finished the task; an invalid vertex
        }
        else{
          // agent is at next_vertex_ID; when doing Update_agent_movable(), it will continue to move from agent_preVertex[i]
          agent_preVertex[i] = next_vertex_ID;
        }
        Update_agent_movable();
      }
    }
  }
}

void SWI::Update_agent_movable(){
  for (int i = 0; i < num_of_agents; i++){
    if (agent_preVertex[i] == -1 or agent_currVertex[i] != -1){ // agent_preVertex[i] == -1 start agent; not carrying anything OR just finished a task
      continue;
    }
    int taskID = vertices[agent_preVertex[i]].taskID;
    int step = vertices[agent_preVertex[i]].traj_step;
    int next_vertex_ID = taskStep2id[std::make_tuple(taskID, step + 1)];
    // check if agent is at the pickup location
    if (agent_currLoc[i] == vertices[agent_preVertex[i]].loc){
      // check if task is valid for pickup (in_degree == 0 && next in_degree == 1)
      if (curr_in_degree[agent_preVertex[i]] == 0 &&  curr_in_degree[next_vertex_ID] == 1){
        // continue to pickup the shelf
        agent_currVertex[i] = agent_preVertex[i];
        if (vertices[agent_currVertex[i]].traj_step + 1 < vertices[agent_currVertex[i]].traj_len){
          agent_nextVertex[i] = taskStep2id[std::make_tuple(vertices[agent_currVertex[i]].taskID, vertices[agent_currVertex[i]].traj_step + 1)];
        }
        else{
          agent_nextVertex[i] = -1;
        }
        cout << "agent " << i << " continue on vertex " << agent_currVertex[i] << "[" << instance.getCoordinate(vertices[agent_currVertex[i]].loc).first << "," << instance.getCoordinate(vertices[agent_currVertex[i]].loc).second << "]" << " ( taskID/shelfID " << vertices[agent_currVertex[i]].taskID << ", step " << vertices[agent_currVertex[i]].traj_step << ") " << endl;
      }
      // else if the next location is switchable; curr_in_degree > 1 mean there are type 2 constraints
      else if (curr_in_degree[agent_preVertex[i]] == 0 && curr_in_degree[next_vertex_ID] > 1){
        if (Switchable(next_vertex_ID)){
          agent_currVertex[i] = agent_preVertex[i];
          agent_nextVertex[i] = next_vertex_ID;
          cout << "Switch succ! agent " << i << " continue on vertex " << agent_currVertex[i] <<  "[" << instance.getCoordinate(vertices[agent_currVertex[i]].loc).first << "," << instance.getCoordinate(vertices[agent_currVertex[i]].loc).second << "]" << " ( taskID/shelfID " << vertices[agent_currVertex[i]].taskID << ", step " << vertices[agent_currVertex[i]].traj_step << ") " << " in_degree " << curr_in_degree[next_vertex_ID] << endl;
        }
        else{
          cout << "Switch fail! agent " << i << " break on vertex " << agent_currVertex[i] <<  "[" << instance.getCoordinate(vertices[agent_currVertex[i]].loc).first << "," << instance.getCoordinate(vertices[agent_currVertex[i]].loc).second << "]" << " ( taskID/shelfID " << vertices[agent_currVertex[i]].taskID << ", step " << vertices[agent_currVertex[i]].traj_step << ") " << " in_degree " << curr_in_degree[next_vertex_ID] << endl;
        }
      }
    }
  }

}