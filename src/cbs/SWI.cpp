#include "SWI.h"
#include <stack>
#include <algorithm>
#include <random>
#include "Hungarian.h"


SWI::~SWI(){
    vertices.clear();
    agent_parkLoc.clear();
    curr_graph.adjList.clear();
    cout << "SWI destructor" << endl;
}

void SWI::Print_agent_paths(){
  cout << "############# AGENT PATHS ###############" << endl;
  for (int i = 0; i < num_of_agents; i++){
    int timestep = 0;
    cout << "agent " << i << " " ;
    for (auto loc_info : agent_paths[i]){
      int loc = std::get<0>(loc_info);
      // location, taskID/shelfID, shelf_step, vertexID
      cout << std::get<0>(loc_info) << "(" << "[" << instance.getCoordinate(loc).first << "," << instance.getCoordinate(loc).second << "]" << "," << std::get<1>(loc_info) << "," << std::get<2>(loc_info) << "," << std::get<3>(loc_info)  << ")@" << timestep << " --> ";
      timestep ++;
    }
    cout << endl;
  }
}

bool SWI::Run_main(){
  


    Move_all_agents();
    // Print_agent_paths();  // debug TODO del


    return true;
}
  

vector<int> SWI::Get_independent_vertex(DirectedGraph input_graph){
  vector<int> independent_vertices;
  // Collect segments with in-degree 0
  for (int vertex_id = 0; vertex_id < curr_in_degree.size(); ++vertex_id)
  {
    if (curr_in_degree[vertex_id] == 0 && vertices[vertex_id].traj_step < vertices[vertex_id].traj_len - 1) // if in_degree is 0 and not the last step of the task
    {
      int taskID = vertices[vertex_id].taskID;
      int traj_step = vertices[vertex_id].traj_step;
      int next_ID = taskStep2id[std::make_tuple(taskID, traj_step + 1)];
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
      curr_in_degree[vertex.id]++;
      curr_graph.addEdge(type1_id, vertex.id);
    }
    for (int type2_id : vertex.type2_idList)
    {
      curr_in_degree[vertex.id]++;
      curr_graph.addEdge(type2_id, vertex.id);
    }
  }

  if (curr_graph.hasCycle()){
    cout << "Initial graph has cycle" << endl;
    exit(1);
  }
  else{
    cout << "Good, initial graph has no cycle" << endl;
  }

  // curr_graph.exportToDOT("initial_graph.dot"); // graph vis
  // init agent_currVertex
  agent_currVertex.resize(num_of_agents, -1);
  // agent_currVertex.resize(num_of_agents, 0); agent_currVertex[0] = -1; // debug TODO del
  Assign_and_pick_shelves(); 

  Print_agent_paths(); // debug TODO del


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
    int vertex_step = vertices[vertex_ID].traj_step;
    int vertex_task = vertices[vertex_ID].taskID;
    
    bool is_switchable = true;
    // last step of the task nothing to switch

    DirectedGraph copiedGraph = curr_graph;  // Uses the copy constructor
    for (int precedent_id : vertices[vertex_ID].type2_idList){
        // if any precedent_id is in agent_currVertex, then it's not is_switchable
        if (std::find(agent_currVertex.begin(), agent_currVertex.end(), precedent_id) != agent_currVertex.end()){
            is_switchable = false;
            break;
        }
        int precedent_step = vertices[precedent_id].traj_step;
        int precedent_task = vertices[precedent_id].taskID;

        if (precedent_step - 1  >= 0 && vertex_step + 1 < vertices[vertex_ID].traj_len){
            int new_fromID = taskStep2id[std::make_tuple(vertex_task, vertex_step + 1)];
            int new_toID = taskStep2id[std::make_tuple(precedent_task, precedent_step - 1)];
            copiedGraph.removeEdge(precedent_id, vertex_ID);
            copiedGraph.addEdge(new_fromID, new_toID);
        }
        else{
            is_switchable = false;
            break;
        }
    }


    if (copiedGraph.hasCycleFromNode(vertex_ID)){
        is_switchable = false;
    }
    
    if (is_switchable){
        cout << "succ switched " << vertex_ID << " (" << vertices[vertex_ID].taskID << "," << vertices[vertex_ID].traj_step << ") " << endl;
        curr_graph = copiedGraph;
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
            // cout << "agent " << i << " loc " << agent_currLoc[i] << " [" << instance.getCoordinate(agent_currLoc[i]).first << "," << instance.getCoordinate(agent_currLoc[i]).second << "] " << endl; //debug TODO del
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
      // cout << " vertex " << vertex_id << " loc " << vertices[vertex_id].loc << " [" << instance.getCoordinate(vertices[vertex_id].loc).first << "," << instance.getCoordinate(vertices[vertex_id].loc).second << "] " << endl; //debug TODO del
    }

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

	for (unsigned int x = 0; x < costMatrix.size(); x++){
    // agent x assign to independent_vertices[assignment[x]]
    Plan_path_to_loc(x, agent_currLoc[x], vertices[independent_vertices[assignment[x]]].loc, independent_vertices[assignment[x]]);
    agent_currVertex[x] = independent_vertices[assignment[x]];
	  cout << "assign " <<  x << " at " << agent_currLoc[x] << " [" << instance.getCoordinate(agent_currLoc[x]).first << "," << instance.getCoordinate(agent_currLoc[x]).second << "] " << " to " << independent_vertices[assignment[x]] << " (taskID/shelfID " << vertices[independent_vertices[assignment[x]]].taskID << " loc " << vertices[independent_vertices[assignment[x]]].loc << " [" << instance.getCoordinate(vertices[independent_vertices[assignment[x]]].loc).first << "," << instance.getCoordinate(vertices[independent_vertices[assignment[x]]].loc).second << "] )" << endl;
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
        cout << "### agent " << i << " move to " << agent_currLoc[i] << " [" << instance.getCoordinate(agent_currLoc[i]).first << "," << instance.getCoordinate(agent_currLoc[i]).second << "] " << " ( taskID/shelfID " << vertices[next_vertex_ID].taskID << ", step " << vertices[next_vertex_ID].traj_step << ") " << endl;
        // update all the indegree 
        for (int toID : curr_graph.adjList[agent_currVertex[i]]){
          curr_in_degree[toID]--;
        }
        curr_graph.removeNode(agent_currVertex[i]);
        agent_currVertex[i] = -1;
        if (curr_step + 1 == vertices[agent_currVertex[i]].traj_len - 1){
          agent_preVertex[i] = -1; // finished the task; an invalid vertex
        }
        else{
          agent_preVertex[i] = next_vertex_ID;
        }
        Update_agent_movable();
      }
    }
  }
}

void SWI::Update_agent_movable(){
  for (int i = 0; i < num_of_agents; i++){
    if (agent_preVertex[i] == -1 or agent_currVertex[i] != -1){ // start agent; not carrying anything OR just finished a task
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
        cout << "agent " << i << " continue on vertex " << agent_currVertex[i] << "[" << instance.getCoordinate(vertices[agent_currVertex[i]].loc).first << "," << instance.getCoordinate(vertices[agent_currVertex[i]].loc).second << "]" << " ( taskID/shelfID " << vertices[agent_currVertex[i]].taskID << ", step " << vertices[agent_currVertex[i]].traj_step << ") " << endl;
      }
      // else if the next location is switchable; curr_in_degree > 1 mean there are type 2 constraints
      else if (curr_in_degree[agent_preVertex[i]] == 0 && curr_in_degree[next_vertex_ID] > 1){
        if (Switchable(next_vertex_ID)){
          agent_currVertex[i] = next_vertex_ID;
          cout << "Switch succ! agent " << i << " continue on vertex " << agent_currVertex[i] <<  "[" << instance.getCoordinate(vertices[agent_currVertex[i]].loc).first << "," << instance.getCoordinate(vertices[agent_currVertex[i]].loc).second << "]" << " ( taskID/shelfID " << vertices[agent_currVertex[i]].taskID << ", step " << vertices[agent_currVertex[i]].traj_step << ") " << endl;
        }
      }
    }
  }

}