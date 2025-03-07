#pragma once
#include "graph.h"
#include "Instance.h"
#include "common.h"


class SWI
{
public:

    // graph variables
    DirectedGraph curr_graph;
    vector<int> curr_in_degree;
    vector<Vertex> vertices;

    // agent variables
    vector<int> agent_parkLoc;
    vector<int> agent_currLoc;
    vector<int> agent_currVertex;
    vector<int> agent_preVertex;
    vector<vector<std::tuple<int, int, int, int>>> agent_paths; // location, taskID/shelfID, shelf_step, vertexID
    // TODO add agent_dummy_path variable

    // system variables
    std::map<std::tuple<int, int>, int> taskStep2id;
    int screen;
    int num_of_tasks;
    int num_of_agents;
    bool ddmapd_instance;
    int num_of_vertices;
    Instance instance;
    bool Run_main();
    
    


    SWI(const Instance& instance, int screen);
    ~SWI();

private:

    int path_planner = 0; // (0) estimate without collision
    void Plan_path_to_loc(int agent, int curr_loc, int target_loc, int vertex_ID); // update agent_paths and agent_currLoc

    bool Switchable(int vertex_ID);
    vector<int> Get_independent_vertex(DirectedGraph input_graph);
    void Assign_and_pick_shelves();
    bool Any_movable();
    void Update_agent_movable();
    void Move_all_agents();
    void Print_agent_paths();

};