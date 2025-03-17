#pragma once
#include "graph.h"
#include "Instance.h"
#include "common.h"


class SWI
{
public:
    // system configuration
    bool shelf_overhead = false;

    // graph variables
    DirectedGraph curr_graph;
    vector<int> curr_in_degree;
    vector<int> init_in_degree;
    vector<bool> passed;
    vector<Vertex> vertices;
    vector<std::map<int, int>> vertex_type2Time; //  (type2_vertex_ID, type2_satisfied_time)

    // agent variables
    vector<int> agent_parkLoc;
    vector<int> agent_currLoc;
    vector<int> agent_nextVertex;
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
    int switch_cost = 0;
    int path_planner = 0; // (0) estimate without collision
    void Plan_path_to_loc(int agent, int agent_curr_loc, int pickup_vertex, int goal_vertex);

    bool Switchable(int vertex_ID);
    vector<int> Get_independent_vertex(DirectedGraph input_graph);
    void Assign_and_pick_shelves();
    bool Any_movable();
    void Update_agent_movable(int agent_id);
    void Move_all_agents();
    void Print_agent_paths();
    void Print_one_agent_path(int agent_id);
    bool Pass_all_vertex(){
        // debug TODO del
        for (int i = 0; i < curr_in_degree.size(); i++){
            if (curr_in_degree[i] > 0){
                cout << "vertex " << i << " in_degree " << curr_in_degree[i] << " depedent vertices: ";
                for (int next_vertex : curr_graph.toFrom[i]){
                    cout << next_vertex << " ";
                }
                cout << endl;
            }
                
        }
        return std::none_of(curr_in_degree.begin(), curr_in_degree.end(), [](int degree) { return degree > 0; });
    };

    int get_type2_sat_time(int vertex_ID){ // time that the sehlf can be move to the current location
        int type2_sat_time =  0;
        if (vertex_type2Time[vertex_ID].size() == 0){
            return 0;
        }
        for (const auto& pair : vertex_type2Time[vertex_ID]) {
            if (pair.second > type2_sat_time) {
                type2_sat_time = pair.second;
            }
        }
        return type2_sat_time;
    };

    int get_next_node(int vertex_ID){
        if (vertices[vertex_ID].traj_step + 1 < vertices[vertex_ID].traj_len){
            return taskStep2id[std::make_tuple(vertices[vertex_ID].taskID, vertices[vertex_ID].traj_step + 1)];
        }
        else{
            return -1;
        }
    };

    int get_pre_node(int vertex_ID){
        if (vertices[vertex_ID].traj_step - 1 >= 0){
            return taskStep2id[std::make_tuple(vertices[vertex_ID].taskID, vertices[vertex_ID].traj_step - 1)];
        }
        else{
            return -1;
        }
    };

};