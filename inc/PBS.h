#pragma once
#include "CBS.h"
#include "graph.h"


unordered_set<int> reachable_set(int source, vector<vector<int>> adj_list);


class PBS: public CBS
{
public:

  // Switchable dd-mapd 
  vector<Vertex> vertices;
  vector<int> agent_parkLoc;
  DirectedGraph initial_graph;

  bool dummy_avoid = true;
  vector<Path> final_paths;
  vector<Path*> curr_dummy_paths;
  vector<Path> agent_dummy_paths;
  vector<Path> agent_future_paths;
  vector<Path> dummy_paths_found_initially;  // contain initial paths found
  double topology_sort_time = 0;
  int branch_a_star_times = 0;
  int sum_a_star_times = 0;
  double a_star_runtime = 0;
  double part_a_time = 0;
  double part_b_time = 0;
  double part_c_time = 0;
  double part_d_time = 0;
  double part_e_time = 0;
  double part_f_time = 0;
  double part_g_time = 0;
  double part_h_time = 0;
  double part_i_time = 0;
  vector<bool> segment_planned;
  bool validateSolution();

  ////////////////////////////////////////////////////////////////////////////////////////////
  // Runs the algorithm until the problem is solved or time is exhausted
  bool solve(double time_limit, int cost_lowerbound = 0, int cost_upperbound = MAX_COST);

  PBS(const Instance& instance, int screen);
  PBS(const Instance& instance);
  // PBS(vector<SingleAgentSolver*>& search_engines,
  //   const vector<ConstraintTable>& constraints,
  //     vector<Path>& paths_found_initially, heuristics_type heuristic, int screen);
  ~PBS();

  // used to store initial priorities
  vector<ConstraintTable> initial_constraints;
  vector<Constraint> initial_priorities;
  void set_heuristic(int h) { heuristic = h; }
  bool run_PBS(vector<int> independent_segments);

  void build_constrain_table(ConstraintTable & curr_ct_table, int agent_id);

private:
  vector<int> agent_start_time;
  vector<ConstraintTable> agent_constraint_table;
  void Update_state(Path planned_path, int dummy_path_length, int agent_id, vector<int> future_connected_segments);
  bool valid_connected_segment(int curr_segment_ID, int next_segment_ID, int current_agent);
  vector<vector<int>> Init_connected_graph; // used for task start and end time estimation
  vector<int> init_inDegree; // used for task start and end time estimation
  vector<int> completion_time;
  vector<int> earlist_start_time;
  vector<int> agent_parkLocations;
  int est_makespan;
  vector<Segment> segments;
  vector<int> get_independent_segment();
  vector<vector<int>> curr_agent_segments;
  std::tuple<std::vector<int>, std::vector<int>> get_combined_segments(int curr_segment_id);

  vector<vector<int>> dependency_graph; // dependency_graph[i] is the list of segment IDs that segment i depends on, segment i can only start after all segments in dependency_graph[i] are finished
  vector<int> curr_inDegree;
  bool valid_to_combine(int curr_segment_id, int combined_segment_id);
  void updateDummyPath(Path & curr_path, Path & dummy_path, int & dummy_path_length);

  void printPaths() const;
  void printResults() const;
  bool ddmapd_instance;
  int get_dis_between_task(int start_taskID, int end_taskID);

  void UpdateTaskEst();

  vector<vector<int>> task_locVal; // the location heuristic for each task, used in DD-MAPD
  int task_gap_threshold = 15; // the threshold for the makespan gap between two tasks; if the gap is smaller than this threshold, the two tasks are considered to be relevant and dummy path should try to avoid them.
  double locVal_offset = 100; // the max heuristic value for the loc for dummy path to avoid; will be divided by the actual task_gap
  int map_size = 0; 
  void Update_task_locVal();

  vector<vector<int>> temporal_adj_list, temporal_adj_list_r;

  void join_paths();

  vector<Path> joined_paths;

  string getSolverName() const;

  vector<pair<int,int>> id2task;
  vector<vector<int>> goal_segmentIDs;
  vector<int> idbase;
  int task2id(pair<int, int> task) const {
    return idbase[task.first] + task.second;
  }

  void get_adj_list(CBSNode* node, vector<vector<int>>& adj_list);
  void get_adj_list(CBSNode* node, vector<vector<int>>& adj_list, vector<vector<int>>& adj_list_r);
  bool topological_sort(vector<vector<int>>& adj_list, vector<int>& planning_order);
  int get_previous_taskID(int current_global_ID);
  int get_next_taskID(int current_global_ID);

  // add paths of agents with higher priorities
  // add all other paths to cat.
  void build_ct(ConstraintTable& ct, int task_id, vector<vector<int>> adj_list_r);
  void build_ct_with_dummypath(ConstraintTable& ct, int agent_id, vector<vector<int>> adj_list_r, vector<int>planned_tasks);
  void remove_dummy_path(int dummy_length, Path & curr_path);
  // void build_ct_remove_dummy_path(vector<Path*> & curr_paths, ConstraintTable& ct, int task_id, vector<vector<int>> adj_list_r);

  int num_of_tasks;

  inline bool is_task_a_final_one(int task);
  inline void updatePaths(CBSNode* curr);
  inline void updatePathsWithDummyPaths(CBSNode* curr);
  void AddDummyPathToAllLastTask(vector<Path*> & raw_paths);
  void AddDummyPathToLastTask(Path & curr_path, Path & dummy_path);
  bool all_planned();
  // 
  // vector<Path*> paths;
  // vector<SingleAgentSolver*> search_engines;  // used to find (single) agents' paths and mdd


  // print and save
  // void printPaths() const;
  // void printResults() const;
  // void printConflicts(const CBSNode& curr) const;

  bool findOneConflict(int task1, int task2);
  bool findOneConflictWithDummyPath(int task1, int task2);
  int heuristic = 1;

  // bool validateSolution() const;
};

class PBS_naive: public CBS
{
public:

  ////////////////////////////////////////////////////////////////////////////////////////////
  // Runs the algorithm until the problem is solved or time is exhausted
  bool solve(double time_limit, int cost_lowerbound = 0, int cost_upperbound = MAX_COST);

  PBS_naive(const Instance& instance, int screen);
  // PBS(vector<SingleAgentSolver*>& search_engines,
  //   const vector<ConstraintTable>& constraints,
  //     vector<Path>& paths_found_initially, heuristics_type heuristic, int screen);
  ~PBS_naive();


private:

  bool generateChild(CBSNode* child, CBSNode* curr);
  bool generateRoot();

  string getSolverName() const;

  // add paths of agents with higher priorities
  // add all other paths to cat.
  void build_ct(ConstraintTable& ct, int agent_id, vector<vector<int>> adj_list_r);

  // inline void updatePaths(CBSNode* curr);

  void get_adj_list(CBSNode* node, vector<vector<int>>& adj_list);
  void get_adj_list(CBSNode* node, vector<vector<int>>& adj_list, vector<vector<int>>& adj_list_r);
  bool topological_sort(vector<vector<int>>& adj_list, vector<int>& planning_order);


  shared_ptr<Conflict> chooseConflict(const CBSNode& node) const;

  bool findOneConflict(int task1, int task2);
  // bool validateSolution() const;

};
