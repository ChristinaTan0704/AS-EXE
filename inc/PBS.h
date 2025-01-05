#pragma once
#include "CBS.h"


unordered_set<int> reachable_set(int source, vector<vector<int>> adj_list);


class PBS: public CBS
{
public:
  vector<Path*> curr_dummy_paths;
  vector<Path> dummy_paths_found_initially;  // contain initial paths found

  ////////////////////////////////////////////////////////////////////////////////////////////
  // Runs the algorithm until the problem is solved or time is exhausted
  bool solve(double time_limit, int cost_lowerbound = 0, int cost_upperbound = MAX_COST);

  PBS(const Instance& instance, int screen);
  // PBS(vector<SingleAgentSolver*>& search_engines,
  //   const vector<ConstraintTable>& constraints,
  //     vector<Path>& paths_found_initially, heuristics_type heuristic, int screen);
  ~PBS();

  // used to store initial priorities
  vector<ConstraintTable> initial_constraints;
  vector<Constraint> initial_priorities;
  void set_heuristic(int h) { pbs_heuristic = h; }

private:
  void updateDummyPath(Path & curr_path, Path & dummy_path, int & dummy_path_length);

  void printPaths() const;
  void printResults() const;
  bool ddmapd_instance;
  int get_task_distance(int start_taskID, int end_taskID);

  int minimumCompletionTime(int num_tasks, vector<pair<int, int>>& priority, vector<int>& earliest_start_time);

  vector<vector<int>> temporal_adj_list, temporal_adj_list_r;

  void join_paths();

  vector<Path> joined_paths;

  bool generateChild(CBSNode* child, CBSNode* curr);
  bool generateRoot();

  string getSolverName() const;

  vector<pair<int,int>> id2task;
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
  void build_ct_with_dummypath(ConstraintTable& ct, int agent_id, vector<vector<int>> adj_list_r);
  void remove_dummy_path(int dummy_length, Path & curr_path);
  // void build_ct_remove_dummy_path(vector<Path*> & curr_paths, ConstraintTable& ct, int task_id, vector<vector<int>> adj_list_r);

  int num_of_tasks;

  inline bool is_task_a_final_one(int task);
  inline void updatePaths(CBSNode* curr);
  inline void updatePathsWithDummyPaths(CBSNode* curr);
  void AddDummyPathToAllLastTask(vector<Path*> & raw_paths);
  void AddDummyPathToLastTask(Path & curr_path, Path & dummy_path);

  // 
  // vector<Path*> paths;
  // vector<SingleAgentSolver*> search_engines;  // used to find (single) agents' paths and mdd


  // print and save
  // void printPaths() const;
  // void printResults() const;
  // void printConflicts(const CBSNode& curr) const;

  bool findOneConflict(int task1, int task2);
  bool findOneConflictWithDummyPath(int task1, int task2);
  int pbs_heuristic = 1;

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
