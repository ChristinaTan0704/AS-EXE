#include "SpaceTimeAStar.h"
#include "common.h"
void MultiLabelSpaceTimeAStar::printPath(const LLNode *goal)
{
	cout << " Path: ";
	const LLNode *curr = goal;
	while (curr != nullptr)
	{
		cout << "[" << instance.getRowCoordinate(curr->location) << "," << instance.getColCoordinate(curr->location) << "]@" << curr->path_idx << " ";
		cout << curr->location << " (g " << curr->g_val << ", h " << curr->h_val << ", t " << curr->path_idx << ", s " << curr->stage << " d " << curr->is_dummy_path << " g2 " << curr->g2_val << ", h2 " << curr->h2_val << " sID " << curr->segmentID << " )  <-- "; // " idx " << idx <<
		curr = curr->parent;
	}
	cout << endl;
}

void MultiLabelSpaceTimeAStar::updatePath(const LLNode *goal, Path &path)
{

	int path_idx;
	if (goal->path_idx == -1)
	{
		path_idx = goal->g_val;
	}
	else
	{
		path_idx = goal->path_idx;
	}

	path.path.resize(path_idx + 1);
	// path.timestamps.resize(goal_location.size(), 0);
	// path.timestamps.back() = path_idx;
	dummy_path_len = -1; // exclude the start point (segment end point) of dummy path

	const LLNode *curr = goal;
	while (curr != nullptr)
	{
		path[path_idx].location = curr->location;
		path[path_idx].segmentID = curr->segmentID;
		path[path_idx].mdd_width = 0;

		if (curr->parent != nullptr && curr->segmentID != curr->parent->segmentID)
		{
			// path.timestamps[curr->parent->segmentID] = path_idx + path.begin_time;
			path[path_idx].is_goal = true;
		}
		else
		{
			path[path_idx].is_goal = false;
		}
		if (curr->is_dummy_path)
		{
			dummy_path_len++;
		}
		curr = curr->parent;
		path_idx -= 1;
	}
}

Path MultiLabelSpaceTimeAStar::findShortestPath(ConstraintTable &constraint_table, const pair<int, int> start_state, int lowerbound)
{

	// for vertex of different stage, the f val should be bounded within the ub.
	vector<int> f_ub(goal_location.size(), INT_MAX);
	if (constraint_table.leq_goal_time[goal_location.size() - 1] != INT_MAX)
	{
		f_ub.back() = constraint_table.leq_goal_time[goal_location.size() - 1];
	}
	for (int i = (int)goal_location.size() - 2; i >= 0; i--)
	{
		if (constraint_table.leq_goal_time[i] != INT_MAX)
		{
			f_ub[i] = min(f_ub[i + 1], constraint_table.leq_goal_time[i] + heuristic_landmark[i]);
		}
		else
		{
			f_ub[i] = f_ub[i + 1];
		}
	}

	// generate start and add it to the OPEN & FOCAL list
	Path path;
	auto start = new MultiLabelAStarNode(start_state.first,					  // location
										 0,									  // g val
										 get_heuristic(0, start_state.first), // h val
										 nullptr,							  // parent
										 start_state.second,				  // timestep
										 0,									  // stage
										 0, false);

	// start->timestamps.resize(goal_location.size());

	num_generated++;
	start->open_handle = open_list.push(start);
	start->focal_handle = focal_list.push(start);
	start->in_openlist = true;

	if (start->location == (unsigned int)goal_location[0] && constraint_table.g_goal_time[0] < start->g_val)
	{
		start->stage += 1;
		if (use_timestamps)
		{
			// timestamps.push_back(0);
		}
	}

	allNodes_table.insert(start);
	min_f_val = (int)start->getFVal();
	int holding_time = constraint_table.getHoldingTime(); // the earliest timestep that the agent can hold its goal location. The length_min is considered here.
	lower_bound = max(holding_time - start_state.second, max(min_f_val, lowerbound));

	while (!open_list.empty())
	{
		updateFocalList(); // update FOCAL if min f-val increased
		auto *curr = popNode();

		// check if the popped node is a goal
		if (curr->location == goal_location.back() &&  // arrive at the goal location
			curr->stage == goal_location.size() - 1 && // reach all previous goals
			!curr->wait_at_goal &&					   // not wait at the goal location
			curr->timestep >= holding_time)			   // the agent can hold the goal location afterward
		{
			updatePath(curr, path);
			break;
		}

		if (curr->timestep >= constraint_table.length_max)
			continue;

		list<int> next_locations = instance.getNeighbors(curr->location);
		next_locations.emplace_back(curr->location);
		// generate child
		for (int next_location : next_locations)
		{
			int next_timestep = curr->timestep + 1;
			// Is the current timestep the longest? Then everyone else finished moving.
			if (max(constraint_table.cat_size, constraint_table.latest_timestep) + 1 < curr->timestep)
			{ // now everything is static, so switch to space A* where we always use the same timestep
				// Exclude wait action, since there is no point not moving.
				if (next_location == curr->location)
				{
					continue;
				}
				next_timestep--;
			}
			// skip if the next_location is constrained at the next_timestep
			if (constraint_table.constrained(next_location, next_timestep) ||
				constraint_table.constrained(curr->location, next_location, next_timestep))
				continue;

			// setting the stage
			auto stage = curr->stage;
			auto timestamps = curr->timestamps;

			// compute cost to next_id via curr node
			int next_g_val = curr->g_val + 1;
			int next_h_val = get_heuristic(stage, next_location);
			if (next_g_val + next_h_val > constraint_table.length_max || next_g_val + next_h_val > f_ub[stage])
				continue;

			if (next_location == (unsigned int)goal_location[stage] && stage < goal_location.size() - 1 && constraint_table.g_goal_time[stage] < curr->g_val + 1)
			{
				stage += 1;
				if (use_timestamps)
				{
					timestamps.push_back(curr->g_val + 1);
				}
			}

			int next_internal_conflicts = curr->num_of_conflicts +
										  constraint_table.getNumOfConflictsForStep(curr->location, next_location, next_timestep);

			// generate (maybe temporary) node
			auto next = new MultiLabelAStarNode(next_location, next_g_val, next_h_val,
												curr, next_timestep, stage, next_internal_conflicts, false);

			next->timestamps = timestamps;
			next->dist_to_next = my_heuristic[stage][next_location];

			if (next->stage == goal_location.size() - 1 && next_location == goal_location.back() && curr->location == goal_location.back())
			{
				next->wait_at_goal = true;
			}

			// try to retrieve it from the hash table
			auto it = allNodes_table.find(next);
			if (it == allNodes_table.end())
			{
				pushNode(next);
				allNodes_table.insert(next);
				continue;
			}
			// update existing node's if needed (only in the open_list)

			auto existing_next = *it;
			if (existing_next->getFVal() > next->getFVal() || // if f-val decreased through this new path
				(existing_next->getFVal() == next->getFVal() &&
				 LLNode::secondary_compare_node_not_random()(existing_next, next)
				 // existing_next->num_of_conflicts > next->num_of_conflicts
				 )) // or it remains the same but there's fewer conflicts
			{
				if (!existing_next->in_openlist) // if its in the closed list (reopen)
				{
					existing_next->copy(*next);
					pushNode(existing_next);
				}
				else
				{
					bool add_to_focal = false;	  // check if it was above the focal bound before and now below (thus need to be inserted)
					bool update_in_focal = false; // check if it was inside the focal and needs to be updated (because f-val changed)
					bool update_open = false;
					if ((next_g_val + next_h_val) <= lower_bound)
					{ // if the new f-val qualify to be in FOCAL
						if (existing_next->getFVal() > lower_bound)
							add_to_focal = true; // and the previous f-val did not qualify to be in FOCAL then add
						else
							update_in_focal = true; // and the previous f-val did qualify to be in FOCAL then update
					}
					if (existing_next->getFVal() > next_g_val + next_h_val)
						update_open = true;

					existing_next->copy(*next); // update existing node

					if (update_open)
						open_list.increase(existing_next->open_handle); // increase because f-val improved
					if (add_to_focal)
						existing_next->focal_handle = focal_list.push(existing_next);
					if (update_in_focal)
						focal_list.update(existing_next->focal_handle); // should we do update? yes, because number of conflicts may go up or down
				}
			}
			delete next; // not needed anymore -- we already generated it before
		} // end for loop that generates successors
	} // end while loop

	releaseNodes();
	return path;
}

Path MultiLabelSpaceTimeAStar::findPathSegment(ConstraintTable &constraint_table, int start_time, int stage, int lowerbound)
{
	int loc = start_location;
	if (stage != 0)
	{
		loc = goal_location[stage - 1];
	}

	// generate start and add it to the OPEN & FOCAL list
	Path path;
	path.begin_time = start_time;
	auto start = new MultiLabelAStarNode(loc,						// location
										 0,							// g val
										 get_heuristic(stage, loc), // h val
										 nullptr,					// parent
										 start_time,				// timestep
										 stage,						// stage
										 0, false);

	start->secondary_keys.push_back(-start->g_val);
	// start->timestamps.resize(goal_location.size());

	num_generated++;
	start->open_handle = open_list.push(start);
	start->focal_handle = focal_list.push(start);
	start->in_openlist = true;
	allNodes_table.insert(start);
	min_f_val = (int)start->getFVal();
	int holding_time = constraint_table.length_min;
	if (stage == goal_location.size() - 1)
	{
		holding_time = constraint_table.getHoldingTime(); // the earliest timestep that the agent can hold its goal location. The length_min is considered here.
	}
	lower_bound = max(holding_time - start_time, max(min_f_val, lowerbound));

	while (!open_list.empty())
	{
		updateFocalList(); // update FOCAL if min f-val increased
		auto *curr = popNode();
		// check if the popped node is a goal
		if (curr->location == goal_location[stage] && // reach all previous goals
			curr->timestep >= holding_time)			  // the agent can hold the goal location afterward
		{
			updatePath(curr, path);
			break;
		}
		if (curr->timestep >= constraint_table.length_max)
			continue;

		list<int> next_locations = instance.getNeighbors(curr->location);
		next_locations.emplace_back(curr->location);
		// generate child
		for (int next_location : next_locations)
		{
			int next_timestep = curr->timestep + 1;
			if (max(constraint_table.cat_size, constraint_table.latest_timestep) + 1 < curr->timestep)
			{ // now everything is static, so switch to space A* where we always use the same timestep
				if (next_location == curr->location)
				{
					continue;
				}
				next_timestep--;
			}

			if (constraint_table.constrained(next_location, next_timestep) ||
				constraint_table.constrained(curr->location, next_location, next_timestep))
				continue;

			// setting the stage
			auto stage = curr->stage;
			auto timestamps = curr->timestamps;

			// compute cost to next_id via curr node
			int next_g_val = curr->g_val + 1;
			// int next_h_val = my_heuristic[next_location];
			int next_h_val = max(get_heuristic(stage, next_location), holding_time - next_timestep);
			// if (next_g_val + next_h_val > constraint_table.length_max || next_g_val + next_h_val > f_ub[stage])
			// 	continue;
			// TODO use CAT
			int next_internal_conflicts = curr->num_of_conflicts;

			// generate (maybe temporary) node
			auto next = new MultiLabelAStarNode(next_location, next_g_val, next_h_val,
												curr, next_timestep, stage, next_internal_conflicts, false);

			next->timestamps = timestamps;
			next->secondary_keys.push_back(-next_g_val);

			next->dist_to_next = my_heuristic[stage][next_location];

			if (next->stage == goal_location.size() - 1 && next_location == goal_location.back() && curr->location == goal_location.back())
			{
				next->wait_at_goal = true;
			}

			// try to retrieve it from the hash table
			// cout << "debug allNodes_table " << allNodes_table.size() << endl;
			auto it = allNodes_table.find(next);
			if (it == allNodes_table.end())
			{
				pushNode(next);
				allNodes_table.insert(next);
				continue;
			}
			// update existing node's if needed (only in the open_list)

			auto existing_next = *it;
			if (existing_next->getFVal() > next->getFVal() || // if f-val decreased through this new path
				(existing_next->getFVal() == next->getFVal() &&
				 LLNode::secondary_compare_node_not_random()(existing_next, next)
				 // existing_next->num_of_conflicts > next->num_of_conflicts
				 )) // or it remains the same but there's fewer conflicts
			{
				if (!existing_next->in_openlist) // if its in the closed list (reopen)
				{
					existing_next->copy(*next);
					pushNode(existing_next);
				}
				else
				{
					bool add_to_focal = false;	  // check if it was above the focal bound before and now below (thus need to be inserted)
					bool update_in_focal = false; // check if it was inside the focal and needs to be updated (because f-val changed)
					bool update_open = false;
					if ((next_g_val + next_h_val) <= lower_bound)
					{ // if the new f-val qualify to be in FOCAL
						if (existing_next->getFVal() > lower_bound)
							add_to_focal = true; // and the previous f-val did not qualify to be in FOCAL then add
						else
							update_in_focal = true; // and the previous f-val did qualify to be in FOCAL then update
					}
					if (existing_next->getFVal() > next_g_val + next_h_val)
						update_open = true;

					existing_next->copy(*next); // update existing node

					if (update_open)
						open_list.increase(existing_next->open_handle); // increase because f-val improved
					if (add_to_focal)
						existing_next->focal_handle = focal_list.push(existing_next);
					if (update_in_focal)
						focal_list.update(existing_next->focal_handle); // should we do update? yes, because number of conflicts may go up or down
				}
			}
			delete next; // not needed anymore -- we already generated it before
		} // end for loop that generates successors
	} // end while loop

	releaseNodes();
	return path;
}

Path MultiLabelSpaceTimeAStar::findPath(const CBSNode &node, const ConstraintTable &initial_constraints,
										const vector<Path *> &paths, int agent, int lowerbound)
{
	num_expanded = 0;
	num_generated = 0;
	// build constraint table
	auto starrt_time = clock();
	ConstraintTable constraint_table(initial_constraints);
	constraint_table.build(node, agent, goal_location.size());

	runtime_build_CT = (double)(clock() - starrt_time) / CLOCKS_PER_SEC;
	if (constraint_table.length_min >= MAX_TIMESTEP || constraint_table.length_min > constraint_table.length_max || // the agent cannot reach
																													// its goal location
		constraint_table.constrained(start_location, 0))															// the agent cannot stay at its start location
	{
		return Path();
	}

	starrt_time = clock();
	constraint_table.buildCAT(agent, paths, node.makespan + 1);
	runtime_build_CAT = (double)(clock() - starrt_time) / CLOCKS_PER_SEC;

	return findShortestPath(constraint_table, make_pair(start_location, 0), lowerbound);
}

Path MultiLabelSpaceTimeAStar::findPathSegmentToParkWithTrajAvoid(ConstraintTable &constraint_table, int start_time, int segment_start, int agent_current_loc, int parking_loc, vector<int> trajectory, vector<int> trajEnds, vector<int> segmentIDs, vector<int> locVal)
{

	// system parameters
	avoid_dummy_path = true;
	findPathSegmentToPark_time = 0;
	timeout = false;
	num_expanded = 0;
	clock_t findPathSegmentToPark_time_start = clock();

	// generate start and add it to the OPEN & FOCAL list
	Path path;
	path.begin_time = start_time;
	int curr_seq_index = 0;
	int segment_start_loc = trajectory.front();
	int segment_end_loc = trajectory.back();
	auto start = new MultiLabelAStarNode(agent_current_loc,																								 // location
										 0,																												 // g-val
										 get_heuristic_ddmapd(agent_current_loc, 0, segment_start_loc, segment_end_loc, trajectory.size(), parking_loc), // h-val
										 nullptr,																										 // parent
										 start_time,																									 // timestep
										 0,																												 // stage
										 0,																												 // num_of_conflicts
										 false);																										 // in_openlist

	start->segment_stage = 0; // segment stage
	start->segmentID = segmentIDs[curr_seq_index];
	start->path_idx = 0;
	start->in_openlist = true;
	start->open_handle = open_list.push(start);

	num_generated++;
	// pushNode(start);
	allNodes_table.insert(start);
	min_f_val = (int)start->getFVal();
	int holding_time = constraint_table.length_min; // the earliest timestep that the agent can hold its goal location. The length_min is considered here.

	// debug delete later print the segment path
	// cout << "segment trajectory ";
	// for (auto loc : agent_segments[stage].trajectory)
	// {
	// 	cout << "[" << instance.getRowCoordinate(loc) << "," << instance.getColCoordinate(loc) << "] @ " << loc << " --> ";
	// }
	// cout << endl;
	// cout << "parking " << instance.start_locations[agent_idx] << endl;

	while (!open_list.empty())
	{
		// TODO  overtime limit
		// if ((double)(clock() - findPathSegmentToPark_time_start) / CLOCKS_PER_SEC > 2)
		// {
		// 	timeout = true;
		// 	break;
		// }

		auto *curr = open_list.top();
		open_list.pop();
		curr->in_openlist = false;
		num_expanded++;

		// debug TODO del
		// debug_agent = 1; // TODO del
		if (debug_agent != -1)
		{
			cout << "## A* pop node loc " << curr->location << " (" << instance.getRowCoordinate(curr->location) << "," << instance.getColCoordinate(curr->location) << ") g-val " << curr->g_val << " h-val "  << curr->h_val   << " f-val " << curr->getFVal() << " t " << curr->timestep << " stage " << curr->stage << " segment_stage " << curr->segment_stage << " num_of_conflicts " << curr->num_of_conflicts << " is_dummy_path " << curr->is_dummy_path << " timestep " << curr->timestep << " segmentID " << curr->segmentID << endl;
			if (curr->location == 190)
			{
				int debuging = 1;
			}
			printPath(curr);
		}
		// debug TODO del

		// reach the safe parking location
		if (curr->segment_stage == 1 && curr->location == instance.start_locations[agent_idx])
		{
			printPath(curr);
			updatePath(curr, path);
			break;
		}

		if (curr->timestep >= constraint_table.length_max)
			continue;

		// if reached the segment start, try to add a action to reach the segment end
		if (curr->segment_stage == 0 && curr->location == trajectory.front() && start_time + curr->g_val >= segment_start)
		{
			// update curr with segment trajectory by creating new node
			MultiLabelAStarNode *curr_copy = curr;
			vector<MultiLabelAStarNode*> traj_nodes;
			bool traj_constrained = false;
			for (int i = 1; i < trajectory.size(); i++)
			{ // trajectory.size() - 1 to exclude the trajectory end location
				auto next = new MultiLabelAStarNode(trajectory[i], curr_copy->g_val + 1, curr_copy->h_val, curr_copy, curr_copy->timestep + 1, 0, curr_copy->num_of_conflicts, false);
				trajNodes_table.push_back(next);
				traj_nodes.push_back(next);
				next->path_idx = curr_copy->path_idx + 1;
				next->segmentID = segmentIDs[curr_seq_index];

				if (debug_agent != -1){
					cout << "expand trajectory node : " << next->location << " (" << instance.getRowCoordinate(next->location) << "," << instance.getColCoordinate(next->location) << ") g-val " << next->g_val << " h-val "  << next->h_val   << " f-val " << next->getFVal() << " t " << next->timestep << " stage " << next->stage << " segment_stage " << next->segment_stage << " num_of_conflicts " << next->num_of_conflicts << " is_dummy_path " << next->is_dummy_path << " timestep " << next->timestep << " segmentID " << next->segmentID << endl;
				}

				// if reach the trajEnds[curr_seq_index] time step, update the segmentID
				if (i == trajEnds[curr_seq_index])
				{
					curr_seq_index++;
				}
				// if the trajectory is constrained, then break
				if (constraint_table.constrained(trajectory[i], next->timestep) ||
					constraint_table.constrained(trajectory[i - 1], trajectory[i], next->timestep))
				{	
					if (debug_agent != -1){
						cout << "constrained " << trajectory[i] << " " << next->timestep << endl;
					}
					traj_constrained = true; // if the trajectory is constrained, then break and keep exploring the normal neighbors
					// delete all the nodes in the traj_nodes
					for (auto node : traj_nodes)
					{
						delete node;
						trajNodes_table.pop_back();
					}
					traj_nodes.clear();
					break;
				}
				curr_copy = next; // update the curr_copy
			}
			traj_nodes.clear();
			// if the trajectory not constrained; curr_copy points to the last node of the trajectory
			if (!traj_constrained)
			{
				auto next_h_val = get_heuristic_ddmapd(curr_copy->location, 1, segment_start_loc, segment_end_loc, trajectory.size(), parking_loc);
				curr_copy->h_val = next_h_val;
				curr_copy->g2_val = locVal[curr_copy->location];
				curr_copy->h2_val = max(next_h_val, holding_time - curr_copy->timestep) - 1;
				curr_copy->segment_stage = 1; // dummy path
				curr_copy->timestamps = curr->timestamps;
				curr_copy->secondary_keys.push_back(-curr_copy->g_val);
				curr_copy->wait_at_goal = true;
				curr_copy->is_dummy_path = true;

				// try to retrieve it from the hash table
				auto it = allNodes_table.find(curr_copy);
				if (it == allNodes_table.end())
				{
					pushNode(curr_copy);
					allNodes_table.insert(curr_copy);
					// // cout << "allNodes_table insert " << curr_copy->location << " " << curr_copy->timestep << " " << curr_copy->segment_stage << " " << curr_copy->segmentID << endl;
					// // debug TODO del
					if (debug_agent != -1)
					{
						cout << "A* push node loc " << curr_copy->location << " (" << instance.getRowCoordinate(curr_copy->location) << "," << instance.getColCoordinate(curr_copy->location) << ") g-val " << curr_copy->g_val << " h-val " << curr_copy->h_val << " t " << curr_copy->timestep << " stage " << curr_copy->stage << " segment_stage " << curr_copy->segment_stage << " num_of_conflicts " << curr_copy->num_of_conflicts << " is_dummy_path " << curr_copy->is_dummy_path <<  " timestep " << curr_copy->timestep << " segmentID " << curr_copy->segmentID << endl;
					}
					// // debug TODO del
				}
				else
				{
					// update existing node's if needed (only in the open_list)
					auto existing_next = *it;
					if (existing_next->getFVal() > curr_copy->getFVal() || // if f-val decreased through this new path
						(existing_next->getFVal() == curr_copy->getFVal() &&
						 existing_next->g2_val + existing_next->g_val > curr_copy->g2_val + curr_copy->g_val)) // or it remains the same but there's fewer conflicts
					{
						if (!existing_next->in_openlist) // if its in the closed list (reopen)
						{
							existing_next->copy(*curr_copy);
							pushNode(existing_next);
							// // debug TODO del
							if (debug_agent != -1)
							{
								cout << "A* push node loc " << curr_copy->location << " (" << instance.getRowCoordinate(curr_copy->location) << "," << instance.getColCoordinate(curr_copy->location) << ") g-val " << curr_copy->g_val << " h-val " << curr_copy->h_val << " t " << curr_copy->timestep << " stage " << curr_copy->stage << " segment_stage " << curr_copy->segment_stage << " num_of_conflicts " << curr_copy->num_of_conflicts << " is_dummy_path " << curr_copy->is_dummy_path << " timestep " << curr_copy->timestep << " segmentID " << curr_copy->segmentID << endl;
							}
							// // debug TODO del
						}
						else
						{
							bool update_open = false;
							if (existing_next->getFVal() > curr_copy->g_val + curr_copy->h_val)
								update_open = true;
							// TODO maybe need a open handle here
							existing_next->copy(*curr_copy); // update existing node
							if (update_open)
								open_list.increase(existing_next->open_handle); // increase because f-val improved
						}
					}
					trajNodes_table.pop_back();
					delete curr_copy;
				}
			}
			else{
				curr_seq_index = 0; // reset the sequence index
			}
		}

		list<int> next_locations = instance.getNeighbors(curr->location);
		AvoidSafeParking(next_locations); // avoid the safe parking location of other agents
		next_locations.emplace_back(curr->location);

		// generate child
		for (int next_location : next_locations)
		{
			int next_timestep = curr->timestep + 1;
			if (max(constraint_table.cat_size, constraint_table.latest_timestep) + 1 < curr->timestep)
			{ // now everything is static, so switch to space A* where we always use the same timestep
				if (next_location == curr->location)
				{
					continue;
				}
				next_timestep--;
			}

			if (constraint_table.constrained(next_location, next_timestep) ||
				constraint_table.constrained(curr->location, next_location, next_timestep))
				continue;

			// setting the stage
			auto stage = curr->stage;
			auto timestamps = curr->timestamps;

			// compute cost to next_id via curr node
			int next_g_val;
			int next_h_val;
			if (curr->segment_stage == 0)
			{
				next_g_val = curr->g_val + 1;
				next_h_val = max(get_heuristic_ddmapd(next_location, curr->segment_stage, segment_start_loc, segment_end_loc, trajectory.size(), parking_loc), holding_time - next_timestep);
			}
			else
			{
				next_g_val = curr->g_val;
				next_h_val = curr->h_val;
			}

			int next_internal_conflicts = curr->num_of_conflicts;

			// generate (maybe temporary) node
			auto next = new MultiLabelAStarNode(next_location, next_g_val, next_h_val,
												curr, next_timestep, stage, next_internal_conflicts, false);

			next->segmentID = curr->segmentID;
			next->timestamps = timestamps;
			next->path_idx = curr->path_idx + 1;
			next->secondary_keys.push_back(-next_g_val);
			next->segment_stage = curr->segment_stage;
			next->is_dummy_path = curr->is_dummy_path;
			// next->dist_to_next = my_heuristic[stage][next_location];

			if (curr->segment_stage == 1)
			{
				next->g2_val = curr->g2_val + 1 + locVal[next_location];
				next->h2_val = max(get_heuristic_ddmapd(next->location, 1, segment_start_loc, segment_end_loc, trajectory.size(), parking_loc), holding_time - next_timestep) - 1;
				next->segmentID = -1; // dummy path not for the segment
			}

			if (next->stage == goal_location.size() - 1 && next_location == goal_location.back() && curr->location == goal_location.back())
			{
				next->wait_at_goal = true;
			}

			// try to retrieve it from the hash table
			// cout << "debug allNodes_table " << allNodes_table.size() << endl;
			auto it = allNodes_table.find(next);
			if (it == allNodes_table.end())
			{
				pushNode(next);
				allNodes_table.insert(next);
				// cout << "allNodes_table insert " << next->location << " " << next->timestep << " " << next->segment_stage << " " << next->segmentID << endl;
				continue;
			}
			// update existing node's if needed (only in the open_list)
			// TODO check the second stange
			auto existing_next = *it;
			if (existing_next->getFVal() > next->getFVal() || // if f-val decreased through this new path
				(existing_next->getFVal() == next->getFVal() &&
				 existing_next->g2_val + existing_next->g_val > next->g2_val + next->g_val)) // or it remains the same but there's fewer conflicts
			{
				if (!existing_next->in_openlist) // if its in the closed list (reopen)
				{
					existing_next->copy(*next);
					pushNode(existing_next);
					// // debug TODO del
					if (debug_agent != -1)
					{
						cout << "A* push node loc " << existing_next->location << " (" << instance.getRowCoordinate(existing_next->location) << "," << instance.getColCoordinate(existing_next->location) << ") g-val " << existing_next->g_val << " h-val " << existing_next->h_val << " t " << existing_next->timestep << " stage " << existing_next->stage << " segment_stage " << existing_next->segment_stage << " num_of_conflicts " << existing_next->num_of_conflicts << " is_dummy_path " << existing_next->is_dummy_path << endl;
					}
					// // debug TODO del
				}
				else
				{
					bool update_open = false;
					if (existing_next->getFVal() > next_g_val + next_h_val)
						update_open = true;

					existing_next->copy(*next); // update existing node

					if (update_open)
						open_list.increase(existing_next->open_handle); // increase because f-val improved
				}
			}
			delete next; // not needed anymore -- we already generated it before
		} // end for loop that generates successors

	} // end while loop

	releaseNodes();

	findPathSegmentToPark_time = (double)(clock() - findPathSegmentToPark_time_start) / CLOCKS_PER_SEC;
	return path;
}

int MultiLabelSpaceTimeAStar::getTravelTime(int start, int end, const ConstraintTable &constraint_table, int upper_bound)
{
	int length = MAX_TIMESTEP;
	if (constraint_table.length_min >= MAX_TIMESTEP || constraint_table.length_min > constraint_table.length_max || // the agent cannot reach
																													// its goal location
		constraint_table.constrained(start, 0))																		// the agent cannot stay at its start location
	{
		return length;
	}
	auto root = new MultiLabelAStarNode(start, 0, compute_heuristic(start, end), nullptr, 0, 0);
	root->open_handle = open_list.push(root); // add root to heap
	allNodes_table.insert(root);			  // add root to hash_table (nodes)
	MultiLabelAStarNode *curr = nullptr;
	while (!open_list.empty())
	{
		curr = open_list.top();
		open_list.pop();
		if (curr->location == end)
		{
			length = curr->g_val;
			break;
		}
		list<int> next_locations = instance.getNeighbors(curr->location);
		next_locations.emplace_back(curr->location);
		for (int next_location : next_locations)
		{
			int next_timestep = curr->timestep + 1;
			int next_g_val = curr->g_val + 1;
			if (constraint_table.latest_timestep <= curr->timestep)
			{
				if (curr->location == next_location)
				{
					continue;
				}
				next_timestep--;
			}
			if (!constraint_table.constrained(next_location, next_timestep) &&
				!constraint_table.constrained(curr->location, next_location, next_timestep))
			{ // if that grid is not blocked

				// setting the stage
				auto stage = curr->stage;
				if (next_location == goal_location[stage] && stage < goal_location.size() - 1)
				{
					stage += 1;
				}

				int next_h_val = compute_heuristic(next_location, end);
				if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
					continue;

				auto next = new MultiLabelAStarNode(next_location, next_g_val, next_h_val, nullptr, next_timestep, stage);
				auto it = allNodes_table.find(next);
				if (it == allNodes_table.end())
				{ // add the newly generated node to heap and hash table
					next->open_handle = open_list.push(next);
					allNodes_table.insert(next);
				}
				else
				{				 // update existing node's g_val if needed (only in the heap)
					delete next; // not needed anymore -- we already generated it before
					auto existing_next = *it;
					if (existing_next->g_val > next_g_val)
					{
						existing_next->g_val = next_g_val;
						existing_next->timestep = next_timestep;
						open_list.increase(existing_next->open_handle);
					}
				}
			}
		}
	}
	releaseNodes();
	return length;
}

inline MultiLabelAStarNode *MultiLabelSpaceTimeAStar::popNode()
{
	auto node = focal_list.top();
	focal_list.pop();
	open_list.erase(node->open_handle);
	node->in_openlist = false;
	num_expanded++;
	return node;
}

inline void MultiLabelSpaceTimeAStar::pushNode(MultiLabelAStarNode *node)
{
	// debug TODO del
	if (debug_agent != -1)
	{
		cout << "A* push node loc " << node->location << " (" << instance.getRowCoordinate(node->location) << "," << instance.getColCoordinate(node->location) << ") g-val " << node->g_val << " h-val " << node->h_val << " f-val " << node->getFVal() << " t " << node->timestep << " stage " << node->stage << " segment_stage " << node->segment_stage << " num_of_conflicts " << node->num_of_conflicts << " is_dummy_path " << node->is_dummy_path << " timestep " << node->timestep << " segmentID " << node->segmentID << endl;
	}
	node->open_handle = open_list.push(node);
	node->in_openlist = true;
	num_generated++;

	// if (node->getFVal() <= lower_bound)
	// 	node->focal_handle = focal_list.push(node);
}

void MultiLabelSpaceTimeAStar::updateFocalList()
{
	auto open_head = open_list.top(); // open_list sorted by f_val and g2_val, h2_val
	if (open_head->getFVal() > min_f_val)
	{
		int new_min_f_val = (int)open_head->getFVal();
		int new_lower_bound = max(lower_bound, new_min_f_val);
		int curr_segment_stage = open_head->segment_stage;
		// for (auto n : open_list)
		// {
		// 	if (n->getFVal() > lower_bound  && n->getFVal() <= new_lower_bound && curr_segment_stage == n->segment_stage){
		// 		n->focal_handle = focal_list.push(n);
		// 	}
		// 	else if (n->getFVal() > lower_bound  && curr_segment_stage == 0 && n->segment_stage == 1){
		// 		n->focal_handle = focal_list.push(n);
		// 	} // skip if curr_segment_stage == 1 && n->segment_stage == 0
		// }

		for (auto n : open_list)
		{
			if (n->getFVal() > lower_bound && n->getFVal() <= new_lower_bound)
				n->focal_handle = focal_list.push(n);
		}
		min_f_val = new_min_f_val;
		lower_bound = new_lower_bound;
	}
}

void MultiLabelSpaceTimeAStar::releaseNodes()
{

	for (auto *node : open_list)
	{
		if (node == nullptr)
		{
			std::cerr << "Error: Null pointer found in heap!" << std::endl;
		}
	}
	open_list.clear();
	focal_list.clear();

	for (auto node : trajNodes_table)
	{
		if (allNodes_table.find(node) != allNodes_table.end())
		{
			allNodes_table.erase(node);
		}
		delete node;
	}

	for (auto node : allNodes_table)
	{
		// if (std::find(trajNodes_table.begin(), trajNodes_table.end(), node) != trajNodes_table.end())
		// {
		//     continue;
		// }
		delete node;
	}

	trajNodes_table.clear();
	allNodes_table.clear();
}

void MultiLabelSpaceTimeAStar::AvoidSafeParking(list<int> &next_locations)
{
	for (auto it = next_locations.begin(); it != next_locations.end();)
	{
		if (instance.AtOtherAgentParking(*it, agent_idx))
			it = next_locations.erase(it);
		else
			++it;
	}
}

int MultiLabelSpaceTimeAStar::get_heuristic_ddmapd(int loc, int segment_stage, int segment_start_loc, int segment_end_loc, int traj_len, int park_loc) const
{
	if (segment_stage == 0) // to the segment start location
	{
		// h to segment start + segment length + segment end to parking location
		return instance.getManhattanDistance(loc, segment_start_loc) + traj_len - 1 + instance.getManhattanDistance(segment_end_loc, park_loc);
	}
	else
	{
		// segment end to parking location
		return instance.getManhattanDistance(loc, park_loc);
	}
}