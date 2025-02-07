#include "SpaceTimeAStar.h"

void MultiLabelSpaceTimeAStar::printPath(const LLNode *goal)
{
	// cout << "Segment_stage : " << goal->segment_stage  << endl;
	cout << " Path: ";
	const LLNode *curr = goal;
	// int idx = 0;
	// int dummy_start_debug = -1;
	while (curr != nullptr)
	{
		// path[curr->g_val].location = curr->location;

		cout << "[" << instance.getRowCoordinate(curr->location) << "," << instance.getColCoordinate(curr->location) << "]@" << curr->path_idx << " ";
		cout << curr->location << " (g " << curr->g_val << ", h " << curr->h_val << ", t " << curr->path_idx << ", s " << curr->stage << " d " << curr->is_dummy_path << " g2 " << curr->g2_val << ", h2 " << curr->h2_val << " )  <-- "; // " idx " << idx <<
		// path[curr->g_val].single = false;
		// path[curr->g_val].mdd_width = 0;
		// if (curr->is_dummy_path && dummy_start_debug == -1)
		// {
		// 	dummy_start_debug = idx + 1;
		// }
		// idx ++ ;
		curr = curr->parent;
	}
	cout << endl;
	// cout << "dummy_start_timestep : " << dummy_start_debug << endl;
}

void MultiLabelSpaceTimeAStar::updatePath(const LLNode *goal, Path &path)
{
	if (avoid_dummy_path)
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
		path.timestamps.resize(goal_location.size(), 0);
		path.timestamps.back() = path_idx;
		dummy_path_len = -1; // exclude the start point (segment end point) of dummy path

		const LLNode *curr = goal;
		while (curr != nullptr)
		{
			path[path_idx].location = curr->location;
			path[path_idx].task = curr->task;
			// path[path_idx].single = false;
			path[path_idx].mdd_width = 0;

			if (curr->parent != nullptr && curr->stage != curr->parent->stage)
			{
				path.timestamps[curr->parent->stage] = path_idx;
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
	else
	{
		path.path.resize(goal->g_val + 1);
		path.timestamps.resize(goal_location.size(), 0);
		path.timestamps.back() = goal->g_val;
		dummy_path_len = -1; // exclude the start point (segment end point) of dummy path

		const LLNode *curr = goal;
		int idx = 0;
		while (curr != nullptr)
		{
			path[curr->g_val].location = curr->location;
			path[curr->g_val].task = curr->task;
			// path[curr->g_val].single = false;
			path[curr->g_val].mdd_width = 0;

			if (curr->parent != nullptr && curr->stage != curr->parent->stage)
			{
				path.timestamps[curr->parent->stage] = curr->g_val;
				path[curr->g_val].is_goal = true;
			}
			else
			{
				path[curr->g_val].is_goal = false;
			}
			if (curr->is_dummy_path)
			{
				dummy_path_len++;
			}
			curr = curr->parent;
		}
	}
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
			// int next_h_val = my_heuristic[next_location];
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

/* a copy of findPathSegmentToPark; able to plan dummy path for each goal

Path MultiLabelSpaceTimeAStar::findPathSegmentToPark(ConstraintTable &constraint_table, int start_time, int stage, int lowerbound)
{

	int loc = start_location;

	// generate start and add it to the OPEN & FOCAL list
	Path path;
	path.begin_time = start_time;
	auto start = new MultiLabelAStarNode(loc,						// location
										 0,							// g val
										 get_heuristic_ddmapd(stage, loc, 0), // h val segment_stage is 0
										 nullptr,					// parent
										 start_time,				// timestep
										 stage,						// stage
										 0, false);

	start->segment_stage = 0; // segment stage
	start->secondary_keys.push_back(-start->g_val);
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

	// TODO delete later print the segment path
	// cout << "segment trajectory ";
	// for (auto loc : agent_segments[stage].trajectory)
	// {
	// 	cout << "[" << instance.getColCoordinate(loc) << "," << instance.getRowCoordinate(loc) << "] @ " << loc << " --> ";
	// }
	// cout << endl;

	while (!open_list.empty())
	{
		updateFocalList(); // update FOCAL if min f-val increased
		auto *curr = popNode();
		// printPath(curr); // TODO del

		if (curr->segment_stage == 1 && curr->location == instance.start_locations[agent_idx]){ // reach the safe parking location
			updatePath(curr, path);
			break;
		}

		if (curr->timestep >= constraint_table.length_max)
			continue;

		list<int> next_locations = instance.getNeighbors(curr->location);
		AvoidSafeParking(next_locations); // avoid the safe parking location of other agents
		next_locations.emplace_back(curr->location);


		// if reached the segment start, try to add a action to reach the segment end
		if (curr->segment_stage == 0 && curr->location == goal_location[stage] && // reach all previous goals
			curr->timestep >= holding_time){
			// update curr with segment trajectory by creating new node
			auto segment_node = curr;
			bool traj_constrained = false;
			auto segment_trajectory = agent_segments[stage].trajectory;
			for (int i = 1; i < segment_trajectory.size(); i++){ // segment_trajectory.size() - 1 to exclude the trajectory end location
				auto next = new MultiLabelAStarNode(segment_trajectory[i], segment_node->g_val + 1, segment_node->h_val, segment_node, segment_node->timestep + 1, stage, segment_node->num_of_conflicts, false);
				if (constraint_table.constrained(segment_trajectory[i], segment_node->timestep + 1) ||
				constraint_table.constrained(segment_node->location, segment_trajectory[i], segment_node->timestep + 1)){
					traj_constrained = true; // if the trajectory is constrained, then break
					break;
				}
				segment_node = next;
				continue;
			}
			// if the trajectory is constrained, then continue, keep exploring the normal neighbors
			if (traj_constrained){
				continue;
			}

			auto next_g_val = segment_node->g_val; // “segment_trajectory.size() - 1” to exclude the start location
			auto next_h_val = get_heuristic_ddmapd(stage, segment_trajectory.back(), 1);
			segment_node->g_val = next_g_val;
			segment_node->h_val = next_h_val;
			segment_node->segment_stage = 1;
			segment_node->timestamps = curr->timestamps;
			segment_node->secondary_keys.push_back(-segment_node->g_val);
			segment_node->wait_at_goal = true;

			// reinitiliaze the curr node with the end location of the trajectory
			// cout << "reach the goal start " << goal_location[stage] << " jump to trajectory end " << agent_segments[stage].trajectory.back() << " moving to end " << instance.start_locations[agent_idx] << endl;
			// printPath(segment_node);

			// try to retrieve it from the hash table
			auto it = allNodes_table.find(segment_node);
			if (it == allNodes_table.end())
			{
				pushNode(segment_node);
				allNodes_table.insert(segment_node);
				continue;
			}
			// update existing node's if needed (only in the open_list)
			auto existing_next = *it;
			if (existing_next->getFVal() > segment_node->getFVal() || // if f-val decreased through this new path
				(existing_next->getFVal() == segment_node->getFVal() &&
				 LLNode::secondary_compare_node_not_random()(existing_next, segment_node)
				 && existing_next->segment_stage == 1)) // or it remains the same but there's fewer conflicts
			{
				if (!existing_next->in_openlist) // if its in the closed list (reopen)
				{
					existing_next->copy(*segment_node);
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

					existing_next->copy(*segment_node); // update existing node

					if (update_open)
						open_list.increase(existing_next->open_handle); // increase because f-val improved
					if (add_to_focal)
						existing_next->focal_handle = focal_list.push(existing_next);
					if (update_in_focal)
						focal_list.update(existing_next->focal_handle); // should we do update? yes, because number of conflicts may go up or down
				}
			}
			delete segment_node; // not needed anymore -- we already generated it before
		}


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
			auto segment_stage = curr->segment_stage;

			// compute cost to next_id via curr node
			int next_g_val = curr->g_val + 1;
			// int next_h_val = my_heuristic[next_location];
			int next_h_val = max(get_heuristic_ddmapd(stage, next_location, segment_stage), holding_time - next_timestep);
			// if (next_g_val + next_h_val > constraint_table.length_max || next_g_val + next_h_val > f_ub[stage])
			// 	continue;
			// TODO use CAT
			int next_internal_conflicts = curr->num_of_conflicts;

			// generate (maybe temporary) node
			auto next = new MultiLabelAStarNode(next_location, next_g_val, next_h_val,
												curr, next_timestep, stage, next_internal_conflicts, false);

			next->timestamps = timestamps;
			next->secondary_keys.push_back(-next_g_val);
			next->segment_stage = segment_stage;

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
			// TODO if  it's the same and the segment state is also the same
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

*/

Path MultiLabelSpaceTimeAStar::findPathSegmentToParkWithTrajAvoid(ConstraintTable &constraint_table, int start_time, int stage, int lowerbound, vector<int> locVal)
{

	avoid_dummy_path = true;
	findPathSegmentToPark_time = 0;
	timeout = false;
	num_expanded = 0;
	clock_t findPathSegmentToPark_time_start = clock();
	int loc;
	if (stage == 0)
	{
		loc = start_location;
	}
	else
	{
		// start from the end of the previous segment
		loc = agent_segments[stage - 1].trajectory.back();
	}

	// generate start and add it to the OPEN & FOCAL list
	Path path;
	path.begin_time = start_time;
	auto start = new MultiLabelAStarNode(loc,								  // location
										 0,									  // g val
										 get_heuristic_ddmapd(stage, loc, 0), // h val segment_stage is 0
										 nullptr,							  // parent
										 start_time,						  // timestep
										 stage,								  // stage
										 0, false);

	start->segment_stage = 0; // segment stage
	start->path_idx = 0;
	start->secondary_keys.push_back(-start->g_val);
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

	// debug delete later print the segment path
	cout << "segment trajectory ";
	for (auto loc : agent_segments[stage].trajectory)
	{
		cout << "[" << instance.getRowCoordinate(loc) << "," << instance.getColCoordinate(loc) << "] @ " << loc << " --> ";
	}
	cout << endl;
	cout << "parking " << instance.start_locations[agent_idx] << endl;

	while (!open_list.empty())
	{
		// TODO add time condition here
		if ((double)(clock() - findPathSegmentToPark_time_start) / CLOCKS_PER_SEC > 2)
		{
			timeout = true;
			break;
		}
		// updateFocalList(); // update FOCAL if min f-val increased
		// auto *curr = popNode();
		// debug TODO del
		// cout << "#### current open_list : " << endl;
		// for (auto one_node : open_list){
		// 	cout << "loc " << one_node->location << " g-val " << one_node->g_val << " h-val " << one_node->h_val << " g2-val " << one_node->g2_val << " h2-val " << one_node->h2_val << " t " << one_node->timestep << " stage " << one_node->stage << " segment_stage " << one_node->segment_stage << " num_of_conflicts " << one_node->num_of_conflicts << " is_dummy_path " << one_node->is_dummy_path << endl;
		// }
		// cout << "#### current open_list END " << endl;

		auto *curr = open_list.top();
		open_list.pop();
		curr->in_openlist = false;
		num_expanded++;

		// debug TODO del
		if (debug_agent != -1)
		{
			printPath(curr);
			cout << "A* pop node loc " << curr->location << " g-val " << curr->g_val << " h-val " << curr->h_val << " t " << curr->timestep << " stage " << curr->stage << " segment_stage " << curr->segment_stage << " num_of_conflicts " << curr->num_of_conflicts << " is_dummy_path " << curr->is_dummy_path << endl;
		}

		if (curr->segment_stage == 1 && curr->location == instance.start_locations[agent_idx])
		{ // reach the safe parking location
			printPath(curr);
			updatePath(curr, path);
			break;
		}

		if (curr->timestep >= constraint_table.length_max)
			continue;

		list<int> next_locations = instance.getNeighbors(curr->location);
		AvoidSafeParking(next_locations); // avoid the safe parking location of other agents
		next_locations.emplace_back(curr->location);

		// if reached the segment start, try to add a action to reach the segment end
		if (curr->segment_stage == 0 && curr->location == goal_location[stage] && // reach all previous goals
			curr->timestep >= constraint_table.length_min)
		{
			// update curr with segment trajectory by creating new node
			auto segment_node = curr;
			bool traj_constrained = false;
			auto segment_trajectory = agent_segments[stage].trajectory;
			for (int i = 1; i < segment_trajectory.size(); i++)
			{ // segment_trajectory.size() - 1 to exclude the trajectory end location
				auto next = new MultiLabelAStarNode(segment_trajectory[i], segment_node->g_val + 1, segment_node->h_val, segment_node, segment_node->timestep + 1, stage, segment_node->num_of_conflicts, false);
				next->task = stage;
				next->path_idx = next->g_val;
				if (constraint_table.constrained(segment_trajectory[i], segment_node->timestep + 1) ||
					constraint_table.constrained(segment_node->location, segment_trajectory[i], segment_node->timestep + 1))
				{
					traj_constrained = true; // if the trajectory is constrained, then break
					break;
				}
				segment_node = next;
			}
			// if the trajectory is constrained, then continue, keep exploring the normal neighbors
			if (!traj_constrained)
			{
				auto next_g_val = segment_node->g_val;
				auto next_h_val = get_heuristic_ddmapd(stage, segment_trajectory.back(), 1);
				segment_node->g_val = next_g_val;
				segment_node->path_idx = next_g_val;
				segment_node->h_val = next_h_val;
				segment_node->g2_val = locVal[segment_node->location];
				segment_node->h2_val = max(get_heuristic_ddmapd(stage, segment_node->location, 1), holding_time - segment_node->timestep) - 1;
				segment_node->segment_stage = 1;
				segment_node->timestamps = curr->timestamps;
				segment_node->secondary_keys.push_back(-segment_node->g_val);
				segment_node->wait_at_goal = true;
				segment_node->is_dummy_path = true;
				// reinitiliaze the curr node with the end location of the trajectory
				// debug TODO del
				cout << "reach the goal start " << goal_location[stage] << " jump to trajectory end " << agent_segments[stage].trajectory.back() << " moving to end " << instance.start_locations[agent_idx] << " g-val " << segment_node->g_val << " h-val " << segment_node->h_val << " g2-val " << segment_node->g2_val << " h2-val " << segment_node->h2_val << endl;
				// printPath(segment_node);

				// try to retrieve it from the hash table
				auto it = allNodes_table.find(segment_node);
				if (it == allNodes_table.end())
				{
					pushNode(segment_node);
					allNodes_table.insert(segment_node);

					// debug TODO del
					// cout << "#### current open_list : " << endl;
					// for (auto one_node : open_list){
					// 	cout << "loc " << one_node->location << " g-val " << one_node->g_val << " h-val " << one_node->h_val << " g2-val " << one_node->g2_val << " h2-val " << one_node->h2_val << " t " << one_node->timestep << " stage " << one_node->stage << " segment_stage " << one_node->segment_stage << " num_of_conflicts " << one_node->num_of_conflicts << " is_dummy_path " << one_node->is_dummy_path << endl;
					// }
					// cout << "#### current open_list END " << endl;
				}
				else
				{
					// update existing node's if needed (only in the open_list)
					auto existing_next = *it;
					if (existing_next->getFVal() > segment_node->getFVal() || // if f-val decreased through this new path
						(existing_next->getFVal() == segment_node->getFVal() &&
						 LLNode::secondary_compare_node_not_random()(existing_next, segment_node) && existing_next->segment_stage == 1)) // or it remains the same but there's fewer conflicts
					{
						if (!existing_next->in_openlist) // if its in the closed list (reopen)
						{
							existing_next->copy(*segment_node);
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

							existing_next->copy(*segment_node); // update existing node

							if (update_open)
								open_list.increase(existing_next->open_handle); // increase because f-val improved
							if (add_to_focal)
								existing_next->focal_handle = focal_list.push(existing_next);
							if (update_in_focal)
								focal_list.update(existing_next->focal_handle); // should we do update? yes, because number of conflicts may go up or down
						}
					}
					delete segment_node; // not needed anymore -- we already generated it before
				}
			}
		}

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
			auto segment_stage = curr->segment_stage;

			// compute cost to next_id via curr node
			int next_g_val;
			int next_h_val;
			if (segment_stage == 0)
			{
				next_g_val = curr->g_val + 1;
				next_h_val = max(get_heuristic_ddmapd(stage, next_location, segment_stage), holding_time - next_timestep);
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

			if (segment_stage == 1)
			{
				next->g2_val = curr->g2_val + 1 + locVal[next_location];
				next->h2_val = max(get_heuristic_ddmapd(stage, next_location, segment_stage), holding_time - next_timestep) - 1;
			}
			next->timestamps = timestamps;
			next->path_idx = curr->path_idx + 1;
			next->secondary_keys.push_back(-next_g_val);
			next->segment_stage = segment_stage;
			next->is_dummy_path = curr->is_dummy_path;

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
			// TODO if  it's the same and the segment state is also the same
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

	findPathSegmentToPark_time = (double)(clock() - findPathSegmentToPark_time_start) / CLOCKS_PER_SEC;
	return path;
}

Path MultiLabelSpaceTimeAStar::findPathSegmentToPark(ConstraintTable &constraint_table, int start_time, int stage, int lowerbound)
{
	int loc;
	if (stage == 0)
	{
		loc = start_location;
	}
	else
	{
		// start from the end of the previous segment
		loc = agent_segments[stage - 1].trajectory.back();
	}

	// generate start and add it to the OPEN & FOCAL list
	Path path;
	path.begin_time = start_time;
	auto start = new MultiLabelAStarNode(loc,								  // location
										 0,									  // g val
										 get_heuristic_ddmapd(stage, loc, 0), // h val segment_stage is 0
										 nullptr,							  // parent
										 start_time,						  // timestep
										 stage,								  // stage
										 0, false);

	start->segment_stage = 0; // segment stage
	start->secondary_keys.push_back(-start->g_val);
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

	// TODO delete later print the segment path
	// cout << "segment trajectory ";
	// for (auto loc : agent_segments[stage].trajectory)
	// {
	// 	cout << "[" << instance.getColCoordinate(loc) << "," << instance.getRowCoordinate(loc) << "] @ " << loc << " --> ";
	// }
	// cout << endl;

	while (!open_list.empty())
	{
		updateFocalList(); // update FOCAL if min f-val increased
		auto *curr = popNode();
		// printPath(curr); // TODO del

		if (debug_agent != -1)
		{
			printPath(curr);
			cout << "A* pop node loc " << curr->location << " g-val " << curr->g_val << " h-val " << curr->h_val << " t " << curr->timestep << " stage " << curr->stage << " segment_stage " << curr->segment_stage << " num_of_conflicts " << curr->num_of_conflicts << " is_dummy_path " << curr->is_dummy_path << endl;
		}

		if (curr->segment_stage == 1 && curr->location == instance.start_locations[agent_idx])
		{ // reach the safe parking location
			printPath(curr);
			updatePath(curr, path);
			break;
		}

		if (curr->timestep >= constraint_table.length_max)
			continue;

		list<int> next_locations = instance.getNeighbors(curr->location);
		AvoidSafeParking(next_locations); // avoid the safe parking location of other agents
		next_locations.emplace_back(curr->location);

		// if reached the segment start, try to add a action to reach the segment end
		if (curr->segment_stage == 0 && curr->location == goal_location[stage] && // reach all previous goals
			curr->timestep >= constraint_table.length_min)
		{
			// update curr with segment trajectory by creating new node
			auto segment_node = curr;
			bool traj_constrained = false;
			auto segment_trajectory = agent_segments[stage].trajectory;
			for (int i = 1; i < segment_trajectory.size(); i++)
			{ // segment_trajectory.size() - 1 to exclude the trajectory end location
				auto next = new MultiLabelAStarNode(segment_trajectory[i], segment_node->g_val + 1, segment_node->h_val, segment_node, segment_node->timestep + 1, stage, segment_node->num_of_conflicts, false);
				if (constraint_table.constrained(segment_trajectory[i], segment_node->timestep + 1) ||
					constraint_table.constrained(segment_node->location, segment_trajectory[i], segment_node->timestep + 1))
				{
					traj_constrained = true; // if the trajectory is constrained, then break
					break;
				}
				segment_node = next;
				continue;
			}
			// if the trajectory is constrained, then continue, keep exploring the normal neighbors
			if (!traj_constrained)
			{
				auto next_g_val = segment_node->g_val; // “segment_trajectory.size() - 1” to exclude the start location
				auto next_h_val = get_heuristic_ddmapd(stage, segment_trajectory.back(), 1);
				segment_node->g_val = next_g_val;
				segment_node->h_val = next_h_val;
				segment_node->segment_stage = 1;
				segment_node->timestamps = curr->timestamps;
				segment_node->secondary_keys.push_back(-segment_node->g_val);
				segment_node->wait_at_goal = true;
				segment_node->is_dummy_path = true;
				// reinitiliaze the curr node with the end location of the trajectory
				// cout << "reach the goal start " << goal_location[stage] << " jump to trajectory end " << agent_segments[stage].trajectory.back() << " moving to end " << instance.start_locations[agent_idx] << endl;
				// printPath(segment_node);

				// try to retrieve it from the hash table
				auto it = allNodes_table.find(segment_node);
				if (it == allNodes_table.end())
				{
					pushNode(segment_node);
					allNodes_table.insert(segment_node);
				}
				else{
					// update existing node's if needed (only in the open_list)
					auto existing_next = *it;
					if (existing_next->getFVal() > segment_node->getFVal() || // if f-val decreased through this new path
						(existing_next->getFVal() == segment_node->getFVal() &&
						LLNode::secondary_compare_node_not_random()(existing_next, segment_node) && existing_next->segment_stage == 1)) // or it remains the same but there's fewer conflicts
					{
						if (!existing_next->in_openlist) // if its in the closed list (reopen)
						{
							existing_next->copy(*segment_node);
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

							existing_next->copy(*segment_node); // update existing node

							if (update_open)
								open_list.increase(existing_next->open_handle); // increase because f-val improved
							if (add_to_focal)
								existing_next->focal_handle = focal_list.push(existing_next);
							if (update_in_focal)
								focal_list.update(existing_next->focal_handle); // should we do update? yes, because number of conflicts may go up or down
						}
					}
					delete segment_node; // not needed anymore -- we already generated it before
				}
			}
		}

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
			auto segment_stage = curr->segment_stage;

			// compute cost to next_id via curr node
			int next_g_val = curr->g_val + 1;
			// int next_h_val = my_heuristic[next_location];
			int next_h_val = max(get_heuristic_ddmapd(stage, next_location, segment_stage), holding_time - next_timestep);
			// if (next_g_val + next_h_val > constraint_table.length_max || next_g_val + next_h_val > f_ub[stage])
			// 	continue;
			// TODO use CAT
			int next_internal_conflicts = curr->num_of_conflicts;

			// generate (maybe temporary) node
			auto next = new MultiLabelAStarNode(next_location, next_g_val, next_h_val,
												curr, next_timestep, stage, next_internal_conflicts, false);

			next->timestamps = timestamps;
			next->secondary_keys.push_back(-next_g_val);
			next->segment_stage = segment_stage;
			next->is_dummy_path = curr->is_dummy_path;

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
			// TODO if  it's the same and the segment state is also the same
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

// Path MultiLabelSpaceTimeAStar::findPathSegmentToPark(ConstraintTable &constraint_table, int start_time, int stage, int lowerbound)
// {
// 	avoid_dummy_path = false;
// 	timeout = false;
// 	findPathSegmentToPark_time = 0;
// 	num_expanded = 0;
// 	clock_t findPathSegmentToPark_time_start = clock();
// 	int loc;
// 	if (stage == 0){
// 		loc = start_location;
// 	}else{
// 		// start from the end of the previous segment
// 		loc = agent_segments[stage - 1].trajectory.back();
// 	}

// 	// generate start and add it to the OPEN & FOCAL list
// 	Path path;
// 	path.begin_time = start_time;
// 	auto start = new MultiLabelAStarNode(loc,						// location
// 										 0,							// g val
// 										 get_heuristic_ddmapd(stage, loc, 0), // h val segment_stage is 0
// 										 nullptr,					// parent
// 										 start_time,				// timestep
// 										 stage,						// stage
// 										 0, false);

// 	start->segment_stage = 0; // segment stage
// 	start->secondary_keys.push_back(-start->g_val);
// 	num_generated++;
// 	start->open_handle = open_list.push(start);
// 	start->focal_handle = focal_list.push(start);
// 	start->in_openlist = true;
// 	allNodes_table.insert(start);
// 	min_f_val = (int)start->getFVal();
// 	int holding_time = constraint_table.length_min;
// 	if (stage == goal_location.size() - 1)
// 	{
// 		holding_time = constraint_table.getHoldingTime(); // the earliest timestep that the agent can hold its goal location. The length_min is considered here.
// 	}
// 	lower_bound = max(holding_time - start_time, max(min_f_val, lowerbound));

// 	while (!open_list.empty())
// 	{
// 		// updateFocalList(); // update FOCAL if min f-val increased
// 		// auto *curr = popNode();
// 		auto *curr = open_list.top();
// 		open_list.pop();
// 		curr->in_openlist = false;
// 		num_expanded++;

// 		if (curr->segment_stage == 1 && curr->location == instance.start_locations[agent_idx]){ // reach the safe parking location
// 			printPath(curr);
// 			updatePath(curr, path);
// 			break;
// 		}

// 		if (curr->timestep >= constraint_table.length_max)
// 			continue;

// 		list<int> next_locations = instance.getNeighbors(curr->location);
// 		AvoidSafeParking(next_locations); // avoid the safe parking location of other agents
// 		next_locations.emplace_back(curr->location);

// 		// if reached the segment start, try to add a action to reach the segment end
// 		if (curr->segment_stage == 0 && curr->location == goal_location[stage] && // reach all previous goals
// 			curr->timestep >= constraint_table.length_min){
// 			// update curr with segment trajectory by creating new node
// 			auto segment_node = curr;
// 			bool traj_constrained = false;
// 			auto segment_trajectory = agent_segments[stage].trajectory;
// 			for (int i = 1; i < segment_trajectory.size(); i++){ // segment_trajectory.size() - 1 to exclude the trajectory end location
// 				auto next = new MultiLabelAStarNode(segment_trajectory[i], segment_node->g_val + 1, segment_node->h_val, segment_node, segment_node->timestep + 1, stage, segment_node->num_of_conflicts, false);
// 				next->task = stage;
// 				if (constraint_table.constrained(segment_trajectory[i], segment_node->timestep + 1) ||
// 				constraint_table.constrained(segment_node->location, segment_trajectory[i], segment_node->timestep + 1)){
// 					traj_constrained = true; // if the trajectory is constrained, then break
// 					break;
// 				}
// 				segment_node = next;
// 				continue;
// 			}
// 			// if the trajectory is constrained, then continue, keep exploring the normal neighbors
// 			if (!traj_constrained){
// 				auto next_g_val = segment_node->g_val; // “segment_trajectory.size() - 1” to exclude the start location
// 				auto next_h_val = get_heuristic_ddmapd(stage, segment_trajectory.back(), 1);
// 				segment_node->g_val = next_g_val;
// 				segment_node->h_val = next_h_val;
// 				segment_node->segment_stage = 1;
// 				segment_node->timestamps = curr->timestamps;
// 				segment_node->secondary_keys.push_back(-segment_node->g_val);
// 				segment_node->wait_at_goal = true;
// 				segment_node->is_dummy_path = true;
// 				// reinitiliaze the curr node with the end location of the trajectory
// 				// cout << "reach the goal start " << goal_location[stage] << " jump to trajectory end " << agent_segments[stage].trajectory.back() << " moving to end " << instance.start_locations[agent_idx] << endl;
// 				// printPath(segment_node);

// 				// try to retrieve it from the hash table
// 				auto it = allNodes_table.find(segment_node);
// 				if (it == allNodes_table.end())
// 				{
// 					pushNode(segment_node);
// 					allNodes_table.insert(segment_node);
// 				}
// 				else{
// 					// update existing node's if needed (only in the open_list)
// 					auto existing_next = *it;
// 					if (existing_next->getFVal() > segment_node->getFVal() || // if f-val decreased through this new path
// 						(existing_next->getFVal() == segment_node->getFVal() &&
// 						LLNode::secondary_compare_node_not_random()(existing_next, segment_node)
// 						&& existing_next->segment_stage == 1)) // or it remains the same but there's fewer conflicts
// 					{
// 						if (!existing_next->in_openlist) // if its in the closed list (reopen)
// 						{
// 							existing_next->copy(*segment_node);
// 							pushNode(existing_next);
// 						}
// 						else
// 						{
// 							bool add_to_focal = false;	  // check if it was above the focal bound before and now below (thus need to be inserted)
// 							bool update_in_focal = false; // check if it was inside the focal and needs to be updated (because f-val changed)
// 							bool update_open = false;
// 							if ((next_g_val + next_h_val) <= lower_bound)
// 							{ // if the new f-val qualify to be in FOCAL
// 								if (existing_next->getFVal() > lower_bound)
// 									add_to_focal = true; // and the previous f-val did not qualify to be in FOCAL then add
// 								else
// 									update_in_focal = true; // and the previous f-val did qualify to be in FOCAL then update
// 							}
// 							if (existing_next->getFVal() > next_g_val + next_h_val)
// 								update_open = true;

// 							existing_next->copy(*segment_node); // update existing node

// 							if (update_open)
// 								open_list.increase(existing_next->open_handle); // increase because f-val improved
// 							if (add_to_focal)
// 								existing_next->focal_handle = focal_list.push(existing_next);
// 							if (update_in_focal)
// 								focal_list.update(existing_next->focal_handle); // should we do update? yes, because number of conflicts may go up or down
// 						}
// 					}
// 				}
// 				delete segment_node; // not needed anymore -- we already generated it before
// 			}
// 		}

// 		// generate child
// 		for (int next_location : next_locations)
// 		{
// 			int next_timestep = curr->timestep + 1;
// 			if (max(constraint_table.cat_size, constraint_table.latest_timestep) + 1 < curr->timestep)
// 			{ // now everything is static, so switch to space A* where we always use the same timestep
// 				if (next_location == curr->location)
// 				{
// 					continue;
// 				}
// 				next_timestep--;
// 			}

// 			if (constraint_table.constrained(next_location, next_timestep) ||
// 				constraint_table.constrained(curr->location, next_location, next_timestep))
// 				continue;

// 			// setting the stage
// 			auto stage = curr->stage;
// 			auto timestamps = curr->timestamps;
// 			auto segment_stage = curr->segment_stage;

// 			// compute cost to next_id via curr node
// 			int next_g_val = curr->g_val + 1;
// 			// int next_h_val = my_heuristic[next_location];
// 			int next_h_val = max(get_heuristic_ddmapd(stage, next_location, segment_stage), holding_time - next_timestep);
// 			// if (next_g_val + next_h_val > constraint_table.length_max || next_g_val + next_h_val > f_ub[stage])
// 			// 	continue;
// 			// TODO use CAT
// 			int next_internal_conflicts = curr->num_of_conflicts;

// 			// generate (maybe temporary) node
// 			auto next = new MultiLabelAStarNode(next_location, next_g_val, next_h_val,
// 												curr, next_timestep, stage, next_internal_conflicts, false);

// 			next->timestamps = timestamps;
// 			next->secondary_keys.push_back(-next_g_val);
// 			next->segment_stage = segment_stage;
// 			next->is_dummy_path = curr->is_dummy_path;

// 			next->dist_to_next = my_heuristic[stage][next_location];

// 			if (next->stage == goal_location.size() - 1 && next_location == goal_location.back() && curr->location == goal_location.back())
// 			{
// 				next->wait_at_goal = true;
// 			}

// 			// try to retrieve it from the hash table
// 			auto it = allNodes_table.find(next);
// 			if (it == allNodes_table.end())
// 			{
// 				pushNode(next);
// 				allNodes_table.insert(next);
// 				continue;
// 			}
// 			// update existing node's if needed (only in the open_list)
// 			// TODO if  it's the same and the segment state is also the same
// 			auto existing_next = *it;
// 			if (existing_next->getFVal() > next->getFVal() || // if f-val decreased through this new path
// 				(existing_next->getFVal() == next->getFVal() &&
// 				 LLNode::secondary_compare_node_not_random()(existing_next, next)
// 				 // existing_next->num_of_conflicts > next->num_of_conflicts
// 				 )) // or it remains the same but there's fewer conflicts
// 			{
// 				if (!existing_next->in_openlist) // if its in the closed list (reopen)
// 				{
// 					existing_next->copy(*next);
// 					pushNode(existing_next);
// 				}
// 				else
// 				{
// 					bool add_to_focal = false;	  // check if it was above the focal bound before and now below (thus need to be inserted)
// 					bool update_in_focal = false; // check if it was inside the focal and needs to be updated (because f-val changed)
// 					bool update_open = false;
// 					if ((next_g_val + next_h_val) <= lower_bound)
// 					{ // if the new f-val qualify to be in FOCAL
// 						if (existing_next->getFVal() > lower_bound)
// 							add_to_focal = true; // and the previous f-val did not qualify to be in FOCAL then add
// 						else
// 							update_in_focal = true; // and the previous f-val did qualify to be in FOCAL then update
// 					}
// 					if (existing_next->getFVal() > next_g_val + next_h_val)
// 						update_open = true;

// 					existing_next->copy(*next); // update existing node

// 					if (update_open)
// 						open_list.increase(existing_next->open_handle); // increase because f-val improved
// 					if (add_to_focal)
// 						existing_next->focal_handle = focal_list.push(existing_next);
// 					if (update_in_focal)
// 						focal_list.update(existing_next->focal_handle); // should we do update? yes, because number of conflicts may go up or down
// 				}
// 			}
// 			delete next; // not needed anymore -- we already generated it before
// 		} // end for loop that generates successors

// 	} // end while loop

// 	releaseNodes();
// 	findPathSegmentToPark_time = (double)(clock() - findPathSegmentToPark_time_start) / CLOCKS_PER_SEC;
// 	return path;
// }

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

Path MultiLabelSpaceTimeAStar::findPath(ConstraintTable &constraint_table, const pair<int, int> start_state, const pair<int, int> goal_state)
{
	/*
	  // generate start and add it to the OPEN & FOCAL list
	  Path path;
	  auto start = new AStarNode(start_state.first,  // location
		  0,  // g val
		  compute_heuristic(start_state.first, goal_state.first),  // h val
		  nullptr,  // parent
		  start_state.second,  // timestep
		  0, false);
	  if (start->timestep + start->h_val > goal_state.second)
		  return path;
	  num_generated++;
	  start->focal_handle = focal_list.push(start);
	  allNodes_table.insert(start);
	  // min_f_val = (int)start->getFVal();

	  while (!focal_list.empty())
	  {
		  auto* curr = focal_list.top();
		  focal_list.pop();
		  curr->in_openlist = false;

		  // check if the popped node is a goal
		  if (curr->location == goal_state.first && // arrive at the goal location
			  curr->timestep == goal_state.second) // at the corresponding timestep
		  {
			  updatePath(curr, path);
			  break;
		  }

		  num_expanded++;
		  list<int> next_locations = instance.getNeighbors(curr->location);
		  next_locations.emplace_back(curr->location);
		  for (int next_location : next_locations)
		  {
			  int next_timestep = curr->timestep + 1;

			  if (constraint_table.constrained(next_location, next_timestep) ||
				  constraint_table.constrained(curr->location, next_location, next_timestep))
				  continue;

			  // compute cost to next_id via curr node
			  int next_g_val = curr->g_val + 1;
			  int next_h_val = compute_heuristic(next_location, goal_state.first);
			  if (next_timestep + next_h_val > goal_state.second)
				  continue;
			  int next_internal_conflicts = curr->num_of_conflicts +
				  constraint_table.getNumOfConflictsForStep(curr->location, next_location, next_timestep);

			  // generate (maybe temporary) node
			  auto next = new AStarNode(next_location, next_g_val, next_h_val,
				  curr, next_timestep, next_internal_conflicts, false);

			  // try to retrieve it from the hash table
			  auto it = allNodes_table.find(next);
			  if (it == allNodes_table.end())
			  {
				  num_generated++;
				  next->focal_handle = focal_list.push(next);
				  next->in_openlist = true;
				  allNodes_table.insert(next);
				  continue;
			  }
			  // update existing node's if needed (only in the open_list)

			  auto existing_next = *it;
			  if (existing_next->num_of_conflicts > next->num_of_conflicts) // if there's fewer conflicts
			  {
				  existing_next->copy(*next);	// update existing node
				  if (!existing_next->in_openlist) // if its in the closed list (reopen)
				  {
					  next->focal_handle = focal_list.push(existing_next);
					  existing_next->in_openlist = true;
				  }
				  else
				  {
					  focal_list.update(existing_next->focal_handle);
				  }
			  }
			  delete next;  // not needed anymore -- we already generated it before
		  }  // end for loop that generates successors
	  }  // end while loop

	  releaseNodes();
	  return path;
	*/
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
		cout << "pushing node location " << node->location << " g_val " << node->g_val << " h_val " << node->h_val << " g2_val " << node->g2_val << " h2_val " << node->h2_val << " f_val " << node->getFVal() << " timestep " << node->timestep << " segment_stage " << node->segment_stage << " stage " << node->stage << endl;
	}
	node->open_handle = open_list.push(node);
	node->in_openlist = true;
	num_generated++;
	if (node->getFVal() <= lower_bound)
		node->focal_handle = focal_list.push(node);
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
	open_list.clear();
	focal_list.clear();
	for (auto node : allNodes_table)
	{
		delete node;
	}
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

int MultiLabelSpaceTimeAStar::get_heuristic_ddmapd(int stage, int loc, int segment_stage) const
{
	if (segment_stage == 0) // to the segment start location
	{
		// h to segment start + segment length + segment end to parking location
		return my_heuristic[stage][loc] + agent_segments[stage].traj_len - 1 + parking_heuristic[agent_segments[stage].trajectory.back()];
	}
	else
	{
		// segment end to parking location
		return parking_heuristic[loc];
	}
}