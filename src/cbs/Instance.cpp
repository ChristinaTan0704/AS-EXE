#include<boost/tokenizer.hpp>
#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include"Instance.h"
#include <sys/stat.h>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

int RANDOM_WALK_STEPS = 100000;

Instance::Instance(const string& map_fname, const string& agent_fname, const string& assignment_folder,
				   int num_of_agents, int num_of_rows, int num_of_cols, int num_of_obstacles, int warehouse_width) :
		map_fname(map_fname), agent_fname(agent_fname), num_of_agents(num_of_agents), assignment_folder(assignment_folder)
{
	struct stat info;
	if (stat(assignment_folder.c_str(), &info) == 0 && (info.st_mode & S_IFDIR)){
		ddmapd_instance = true;
	}

	bool succ = loadMap();
	if (!succ)
	{
		if (num_of_rows > 0 && num_of_cols > 0 && num_of_obstacles >= 0 && 
			num_of_obstacles < num_of_rows * num_of_cols) // generate random grid
		{
			generateConnectedRandomGrid(num_of_rows, num_of_cols, num_of_obstacles);
			saveMap();
		}
		else
		{
			cerr << "Map file " << map_fname << " not found." << endl;
			exit(-1);
		}
	}
	


	if (stat(assignment_folder.c_str(), &info) == 0 && (info.st_mode & S_IFDIR))
	{
		// It's a directory, read all files in the directory
		succ = loadAgentsJson();
	}
	else
	{
		// Neither a directory nor a file
		cerr << "Agent file " << agent_fname << " not found." << endl;
		exit(-1);
	}

	if (!succ)
	{
		if (num_of_agents > 0)
		{
			generateRandomAgents(warehouse_width);
			saveAgents();
		}
		else
		{
			cerr << "Agent file " << agent_fname << " not found." << endl;
			exit(-1);
		}
	}

}


bool Instance::loadAgentsJson(){
	// Ensure assignment_folder ends with a slash
	std::string folder = assignment_folder;
	if (folder.back() != '/')
	{
		folder += '/';
	}
	// read agent sequence and segment info
	std::string agent_locs_path = folder + "agent_locs.json";
	std::string vertex_info_path = folder + "vertex_info.json";


	std::ifstream agent_locs_file(agent_locs_path);
	if (!agent_locs_file.is_open())
	{
		std::cerr << "Agent sequence file " << agent_locs_path << " not found." << std::endl;
		return false;
	}

	std::ifstream vertex_info_file(vertex_info_path);
	if (!vertex_info_file.is_open())
	{
		std::cerr << "Segment info file " << vertex_info_path << " not found." << std::endl;
		return false;
	}
	// json agent_seq_js
	json agent_locs_json = json::parse(agent_locs_file);
	json vertex_info_json = json::parse(vertex_info_file);

	num_of_agents = agent_locs_json.size();
	// Initialize segments
	vertices.resize(vertex_info_json.size());
	agent_parkLoc.resize(num_of_agents);

	// get agent start_loc
    for (json::iterator it = agent_locs_json.begin(); it != agent_locs_json.end(); ++it) {
        int agent = std::stoi(it.key());
        vector<int> loc = it.value();
		int linear_loc = linearizeCoordinate(loc[0], loc[1]);
		agent_parkLoc[agent] = linear_loc;
    }


	// get segment info
	for (auto [key, value] : vertex_info_json.items())
	{
		int id = std::stoi(key);
		vector<int> locs = value["loc"];
		int loc = linearizeCoordinate(locs[0], locs[1]);
		int traj_len = value["traj_len"];
		int taskID = value["taskID"];
		int traj_step = value["traj_step"];
		vector<int> type1_idList = value["type1_id"];
		vector<int> type2_idList = value["type2_id"];
		vertices[id] = Vertex(id, loc, traj_len, taskID, traj_step, type1_idList, type2_idList);
		taskStep2id[std::make_tuple(taskID, traj_step)] = id;
	}

	// assert len(myMap) == num_of_agents
//	assert(taskStep2id.size() == vertices, "Number of agents does not match the number of agents in the agent sequence file.");

	return true;
}



int Instance::randomWalk(int curr, int steps) const
{
	for (int walk = 0; walk < steps; walk++)
	{
		list<int> l = getNeighbors(curr);
		vector<int> next_locations(l.cbegin(), l.cend());
		auto rng = std::default_random_engine{};
		std::shuffle(std::begin(next_locations), std::end(next_locations), rng);
		for (int next : next_locations)
		{
			if (validMove(curr, next))
			{
				curr = next;
				break;
			}
		}
	}
	return curr;
}

void Instance::generateRandomAgents(int warehouse_width)
{
}

bool Instance::validMove(int curr, int next) const
{
	if (next < 0 || next >= map_size)
		return false;
	if (my_map[next])
		return false;
	return getManhattanDistance(curr, next) < 2;
}

bool Instance::addObstacle(int obstacle)
{
	if (my_map[obstacle])
		return false;
	my_map[obstacle] = true;
	int obstacle_x = getRowCoordinate(obstacle);
	int obstacle_y = getColCoordinate(obstacle);
	int x[4] = { obstacle_x, obstacle_x + 1, obstacle_x, obstacle_x - 1 };
	int y[4] = { obstacle_y - 1, obstacle_y, obstacle_y + 1, obstacle_y };
	int start = 0;
	int goal = 1;
	while (start < 3 && goal < 4)
	{
		if (x[start] < 0 || x[start] >= num_of_rows || y[start] < 0 || y[start] >= num_of_cols 
			|| my_map[linearizeCoordinate(x[start], y[start])])
			start++;
		else if (goal <= start)
			goal = start + 1;
		else if (x[goal] < 0 || x[goal] >= num_of_rows || y[goal] < 0 || y[goal] >= num_of_cols
				 || my_map[linearizeCoordinate(x[goal], y[goal])])
			goal++;
		else if (isConnected(linearizeCoordinate(x[start], y[start]),
							 linearizeCoordinate(x[goal], y[goal]))) // cannot find a path from start to goal
		{
			start = goal;
			goal++;
		}
		else
		{
			my_map[obstacle] = false;
			return false;
		}
	}
	return true;
}

bool Instance::isConnected(int start, int goal)
{
	std::queue<int> open;
	vector<bool> closed(map_size, false);
	open.push(start);
	closed[start] = true;
	while (!open.empty())
	{
		int curr = open.front(); open.pop();
		if (curr == goal)
			return true;
		for (int next : getNeighbors(curr))
		{
			if (closed[next])
				continue;
			open.push(next);
			closed[next] = true;
		}
	}
	return false;
}

void Instance::generateConnectedRandomGrid(int rows, int cols, int obstacles)
{
	cout << "Generate a " << rows << " x " << cols << " grid with " << obstacles << " obstacles. " << endl;
	int i, j;
	num_of_rows = rows + 2;
	num_of_cols = cols + 2;
	map_size = num_of_rows * num_of_cols;
	my_map.resize(map_size, false);
	// Possible moves [WAIT, NORTH, EAST, SOUTH, WEST]
	/*moves_offset[Instance::valid_moves_t::WAIT_MOVE] = 0;
	moves_offset[Instance::valid_moves_t::NORTH] = -num_of_cols;
	moves_offset[Instance::valid_moves_t::EAST] = 1;
	moves_offset[Instance::valid_moves_t::SOUTH] = num_of_cols;
	moves_offset[Instance::valid_moves_t::WEST] = -1;*/

	// add padding
	i = 0;
	for (j = 0; j < num_of_cols; j++)
		my_map[linearizeCoordinate(i, j)] = true;
	i = num_of_rows - 1;
	for (j = 0; j < num_of_cols; j++)
		my_map[linearizeCoordinate(i, j)] = true;
	j = 0;
	for (i = 0; i < num_of_rows; i++)
		my_map[linearizeCoordinate(i, j)] = true;
	j = num_of_cols - 1;
	for (i = 0; i < num_of_rows; i++)
		my_map[linearizeCoordinate(i, j)] = true;

	// add obstacles uniformly at random
	i = 0;
	while (i < obstacles)
	{
		int loc = rand() % map_size;
		if (addObstacle(loc))
		{
			printMap();
			i++;
		}
	}
}

bool Instance::loadMap()
{
	using namespace boost;
	using namespace std;
	ifstream myfile(map_fname.c_str());
	if (!myfile.is_open())
		return false;
	string line;
	tokenizer<char_separator<char>>::iterator beg;
	getline(myfile, line);
	if (line[0] == 't') // Nathan's benchmark
	{
		char_separator<char> sep(" ");
		getline(myfile, line);
		tokenizer<char_separator<char>> tok(line, sep);
		beg = tok.begin();
		beg++;
		num_of_rows = atoi((*beg).c_str()); // read number of rows
		getline(myfile, line);
		tokenizer<char_separator<char>> tok2(line, sep);
		beg = tok2.begin();
		beg++;
		num_of_cols = atoi((*beg).c_str()); // read number of cols
		getline(myfile, line); // skip "map"
	}
	else // my benchmark
	{
		char_separator<char> sep(",");
		tokenizer<char_separator<char>> tok(line, sep);
		beg = tok.begin();
		num_of_rows = atoi((*beg).c_str()); // read number of rows
		beg++;
		num_of_cols = atoi((*beg).c_str()); // read number of cols
	}
	map_size = num_of_cols * num_of_rows;
	my_map.resize(map_size, false);
	
	if (!ddmapd_instance){ // shelf map don't have obstacles
		// read map (and start/goal locations)
		for (int i = 0; i < num_of_rows; i++)
		{
			getline(myfile, line);
			for (int j = 0; j < num_of_cols; j++)
			{
				my_map[linearizeCoordinate(i, j)] = (line[j] != '.');
			}
		}
	}

	myfile.close();

	// initialize moves_offset array
	/*moves_offset[Instance::valid_moves_t::WAIT_MOVE] = 0;
	moves_offset[Instance::valid_moves_t::NORTH] = -num_of_cols;
	moves_offset[Instance::valid_moves_t::EAST] = 1;
	moves_offset[Instance::valid_moves_t::SOUTH] = num_of_cols;
	moves_offset[Instance::valid_moves_t::WEST] = -1;*/
	return true;
}


void Instance::printMap() const
{
	for (int i = 0; i < num_of_rows; i++)
	{
		for (int j = 0; j < num_of_cols; j++)
		{
			if (this->my_map[linearizeCoordinate(i, j)])
				cout << '@';
			else
				cout << '.';
		}
		cout << endl;
	}
}


void Instance::saveMap() const
{
	ofstream myfile;
	myfile.open(map_fname);
	if (!myfile.is_open())
	{
		cout << "Fail to save the map to " << map_fname << endl;
		return;
	}
	myfile << num_of_rows << "," << num_of_cols << endl;
	for (int i = 0; i < num_of_rows; i++)
	{
		for (int j = 0; j < num_of_cols; j++)
		{
			if (my_map[linearizeCoordinate(i, j)])
				myfile << "@";
			else
				myfile << ".";
		}
		myfile << endl;
	}
	myfile.close();
}




void Instance::printAgents() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		cout << "Agent" << i << " : S=(" << getRowCoordinate(start_locations[i]) << "," << getColCoordinate(start_locations[i])
			 << ") ; 0: (" << getRowCoordinate(goal_locations[i][0]) << "," << getColCoordinate(goal_locations[i][0]) << ")" ;
    for (int j = 1; j < goal_locations[i].size(); j++){
      cout << " =>" << j << ": (" << getRowCoordinate(goal_locations[i][j]) << "," << getColCoordinate(goal_locations[i][j]) << ")" ;
    }
    cout << endl;
	}
}


void Instance::saveAgents() const
{
	ofstream myfile;
	myfile.open(agent_fname);
	if (!myfile.is_open())
	{
		cout << "Fail to save the agents to " << agent_fname << endl;
		return;
	}
	myfile << num_of_agents << endl;
	for (int i = 0; i < num_of_agents; i++){
		myfile << getRowCoordinate(start_locations[i]) << "," << getColCoordinate(start_locations[i]) << "," << goal_locations[i].size() << endl;

    for (auto g:goal_locations[i]){
      cout << getRowCoordinate(g) << "," << getColCoordinate(g) << "," ;
    }
  }
  cout << endl;
	myfile.close();
}


list<int> Instance::getNeighbors(int curr) const
{
	list<int> neighbors;
	int candidates[4] = { curr + 1, curr - 1, curr + num_of_cols, curr - num_of_cols };
	for (int next : candidates)
	{
		if (validMove(curr, next))
			neighbors.emplace_back(next);
	}
	return neighbors;
}

// return the list of segments which id in id_list
vector<Segment> Instance::getSegments(vector<int> id_list) const
{
	vector<Segment> rst;
	for (int id : id_list)
	{
		rst.emplace_back(segments[id]);
	}
	return rst;
}


