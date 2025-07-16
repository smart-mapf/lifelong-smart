#include "instance.h"

int RANDOM_WALK_STEPS = 100000;

Instance::Instance(shared_ptr<Graph> graph, const string& in_agent_fname,
                   int in_agents_num, const string& in_agent_indices,
                   bool use_partial_expansion, int used_sps_solver)
    : graph(graph),
      agent_fname(in_agent_fname),
      agent_indices(in_agent_indices),
      num_of_agents(in_agents_num),
      use_pe(use_partial_expansion),
      use_sps_type(used_sps_solver) {
    // bool succ = loadMap();
    // if (!succ)
    // {
    // 	if (num_of_rows > 0 && num_of_cols > 0 && num_of_obstacles >= 0 &&
    // 		num_of_obstacles < num_of_rows * num_of_cols) // generate random
    // grid
    // 	{
    // 		generateConnectedRandomGrid(num_of_rows, num_of_cols,
    // num_of_obstacles); 		saveMap();
    // 	}
    // 	else
    // 	{
    // 		cerr << "Map file " << map_fname << " not found." << endl;
    // 		exit(-1);
    // 	}
    // }
    GetRawReservationTable();
    bool succ = loadAgents();
    if (!succ) {
        // if (num_of_agents > 0)
        // {
        // 	generateRandomAgents(warehouse_width);
        // 	saveAgents();
        // }
        // else
        // {
        cerr << "Agent file " << agent_fname << " not found." << endl;
        exit(-1);
        // }
    }
    for (int i = 0; i < num_of_agents; i++) {
        agents[i].id = i;
    }
}

void Instance::GetRawReservationTable() {
    raw_rv_tbl.resize(this->graph->map_size);
    TimeInterval blocked_interval;
    blocked_interval.t_min = 0;
    blocked_interval.t_max = INF;
    blocked_interval.agent_id = -1;
    for (int i = 0; i < this->graph->map_size; i++) {
        if (this->graph->my_map[i]) {
            raw_rv_tbl[i].push_back(blocked_interval);
        }
    }
}

void Instance::GetAgents(std::vector<Agent>& agents_list) {
    agents_list = agents;
}

// void Instance::generateRandomAgents(int warehouse_width)
// {
// 	cout << "Generate " << num_of_agents << " random start and goal
// locations " << endl; 	vector<bool> starts(this->graph->map_size,
// false); 	vector<bool> goals(this->graph->map_size, false);
// 	agents.resize(num_of_agents);

// 	if (warehouse_width == 0)//Generate agents randomly
// 	{
// 		// Choose random start locations
// 		int k = 0;
// 		while (k < num_of_agents)
// 		{
// 			int x = rand() % num_of_rows, y = rand() % num_of_cols;
// 			int start = this->graph->linearizeCoordinate(x, y);
// 			if (my_map[start] || starts[start])
// 				continue;

// 			// update start
// 			starts[start] = true;

// 			// find goal
// 			int goal = randomWalk(start, RANDOM_WALK_STEPS);
// 			while (goals[goal])
// 				goal = randomWalk(goal, 1);

// 			//update goal
// 			goals[goal] = true;
// 			agents[k] = Agent(start, goal);
// 			k++;
// 		}
// 	}
// 	else //Generate agents for warehouse scenario
// 	{
// 		// Choose random start locations
// 		int k = 0;
// 		while (k < num_of_agents)
// 		{
// 			int x = rand() % num_of_rows, y = rand() %
// warehouse_width; 			if (k % 2 == 0)
// y = num_of_cols - y - 1; 			int start =
// this->graph->linearizeCoordinate(x, y); 			if
// (starts[start]) 				continue;
// 			// update start

// 			agents[k] = Agent(start, 0);
// 			starts[start] = true;

// 			k++;
// 		}
// 		// Choose random goal locations
// 		k = 0;
// 		while (k < num_of_agents)
// 		{
// 			int x = rand() % num_of_rows, y = rand() %
// warehouse_width; 			if (k % 2 == 1)
// y = num_of_cols - y - 1; 			int goal =
// this->graph->linearizeCoordinate(x, y); 			if (goals[goal])
// continue;
// 			// update goal
// 			agents[k].goal_location = goal;
// 			goals[goal] = true;
// 			k++;
// 		}
// 	}
// }

// bool Instance::addObstacle(int obstacle)
// {
// 	if (my_map[obstacle])
// 		return false;
// 	my_map[obstacle] = true;
// 	int obstacle_x = this->graph->getRowCoordinate(obstacle);
// 	int obstacle_y = this->graph->getColCoordinate(obstacle);
// 	int x[4] = { obstacle_x, obstacle_x + 1, obstacle_x, obstacle_x - 1 };
// 	int y[4] = { obstacle_y - 1, obstacle_y, obstacle_y + 1, obstacle_y };
// 	int start = 0;
// 	int goal = 1;
// 	while (start < 3 && goal < 4)
// 	{
// 		if (x[start] < 0 || x[start] >= num_of_rows || y[start] < 0 ||
// y[start] >= num_of_cols
// 			|| my_map[this->graph->linearizeCoordinate(x[start],
// y[start])]) 			start++; 		else if (goal <= start)
// goal = start + 1; 		else if (x[goal] < 0 || x[goal] >= num_of_rows
// || y[goal] < 0 || y[goal] >= num_of_cols
// 				 ||
// my_map[this->graph->linearizeCoordinate(x[goal], y[goal])])
// goal++; 		else if
// (graph->isConnected(this->graph->linearizeCoordinate(x[start], y[start]),
// 							 this->graph->linearizeCoordinate(x[goal],
// y[goal]))) // cannot find a path from start to goal
// 		{
// 			start = goal;
// 			goal++;
// 		}
// 		else
// 		{
// 			my_map[obstacle] = false;
// 			return false;
// 		}
// 	}
// 	return true;
// }

// void Instance::generateConnectedRandomGrid(int rows, int cols, int obstacles)
// {
// 	cout << "Generate a " << rows << " x " << cols << " grid with " <<
// obstacles << " obstacles. " << endl; 	int i, j; 	num_of_rows =
// rows + 2; 	num_of_cols = cols + 2; 	this->graph->map_size = num_of_rows *
// num_of_cols; 	my_map.resize(this->graph->map_size, false);
// 	// add padding
// 	i = 0;
// 	for (j = 0; j < num_of_cols; j++)
// 		my_map[this->graph->linearizeCoordinate(i, j)] = true;
// 	i = num_of_rows - 1;
// 	for (j = 0; j < num_of_cols; j++)
// 		my_map[this->graph->linearizeCoordinate(i, j)] = true;
// 	j = 0;
// 	for (i = 0; i < num_of_rows; i++)
// 		my_map[this->graph->linearizeCoordinate(i, j)] = true;
// 	j = num_of_cols - 1;
// 	for (i = 0; i < num_of_rows; i++)
// 		my_map[this->graph->linearizeCoordinate(i, j)] = true;

// 	// add obstacles uniformly at random
// 	i = 0;
// 	while (i < obstacles)
// 	{
// 		int loc = rand() % this->graph->map_size;
// 		if (addObstacle(loc))
// 		{
// 			printMap();
// 			i++;
// 		}
// 	}
// }

// bool Instance::loadMap()
// {
// 	using namespace boost;
// 	using namespace std;
// 	ifstream myfile(map_fname.c_str());
// 	if (!myfile.is_open())
// 		return false;
// 	string line;
// 	tokenizer<char_separator<char>>::iterator beg;
// 	getline(myfile, line);
// 	if (line[0] == 't') // Nathan's benchmark
// 	{
// 		char_separator<char> sep(" ");
// 		getline(myfile, line);
// 		tokenizer<char_separator<char>> tok(line, sep);
// 		beg = tok.begin();
// 		beg++;
// 		num_of_rows = atoi((*beg).c_str()); // read number of rows
// 		getline(myfile, line);
// 		tokenizer<char_separator<char>> tok2(line, sep);
// 		beg = tok2.begin();
// 		beg++;
// 		num_of_cols = atoi((*beg).c_str()); // read number of cols
// 		getline(myfile, line); // skip "map"
// 	}
// 	else // my benchmark
// 	{
// 		char_separator<char> sep(",");
// 		tokenizer<char_separator<char>> tok(line, sep);
// 		beg = tok.begin();
// 		num_of_rows = atoi((*beg).c_str()); // read number of rows
// 		beg++;
// 		num_of_cols = atoi((*beg).c_str()); // read number of cols
// 	}
// 	this->graph->map_size = num_of_cols * num_of_rows;
// 	my_map.resize(this->graph->map_size, false);
// 	// read map (and start/goal locations)
// 	for (int i = 0; i < num_of_rows; i++)
// 	{
// 		getline(myfile, line);
// 		for (int j = 0; j < num_of_cols; j++)
// 		{
// 			my_map[this->graph->linearizeCoordinate(i, j)] =
// (line[j] != '.');
// 		}
// 	}
// 	myfile.close();
// 	return true;
// }

bool Instance::loadAgents() {
    using namespace std;
    using namespace boost;

    string line;
    ifstream myfile(agent_fname.c_str());
    if (!myfile.is_open())
        return false;

    getline(myfile, line);
    if (line[0] == 'v')  // Nathan's benchmark
    {
        if (num_of_agents == 0) {
            cerr << "The number of agents should be larger than 0" << endl;
            exit(-1);
        }
        // start_locations.resize(num_of_agents);
        // goal_locations.resize(num_of_agents);
        agents.resize(num_of_agents);

        vector<int> ids(num_of_agents);
        if (agent_indices != "") {
            char_separator<char> sep(",");
            tokenizer<char_separator<char>> chars(agent_indices, sep);
            int i = 0;
            for (auto c : chars) {
                ids[i] = atoi(c.c_str());
                if (i > 0 && ids[i] <= ids[i - 1]) {
                    printf("the indices of the agents should be strictly "
                           "increasing!\n");
                    exit(-1);  // the indices of the agents should be strictly
                               // increasing!
                }

                i++;
            }
        } else {
            for (int i = 0; i < num_of_agents; i++)
                ids[i] = i;
        }
        char_separator<char> sep("\t");
        int count = 0;
        int i = 0;
        while (i < num_of_agents) {
            getline(myfile, line);
            if (count == ids[i]) {
                tokenizer<char_separator<char>> tok(line, sep);
                tokenizer<char_separator<char>>::iterator beg = tok.begin();
                beg++;  // skip the first number
                beg++;  // skip the map name
                beg++;  // skip the columns
                beg++;  // skip the rows
                        // read start [row,col] for agent i
                int col = atoi((*beg).c_str());
                beg++;
                int row = atoi((*beg).c_str());
                int start = this->graph->linearizeCoordinate(row, col);
                // start_locations[i] = this->graph->linearizeCoordinate(row,
                // col); read goal [row,col] for agent i
                beg++;
                col = atoi((*beg).c_str());
                beg++;
                row = atoi((*beg).c_str());
                int goal = this->graph->linearizeCoordinate(row, col);
                // goal_locations[i] = this->graph->linearizeCoordinate(row,
                // col);
                agents[i] = Agent(start, goal);
                i++;
            }
            count++;
        }
    } else  // My benchmark
    {
        char_separator<char> sep(",");
        tokenizer<char_separator<char>> tok(line, sep);
        tokenizer<char_separator<char>>::iterator beg = tok.begin();
        int tmp = atoi((*beg).c_str());
        // start_locations.resize(num_of_agents);
        // goal_locations.resize(num_of_agents);
        //        printf("Total number of agents: %d\n", num_of_agents);
        agents.resize(num_of_agents);
        for (int i = 0; i < num_of_agents; i++) {
            getline(myfile, line);
            tokenizer<char_separator<char>> col_tok(line, sep);
            tokenizer<char_separator<char>>::iterator c_beg = col_tok.begin();
            // pair<int, int> curr_pair;
            // read start [row,col] for agent i
            int row = atoi((*c_beg).c_str());
            c_beg++;
            int col = atoi((*c_beg).c_str());
            int start = this->graph->linearizeCoordinate(row, col);
            // start_locations[i] = this->graph->linearizeCoordinate(row, col);
            // read goal [row,col] for agent i
            c_beg++;
            row = atoi((*c_beg).c_str());
            c_beg++;
            col = atoi((*c_beg).c_str());
            int goal = this->graph->linearizeCoordinate(row, col);
            agents[i] = Agent(start, goal);
            agents[i].id = i;
        }
    }
    myfile.close();
    return true;
}

void Instance::printAgents() const {
    for (int i = 0; i < num_of_agents; i++) {
        Agent curr_agent = agents[i];
        cout << "Agent " << i << " with id: " << curr_agent.id << " : S=("
             << this->graph->getRowCoordinate(curr_agent.start_location) << ","
             << this->graph->getColCoordinate(curr_agent.start_location)
             << ") ; G=("
             << this->graph->getRowCoordinate(curr_agent.goal_location) << ","
             << this->graph->getColCoordinate(curr_agent.goal_location) << ")"
             << endl;
    }
}

void Instance::saveAgents() const {
    ofstream myfile;
    myfile.open(agent_fname);
    if (!myfile.is_open()) {
        cout << "Fail to save the agents to " << agent_fname << endl;
        return;
    }
    myfile << num_of_agents << endl;
    for (int i = 0; i < num_of_agents; i++) {
        Agent curr_agent = agents[i];
        myfile << this->graph->getRowCoordinate(curr_agent.start_location)
               << ","
               << this->graph->getColCoordinate(curr_agent.start_location)
               << "," << this->graph->getRowCoordinate(curr_agent.goal_location)
               << "," << this->graph->getColCoordinate(curr_agent.goal_location)
               << "," << endl;
    }
    myfile.close();
}
