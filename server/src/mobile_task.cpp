#include "mobile_task.h"

MobileTaskManager::MobileTaskManager(int num_agents, int num_picker, std::string& map_fname): TaskManager(num_agents, map_fname),
  num_picker_(num_picker){
  for (auto& tmp_station: user_map.all_stations) {
    active_stations.insert({tmp_station->idx, tmp_station});
  }
  for (int i = 0; i < num_agents; ++i) {
    agent_task_status.emplace_back(i);
  }
  all_picker_tasks.resize(num_picker_);
}


double computeDistance(const Location& agent_loc, const Location& task_loc) {
  double dx = agent_loc.first - task_loc.first;
  double dy = agent_loc.second - task_loc.second;
  return std::sqrt(dx * dx + dy * dy);
}

void MobileTaskManager::cleanTasks() {
  for (int i = 0; i < num_picker_; ++i) {
    // The task is not allocated yet
    while (not all_picker_tasks[i].empty() and
      (all_picker_tasks[i].front()->status == DELIVER or all_picker_tasks[i].front()->status == DONE)) {
      all_picker_tasks[i].pop_front();
    }
  }
}

void MobileTaskManager::getTask(const std::vector<std::pair<double, double>>& robots_location,
  std::vector<std::deque<std::shared_ptr<MobileRobotTask>>>& new_tasks) {

  cleanTasks();
  std::deque<std::shared_ptr<MobileRobotTask>> front_tasks;
  for (int i = 0; i < num_picker_; ++i) {
    if (not all_picker_tasks[i].empty()) {
      // The task is not allocated yet
      if (all_picker_tasks[i].front()->agent_id == -1) {
        front_tasks.push_back(all_picker_tasks[i].front());
      }
    }
  }

  std::vector<int> free_agents;
  for (int agent_idx = 0; agent_idx < num_robots_; agent_idx++) {
    if (agent_task_status[agent_idx].assigned_tasks.empty()) {
      free_agents.push_back(agent_idx);
    }
  }
  if (not free_agents.empty() and not front_tasks.empty()) {
    std::cout << "Find free agents" << std::endl;
    vector<vector<double> > costMatrix;
    for (int robot_id: free_agents) {
      std::vector<double> cost_tmp;
      for (int j = 0; j < front_tasks.size(); ++j) {
        double tmp_dis = computeDistance(robots_location[robot_id], front_tasks[j]->goal_position);
        cost_tmp.push_back(tmp_dis);
      }
      costMatrix.push_back(cost_tmp);
    }
    std::cout << "Get cost matrix" << std::endl;

    vector<int> assignment;
    HungarianAlgorithm HungAlgo;

    double cost = HungAlgo.Solve(costMatrix, assignment);


    std::cout << "Finish solve hungarian, the cost is: " << cost << std::endl;
    for (int i = 0; i < assignment.size(); ++i) {
      if (assignment[i] >= 0) {
        cout << "Agent " << free_agents[i] << " assigned to Task " << assignment[i] << endl;
        setTask(free_agents[i], front_tasks[assignment[i]], PICK);
      }
    }
  }
  new_tasks.clear();
  new_tasks.resize(num_robots_);
  for (int i = 0; i < num_robots_; ++i) {
    new_tasks[i] = agent_task_status[i].assigned_tasks;
  }

}

void MobileTaskManager::finishTask(int agent_id, std::shared_ptr<MobileRobotTask>& task) {
  if (task->status == PICK) {
    setTask(agent_id, task, DELIVER);
  } else if (task->status == DELIVER) {
    setTask(agent_id, task, DONE);
  } else {
    std::cerr << "MobileTaskManager::finishTask: unknown action" << std::endl;
    exit(-1);
  }
}


void MobileTaskManager::setTask(int agent_id, std::shared_ptr<MobileRobotTask>& task, MobileAction status) {
  if (task->status == NONE) {
    if (status == PICK) {
#ifdef DEBUG
      std::cout << "adding new task with id: " << task->id << std::endl;
#endif
      assert(agent_task_status[agent_id].assigned_tasks.empty());
      agent_task_status[agent_id].assigned_tasks.emplace_back(task);
      task->agent_id = agent_id;
    } else {
      // std::cerr << "Its a NONE task, but got " << status << std::endl;
    }
  } else if (task->status == PICK) {
    if (status == DELIVER) {
      ;
    } else {
      // std::cerr << "Its a PICK task, but got " << status << std::endl;
      return;
    }
  } else if (task->status == DELIVER) {
    if (status == DONE) {
      if (not agent_task_status[agent_id].assigned_tasks.empty()) {
        if (agent_task_status[agent_id].assigned_tasks.front()->id == task->id) {
          agent_task_status[agent_id].assigned_tasks.pop_front();
          total_finished_tasks_++;
          std::cout << "Total number of tasks finished: " << total_finished_tasks_ << " finished" << std::endl;
        }
      }
    } else {
      // std::cerr << "Its a DELIVER task, but got " << status << std::endl;
      // exit(-1);
      return;
    }
  } else if (task->status == DONE) {
    // std::cerr << "Task already finished!" << std::endl;
  }
  task->status = status;
}

std::pair<int, int> MobileTaskManager::findNearbyFreeCell(int x, int y) {
  if ((x / 4) % 2 == 0 ) {
    return std::make_pair(x, y-1);
  } else {
    return std::make_pair(x, y+1);
  }
}

std::pair<int, int> MobileTaskManager::findPalletizer(int picker_id) {
  int station_idx = picker_id/2;

  static std::mt19937 rng(std::random_device{}());
  std::uniform_int_distribution<int> dist(0, 1);
  station_idx = station_idx*2 + dist(rng);
  // std::srand(std::time(0)); // Seed the random number generator with current time
  // station_idx = std::rand() % active_stations.size();
  std::cout << "active stations size: " << active_stations.size() << ", picked station idx: " << station_idx << std::endl;
  auto it = active_stations.begin();
  std::advance(it, station_idx);
  auto tmp_station = it->second;
  int x = tmp_station->x;
  int y = tmp_station->y;
  return std::make_pair(x, y);
}

int MobileTaskManager::insertPickerTask(int picker_id, int goal_x, int goal_y) {
  std::pair<int, int> free_loc = findNearbyFreeCell(goal_x, goal_y);
  std::pair<int, int> picker_loc = findPalletizer(picker_id);
  std::shared_ptr<MobileRobotTask> new_pickup_task = std::make_shared<MobileRobotTask>(curr_task_idx++, -1,
    free_loc, -1, MobileAction::NONE, picker_id, picker_loc);
  // picker_tasks.push_back(new_pickup_task);
  all_picker_tasks[picker_id].push_back(new_pickup_task);
  return new_pickup_task->id;
}