#include "mobile_task.h"

MobileTaskManager::MobileTaskManager(int num_agents, std::string& map_fname): TaskManager(num_agents, map_fname) {
  for (auto& tmp_station: user_map.all_stations) {
    active_stations.insert({tmp_station->idx, tmp_station});
  }
  for (int i = 0; i < num_agents; ++i) {
    agent_task_status.emplace_back(i);
  }
  all_picker_tasks.resize(num_agents);
}


void MobileTaskManager::getTask(std::vector<std::deque<std::shared_ptr<MobileRobotTask>>>& new_tasks) {
  int agent_idx_offset = getFirstAgentInQueue();
  for (int i = 0; i < num_robots_; i++) {
    int agent_idx = (agent_idx_offset + i)%num_robots_;
    assert(agent_task_status[agent_idx].curr_loads >= 0);
#ifdef DEBUG
    std::cout << "Total task for Agent " << agent_idx << ", is: " << agent_task_status[agent_idx].assigned_tasks.size() << std::endl;
#endif
    while (agent_task_status[agent_idx].assigned_tasks.size() < MAX_NUM_GOALS) {
      auto tmp_new_task = genTask(agent_idx);
      if (tmp_new_task == nullptr) {
        // Do nothing
        // std::cerr << "Failed to generate new random task" << std::endl;
        break;
      }
      setTask(agent_idx, tmp_new_task, true);
      std::cout << "Create new task for agent " << agent_idx << ", with goal at: "
      << tmp_new_task->goal_position.first << ", " << tmp_new_task->goal_position.second << std::endl;
    }
  }

  // TODO@jingtian: consider more efficient implementation
  new_tasks.clear();
  new_tasks.resize(num_robots_);
  for (int agent_idx = 0; agent_idx < num_robots_; ++agent_idx) {
    new_tasks[agent_idx] = agent_task_status[agent_idx].assigned_tasks;
  }
}

std::shared_ptr<MobileRobotTask> MobileTaskManager::genTask(int agent_id) {
  if (agent_task_status[agent_id].curr_loads < MAX_LOADS) {
    std::cout << "Pick a new random picker" << std::endl;
    auto new_task = pickPicker(agent_id);
    return new_task;
  } else {
    std::cout << "Pick a new random Station" << std::endl;
    auto new_task = pickStation(agent_id);
    return new_task;
  }
}


void padMatrix(vector<vector<double>>& cost, int& n) {
  int rows = cost.size();
  int cols = cost[0].size();
  n = max(rows, cols);
  for (auto& row : cost) {
    row.resize(n, INF);
  }
  while (cost.size() < n) {
    cost.push_back(vector<double>(n, INF));
  }
}

vector<int> hungarian(const vector<vector<double>>& cost) {
  int n = cost.size();
  vector<int> u(n + 1), v(n + 1), p(n + 1), way(n + 1);
  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    vector<int> minv(n + 1, INF);
    vector<bool> used(n + 1, false);
    int j0 = 0;
    do {
      used[j0] = true;
      int i0 = p[j0], delta = INF, j1 = 0;
      for (int j = 1; j <= n; ++j) {
        if (!used[j]) {
          int cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
          if (cur < minv[j]) {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
      }
      for (int j = 0; j <= n; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }
  vector<int> result(n);
  for (int j = 1; j <= n; ++j) {
    result[p[j] - 1] = j - 1;
  }
  return result;
}

double computeDistance(int agent_id, std::shared_ptr<MobileRobotTask>& task) {
  return 0.0;
}

vector<int> MobileTaskManager::pickPicker(int agent_id) {
  std::deque<std::shared_ptr<MobileRobotTask>> front_tasks;
  for (int i = 0; i < num_robots_; ++i) {
    if (not all_picker_tasks[i].empty()) {
      front_tasks.push_back(all_picker_tasks[i].front());
    }
  }
  vector<vector<double>> cost;
  for (int i = 0; i < num_robots_; ++i) {
    std::vector<double> cost_tmp;
    for (int j = 0; j < front_tasks.size(); ++j) {
      double tmp_dis = computeDistance(i, front_tasks[j]);
      cost_tmp.push_back(tmp_dis);
    }
    cost.push_back(cost_tmp);
  }

  int n;
  padMatrix(cost, n);
  vector<int> assignment = hungarian(cost);
  for (int i = 0; i < assignment.size(); ++i) {
    if (cost[i][assignment[i]] < INF) {
      cout << "Agent " << i << " assigned to Task " << assignment[i] << endl;
    }
  }

  // if (picker_tasks.empty()) {
  //   return nullptr;
  // }
  // // std::srand(std::time(0));  // Seed the random number generator
  // // std::srand(0);  // Seed the random number generator
  // //
  // // int picker_idx = std::rand() % picker_tasks.size();
  // // auto it = picker_tasks.begin();
  // // std::advance(it, picker_idx);
  // // auto tmp_pick_task = *it;
  // // picker_tasks.erase(it);
  // // tmp_pick_task->agent_id = agent_id;
  // auto tmp_pick_task = picker_tasks.front();
  // picker_tasks.pop_front();
  // tmp_pick_task->agent_id = agent_id;
  return assignment;
}


std::shared_ptr<MobileRobotTask> MobileTaskManager::pickStation(int agent_id) {
//  Task tmp_task;
  if (active_stations.empty()) {
    return nullptr;
  }
  // std::srand(std::time(0));  // Seed the random number generator
  std::srand(0);  // Seed the random number generator

  int station_idx = std::rand() % active_stations.size();
  std::cout << "active stations size: " << active_stations.size() << ", picked station idx: " << station_idx << std::endl;
  auto it = active_stations.begin();
  std::advance(it, station_idx);
  auto tmp_station = it->second;
  int x = tmp_station->x;
  int y = tmp_station->y;
  return std::make_shared<MobileRobotTask>(curr_task_idx++, agent_id, std::make_pair(x, y),
    tmp_station->idx, MobileAction::DELIVER);
}

void MobileTaskManager::setTask(int agent_id, std::shared_ptr<MobileRobotTask>& task, bool status) {
  if (task->status == status and status == false) {
    return;
  }
  if (task->act == MobileAction::DELIVER) {
    setStation(task->operate_obj_idx, status);
  }
  if (status) {
    // Add new task to the agent
#ifdef DEBUG
    std::cout << "adding new task with id: " << task->id << std::endl;
#endif
    agent_task_status[agent_id].assigned_tasks.emplace_back(task);
  } else {
    // Remove finished task
#ifdef DEBUG
    std::cout << "remove finished task with id: " << task->id << std::endl;
    for (auto& tmp_new_task: agent_task_status[agent_id].assigned_tasks) {
      std::cout << "task for agent " << agent_id << ", with goal at: "
      << tmp_new_task->goal_position.first << ", " << tmp_new_task->goal_position.second << ", status: "
      << tmp_new_task->status << std::endl;
    }
#endif
    if (agent_task_status[agent_id].assigned_tasks.empty()) {
      std::cout << "No task for agent " << agent_id << std::endl;
      exit(-1);
    }
    assert(task == agent_task_status[agent_id].assigned_tasks.front());
    agent_task_status[agent_id].assigned_tasks.pop_front();
    if (task->act == MobileAction::DELIVER) {
      agent_task_status[agent_id].curr_loads = 0;
    } else {
      agent_task_status[agent_id].curr_loads += 1;
    }
  }
  task->status = status;
}

bool MobileTaskManager::setStation(int station_idx, bool status) {
  if (status) {
    auto tmp_station = active_stations.find(station_idx);
    if (tmp_station == active_stations.end()) {
      std::cerr << "station_idx " << station_idx << " is not free!" << std::endl;
      return false;
    }
    tmp_station->second->status = status;
    assert(tmp_station != nullptr);
    active_stations.erase(tmp_station);
    occupied_stations.insert({station_idx, tmp_station->second});
  } else {
    auto tmp_station = occupied_stations.find(station_idx);
    if (tmp_station == occupied_stations.end()) {
      std::cerr << "station_idx " << station_idx << " is not in use!" << std::endl;
      return false;
    }
    tmp_station->second->status = status;
    assert(tmp_station != nullptr);
    occupied_stations.erase(tmp_station);
    active_stations.insert({station_idx, tmp_station->second});
  }
  return true;
}

std::pair<int, int> MobileTaskManager::findNearbyFreeCell(int x, int y) {
  if ((y / 4) % 2 == 0 ) {
    return std::make_pair(x, y+1);

  } else {
    return std::make_pair(x, y-1);
  }

  if (user_map.isValid(x, y+1)) {
    return std::make_pair(x, y+1);
  }
  if (user_map.isValid(x, y-1)) {
    return std::make_pair(x, y-1);
  }
  std::cerr << "Error in finding available position to pod! Error in location! Exiting" << std::endl;
  exit(-1);
}

int MobileTaskManager::insertPickerTask(int picker_id, int goal_x, int goal_y) {
  std::pair<int, int> free_loc = findNearbyFreeCell(goal_x, goal_y);
  std::shared_ptr<MobileRobotTask> new_pickup_task = std::make_shared<MobileRobotTask>(curr_task_idx++, -1,
    free_loc, -1, MobileAction::PICK);
  // picker_tasks.push_back(new_pickup_task);
  all_picker_tasks[picker_id].push_back(new_pickup_task);
  return new_pickup_task->id;
}