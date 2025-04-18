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

double computeDistance(const Location& agent_loc, const Location& task_loc) {
  double dx = agent_loc.first - task_loc.first;
  double dy = agent_loc.second - task_loc.second;
  return std::sqrt(dx * dx + dy * dy);
}

void MobileTaskManager::getTask(const std::vector<std::pair<double, double>>& robots_location,
  std::vector<std::deque<std::shared_ptr<MobileRobotTask>>>& new_tasks) {
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

  vector<vector<double>> cost;
  for (int robot_id: free_agents) {
    std::vector<double> cost_tmp;
    for (int j = 0; j < front_tasks.size(); ++j) {
      double tmp_dis = computeDistance(robots_location[robot_id], front_tasks[j]->goal_position);
      cost_tmp.push_back(tmp_dis);
    }
    cost.push_back(cost_tmp);
  }

  int n;
  padMatrix(cost, n);
  vector<int> assignment = hungarian(cost);
  new_tasks.clear();
  new_tasks.resize(num_robots_);
  for (int i = 0; i < assignment.size(); ++i) {
    if (cost[i][assignment[i]] < INF) {
      cout << "Agent " << free_agents[i] << " assigned to Task " << assignment[i] << endl;
      setTask(free_agents[i], front_tasks[assignment[i]], PICK);
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
      agent_task_status[agent_id].assigned_tasks.emplace_back(task);
    } else {
      std::cerr << "impossible case, exiting..." << std::endl;
      exit(-1);
    }
  } else if (task->status == PICK) {
    if (status == DELIVER) {
    } else {
      std::cerr << "impossible case, exiting..." << std::endl;
      exit(-1);
      return;
    }
  } else if (task->status == DELIVER) {
    if (status == DONE) {
      agent_task_status[agent_id].assigned_tasks.pop_front();
    } else {
      std::cerr << "impossible case, exiting..." << std::endl;
      exit(-1);
      return;
    }
  } else if (task->status == DONE) {
    std::cerr << "Task already finished!" << std::endl;
  }
  task->status = status;
}

std::pair<int, int> MobileTaskManager::findNearbyFreeCell(int x, int y) {
  if ((y / 4) % 2 == 0 ) {
    return std::make_pair(x, y+1);
  } else {
    return std::make_pair(x, y-1);
  }
}

std::pair<int, int> MobileTaskManager::findPalletizer(int picker_id) {
  int station_idx = std::rand() % active_stations.size();
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