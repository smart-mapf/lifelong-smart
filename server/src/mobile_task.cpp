#include "mobile_task.h"

MobileTaskManager::MobileTaskManager(int num_agents, std::string& map_fname): TaskManager(num_agents, map_fname) {
  for (auto& tmp_station: user_map.all_stations) {
    active_stations.insert({tmp_station->idx, tmp_station});
  }
  for (auto& tmp_pod: user_map.all_pods) {
    active_pods.insert({tmp_pod->idx, tmp_pod});
  }
  for (int i = 0; i < num_agents; ++i) {
    agent_task_status.emplace_back(i);
  }
}


void MobileTaskManager::getTask(std::vector<std::deque<std::shared_ptr<Task>>>& new_tasks) {
  for (int agent_idx = 0; agent_idx < num_robots_; agent_idx++) {
    assert(agent_task_status[agent_idx].curr_loads >= 0);
#ifdef DEBUG
    std::cout << "Total task for Agent " << agent_idx << ", is: " << agent_task_status[agent_idx].assigned_tasks.size() << std::endl;
#endif
    if (agent_task_status[agent_idx].assigned_tasks.empty()) {
      auto tmp_new_task = genRandomTask(agent_idx);
      if (tmp_new_task == nullptr) {
        // Do nothing
        std::cerr << "Failed to generate new random task" << std::endl;
        continue;
      }
      setTask(agent_idx, tmp_new_task, true);
      std::cout << "Create new task for agent " << agent_idx << ", with goal at: "
      << tmp_new_task->goal_position.first << ", " << tmp_new_task->goal_position.second << std::endl;
    } else {
      auto tmp_new_task = agent_task_status[agent_idx].assigned_tasks.front();
      std::cout << "Existing task for agent " << agent_idx << ", with goal at: "
      << tmp_new_task->goal_position.first << ", " << tmp_new_task->goal_position.second << ", status: "
      << tmp_new_task->status << std::endl;
    }
  }

  // TODO@jingtian: consider more efficient implementation
  new_tasks.clear();
  new_tasks.resize(num_robots_);
  for (int agent_idx = 0; agent_idx < num_robots_; ++agent_idx) {
    new_tasks[agent_idx] = agent_task_status[agent_idx].assigned_tasks;
  }
}

std::shared_ptr<Task> MobileTaskManager::genRandomTask(int agent_id) {
  if (agent_task_status[agent_id].curr_loads < MAX_LOADS) {
    std::cout << "Pick a new random pod" << std::endl;
    auto new_task = pickRandomPod(agent_id);
    // if (new_task != nullptr) {
    //   setPod(new_task->operate_obj_idx, true);
    // }
    return new_task;
  } else {
    std::cout << "Pick a new random Station" << std::endl;
    auto new_task = pickRandomStation(agent_id);
    // if (new_task != nullptr) {
    //   // TODO@jingtian: set this after a goal is reached
    //   // agent_task_status[agent_id].curr_loads = 0;
    //   setStation(new_task->operate_obj_idx, true);
    // }
    return new_task;
  }
}

std::shared_ptr<Task> MobileTaskManager::pickRandomPod(int agent_id) {
//  Task tmp_task;
  if (active_pods.empty()) {
    return nullptr;
  }
  // std::srand(std::time(0));  // Seed the random number generator
  std::srand(0);  // Seed the random number generator
  std::cout << "active pods size: " << active_pods.size() << std::endl;

  int pod_idx = std::rand() % (active_pods.size() - 1);
  auto it = active_pods.begin();
  std::advance(it, pod_idx);
  auto tmp_pod = it->second;
  int x = tmp_pod->x;
  int y = tmp_pod->y;
  if (user_map.isValid(x, y+1)) {
    return std::make_shared<Task>(curr_task_idx++, agent_id, std::make_pair(x, y+1), std::make_pair(x, y), tmp_pod->idx, 1);
  }
  if (user_map.isValid(x, y-1)) {
    return std::make_shared<Task>(curr_task_idx++, agent_id, std::make_pair(x, y-1), std::make_pair(x, y), tmp_pod->idx, 1);
  }
  // if (user_map.isValid(x+1, y)) {
  //   return std::make_shared<Task>(curr_task_idx++, agent_id, std::make_pair(x+1, y), std::make_pair(x, y), pod_idx, 1);
  // }
  // if (user_map.isValid(x-1, y)) {
  //   return std::make_shared<Task>(curr_task_idx++, agent_id, std::make_pair(x-1, y), std::make_pair(x, y), pod_idx, 1);
  // }
  std::cerr << "Error in finding available position to pod!" << std::endl;
  return nullptr;
}

std::shared_ptr<Task> MobileTaskManager::pickRandomStation(int agent_id) {
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
  return std::make_shared<Task>(curr_task_idx++, agent_id, std::make_pair(x, y), std::make_pair(x, y), tmp_station->idx, 0);
}

void MobileTaskManager::setTask(int agent_id, std::shared_ptr<Task>& task, bool status) {
  if (task->status == status and status == false) {
    return;
  }
  if (task->flag == 0) {
    setStation(task->operate_obj_idx, status);
  } else if (task->flag == 1) {
    setPod(task->operate_obj_idx, status);
  } else {
    std::cerr << "Error in task type!" << std::endl;
    exit(-1);
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
    if (task->flag == 0) {
      agent_task_status[agent_id].curr_loads = 0;
    } else {
      agent_task_status[agent_id].curr_loads += 1;
    }
  }
  task->status = status;
}

bool MobileTaskManager::setPod(int pod_idx, bool status) {
  if (status) {
    auto tmp_pod = active_pods.find(pod_idx);
    if (tmp_pod == active_pods.end()) {
      std::cerr << "pod_idx " << pod_idx << " is not free!" << std::endl;
      return false;
    }
    tmp_pod->second->status = status;
    assert(tmp_pod != nullptr);
    active_pods.erase(tmp_pod);
    occupied_pods.insert({tmp_pod->first, tmp_pod->second});
  } else {
    auto tmp_pod = occupied_pods.find(pod_idx);
    if (tmp_pod == occupied_pods.end()) {
      std::cerr << "pod_idx " << pod_idx << " is not in use!" << std::endl;
      return false;
    }
    tmp_pod->second->status = status;
    assert(tmp_pod != nullptr);
    occupied_pods.erase(tmp_pod);
    active_pods.insert({tmp_pod->first, tmp_pod->second});
  }
  return true;
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