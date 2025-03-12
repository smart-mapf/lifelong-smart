#include "random_task.h"

RandomTask::RandomTask(int num_agents, std::string& map_fname): TaskManager(num_agents, map_fname) {
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


void RandomTask::getTask(std::vector<std::deque<std::shared_ptr<Task>>>& new_tasks) {
  for (int agent_idx = 0; agent_idx < num_robots_; agent_idx++) {
    assert(agent_task_status[agent_idx].curr_loads >= 0);
    if (agent_task_status[agent_idx].assigned_tasks.empty()) {
      auto tmp_new_task = genRandomTask(agent_idx);
      if (tmp_new_task == nullptr) {
        // Do nothing
        std::cerr << "Failed to generate new random task" << std::endl;
        exit(-1);
        continue;
      }
      agent_task_status[agent_idx].assigned_tasks.emplace_back(tmp_new_task);
      setTask(agent_idx, tmp_new_task, true);
    }
  }

  // TODO@jingtian: consider more efficient implementation
  new_tasks.clear();
  new_tasks.resize(num_robots_);
  for (int agent_idx = 0; agent_idx < num_robots_; ++agent_idx) {
    new_tasks[agent_idx] = agent_task_status[agent_idx].assigned_tasks;
  }
}

std::shared_ptr<Task> RandomTask::genRandomTask(int agent_id) {
  if (agent_task_status[agent_id].curr_loads < MAX_LOADS) {
    auto new_task = pickRandomPod(agent_id);
    // if (new_task != nullptr) {
    //   setPod(new_task->operate_obj_idx, true);
    // }
    return new_task;
  } else {
    auto new_task = pickRandomStation(agent_id);
    // if (new_task != nullptr) {
    //   // TODO@jingtian: set this after a goal is reached
    //   // agent_task_status[agent_id].curr_loads = 0;
    //   setStation(new_task->operate_obj_idx, true);
    // }
    return new_task;
  }
}

std::shared_ptr<Task> RandomTask::pickRandomPod(int agent_id) {
//  Task tmp_task;
  if (active_pods.empty()) {
    return nullptr;
  }
  std::srand(std::time(0));  // Seed the random number generator
  int pod_idx = std::rand() % (active_pods.size() - 1);
  auto tmp_pod = active_pods.at(pod_idx);
  int x = tmp_pod->x;
  int y = tmp_pod->y;
  // TODO@jingtian: check correctness here
  if (user_map.isValid(x+1, y)) {
    return std::make_shared<Task>(curr_task_idx++, agent_id, std::make_pair(x+1, y), std::make_pair(x, y), pod_idx, 1);
  }
  if (user_map.isValid(x-1, y)) {
    return std::make_shared<Task>(curr_task_idx++, agent_id, std::make_pair(x-1, y), std::make_pair(x, y), pod_idx, 1);
  }
  if (user_map.isValid(x, y+1)) {
    return std::make_shared<Task>(curr_task_idx++, agent_id, std::make_pair(x, y+1), std::make_pair(x, y), pod_idx, 1);
  }
  if (user_map.isValid(x, y-1)) {
    return std::make_shared<Task>(curr_task_idx++, agent_id, std::make_pair(x, y-1), std::make_pair(x, y), pod_idx, 1);
  }
  std::cerr << "Error in finding available position to pod!" << std::endl;
  return nullptr;
}

std::shared_ptr<Task> RandomTask::pickRandomStation(int agent_id) {
//  Task tmp_task;
  if (active_stations.empty()) {
    return nullptr;
  }
  std::srand(std::time(0));  // Seed the random number generator
  int station_idx = std::rand() % (active_stations.size() - 1);
  auto tmp_station = active_stations.at(station_idx);
  int x = tmp_station->x;
  int y = tmp_station->y;
  return std::make_shared<Task>(curr_task_idx++, agent_id, std::make_pair(x, y), std::make_pair(x, y), station_idx, 0);
}

void RandomTask::setTask(int agent_id, std::shared_ptr<Task>& task, bool status) {
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
    agent_task_status[agent_id].assigned_tasks.emplace_back(task);
  } else {
    // Remove finished task
    agent_task_status[agent_id].assigned_tasks.pop_front();
    agent_task_status[agent_id].curr_loads += 1;
  }
}

bool RandomTask::setPod(int pod_idx, bool status) {
  if (status) {
    auto tmp_pod = active_pods.find(pod_idx);
    if (tmp_pod == active_pods.end()) {
      std::cerr << "pod_idx " << pod_idx << " is not free!" << std::endl;
      return false;
    }
    tmp_pod->second->status = status;
    active_pods.erase(tmp_pod);
    occupied_pods.insert({tmp_pod->first, tmp_pod->second});
  } else {
    auto tmp_pod = occupied_pods.find(pod_idx);
    if (tmp_pod == occupied_pods.end()) {
      std::cerr << "pod_idx " << pod_idx << " is not in use!" << std::endl;
      return false;
    }
    tmp_pod->second->status = status;
    occupied_pods.erase(tmp_pod);
    active_pods.insert({tmp_pod->first, tmp_pod->second});
  }
  return true;
}

bool RandomTask::setStation(int station_idx, bool status) {
  if (status) {
    auto tmp_station = active_stations.find(station_idx);
    if (tmp_station == active_stations.end()) {
      std::cerr << "station_idx " << station_idx << " is not free!" << std::endl;
      return false;
    }
    tmp_station->second->status = status;
    active_stations.erase(tmp_station);
    occupied_stations.insert({station_idx, tmp_station->second});
  } else {
    auto tmp_station = occupied_stations.find(station_idx);
    if (tmp_station == occupied_stations.end()) {
      std::cerr << "station_idx " << station_idx << " is not in use!" << std::endl;
      return false;
    }
    tmp_station->second->status = status;
    occupied_stations.erase(tmp_station);
    active_stations.insert({station_idx, tmp_station->second});
  }
  return true;
}