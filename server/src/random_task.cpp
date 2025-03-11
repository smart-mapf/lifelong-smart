#include "random_task.h"

//RandomTask::RandomTask(int num_char, int num_robots);

Task RandomTask::genRandomTask(int agent_id) {
  if (agent_task_status[agent_id].curr_loads < MAX_LOADS) {
    return pickRandomPod(agent_id);
  } else {
    Task new_task = pickRandomStation(agent_id);
    if (new_task.id != 1) {
      agent_task_status[agent_id].curr_loads = 0;
    }
    return new_task;
  }
}

Task RandomTask::pickRandomPod(int agent_id) {
//  Task tmp_task;
  if (active_pods.empty()) {
    return Task{-1, agent_id, std::make_pair(-1, -1)};
  }
  std::srand(std::time(0));  // Seed the random number generator
  int pod_idx = std::rand() % (active_pods.size() - 1);
  auto tmp_pod = active_pods.at(pod_idx);
  int x = tmp_pod->x;
  int y = tmp_pod->y;
  // TODO@jingtian: check correctness here
  if (map.isValid(x+1, y)) {
    return Task{curr_task_idx++, agent_id, std::make_pair(x+1, y), std::make_pair(x, y)};
  }
  if (map.isValid(x-1, y)) {
    return Task{curr_task_idx++, agent_id, std::make_pair(x-1, y), std::make_pair(x, y)};
  }
  if (map.isValid(x, y+1)) {
    return Task{curr_task_idx++, agent_id, std::make_pair(x, y+1), std::make_pair(x, y)};
  }
  if (map.isValid(x, y-1)) {
    return Task{curr_task_idx++, agent_id, std::make_pair(x, y-1), std::make_pair(x, y)};
  }
  std::cerr << "Error in finding available position to pod!" << std::endl;
  exit(-1);
}


Task RandomTask::pickRandomStation(int agent_id) {
//  Task tmp_task;
  if (active_stations.empty()) {
    return Task{-1, agent_id, std::make_pair(-1, -1)};
  }
  std::srand(std::time(0));  // Seed the random number generator
  int station_idx = std::rand() % (active_stations.size() - 1);
  auto tmp_station = active_stations.at(station_idx);
  int x = tmp_station->x;
  int y = tmp_station->y;
  return Task{curr_task_idx++, agent_id, std::make_pair(x, y), std::make_pair(x, y)};
}