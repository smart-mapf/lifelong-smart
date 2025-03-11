#pragma once

#include "task_assigner.h"

struct AgentTaskStatus {
  // num of packs agent is carrying
  int curr_loads = 0;
  int agent_idx;
  std::deque<std::shared_ptr<Task>> assigned_tasks;
  explicit AgentTaskStatus(int agent_idx): agent_idx(agent_idx) {}
};

class RandomTask : private TaskManager {
public:
  RandomTask(int num_agents, std::string& map_fname);
  std::shared_ptr<Task> genRandomTask(int agent_id);
  std::shared_ptr<Task> pickRandomPod(int agent_id);
  std::shared_ptr<Task> pickRandomStation(int agent_id);
  void getTask(std::vector<std::deque<std::shared_ptr<Task>>>& new_tasks) override;
  void getTask(std::vector<std::deque<std::shared_ptr<Task>>>& new_tasks, std::vector<int> num_new_tasks) override;
  bool setPod(int pod_idx, bool status);
  bool setStation(int station_idx, bool status);

private:
  std::vector<AgentTaskStatus> agent_task_status;
  std::map<int, std::shared_ptr<Station>> active_stations;
  std::map<int, std::shared_ptr<Station>> occupied_stations;
  std::map<int, std::shared_ptr<Pod>> active_pods;
  std::map<int, std::shared_ptr<Pod>> occupied_pods;
};