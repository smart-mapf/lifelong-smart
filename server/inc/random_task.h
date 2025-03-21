#pragma once

#include "task_assigner.h"

struct Task
{
    int id;
    int agent_id;
    std::pair<int, int> goal_position;
    std::pair<int, int> obj_position;
    int goal_orient=0;
    // Maybe find a better way?
    // For now, 0 is to station, 1 is to pod
    int flag = -1;
    int operate_obj_idx = -1;

    bool status=true; // false if finished, true otherwise
    Task(int id, int agent_id, std::pair<int, int> goal_position): id(id), agent_id(agent_id), goal_position(std::move(goal_position)) {
        obj_position=goal_position;
    }
    Task(int id, int agent_id, std::pair<int, int> goal_position, std::pair<int, int> obj_pos, int obj_idx, int flag):
        id(id), agent_id(agent_id), goal_position(std::move(goal_position)), obj_position(std::move(obj_pos)),
        operate_obj_idx(obj_idx), flag(flag) {}
};

struct AgentTaskStatus {
  // num of packs agent is carrying
  int curr_loads = 0;
  int agent_idx;
  std::deque<std::shared_ptr<Task>> assigned_tasks;
  explicit AgentTaskStatus(int agent_idx): agent_idx(agent_idx) {}
};

class RandomTask : TaskManager {
public:
  RandomTask(int num_agents, std::string& map_fname);
  std::shared_ptr<Task> genRandomTask(int agent_id);
  void getTask(std::vector<std::deque<std::shared_ptr<Task>>>& new_tasks);
  // void getTask(std::vector<std::deque<std::shared_ptr<Task>>>& new_tasks, std::vector<int> num_new_tasks) override;
  void setTask(int agent_id, std::shared_ptr<Task>& task, bool status);
  // ~RandomTask() override = default;

private:
  std::shared_ptr<Task> pickRandomPod(int agent_id);
  std::shared_ptr<Task> pickRandomStation(int agent_id);
  bool setPod(int pod_idx, bool status);
  bool setStation(int station_idx, bool status);

private:
  std::vector<std::deque<std::shared_ptr<Task>>> all_tasks;
  std::vector<AgentTaskStatus> agent_task_status;
  std::map<int, std::shared_ptr<Station>> active_stations;
  std::map<int, std::shared_ptr<Station>> occupied_stations;
  std::map<int, std::shared_ptr<Pod>> active_pods;
  std::map<int, std::shared_ptr<Pod>> occupied_pods;
};