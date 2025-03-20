#pragma once

#include <list>

#include "common.h"
#include "task_assigner.h"

struct PickerRobotState {
  // num of packs agent is carrying
  int curr_loads = 0;
  int agent_idx;
  std::deque<std::shared_ptr<MobileRobotTask>> assigned_tasks;
  explicit PickerRobotState(int agent_idx): agent_idx(agent_idx) {}
};

class MobileTaskManager : TaskManager {
public:
  MobileTaskManager(int num_agents, std::string& map_fname);
  void getTask(std::vector<std::deque<std::shared_ptr<MobileRobotTask>>>& new_tasks);
  void setTask(int agent_id, std::shared_ptr<MobileRobotTask>& task, bool status);
  void insertPickerTask(std::shared_ptr<MobileRobotTask> task) {
    picker_tasks.push_back(task);
  }

private:
  std::shared_ptr<MobileRobotTask> genTask(int agent_id);
  std::shared_ptr<MobileRobotTask> pickPicker(int agent_id);
  std::shared_ptr<MobileRobotTask> pickStation(int agent_id);
  bool setStation(int station_idx, bool status);

private:
  std::vector<PickerRobotState> agent_task_status;
  std::list<std::shared_ptr<MobileRobotTask>> picker_tasks;
  std::unordered_map<int, std::shared_ptr<Station>> active_stations;
  std::unordered_map<int, std::shared_ptr<Station>> occupied_stations;
};