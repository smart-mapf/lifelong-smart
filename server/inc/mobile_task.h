#pragma once

#include <list>

#include "common.h"
#include "task_assigner.h"

struct MobileRobotState {
  // num of packs agent is carrying
  int curr_loads = 0;
  int agent_idx;
  std::deque<std::shared_ptr<MobileRobotTask>> assigned_tasks;
  explicit MobileRobotState(int agent_idx): agent_idx(agent_idx) {}
};

class MobileTaskManager : public TaskManager {
public:
  MobileTaskManager(int num_agents, std::string& map_fname);
  void getTask(std::vector<std::deque<std::shared_ptr<MobileRobotTask>>>& new_tasks);
  void setTask(int agent_id, std::shared_ptr<MobileRobotTask>& task, bool status);
  int insertPickerTask(int goal_x, int goal_y);

private:
  std::shared_ptr<MobileRobotTask> genTask(int agent_id);
  std::shared_ptr<MobileRobotTask> pickPicker(int agent_id);
  std::shared_ptr<MobileRobotTask> pickStation(int agent_id);
  bool setStation(int station_idx, bool status);
  std::pair<int, int> findNearbyFreeCell(int x, int y);

private:
  std::vector<MobileRobotState> agent_task_status;
  std::list<std::shared_ptr<MobileRobotTask>> picker_tasks;
  std::unordered_map<int, std::shared_ptr<Station>> active_stations;
  std::unordered_map<int, std::shared_ptr<Station>> occupied_stations;
};