#pragma once

#include <list>

#include "common.h"
#include "task_assigner.h"
#include "hungarian.h"

struct MobileRobotState {
  // num of packs agent is carrying
  // int curr_loads = 0;
  int agent_idx;
  Location loc;
  std::deque<std::shared_ptr<MobileRobotTask>> assigned_tasks;
  explicit MobileRobotState(int agent_idx): agent_idx(agent_idx) {}
};

class MobileTaskManager : public TaskManager {
public:
  MobileTaskManager(int num_agents, int num_picker, std::string& map_fname);
  void getTask(const std::vector<std::pair<double, double>>& robots_location,
    std::vector<std::deque<std::shared_ptr<MobileRobotTask>>>& new_tasks);

  void setTask(int agent_id, std::shared_ptr<MobileRobotTask>& task, MobileAction status);
  void finishTask(int agent_id, std::shared_ptr<MobileRobotTask>& task);
  int insertPickerTask(int picker_id, int goal_x, int goal_y);
  int total_finished_tasks_ = 0;

private:
  std::pair<int, int> findNearbyFreeCell(int x, int y);
  std::pair<int, int> findPalletizer(int picker_id);
  void cleanTasks();

private:
  std::vector<MobileRobotState> agent_task_status;
  // std::deque<std::shared_ptr<MobileRobotTask>> picker_tasks;
  std::vector< std::deque<std::shared_ptr<MobileRobotTask>> > all_picker_tasks;
  std::unordered_map<int, std::shared_ptr<Station>> active_stations;
  std::unordered_map<int, std::shared_ptr<Station>> occupied_stations;
  int prev_agent_idx = -1;
  int num_picker_ = 0;
};