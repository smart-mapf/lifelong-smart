#pragma once

#include <task_assigner.h>

#include "common.h"

// typedef Task PickerTask;

class PickerRobotState {
  public:
    explicit PickerRobotState(int agent_idx): agent_idx(agent_idx) {}
    // void deliverPkg();
    // void pickPkg();
    // num of packs agent is carrying
    int curr_loads = 0;
    int agent_idx;
};


class PickTaskManager: TaskManager {
public:
  PickTaskManager(int num_agents, int num_genre, std::string& map_fname);

  // read all tasks from order.csv
  void readTask(std::string& order_file) {
    // TODO@jingtian
  };

  // generate random tasks
  void genRandomTask();

  std::shared_ptr<PickerTask> pickRandomPod(int genre);

  // read path/route picker need to follow
  void readConfig(std::string& config_file) {
    // TODO@jingtian
  }

  void getTask(std::vector<std::shared_ptr<PickerTask>>& new_tasks);

  bool confirmTask(int agent_id, int task_id, int& genre_id);

  // void mobileRobotRequest();

  void finishDeliver(int agent_id);

private:
  int num_task_genre;
  // NOTE@jingtian: Unused for now, maybe add this later?
  std::map< int, int > robot2genre;
  std::vector<PickerRobotState> pickers;
  std::vector< std::map< int, std::shared_ptr<PickerTask> > > tasks_by_genre;
  std::map< int, int > task2genre;
};