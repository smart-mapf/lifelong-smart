#pragma once

#include <task_assigner.h>

#include "common.h"

typedef Task PickerTask;

class PickerRobotState {
  public:
    explicit PickerRobotState(int agent_idx): agent_idx(agent_idx) {}
    void deliverPkg();
    void pickPkg();
    // num of packs agent is carrying
    int curr_loads = 0;
    int agent_idx;
};

// struct PickerTask:Task
// {
//   int id;
//   int agent_id;
//   std::pair<int, int> goal_position;
//   std::pair<int, int> obj_position;
//   int goal_orient=0;
//   // Maybe find a better way?
//   // For now, 0 is to station, 1 is to pod
//   int flag = -1;
//   int operate_obj_idx = -1;
//
//   bool status=true; // false if finished, true otherwise
//   PickerTask(int id, int agent_id, std::pair<int, int> goal_position): Task(id, agent_id, goal_position) {
//     ;
//   }
//   PickerTask(int id, int agent_id, std::pair<int, int> goal_position, std::pair<int, int> obj_pos, int obj_idx, int flag):
//       Task(id, agent_id, goal_position, obj_pos, obj_idx, flag) {
//     ;
//   }
// };


class PickTask: TaskManager {
public:
  PickTask(int num_agents, int num_genre, std::string& map_fname);

  // read all tasks from order.csv
  void readTask(std::string& order_file) {
    // TODO@jingtian
  };

  // generate random tasks
  void genRandomTask();

  std::shared_ptr<Task> pickRandomPod(int genre);

  // read path/route picker need to follow
  void readConfig(std::string& config_file) {
    // TODO@jingtian
  }

  void getTask(std::vector<std::deque<std::shared_ptr<Task>>>& new_tasks);

  bool confirmTask(int agent_id, std::shared_ptr<Task>& task);

  void mobileRobotRequest();

  void mobileRobotUpdate();

private:
  int num_task_genre;
  // NOTE@jingtian: Unused for now, maybe add this later?
  std::map< int, int > robot2genre;
  std::vector<PickerRobotState> pickers;
  std::vector< std::map< int, std::shared_ptr<PickerTask> > > tasks_by_genre;
};