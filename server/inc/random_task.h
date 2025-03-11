#include "task_assigner.h"



struct AgentTaskStatus {
  // num of packs agent is carrying
  int curr_loads;
  int agent_idx;
};

class RandomTask : private TaskManager {
public:
  RandomTask() = default;
  Task genRandomTask(int agent_id);
  Task pickRandomPod(int agent_id);
  Task pickRandomStation(int agent_id);

private:
  std::vector<Task> allocated_tasks;
  Map map;
  std::vector<AgentTaskStatus> agent_task_status;
};