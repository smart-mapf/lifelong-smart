#include "task_assigner.h"


class RandomTask : public TaskManager {
public:
  RandomTask() = default;
  Task genRandomTask(int robot_id);

};