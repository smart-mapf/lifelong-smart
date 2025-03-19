#include "picker_task.h"

PickTask::PickTask(int num_agents, int num_genre, std::string& map_fname): TaskManager(num_agents, map_fname),
num_task_genre(num_genre)
{
  for (int i = 0; i < num_agents; i++) {
    pickers.emplace_back(i);
  }
  genRandomTask();
}


void PickTask::getTask(std::vector<std::deque<std::shared_ptr<PickerTask>>>& new_tasks) {
  genRandomTask();

}

void PickTask::genRandomTask() {
  for (int genre = 0; genre < num_task_genre; genre++) {
    while (tasks_by_genre[genre].size() < MAX_TASKS) {
      auto new_task = pickRandomPod(genre);
      tasks_by_genre[genre].insert(std::make_pair(new_task->id, new_task));
    }
  }
}

std::shared_ptr<PickerTask> PickTask::pickRandomPod(int genre) {
  // std::srand(std::time(0));  // Seed the random number generator
  std::srand(0);  // Seed the random number generator
  int pod_idx = std::rand() % static_cast<int> (user_map.pods_by_genre[genre].size());
  auto tmp_pod = user_map.pods_by_genre[genre][pod_idx];
  int x = tmp_pod->x;
  int y = tmp_pod->y;
  // TODO@jingtian: I use agent id entry in Task as genre id
  if (user_map.isValid(x, y+1)) {
    return std::make_shared<PickerTask>(curr_task_idx++, genre, std::make_pair(x, y+1), std::make_pair(x, y), tmp_pod->idx, 1);
  }
  if (user_map.isValid(x, y-1)) {
    return std::make_shared<PickerTask>(curr_task_idx++, genre, std::make_pair(x, y-1), std::make_pair(x, y), tmp_pod->idx, 1);
  }
  std::cerr << "Error in finding available position to pod!" << std::endl;
  exit(-1);
}



bool PickTask::confirmTask(int agent_id, std::shared_ptr<PickerTask>& task) {

}