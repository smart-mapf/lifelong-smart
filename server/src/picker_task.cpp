#include "picker_task.h"

PickTaskManager::PickTaskManager(int num_agents, int num_genre, std::string& map_fname): TaskManager(num_agents, map_fname),
num_task_genre(num_genre)
{
  for (int i = 0; i < num_agents; i++) {
    pickers.emplace_back(i);
  }
  tasks_by_genre.resize(num_genre);
  genRandomTask();
}


void PickTaskManager::getTask(std::vector<std::shared_ptr<PickerTask>>& new_tasks) {
  genRandomTask();
  new_tasks.clear();
  for (auto& genre_task : tasks_by_genre) {
    for (auto& task : genre_task) {
      new_tasks.push_back(task.second);
    }
  }
}

void PickTaskManager::genRandomTask() {
  for (int genre = 0; genre < num_task_genre; genre++) {
    while (tasks_by_genre[genre].size() < MAX_TASKS) {
      auto new_task = pickRandomPod(genre);
      tasks_by_genre[genre].insert(std::make_pair(new_task->id, new_task));
    }
  }
}

std::shared_ptr<PickerTask> PickTaskManager::pickRandomPod(int genre) {
  // std::srand(std::time(0));  // Seed the random number generator
  std::srand(genre*tasks_by_genre[genre].size());  // Seed the random number generator
  int pod_idx = std::rand() % static_cast<int> (user_map.pods_by_genre[genre].size());
  auto tmp_pod = user_map.pods_by_genre[genre][pod_idx];
  int x = tmp_pod->x;
  int y = tmp_pod->y;
  std::cout << "Genre id " << genre <<  ", choose pod: " << pod_idx << ", at loc " << x << ", " << y << std::endl;
  return std::make_shared<PickerTask>(curr_task_idx++, genre, std::make_pair(x, y));
  // if (user_map.isValid(x, y+1)) {
  //   return std::make_shared<PickerTask>(curr_task_idx++, genre, std::make_pair(x, y+1));
  // }
  // if (user_map.isValid(x, y-1)) {
  //   return std::make_shared<PickerTask>(curr_task_idx++, genre, std::make_pair(x, y-1));
  // }
  // std::cerr << "Error in finding available position to pod!" << std::endl;
  // exit(-1);
}



bool PickTaskManager::confirmTask(int agent_id, int task_id) {
  std::shared_ptr<PickerTask> tmp_task = nullptr;
  for (int genre = 0; genre < num_task_genre; genre++) {
    auto it = tasks_by_genre[genre].find(task_id);
    if (it != tasks_by_genre[genre].end()) {
      tmp_task = it->second;
      break;
    }
  }
  assert(tmp_task != nullptr and tmp_task->id == task_id);
  if (tmp_task->status == false) {
    std::cerr << "Task already finished!" << std::endl;
    return false;
  }
  tmp_task->status = false;
  tasks_by_genre[tmp_task->genre].erase(tasks_by_genre[tmp_task->genre].find(tmp_task->id));
  pickers[agent_id].curr_loads += 1;
  return true;
}

// void PickTask::mobileRobotRequest() {
//
// }

void PickTaskManager::finishDeliver(int agent_id) {
  pickers[agent_id].curr_loads = 0;
}