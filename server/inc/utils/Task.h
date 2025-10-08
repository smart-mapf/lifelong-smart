#pragma once
#include "utils/common.h"

struct Task {
    int location;        // location of the goal
    int orientation;     // orientation of the goal
    int task_wait_time;  // the number of timesteps the agent should wait at the
                         // goal
    int hold_time;  // the time when the task is held. Used by `hold_endpoints`
    int agent_id;   // the agent who is doing the task
    int finish_t;   // the timestep when the task is finished
    bool is_parking;  // whether the task is a parking task
    bool movable;  // whether the agent can move away from the task if it block
                   // other agents. Used only by GreyOrange system.
    int id;        // Unique ID for the task

    Task()
        : location(-1),
          orientation(-1),
          task_wait_time(0),
          hold_time(0),
          agent_id(-1),
          finish_t(-1),
          is_parking(false),
          movable(false),
          id(-1) {
    }

    Task(int loc, int ori = -1, int wait_time = 0, int hold_time = 0,
         int agent_id = -1, int finish_t = -1, bool is_parking = false,
         bool movable = false, int id = -1)
        : location(loc),
          orientation(ori),
          task_wait_time(wait_time),
          hold_time(hold_time),
          agent_id(agent_id),
          finish_t(finish_t),
          is_parking(is_parking),
          movable(movable),
          id(id) {
    }

    Task(const Task& other) {
        location = other.location;
        orientation = other.orientation;
        task_wait_time = other.task_wait_time;
        hold_time = other.hold_time;
        agent_id = other.agent_id;
        finish_t = other.finish_t;
        is_parking = other.is_parking;
        movable = other.movable;
        id = other.id;
    }

    struct Hasher {
        std::size_t operator()(const Task& n) const {
            size_t loc_hash = std::hash<int>()(n.location);
            size_t ori_hash = std::hash<int>()(n.orientation);
            size_t wait_hash = std::hash<int>()(n.task_wait_time);
            size_t hold_hash = std::hash<int>()(n.hold_time);
            // size_t agent_hash = std::hash<int>()(n.agent_id);
            // size_t finish_hash = std::hash<int>()(n.finish_t);
            size_t is_parking_hash = std::hash<bool>()(n.is_parking);
            size_t movable_hash = std::hash<bool>()(n.movable);
            size_t id_hash = std::hash<int>()(n.id);
            // Hashing function
            // Combine the hashes using XOR and bit shifting
            // to create a unique hash value
            return (loc_hash ^ (ori_hash << 1) ^ (wait_hash << 2) ^
                    (hold_hash << 3) ^ (is_parking_hash << 4) ^
                    (movable_hash << 5)) ^
                   (id_hash << 6);
        }
    };

    void operator=(const Task& other) {
        location = other.location;
        orientation = other.orientation;
        task_wait_time = other.task_wait_time;
        hold_time = other.hold_time;
        agent_id = other.agent_id;
        finish_t = other.finish_t;
        is_parking == other.is_parking;
        movable = other.movable;
        id = other.id;
    }

    bool operator==(const Task& other) const {
        return location == other.location && orientation == other.orientation &&
               task_wait_time == other.task_wait_time &&
               hold_time == other.hold_time && agent_id == other.agent_id &&
               finish_t == other.finish_t && is_parking == other.is_parking &&
               movable == other.movable && id == other.id;
    }

    bool operator!=(const Task& other) const {
        return location != other.location || orientation != other.orientation ||
               task_wait_time != other.task_wait_time ||
               hold_time != other.hold_time || agent_id != other.agent_id ||
               finish_t != other.finish_t || is_parking != other.is_parking ||
               movable != other.movable || id != other.id;
    }
};

std::ostream& operator<<(std::ostream& os, const Task& task);

vector<list<Task>> read_task_vec(const std::string& fname, int num_of_drives);
vector<tuple<int, int>> read_start_vec(const std::string& fname,
                                       int num_of_drives);
void from_json(const json& j, Task& t);
void to_json(json& j, const Task& t);