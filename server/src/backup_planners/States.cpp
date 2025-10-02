#include "backup_planners/States.h"

std::ostream &operator<<(std::ostream &out, const State &s) {
    out << s.location << "," << s.orientation << "," << s.timestep << ","
        << s.is_tasking_wait << "," << s.is_rotating;
    return out;
}

std::ostream &operator<<(std::ostream &out, const Path &path) {
    for (auto state : path) {
        // if(state.location < 0)
        //     continue;
        out << "(" << state.location << "," << state.orientation << ","
            << state.timestep << "," << state.is_tasking_wait << ","
            << state.is_rotating << ")->";
    }
    out << std::endl;
    return out;
}

void from_json(const json &j, State &s) {
    j.at("location").get_to(s.location);
    j.at("timestep").get_to(s.timestep);
    j.at("orientation").get_to(s.orientation);
    j.at("is_tasking_wait").get_to(s.is_tasking_wait);
    j.at("is_rotating").get_to(s.is_rotating);
}