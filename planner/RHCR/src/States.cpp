#include "States.h"


std::ostream & operator << (std::ostream &out, const State &s)
{
    out << s.location << ","
        << s.orientation << ","
        << s.timestep << ","
        << s.is_tasking_wait << ","
        << s.is_rotating;
    return out;
}

std::ostream & operator << (std::ostream &out, const Path &path)
{
    for (auto state : path)
    {
        // if(state.location < 0)
        //     continue;
        out << "("
            << state.location << ","
            << state.orientation << ","
            << state.timestep << ","
            << state.is_tasking_wait << ","
            << state.is_rotating << ")->";
    }
    out << std::endl;
    return out;
}

void to_json(json& j, const State& s)
{
    j = json{
        {"location", s.location},
        {"timestep", s.timestep},
        {"orientation", s.orientation},
        {"is_tasking_wait", s.is_tasking_wait},
        {"is_rotating", s.is_rotating}
    };
}