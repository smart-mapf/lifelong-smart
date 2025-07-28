#include "states.h"


std::ostream & operator << (std::ostream &out, const State &s)
{
    out << s.location << ","
        << s.orientation << ","
        << s.timestep << ","
        << s.is_tasking_wait << ","
        << s.is_rotating;
    return out;
}