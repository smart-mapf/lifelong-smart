#pragma once

#include <iostream>
#include <json.hpp>

using json = nlohmann::json;

inline bool& visualization_events_enabled() {
    static bool enabled = false;
    return enabled;
}

inline void set_visualization_events_enabled(bool enabled) {
    visualization_events_enabled() = enabled;
}

/// Emit a single JSON-line event to stdout with immediate flush.
/// Used by the server to stream visualization events to the web runner.
inline void emit_event(const json& event) {
    if (visualization_events_enabled()) {
        std::cout << event.dump() << std::endl;
    }
}
