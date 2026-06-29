#pragma once

#include <iostream>
#include <json.hpp>

using json = nlohmann::json;

/// Emit a single JSON-line event to stdout with immediate flush.
/// Used by the server to stream visualization events to the web runner.
inline void emit_event(const json& event) {
    std::cout << event.dump() << std::endl;
}
