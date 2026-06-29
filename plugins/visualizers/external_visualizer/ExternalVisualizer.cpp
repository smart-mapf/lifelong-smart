#include "ExternalVisualizer.h"

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <algorithm>
#include <cstdio>
#include <fmt/core.h>
#include <string>
#include <vector>

using namespace argos;
using namespace std;

void ExternalVisualizer::Capture() {
    auto& space = CSimulator::GetInstance().GetSpace();

    // Collect foot-bot entities with their index attribute
    vector<pair<string, CFootBotEntity*>> bots;
    for (const auto& it : space.GetEntitiesByType("foot-bot")) {
        auto& bot = *any_cast<CFootBotEntity*>(it.second);
        auto index =
            bot.GetConfigurationNode()->GetAttributeOrDefault("index", "0");
        bots.emplace_back(index, &bot);
    }

    // Sort by index for stable ordering
    sort(bots.begin(), bots.end(),
         [](const auto& a, const auto& b) {
             return stoi(a.first) < stoi(b.first);
         });

    // Build agents array
    string agents_json;
    for (size_t i = 0; i < bots.size(); ++i) {
        auto& bot = *bots[i].second;
        auto& anchor = bot.GetEmbodiedEntity().GetOriginAnchor();
        auto& rotation = anchor.Orientation;

        CRadians cZAngle, cYAngle, cXAngle;
        rotation.ToEulerAngles(cZAngle, cYAngle, cXAngle);

        auto id = bot.GetConfigurationNode()->GetAttributeOrDefault(
            "index", "0");

        if (i > 0) agents_json += ",";
        agents_json += fmt::format(
            R"({{"id":{},"x":{:.2f},"y":{:.2f},"z":{:.2f},"rx":{:.2f},"ry":{:.2f},"rz":{:.2f}}})",
            id, anchor.Position.GetX(), anchor.Position.GetY(),
            anchor.Position.GetZ(), cXAngle.GetValue(), cYAngle.GetValue(),
            cZAngle.GetValue());
    }

    // Emit tick line
    fmt::print(
        R"({{"type":"tick","clock":{},"agents":[{}]}})",
        space.GetSimulationClock(), agents_json);
    fmt::print("\n");
    fflush(stdout);
}

void ExternalVisualizer::Init(TConfigurationNode& t_tree) {}

void ExternalVisualizer::Reset() {}

void ExternalVisualizer::Destroy() {}

void ExternalVisualizer::Execute() {
    auto& instance = CSimulator::GetInstance();
    while (!instance.IsExperimentFinished()) {
        auto t = instance.GetSpace().GetSimulationClock();
        if (t % 1 == 0) {
            ExternalVisualizer::Capture();
        }
        instance.UpdateSpace();
    }
}

REGISTER_VISUALIZATION(
    ExternalVisualizer,
    "external_visualizer",
    "LSMART Team",
    "0.0.1",
    "External Visualizer",
    "Streams tick frames to stdout for web-based visualization",
    "Experimental"
)
