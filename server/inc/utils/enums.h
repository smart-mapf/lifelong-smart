#pragma once
// #include "utils/common.h"

enum heuristics_type { ZERO, CG, DG, WDG, STRATEGY_COUNT };

enum class SMARTGridType {
    REGULAR,            // Regular grid
    ONE_BOT_PER_AISLE,  // At most one bot per endpoint paisle
};

enum MobileAction {
    // DELIVER = 0, PICK = 1, NONE = 2
    NONE = 0,
    PICK = 1,
    DELIVER = 2,
    DONE = 3
};

static boost::unordered_map<std::string, SMARTGridType> const convert_G_type = {
    {"regular", SMARTGridType::REGULAR},
    {"one_bot_per_aisle", SMARTGridType::ONE_BOT_PER_AISLE}};