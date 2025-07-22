#pragma once
#include "common.h"

enum class SMARTGridType {
    REGULAR,            // Regular grid
    ONE_BOT_PER_AISLE,  // At most one bot per endpoint paisle
};

static boost::unordered_map<std::string, SMARTGridType> const convert_G_type = {
    {"regular", SMARTGridType::REGULAR},
    {"one_bot_per_aisle", SMARTGridType::ONE_BOT_PER_AISLE}};