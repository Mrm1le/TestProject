#pragma once

namespace msquare {
static constexpr double MIN_DIST_TO_INTSECT = 100.0;
static constexpr double MIN_DIST_TO_RAMP = 1000.0;
static constexpr double MIN_ALC_SPEED = 80.0;
static constexpr double EFFICIENCY_COST_FILTER_TIME = 2.0;
static constexpr double BLOCKED_UNIT_COST = 1.0;
static constexpr double CONSERVATIVE_UNIT_COST = 3.5;
static constexpr double MIN_SAFE_DIST = 5.0;
static constexpr double NONE_TO_TRIGGER_THRESH = 7.0;
static constexpr double BALANCE_FACTOR = 0.6;
static constexpr double MIN_SAFE_BACK_ONE_TTC = 8.0;
static constexpr double CAR_FOLLOWING_FACTOR = 0.85;
static constexpr double BASE_TTC_FACTOR = 1.5;
static constexpr double MIN_WAIT_TIME_AFTER_CHANGE = 30.0;
} // namespace msquare