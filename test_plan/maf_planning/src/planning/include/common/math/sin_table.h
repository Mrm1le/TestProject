/**
 * @file
 * @brief Exports the SIN_TABLE, used by the Angle class.
 */

#pragma once

namespace msquare {
namespace planning_math {

//! Used by Angle class to speed-up computation of trigonometric functions.
#define SIN_TABLE_SIZE 16385
extern const float SIN_TABLE[SIN_TABLE_SIZE];

} // namespace planning_math
} // namespace msquare
