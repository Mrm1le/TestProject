#ifndef OBSTACLE_HACKS_HPP
#define OBSTACLE_HACKS_HPP
#include <map>
#include <string>
#include <utility>

namespace msquare {

const std::map<long long, std::pair<bool, bool>> obstacle_hacks = {
    {3074840, {false, true}}, {3057120, {true, false}},
    {3080209, {false, true}}, {3074901, {true, true}},
    {3075289, {false, true}}, {3075254, {false, true}},
    {3076131, {true, true}},  {3076103, {false, true}},
    {3076742, {false, true}}, {3076594, {false, true}},
    {3076484, {true, false}}, {3075893, {false, true}},
    {3057109, {true, false}}};

} // namespace msquare

#endif
