#pragma once

#include "pnc/define/planner_constants.hpp"
#include <array>

// Fifth order Gauss quadrature for high precision numerical integration
// Gauss quadrature point is shifted from [-1, 1] to [0, 1] range
// Ref1: https://cloud.tencent.com/developer/article/1625372
// Ref2: http://bbs.fcode.cn/thread-219-1-1.html

// clang-format off

// Gausss quadrature point sampling position
static constexpr std::array<double, path_planner::QUADRATURE_ORDER> GAUSS_QUAD_5TH_POS = {
    0.5 * (1.0 - 0.9061798459),    
    0.5 * (1.0 - 0.5384693101),    
    0.5 * 1.0,    
    0.5 * (1.0 + 0.5384693101),    
    0.5 * (1.0 + 0.9061798459),
};

static constexpr std::array<double, path_planner::QUADRATURE_ORDER + 2> GAUSS_QUAD_5TH_POS_WITH_END = {
    0.0,
    0.5 * (1.0 - 0.9061798459),    
    0.5 * (1.0 - 0.5384693101),    
    0.5 * 1.0,    
    0.5 * (1.0 + 0.5384693101),    
    0.5 * (1.0 + 0.9061798459),
    1.0,
};

// Guass quadrature point sampling weight
static constexpr std::array<double, path_planner::QUADRATURE_ORDER> GAUSS_QUAD_5TH_WT = {
    0.5 * 0.2369268850,    
    0.5 * 0.4786286705,    
    0.5 * 0.5688888888,    
    0.5 * 0.4786286705,    
    0.5 * 0.2369268850,
};

// clang-format on