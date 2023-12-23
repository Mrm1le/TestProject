/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * @file
 */

#pragma once

#include <omp.h>

#include "common/math/math_utils.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/State.hpp"
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace msquare {

struct ReedSheppPath {
  std::vector<double> segs_lengths;
  std::vector<char> segs_types;
  double total_length = 0.0;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  // true for driving forward and false for driving backward
  std::vector<int> gear;

  void reset() {
    total_length = 0.0;
    segs_lengths.clear();
    segs_types.clear();
    x.clear();
    y.clear();
    phi.clear();
    gear.clear();
  }
};

struct RSPParam {
  bool flag = false;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
};

class ReedShepp {
public:
  ReedShepp();
  ReedShepp(double max_kappa, double step_size);
  virtual ~ReedShepp() = default;
  bool ShortestRSP(const std::shared_ptr<SearchNode> start_node,
                   const std::shared_ptr<SearchNode> end_node,
                   std::shared_ptr<ReedSheppPath> optimal_path);
  bool AllRSPs(const std::shared_ptr<SearchNode> start_node,
               const std::shared_ptr<SearchNode> end_node,
               std::vector<ReedSheppPath> *all_possible_path);
  bool ShortestRSP(double start_x, double start_y, double start_theta,
                   double end_x, double end_y, double end_theta,
                   std::shared_ptr<ReedSheppPath> optimal_path);
  bool AllRSPs(double start_x, double start_y, double start_theta, double end_x,
               double end_y, double end_theta,
               std::vector<ReedSheppPath> *all_possible_traj);

protected:
  // Set the general profile of the movement primitives
  bool GenerateRSP(double start_x, double start_y, double start_theta,
                   double end_x, double end_y, double end_theta);
  // Set local exact configurations profile of each movement primitive
  bool GenerateLocalConfigurations(double start_x, double start_y,
                                   double start_theta, double end_x,
                                   double end_y, double end_theta,
                                   ReedSheppPath *shortest_path);

  // Interpolation usde in GenetateLocalConfiguration
  void Interpolation(const int index, const double pd, const char m,
                     const double ox, const double oy, const double ophi,
                     std::vector<double> *px, std::vector<double> *py,
                     std::vector<double> *pphi, std::vector<int> *pgear);
  // motion primitives combination setup function
  bool SetRSP(const int size, const double *lengths, const char *types);
  // setRSP parallel version
  bool SetRSPPar(const int size, const double *lengths,
                 const std::string &types, const int idx);
  // Six different combination of motion primitive in Reed Shepp path used in
  // GenerateRSP()
  bool SCS(const double x, const double y, const double phi);
  bool CSC(const double x, const double y, const double phi);
  bool CCC(const double x, const double y, const double phi);
  bool CCCC(const double x, const double y, const double phi);
  bool CCSC(const double x, const double y, const double phi);
  bool CCSCC(const double x, const double y, const double phi);
  // different options for different combination of motion primitives
  void LSL(const double x, const double y, const double phi, RSPParam *param);
  void LSR(const double x, const double y, const double phi, RSPParam *param);
  void LRL(const double x, const double y, const double phi, RSPParam *param);
  void SLS(const double x, const double y, const double phi, RSPParam *param);
  void LRLRn(const double x, const double y, const double phi, RSPParam *param);
  void LRLRp(const double x, const double y, const double phi, RSPParam *param);
  void LRSR(const double x, const double y, const double phi, RSPParam *param);
  void LRSL(const double x, const double y, const double phi, RSPParam *param);
  void LRSLR(const double x, const double y, const double phi, RSPParam *param);
  std::pair<double, double> calc_tau_omega(const double u, const double v,
                                           const double xi, const double eta,
                                           const double phi);

protected:
  std::vector<ReedSheppPath> all_possible_paths_;
  int paths_max_size_ = 48;
  size_t paths_size_ = 0;
  double step_size_;
  double max_kappa_;
};

} // namespace msquare
