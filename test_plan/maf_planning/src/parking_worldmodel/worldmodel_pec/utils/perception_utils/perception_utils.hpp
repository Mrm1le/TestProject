/*
 * perception_common.hpp
 *
 *  Created on: Nov 24, 2020
 *      Author: dozen
 */

#ifndef UTILS_PERCEPTION_VISION_BRUSH_PERCEPTION_COMMON_HPP_
#define UTILS_PERCEPTION_VISION_BRUSH_PERCEPTION_COMMON_HPP_
#include "maf_interface.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

class PerceptionCommon {
public:
  static int tagToCamIdOld(std::string tag) {
    if (tag == "fish_front") {
      return 0;
    } else if (tag == "fish_rr") {
      return 1;
    } else if (tag == "fish_rear") {
      return 2;
    } else if (tag == "fish_lr") {
      return 3;
    } else
      return -1;
  }

  static int tagToCamId(std::string tag) {
    if (tag == "Front195") {
      return 0;
    } else if (tag == "Right195") {
      return 1;
    } else if (tag == "Rear195") {
      return 2;
    } else if (tag == "Left195") {
      return 3;
    } else
      return -1;
  }

private:
  PerceptionCommon() = delete;
  ~PerceptionCommon(){};
};

#endif /* UTILS_PERCEPTION_VISION_BRUSH_PERCEPTION_COMMON_HPP_ */
