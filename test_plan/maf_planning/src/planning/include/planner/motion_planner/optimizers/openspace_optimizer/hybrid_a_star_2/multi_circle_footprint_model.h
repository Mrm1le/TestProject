#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../sbp_obstacle_interface.h"
#include "common/math/vec2d.h"
#include "common/utils/yaml_utils_parking.h"

namespace msquare {

namespace hybrid_a_star_2 {

class MultiCircleFootprintModel {
public:
  struct Circle {
    float center_x;
    float center_y;
    float radius;
  };

  struct Model {
    int index;
    std::string description;
    std::vector<int> include_model;
    std::vector<int> circle_index;
    ObstacleHeightType height_type;
  };

  MultiCircleFootprintModel() { deInit(); }

  ~MultiCircleFootprintModel() {}

  bool inited() const { return inited_; }
  const std::string &vehicleType() const { return vehicle_type_; }
  const std::vector<Model> &models() const { return models_; }
  const std::vector<Circle> &circles() const { return circles_local_; }
  const std::vector<Model> &modelsByHeight(ObstacleHeightType height) const {
    return models_by_heights_[int(height)];
  }

  void loadFromFile(const std::string &file_name) {
    std::ifstream ifs(file_name);
    if (!ifs.is_open()) {
      deInit();
      return;
    }
    ifs.seekg(0, std::ios::end);
    std::string buffer(std::size_t(ifs.tellg()), ' ');
    ifs.seekg(0);
    ifs.read(&buffer[0], buffer.size());
    loadFromContent(buffer);
  }

  void loadFromContent(const std::string &content) {
    deInit();
    if (loadFromContentInternal(content)) {
      inited_ = true;
    } else {
      deInit();
    }
  }

  void updatePose(const Pose2D &current_pose) {
    updatePose(current_pose.x, current_pose.y, std::cos(current_pose.theta),
               std::sin(current_pose.theta));
  }

  void updatePose(float x, float y, float cos_theta, float sin_theta) {
    for (std::size_t i = 0; i < circles_vehicle_.size(); i++) {
      circles_local_[i].center_x = circles_vehicle_[i].center_x * cos_theta -
                                   circles_vehicle_[i].center_y * sin_theta + x;
      circles_local_[i].center_y = circles_vehicle_[i].center_x * sin_theta +
                                   circles_vehicle_[i].center_y * cos_theta + y;
    }
  }

  void deInit() {
    inited_ = false;
    vehicle_type_ = "";
    param_version_ = "";
    circles_vehicle_.clear();
    circles_local_.clear();
    models_.clear();
    models_by_heights_.clear();
  }

private:
  bool loadFromContentInternal(const std::string &content) {
    YAML::Node yaml_config = YAML::Load(content);
    param_version_ = yaml_config["param_version"].as<std::string>();

    //[in dev] only support specific version
    std::string supported_version = "multi_circle_footprint_model_v1.1";
    if (param_version_ != supported_version) {
      return false;
    }

    vehicle_type_ = yaml_config["type"].as<std::string>();

    int circle_num = yaml_config["circle_num"].as<int>();
    circles_vehicle_.resize(circle_num);
    circles_local_.resize(circle_num);
    for (int i = 0; i < circle_num; i++) {
      std::string circle_name = std::string("circle") + std::to_string(i);
      circles_vehicle_[i].center_x = yaml_config[circle_name]["x"].as<float>();
      circles_vehicle_[i].center_y = yaml_config[circle_name]["y"].as<float>();
      circles_vehicle_[i].radius = yaml_config[circle_name]["r"].as<float>();
      circles_local_[i] = circles_vehicle_[i];
    }

    const int EXPECTED_MODEL_NUM = 3;

    std::vector<std::string> expected_descrption(EXPECTED_MODEL_NUM);
    expected_descrption[0] = "low_wheels";
    expected_descrption[1] = "not_allow_close";
    expected_descrption[2] = "full_body";

    int model_num = yaml_config["model_num"].as<int>();
    if (model_num != EXPECTED_MODEL_NUM) {
      return false;
    }

    models_.resize(model_num);
    for (int i = 0; i < model_num; i++) {
      Model &m = models_[i];
      m.index = i;
      std::string model_name = std::string("model") + std::to_string(i);
      m.description = yaml_config[model_name]["description"].as<std::string>();
      m.include_model =
          yaml_config[model_name]["include_model"].as<std::vector<int>>();
      m.circle_index = yaml_config[model_name]["index"].as<std::vector<int>>();

      if (m.include_model.size() > 1) {
        return false;
      }

      if (!m.include_model.empty()) {
        if (m.include_model[0] >= m.index) {
          // include model index must be less than model_index
          return false;
        }
      }

      if (m.description != expected_descrption[i]) {
        return false;
      }

      std::string height_type_str =
          yaml_config[model_name]["height"].as<std::string>();
      if (height_type_str == "LOW") {
        m.height_type = ObstacleHeightType::LOW;
      } else if (height_type_str == "MID") {
        m.height_type = ObstacleHeightType::MID;
      } else if (height_type_str == "HIGH") {
        m.height_type = ObstacleHeightType::HIGH;
      } else if (height_type_str == "HANGING_HIGH") {
        m.height_type = ObstacleHeightType::HANGING_HIGH;
      }
    }

    models_by_heights_.resize(int(ObstacleHeightType::HEIGHT_TYPE_NUM));
    for (std::size_t i = 0; i < models_.size(); i++) {
      models_by_heights_[int(models_[i].height_type)].push_back(models_[i]);
    }
    return true;
  }

  bool inited_;
  std::string vehicle_type_;
  std::string param_version_;

  // circles in vehicle coordinate system
  std::vector<Circle> circles_vehicle_;
  // circles in local coordinate system
  std::vector<Circle> circles_local_;

  std::vector<Model> models_;
  std::vector<std::vector<Model>> models_by_heights_;
};

} // namespace hybrid_a_star_2

} // namespace msquare
