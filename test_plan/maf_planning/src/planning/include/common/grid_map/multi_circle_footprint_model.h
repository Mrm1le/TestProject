#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "common/math/vec2d.h"
#include "common/utils/yaml_utils.h"

#include "planner/motion_planner/optimizers/openspace_optimizer/sbp_obstacle_interface.h"

//#include "common/grid_map/util.h"



#ifdef HASTAR2_HAS_INTERNAL_MODEL
static std::string CarConfigsMultiCircleFootprintModelLacarYamlContent() {
  extern std::uint8_t start[] asm(
      "_binary_car_configs_multi_circle_footprint_model_lacar_yaml_start");
  extern std::uint8_t end[] asm(
      "_binary_car_configs_multi_circle_footprint_model_lacar_yaml_end");
  return std::string(start, end);
}
static std::string CarConfigsMultiCircleFootprintModelWlsC03YamlContent() {
  extern std::uint8_t start[] asm(
      "_binary_car_configs_multi_circle_footprint_model_wls_c03_yaml_start");
  extern std::uint8_t end[] asm(
      "_binary_car_configs_multi_circle_footprint_model_wls_c03_yaml_end");
  return std::string(start, end);
}
static std::string CarConfigsMultiCircleFootprintModelLhsSgYamlContent() {
  extern std::uint8_t start[] asm(
      "_binary_car_configs_multi_circle_footprint_model_lhs_sg_yaml_start");
  extern std::uint8_t end[] asm(
      "_binary_car_configs_multi_circle_footprint_model_lhs_sg_yaml_end");
  return std::string(start, end);
}
static std::string CarConfigsMultiCircleFootprintModelLhsUxeYamlContent() {
  extern std::uint8_t start[] asm(
      "_binary_car_configs_multi_circle_footprint_model_lhs_uxe_yaml_start");
  extern std::uint8_t end[] asm(
      "_binary_car_configs_multi_circle_footprint_model_lhs_uxe_yaml_end");
  return std::string(start, end);
}
static std::string CarConfigsMultiCircleFootprintModelLbcarYamlContent() {
  extern std::uint8_t start[] asm(
      "_binary_car_configs_multi_circle_footprint_model_lbcar_yaml_start");
  extern std::uint8_t end[] asm(
      "_binary_car_configs_multi_circle_footprint_model_lbcar_yaml_end");
  return std::string(start, end);
}
static std::string CarConfigsMultiCircleFootprintModelBysA02YamlContent() {
  extern std::uint8_t start[] asm(
      "_binary_car_configs_multi_circle_footprint_model_bys_a02_yaml_start");
  extern std::uint8_t end[] asm(
      "_binary_car_configs_multi_circle_footprint_model_bys_a02_yaml_end");
  return std::string(start, end);
}
static std::string CarConfigsMultiCircleFootprintModelHarzS450LYamlContent() {
  extern std::uint8_t start[] asm(
      "_binary_car_configs_multi_circle_footprint_model_harz_s450l_yaml_start");
  extern std::uint8_t end[] asm(
      "_binary_car_configs_multi_circle_footprint_model_harz_s450l_yaml_end");
  return std::string(start, end);
}
static std::string CarConfigsMultiCircleFootprintModelRockyLyriqYamlContent() {
  extern std::uint8_t start[] asm("_binary_car_configs_multi_circle_footprint_"
                                  "model_rocky_lyriq_yaml_start");
  extern std::uint8_t end[] asm(
      "_binary_car_configs_multi_circle_footprint_model_rocky_lyriq_yaml_end");
  return std::string(start, end);
}
static std::string CarConfigsMultiCircleFootprintModelLhsEseaYamlContent() {
  extern std::uint8_t start[] asm(
      "_binary_car_configs_multi_circle_footprint_model_lhs_esea_yaml_start");
  extern std::uint8_t end[] asm(
      "_binary_car_configs_multi_circle_footprint_model_lhs_esea_yaml_end");
  return std::string(start, end);
}
static std::string CarConfigsMultiCircleFootprintModelLccarYamlContent() {
  extern std::uint8_t start[] asm(
      "_binary_car_configs_multi_circle_footprint_model_lccar_yaml_start");
  extern std::uint8_t end[] asm(
      "_binary_car_configs_multi_circle_footprint_model_lccar_yaml_end");
  return std::string(start, end);
}
static std::string HeuristicOfflineBoundaryCostBinContent() {
  extern std::uint8_t start[] asm(
      "_binary_heuristic_offline_boundary_cost_bin_start");
  extern std::uint8_t end[] asm(
      "_binary_heuristic_offline_boundary_cost_bin_end");
  return std::string(start, end);
}
#else  // HASTAR2_HAS_INTERNAL_MODEL
static std::string CarConfigsMultiCircleFootprintModelLacarYamlContent() {
  return std::string();
}
static std::string CarConfigsMultiCircleFootprintModelWlsC03YamlContent() {
  return std::string();
}
static std::string CarConfigsMultiCircleFootprintModelLhsSgYamlContent() {
  return std::string();
}
static std::string CarConfigsMultiCircleFootprintModelLhsUxeYamlContent() {
  return std::string();
}
static std::string CarConfigsMultiCircleFootprintModelLbcarYamlContent() {
  return std::string();
}
static std::string CarConfigsMultiCircleFootprintModelBysA02YamlContent() {
  return std::string();
}
static std::string CarConfigsMultiCircleFootprintModelHarzS450LYamlContent() {
  return std::string();
}
static std::string CarConfigsMultiCircleFootprintModelRockyLyriqYamlContent() {
  return std::string();
}
static std::string CarConfigsMultiCircleFootprintModelLhsEseaYamlContent() {
  return std::string();
}
static std::string CarConfigsMultiCircleFootprintModelLccarYamlContent() {
  return std::string();
}
static std::string HeuristicOfflineBoundaryCostBinContent() {
  return std::string();
}
#endif // HASTAR2_HAS_INTERNAL_MODEL


namespace msquare {
namespace grid {

class MultiCircleFootprintModel {
public:
  struct Circle {
    planning_math::Vec2d center_;
    double radius_;
  };

  struct Model {
    int model_index_;
    std::string description_;
    std::vector<int> include_model_;
    std::vector<int> circle_index_;
    ObstacleHeightType height_type_;
  };

  bool inited_;
  int circle_num_;
  // circles in ego vehicle coordinate
  std::vector<Circle> circles_vehicle_;

  // circles in local coordinate (as geometry cache)
  std::vector<Circle> circles_local_;

  std::string vehicle_type_;
  std::string param_version_;

  int model_num_;
  std::vector<Model> models_;
  std::vector<std::vector<Model>> models_by_heights_;

public:
  MultiCircleFootprintModel(const std::string &config_folder = std::string()) {
    circle_num_ = 0;
    model_num_ = 0;
    inited_ = false;

  bool load_result = false;
  if (CarParams::GetInstance()->type == "C03") {
    if (config_folder == "") {
      load_result = this->loadFromContent(
          CarConfigsMultiCircleFootprintModelWlsC03YamlContent());
    } else {
      load_result = this->loadFromFile(
          config_folder +
          "car_configs/multi_circle_footprint_model_wls_c03.yaml");
    }
  } else if (CarParams::GetInstance()->type == "SG") {
    if (config_folder == "") {
      load_result = this->loadFromContent(
          CarConfigsMultiCircleFootprintModelLhsSgYamlContent());
    } else {
      load_result = this->loadFromFile(
          config_folder +
          "car_configs/multi_circle_footprint_model_lhs_sg.yaml");
    }
  } else if (CarParams::GetInstance()->type == "UXE") {
    if (config_folder == "") {
      load_result = this->loadFromContent(
          CarConfigsMultiCircleFootprintModelLhsUxeYamlContent());
    } else {
      load_result = this->loadFromFile(
          config_folder +
          "car_configs/multi_circle_footprint_model_lhs_uxe.yaml");
    }
  } else if (CarParams::GetInstance()->type == "LS7") {
    if (config_folder == "") {
      load_result = this->loadFromContent(
          CarConfigsMultiCircleFootprintModelLbcarYamlContent());
    } else {
      load_result = this->loadFromFile(
          config_folder +
          "car_configs/multi_circle_footprint_model_lbcar.yaml");
    }
  } else if (CarParams::GetInstance()->type == "A02") {
    if (config_folder == "") {
      load_result = this->loadFromContent(
          CarConfigsMultiCircleFootprintModelBysA02YamlContent());
    } else {
      load_result = this->loadFromFile(
          config_folder +
          "car_configs/multi_circle_footprint_model_bys_a02.yaml");
    }
  } else if (CarParams::GetInstance()->type == "S450L") {
    if (config_folder == "") {
      load_result = this->loadFromContent(
          CarConfigsMultiCircleFootprintModelHarzS450LYamlContent());
    } else {
      load_result = this->loadFromFile(
          config_folder +
          "car_configs/multi_circle_footprint_model_harz_s450l.yaml");
    }
  } else if (CarParams::GetInstance()->type == "LYRIQ") {
    if (config_folder == "") {
      load_result = this->loadFromContent(
          CarConfigsMultiCircleFootprintModelRockyLyriqYamlContent());
    } else {
      load_result = this->loadFromFile(
          config_folder +
          "car_configs/multi_circle_footprint_model_rocky_lyriq.yaml");
    }
  } else if (CarParams::GetInstance()->type == "ESEA") {
    if (config_folder == "") {
      load_result = this->loadFromContent(
          CarConfigsMultiCircleFootprintModelLhsEseaYamlContent());
    } else {
      load_result = this->loadFromFile(
          config_folder +
          "car_configs/multi_circle_footprint_model_lhs_esea.yaml");
    }
  } else if (CarParams::GetInstance()->type == "LC6") {
    if (config_folder == "") {
      load_result = this->loadFromContent(
          CarConfigsMultiCircleFootprintModelLccarYamlContent());
    } else {
      load_result = this->loadFromFile(
          config_folder +
          "car_configs/multi_circle_footprint_model_lccar.yaml");
    }
  } else {
    // TODO: Jinwei: Lacar by default, might be confusing
    if (config_folder == "") {
      load_result = this->loadFromContent(
          CarConfigsMultiCircleFootprintModelLacarYamlContent());
    } else {
      load_result = this->loadFromFile(
          config_folder +
          "car_configs/multi_circle_footprint_model_lacar.yaml");
    }
  }

  // TODO(ckl): do not have 
  updateConfig();
  }

  ~MultiCircleFootprintModel() {}

  void updateConfig() {
    if (this->vehicle_type_ !=
        VehicleParam::Instance()->car_type) {
    }
  }

  bool loadFromFile(const std::string &file_name) {
    std::ifstream ifs(file_name);
    if (!ifs.is_open()) {
      return false;
    }
    ifs.seekg(0, std::ios::end);
    std::string buffer(std::size_t(ifs.tellg()), ' ');
    ifs.seekg(0);
    ifs.read(&buffer[0], buffer.size());
    return loadFromContent(buffer);
  }

  bool loadFromContent(const std::string &content) {
    YAML::Node yaml_config = YAML::Load(content);
    param_version_ = yaml_config["param_version"].as<std::string>();

    //[in dev] only support specific version
    std::string supported_version = "multi_circle_footprint_model_v1.1";
    if (param_version_ != supported_version) {
      return false;
    }

    vehicle_type_ = yaml_config["type"].as<std::string>();

    circle_num_ = yaml_config["circle_num"].as<int>();
    circles_vehicle_.clear();
    circles_local_.clear();
    for (int i = 0; i < circle_num_; i++) {
      Circle c;
      std::string circle_name = std::string("circle") + std::to_string(i);
      c.center_.set_x(yaml_config[circle_name]["x"].as<double>());
      c.center_.set_y(yaml_config[circle_name]["y"].as<double>());
      c.radius_ = yaml_config[circle_name]["r"].as<double>();
      circles_vehicle_.push_back(c);
      circles_local_.push_back(c);
    }

    const int EXPECTED_MODEL_NUM = 3;

    std::vector<std::string> expected_descrption(EXPECTED_MODEL_NUM);
    expected_descrption[0] = "low_wheels";
    expected_descrption[1] = "not_allow_close";
    expected_descrption[2] = "full_body";
    
    model_num_ = yaml_config["model_num"].as<int>();

    if (model_num_ != EXPECTED_MODEL_NUM) {
      return false;
    }

    for (int i = 0; i < model_num_; i++) {
      Model m;
      m.model_index_ = i;
      std::string model_name = std::string("model") + std::to_string(i);
      m.description_ = yaml_config[model_name]["description"].as<std::string>();
      m.include_model_ =
          yaml_config[model_name]["include_model"].as<std::vector<int>>();
      m.circle_index_ = yaml_config[model_name]["index"].as<std::vector<int>>();
      
      if(m.include_model_.size() >1){
        return false;
      }

      if (!m.include_model_.empty()) {
        if (m.include_model_[0] >= m.model_index_) {
          //include model index must be less than model_index
          return false;
        }
      }

      if(m.description_ != expected_descrption[i]){
        return false;
      }

      std::string height_type_str = yaml_config[model_name]["height"].as<std::string>();

      if(height_type_str == "LOW"){
        m.height_type_ = ObstacleHeightType::LOW;
      }
      else if(height_type_str == "MID"){
        m.height_type_ = ObstacleHeightType::MID;
      }
      else if(height_type_str == "HIGH"){
        m.height_type_ = ObstacleHeightType::HIGH;
      }
      else if(height_type_str == "HANGING_HIGH"){
        m.height_type_ = ObstacleHeightType::HANGING_HIGH;
      }    
      models_.push_back(m);
    }

    models_by_heights_.resize((int)ObstacleHeightType::HEIGHT_TYPE_NUM);
    
    for (int i = 0; i < model_num_; i++) {
      ObstacleHeightType height = models_[i].height_type_; 
      models_by_heights_[(int)height].push_back(models_[i]);
    }

    return true;
  }

  bool isInited() const { return inited_; }

  bool updatePose(const Pose2D &current_pose) {
    return updatePose(current_pose.x, current_pose.y,
                      std::cos(current_pose.theta),
                      std::sin(current_pose.theta));
  }

  bool updatePose(double x, double y, double cos_theta, double sin_theta) {
    for (int i = 0; i < circle_num_; i++) {
      double new_x = circles_vehicle_[i].center_.x() * cos_theta -
                     circles_vehicle_[i].center_.y() * sin_theta + x;
      double new_y = circles_vehicle_[i].center_.x() * sin_theta +
                     circles_vehicle_[i].center_.y() * cos_theta + y;
      circles_local_[i].center_.set_point(new_x, new_y);
    }
    return true;
  }
  //a deep clone function is needed because this class contains vectors
  MultiCircleFootprintModel clone() const
  {

    MultiCircleFootprintModel result;
    result.param_version_ = param_version_;
    result.circle_num_ = circle_num_;
    result.vehicle_type_ = vehicle_type_;
    result.circles_vehicle_.resize(circle_num_);
    result.circles_local_.resize(circle_num_);
    for(int i=0; i<circle_num_; i++)
    {    
      result.circles_vehicle_[i] = circles_vehicle_[i];
      result.circles_local_[i] = circles_local_[i];
    }
    result.model_num_ = model_num_;
    result.models_.resize(model_num_);
    for (int i = 0; i < model_num_; i++) {
      result.models_[i] = models_[i];
    }
    result.models_by_heights_.resize(models_by_heights_.size());
    for (std::size_t i = 0; i < models_by_heights_.size(); i++) {
      result.models_by_heights_[i].assign(models_by_heights_[i].begin(),models_by_heights_[i].end());
    }
    
    result.inited_ = true;
    return result;
  }
};

} // namespace grid
} // namespace msquare
