
#include "common/planning_context.h"

#include "common/utils/yaml_utils.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace msquare {

namespace parking {

class ParkingConfigLoader {
private:
  ParkingConfigLoader();
  ~ParkingConfigLoader();

public:
  //[fenix.refactor.sm] moved from parking_scenario_manager.cpp
  static bool
  load_config(StatusType status_type,
              uint8_t parking_slot_type = ParkingSlotType::PERPENDICULAR) {
    std::string config_file;

    if (status_type == StatusType::APA) {
      if (parking_slot_type == ParkingSlotType::PARALLEL) {
        if (CarParams::GetInstance()
                ->car_config.common_config.use_sop_openspace_planner_parallel) {
          config_file = PlanningConfig::Instance()
                            ->config_files()
                            .openspace_apa_parallel_sop_planner_config_file;
        } else {
          config_file = PlanningConfig::Instance()
                            ->config_files()
                            .openspace_apa_parallel_planner_config_file;
        }
      } else {
        if (CarParams::GetInstance()
                ->car_config.common_config.use_sop_openspace_planner) {
          config_file = PlanningConfig::Instance()
                            ->config_files()
                            .openspace_apa_sop_planner_config_file;
        } else {
          config_file = PlanningConfig::Instance()
                            ->config_files()
                            .openspace_apa_planner_config_file;
        }
      }
    } else if (status_type == StatusType::APOA) {
      if (CarParams::GetInstance()->car_config.apoa_config.use_legacy_parkout) {
        if (parking_slot_type == ParkingSlotType::PARALLEL) {
          config_file = PlanningConfig::Instance()
                            ->config_files()
                            .legacy_openspace_apoa_parallel_planner_config_file;
        } else {
          config_file = PlanningConfig::Instance()
                            ->config_files()
                            .legacy_openspace_apoa_planner_config_file;
        }
      } else {
        config_file = parking_slot_type != ParkingSlotType::PARALLEL
                          ? PlanningConfig::Instance()
                                ->config_files()
                                .openspace_apoa_planner_config_file
                          : PlanningConfig::Instance()
                                ->config_files()
                                .openspace_apoa_parallel_planner_config_file;
    }
    } else if (status_type == StatusType::RPA_STRAIGHT) {
      config_file = PlanningConfig::Instance()
                        ->config_files()
                        .openspace_rpa_straight_planner_config_file;
    } else {
      config_file = PlanningConfig::Instance()
                        ->config_files()
                        .openspace_avp_planner_config_file;
    }
    // std::cout << "hzmdebug: openspace config is " << config_file <<
    // std::endl;

    if (!HybridAstarConfig::GetInstance()->loadFile(config_file)) {
      MSD_LOG(INFO, "openspace hybrid-astar config file load failed!\n");
      return false;
    }
    if (!TrajectoryOptimizerConfig::GetInstance()->loadFile(config_file)) {
      MSD_LOG(INFO, "openspace OBCA config file load failed!\n");
      return false;
    }
    if (!CarParams::GetInstance()->loadFile4Plan(config_file)) {
      MSD_LOG(INFO, "openspace CarParams config file load failed!\n");
      return false;
    }
    if (!StrategyParams::GetInstance()->loadFile(config_file)) {
      MSD_LOG(INFO, "openspace strategy params config file load failed!\n");
      return false;
    }

    auto config_file_dir = PlanningContext::Instance()->get_config_file_dir();
    auto scenario_config_file_dir = config_file_dir + "/scenario_configs_json";
    if (!CarParams::GetInstance()->loadFile4Car(scenario_config_file_dir)) {
      MSD_LOG(INFO, "car config file load failed!\n");
      return false;
    }
    const auto &car_type = VehicleParam::Instance()->car_type;
    const auto &architecture = VehicleParam::Instance()->architecture;
    MSD_LOG(ERROR, "loadFile4Car: car_type: %s, architecture: %s",
            car_type.c_str(), architecture.c_str());

    rewriteKino(status_type, parking_slot_type);
    MSD_LOG(ERROR, "vel_backward:%f, vel_forward:%f",   
      msquare::TrajectoryOptimizerConfig::GetInstance()->param_max_speed_reverse,
      msquare::TrajectoryOptimizerConfig::GetInstance()->param_max_speed_forward);

    return true;
  }

  //[fenix.refactor.sm] moved from parking_scenario_manager.cpp
  static bool load_params(const std::string config_file_dir) {
    ConfigFiles *config_files =
        PlanningConfig::Instance()->mutable_config_files();
    config_files->parking_task_config_file =
        config_file_dir + "/parking/parking_task_config.yaml";
    config_files->openspace_decider_config_file =
        config_file_dir + "/parking/openspace_decider.yaml";
    config_files->openspace_apa_planner_config_file =
        config_file_dir + "/parking/apa.yaml";
    config_files->openspace_apa_sop_planner_config_file =
        config_file_dir + "/parking/apa_sop.yaml";
    config_files->openspace_avp_planner_config_file =
        config_file_dir + "/parking/avp.yaml";
    config_files->openspace_apoa_planner_config_file =
        config_file_dir + "/parking/apoa_sop.yaml";
    config_files->openspace_apa_parallel_planner_config_file =
        config_file_dir + "/parking/apa_parallel.yaml";
    config_files->openspace_apa_parallel_sop_planner_config_file =
        config_file_dir + "/parking/apa_parallel_sop.yaml";
    config_files->openspace_apoa_parallel_planner_config_file =
        config_file_dir + "/parking/apoa_parallel_sop.yaml";
    config_files->legacy_openspace_apoa_planner_config_file =
        config_file_dir + "/parking/apoa.yaml";
    config_files->legacy_openspace_apoa_parallel_planner_config_file =
        config_file_dir + "/parking/apoa_parallel.yaml";
    config_files->openspace_rpa_straight_planner_config_file =
        config_file_dir + "/parking/rpa_straight.yaml";
    //   config_files->log_path = "/home/ros/Downloads/pnc_logs/t_line/";

    char *calib_dir_env = std::getenv("CALIB_DIR");

    if (calib_dir_env == nullptr) {
      config_files->vehicle_param_file =
          config_file_dir + "/parking/vehicle_param.yaml";
    } else {
      std::string calib_dir(calib_dir_env);
      std::string vehicle_param_path = calib_dir + "/vehicle.yaml";
      MSD_LOG(INFO, "VehicleParam: CALIB_DIR %s yaml dir %s", calib_dir.c_str(),
              vehicle_param_path.c_str());
      if (access(vehicle_param_path.c_str(), F_OK) == -1) {
        MSD_LOG(ERROR, "VehicleParam: vehicle.yaml not exist!");
        config_files->vehicle_param_file =
            config_file_dir + "/parking/vehicle_param.yaml";
      } else {
        config_files->vehicle_param_file = vehicle_param_path;
      }
    }
    config_files->teb_config_file =
        config_file_dir + "/parking/teb_local_planner_params.yaml";
    config_files->teb_footprint_file =
        config_file_dir + "/parking/footprint_model.yaml";
    config_files->parking_lot_config_file =
        config_file_dir + "/parking/parking_lot.yaml";
    config_files->parallel_parking_slot_config_file =
        config_file_dir + "/parking/parallel_parking_slot.yml";
    config_files->lot_config_pre_apa_file =
        config_file_dir + "/parking/lot_config_pre_apa.yaml";
    config_files->state_parking_config_file =
        config_file_dir + "/parking/state_parking_config.yaml";
    config_files->trajectory_file =
        config_file_dir + "/parking/keyframe_ts.txt";
    config_files->parking_lateral_behavior_planner_config_file =
        config_file_dir + "/parking/lateral_behavior_planner_config.yaml";
    config_files->teb_openspace_decider_config_file =
        config_file_dir + "/parking/teb_openspace_decider_config.yaml";
    config_files->via_point_decider_config_file =
        config_file_dir + "/parking/via_point_decider_config.yaml";
    config_files->apf_decider_config_file =
        config_file_dir + "/parking/apf_decider_config.yaml";

    if (!ParkingLateralBehaviorPlannerConfig::Instance()->loadFile(
            PlanningConfig::Instance()
                ->config_files()
                .parking_lateral_behavior_planner_config_file)) {
      MSD_LOG(
          ERROR,
          "std::logic_error: CarParams load from vehicle_param_file failed!");
    }

    if (!TebOpenspaceDeciderConfig::Instance()->loadFile(
            PlanningConfig::Instance()
                ->config_files()
                .teb_openspace_decider_config_file)) {
      MSD_LOG(
          ERROR,
          "std::logic_error: CarParams load from vehicle_param_file failed!");
    }

    if (!ViaPointDeciderConfig::Instance()->loadFile(
            PlanningConfig::Instance()
                ->config_files()
                .via_point_decider_config_file)) {
      MSD_LOG(
          ERROR,
          "std::logic_error: CarParams load from vehicle_param_file failed!");
    }

    if (!CarParams::GetInstance()->loadFile(
            PlanningConfig::Instance()->config_files().vehicle_param_file)) {
      MSD_LOG(
          ERROR,
          "std::logic_error: CarParams load from vehicle_param_file failed!");
    }

    if (!VehicleParam::Instance()->loadFile(
            PlanningConfig::Instance()->config_files().vehicle_param_file)) {
      MSD_LOG(ERROR, "std::logic_error: VehicleParam load from "
                     "vehicle_param_file failed!");
    }

    if (!ApfDeciderConfig::Instance()->loadFile(PlanningConfig::Instance()
                                                    ->config_files()
                                                    .apf_decider_config_file)) {
      MSD_LOG(ERROR, "std::logic_error: apf decider config load from "
                     "apf_decider_config_file failed!");
    }
    if (!CarParams::GetInstance()->loadFile4Car(config_file_dir)) {
      MSD_LOG(ERROR, "car config file load failed!\n");
      return false;
    }

    return true;
  }

static void rewriteKino(StatusType status_type, uint8_t parking_slot_type){
  msquare::KinoDynamicConfig kino_config;
    if (status_type == StatusType::APA) {
        if(parking_slot_type == ParkingSlotType::PARALLEL){
            kino_config = CarParams::GetInstance()->car_config.kino_config.apa_parallel;
        }else if(parking_slot_type == ParkingSlotType::PERPENDICULAR){
            kino_config = CarParams::GetInstance()->car_config.kino_config.apa_vertical;
        }else if(parking_slot_type == ParkingSlotType::OBLIQUE){
            kino_config = CarParams::GetInstance()->car_config.kino_config.apa_oblique;
        }else{
            kino_config = CarParams::GetInstance()->car_config.kino_config.apa_vertical;
        }
    } else if (status_type == StatusType::APOA) {
        if(parking_slot_type == ParkingSlotType::PARALLEL){
            kino_config = CarParams::GetInstance()->car_config.kino_config.apoa_parallel;
        }else if(parking_slot_type == ParkingSlotType::PERPENDICULAR){
            kino_config = CarParams::GetInstance()->car_config.kino_config.apoa_vertical;
        }else if(parking_slot_type == ParkingSlotType::OBLIQUE){
            kino_config = CarParams::GetInstance()->car_config.kino_config.apoa_oblique;
        }else{
            kino_config = CarParams::GetInstance()->car_config.kino_config.apoa_vertical;
        }
    } else if (status_type == StatusType::RPA_STRAIGHT) {
        kino_config = CarParams::GetInstance()->car_config.kino_config.rpa;
    } else {
        kino_config = CarParams::GetInstance()->car_config.kino_config.apa_vertical;
    }
    msquare::TrajectoryOptimizerConfig::GetInstance()->param_max_speed_reverse = kino_config.vel_backward;
    msquare::TrajectoryOptimizerConfig::GetInstance()->param_max_speed_forward = kino_config.vel_forward;
}
};

} // namespace parking

} // namespace msquare
