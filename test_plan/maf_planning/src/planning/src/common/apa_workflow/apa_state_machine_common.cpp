#include "common/apa_workflow/apa_state_machine.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planning/common/common.h"
#ifdef USE_CONFIG_SERVICE
#include <mpf/core/parameter/parameter.h>
#endif

#include "common/apa_workflow/parking_config_loader.hpp"

namespace msquare {

namespace parking {

ApaStateMachine::ApaStateMachine(const std::shared_ptr<WorldModel> &world_model)
    : world_model_(world_model) {

  //[fenix.refactor.sm] Original ScenarioMannger construct functions
  // auto config_file_dir = SCENARIO_CONFIG_JSON_FILE_DIR;
  auto config_file_dir = PlanningContext::Instance()->get_config_file_dir();
  auto scenario_config_file_dir = config_file_dir + "/scenario_configs_json";
  (void)ParkingConfigLoader::load_params(scenario_config_file_dir);

  YAML::Node state_parking_config_node = YAML::LoadFile(
      PlanningConfig::Instance()->config_files().state_parking_config_file);
  state_parking_cfg_.apa_min_adjust_threshold.x =
      state_parking_config_node["apa_min_adjust_threshold"]["x"].as<double>();
  state_parking_cfg_.apa_min_adjust_threshold.y =
      state_parking_config_node["apa_min_adjust_threshold"]["y"].as<double>();
  state_parking_cfg_.apa_min_adjust_threshold.theta =
      state_parking_config_node["apa_min_adjust_threshold"]["theta"]
          .as<double>();
  state_parking_cfg_.apa_psd_drift_threshold.x =
      state_parking_config_node["apa_psd_drift_threshold"]["x"].as<double>();
  state_parking_cfg_.apa_psd_drift_threshold.y =
      state_parking_config_node["apa_psd_drift_threshold"]["y"].as<double>();
  state_parking_cfg_.apa_psd_drift_threshold.theta =
      state_parking_config_node["apa_psd_drift_threshold"]["theta"]
          .as<double>();
  state_parking_cfg_.apa_simple_mpc_entry_threshold =
      state_parking_config_node["apa_simple_mpc_entry_threshold"].as<double>();
  state_parking_cfg_.mpc_predict_threshold.x =
      state_parking_config_node["mpc_predict_threshold"]["x"].as<double>();
  state_parking_cfg_.mpc_predict_threshold.y =
      state_parking_config_node["mpc_predict_threshold"]["y"].as<double>();
  state_parking_cfg_.mpc_predict_threshold.theta =
      state_parking_config_node["mpc_predict_threshold"]["theta"].as<double>();

  (void)ParkingConfigLoader::load_config(StatusType::AVP);

  parking_longitudinal_behavior_planner_ =
      std::make_shared<ParkingLongitudinalBehaviorPlanner>(world_model_);
  parking_longitudinal_motion_planner_ =
      std::make_unique<ParkingLongitudinalMotionPlanner>(world_model_);

  // init lon_behavior_planning
  PlanningContext::Instance()->lon_mc_footprint_model() =
      std::make_shared<grid::MultiCircleFootprintModel>(
          PlanningContext::Instance()->get_config_file_dir() +
          "/scenario_configs_json/parking/");

  leader_decider_ = std::make_unique<LeaderDecider>(world_model_);
  //   freespace_decider_ = std::make_unique<FreespaceDecider>(world_model_);
  // openspace_motion_planner_ =
  //     std::make_unique<OpenspaceMotionPlanner>(world_model_);
  // teb_openspace_decider_ =
  // std::make_unique<TebOpenspaceDecider>(world_model_); teb_motion_planner_ =
  // std::make_unique<TebMotionPlanner>(world_model_);

  //[fenix.refactor.sm] END Original ScenarioMannger construct functions

  path_sampler_ = std::make_shared<PathSampler>(world_model_);

  collision_checker_ = CollisionChecker();
  parking_task_config_ = ParkingTaskConfig();
  // apa_key_point_selector_ = ApaKeyPointSelector(world_model);
  parking_slot_manager_ = ParkingSlotManager(world_model);

  openspace_state_machine_ =
      std::make_unique<openspace_state_machine::OpenspaceStateMachine>(
          world_model, path_sampler_);

  behavior_decider_parkin_ = std::make_unique<APABehaviorDeciderParkIn>(
      world_model, parking_longitudinal_behavior_planner_);
  behavior_calculator_parkin_ =
      std::make_unique<APABehaviorCalculatorParkIn>(world_model);

  behavior_calculator_parkout_ =
      std::make_unique<APABehaviorCalculatorParkOut>(world_model);

  init();
}

ApaStateMachine::~ApaStateMachine() {
  openspace_state_machine_->destroyMachine();
}

void ApaStateMachine::init() {
  if (!parking_task_config_.LoadYAML()) {
    MSD_LOG(INFO, "Load Parking Task Config Error!!!");
  }
  setupCallbackFunctions();
  unavailable_parking_lot_list_.clear();
  machine_ =
      std::make_unique<hfsm::Machine<Context>::PeerRoot<COMPOSITE>>(context_);
  current_state_ = StatusType::WAIT;
  PlanningContext::Instance()->mutable_planning_status()->task_status.task =
      StatusType::WAIT;
  PlanningContext::Instance()->mutable_planning_status()->task_status.status =
      TaskStatusType::RUNNING;
  // PlanningContext::Instance()
  //     ->mutable_planning_status()
  //     ->last_task_status.task = StatusType::WAIT;
  // PlanningContext::Instance()
  //     ->mutable_planning_status()
  //     ->last_task_status.status = TaskStatusType::RUNNING;
}

void ApaStateMachine::setupCallbackFunctions() {
  context_.register_entry_callback<ParkIn>(
      std::bind(&ApaStateMachine::onEntryParkIn, this));
  context_.register_entry_callback<ParkOut>(
      std::bind(&ApaStateMachine::onEntryParkOut, this));
  context_.register_entry_callback<Wait>(
      std::bind(&ApaStateMachine::onEntryWait, this));
  context_.register_entry_callback<RpaStraightStandby>(
      std::bind(&ApaStateMachine::onEntryRpaStraightStandby, this));
  context_.register_entry_callback<RpaStraight>(
      std::bind(&ApaStateMachine::onEntryRpaStraight, this));
  context_.register_update_callback<ParkIn>(
      std::bind(&ApaStateMachine::onUpdateParkIn, this));
  context_.register_update_callback<ParkOut>(
      std::bind(&ApaStateMachine::onUpdateParkOut, this));
  context_.register_update_callback<Wait>(
      std::bind(&ApaStateMachine::onUpdateWait, this));
  context_.register_update_callback<RpaStraightStandby>(
      std::bind(&ApaStateMachine::onUpdateRpaStraightStandby, this));
  context_.register_update_callback<RpaStraight>(
      std::bind(&ApaStateMachine::onUpdateRpaStraight, this));
  context_.register_transition_callback<ParkIn>(std::bind(
      &ApaStateMachine::onTransitionParkIn, this, std::placeholders::_1));
  context_.register_transition_callback<ParkOut>(std::bind(
      &ApaStateMachine::onTransitionParkOut, this, std::placeholders::_1));
  context_.register_transition_callback<Wait>(std::bind(
      &ApaStateMachine::onTransitionWait, this, std::placeholders::_1));
  context_.register_transition_callback<RpaStraightStandby>(
      std::bind(&ApaStateMachine::onTransitionRpaStraightStandby, this,
                std::placeholders::_1));
  context_.register_transition_callback<RpaStraight>(std::bind(
      &ApaStateMachine::onTransitionRpaStraight, this, std::placeholders::_1));
  context_.register_leave_callback<ParkIn>(
      std::bind(&ApaStateMachine::onLeaveParkIn, this));
  context_.register_leave_callback<ParkOut>(
      std::bind(&ApaStateMachine::onLeaveParkOut, this));
  context_.register_leave_callback<Wait>(
      std::bind(&ApaStateMachine::onLeaveWait, this));
  context_.register_leave_callback<RpaStraightStandby>(
      std::bind(&ApaStateMachine::onLeaveRpaStraightStandby, this));
  context_.register_leave_callback<RpaStraight>(
      std::bind(&ApaStateMachine::onLeaveRpaStraight, this));
}

bool ApaStateMachine::is_request_to_ego_slot() {
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->is_request_to_ego_slot = is_in_parking_slot();
  return true;
}

bool ApaStateMachine::generate_designate_parkin_tasks() {
  PlanningRequest planning_request = world_model_->get_planning_request();
  if (parking_task_config_.enable_control_test) {
    const int kTestCaseNUm = 36;
    control_test_id_ = control_test_id_ % kTestCaseNUm + 1;
    world_model_->set_control_test_flag(true);
    PlanningContext::Instance()
        ->mutable_openspace_decider_output()
        ->pattern_path = PlanningContext::Instance()->get_config_file_dir() +
                         "../test_case_config/APA_TFT_" +
                         std::to_string(control_test_id_) + ".json";
  }
  parking_task_list_.clear();
  ParkingTask task;
  task.task_name = "WAIT";
  task.task_type = ParkingTaskType::WAIT;
  task.poi_id = planning_request.id;
  task.poi_type = "PARKING_LOT";
  parking_task_list_.emplace_back(task);

  task.task_name = "APA";
  task.task_type = ParkingTaskType::PARKIN;
  task.poi_id = planning_request.id;
  task.poi_type = "PARKING_LOT";
  parking_task_list_.emplace_back(task);
  task.task_name = "WAIT_FOR_CALL";
  task.task_type = ParkingTaskType::WAIT;
  task.poi_id = planning_request.id;
  task.poi_type = "PARKING_LOT";
  parking_task_list_.emplace_back(task);
  current_task_ = parking_task_list_.begin();
  return true;
}

bool ApaStateMachine::generate_parkout_tasks() {
  PlanningRequest planning_request = world_model_->get_planning_request();
  parking_task_list_.clear();
  ParkingTask task;
  task.task_name = "WAIT";
  task.task_type = ParkingTaskType::WAIT;
  task.poi_id = world_model_->get_map_info().current_parking_lot_id;
  task.poi_type = "PARKING_LOT";
  parking_task_list_.emplace_back(task);
  task.task_name = "APOA";
  task.task_type = ParkingTaskType::PARKOUT;
  task.poi_id = world_model_->get_map_info().current_parking_lot_id;
  task.poi_type = "PARKING_LOT";
  task.park_out_direction.value = planning_request.park_out_direction.value;
  parking_task_list_.emplace_back(task);

  task.task_name = "WAIT_FOR_TAKEOVER";
  task.task_type = ParkingTaskType::WAIT;
  task.poi_id = parking_task_config_.done_id;
  task.poi_type = "EXIT";
  parking_task_list_.emplace_back(task);
  current_task_ = parking_task_list_.begin();
  return true;
}

bool ApaStateMachine::generate_rpa_straight_tasks() {
  PlanningRequest planning_request = world_model_->get_planning_request();
  parking_task_list_.clear();
  ParkingTask task;
  task.task_name = "RPA_STRAIGHT_STANDBY";
  task.task_type = ParkingTaskType::RPA_STRAIGHT_STANDBY;
  parking_task_list_.emplace_back(task);
  task.task_name = "RPA_STRAIGHT";
  task.task_type = ParkingTaskType::RPA_STRAIGHT;
  task.rpa_straight_direction.value =
      planning_request.rpa_straight_direction.value;
  parking_task_list_.emplace_back(task);
  task.task_name = "RPA_STRAIGHT_STANDBY";
  task.task_type = ParkingTaskType::RPA_STRAIGHT_STANDBY;
  parking_task_list_.emplace_back(task);
  current_task_ = parking_task_list_.begin();
  return true;
}

bool ApaStateMachine::update() {
  //[fenix.refactor.sm] Original ScenarioManager::execute_planning Part1

  MSD_LOG(INFO, "--------------------------------------------------------"
                "scenario manager.");

  double start4 = MTIME()->timestamp().sec();
  box_model_ =
      std::make_shared<BoxFootprintModel>(VehicleParam::Instance(), 0, 0);

  //   freespace_decider_->execute();

  // if (world_model_->is_parking_svp()) {
  if (msquare::CarParams::GetInstance()
          ->car_config.lon_config.use_sop_algorithm) {
    (void)path_sampler_->calculate_sop();
  } else {
    (void)path_sampler_->calculate();
  }

  if (world_model_->get_pause_status()) {
    if (world_model_->get_force_p_gear() &&
        world_model_->get_ego_state().is_static) {
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->planning_result.gear = GearState::PARK;
    }
  }

  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->is_last_path = path_sampler_->is_at_last_segment();
  double start3 = MTIME()->timestamp().sec();

  (void)leader_decider_->execute();
  (void)parking_longitudinal_behavior_planner_->calculate();
  (void)parking_longitudinal_motion_planner_->calculate();

  double after_update_msgs3 = MTIME()->timestamp().sec();

  auto update_msgs_time3 = after_update_msgs3 - start3;

  //[fenix.refactor.sm] END Original ScenarioManager::execute_planning Part1

  PlanningContext::Instance()
      ->mutable_planning_status()
      ->planning_result.bag_recorder_filter_scenario.reparkin = false;

  MSD_LOG(INFO, "----- task before update: -----");

  if (current_task_ != parking_task_list_.end()) {
    MSD_LOG(INFO, "task name: %s", current_task_->task_name.c_str());
    MSD_LOG(INFO, "task type: %d", (int)current_task_->task_type);
    if ((current_task_ + 1) != parking_task_list_.end()) {
      MSD_LOG(INFO, "next task name: %s",
              (current_task_ + 1)->task_name.c_str());
      MSD_LOG(INFO, "next task type: %d", (int)(current_task_ + 1)->task_type);
    } else {
      MSD_LOG(INFO, "Next task is None!");
    }
  } else {
    MSD_LOG(INFO, "Current task is None!");
  }
  machine_->update();
  MSD_LOG(INFO, "----- task after update: -----");

  if (current_task_ != parking_task_list_.end()) {
    MSD_LOG(INFO, "task name: %s", current_task_->task_name.c_str());
    MSD_LOG(INFO, "task type: %d", (int)current_task_->task_type);
    if ((current_task_ + 1) != parking_task_list_.end()) {
      MSD_LOG(INFO, "next task name: %s",
              (current_task_ + 1)->task_name.c_str());
      MSD_LOG(INFO, "next task type: %d", (int)(current_task_ + 1)->task_type);
    } else {
      MSD_LOG(INFO, "Next task is None!");
    }
  } else {
    MSD_LOG(INFO, "Current task is None!");
  }

  (void)update_status();

  auto planning_status_ =
      PlanningContext::Instance()->mutable_planning_status();
  planning_status_->scenario.scenario_type = status_text;
  planning_status_->scenario.stage_type = status_text;
  planning_status_->scenario.status_type = current_state_;
  planning_status_->scenario.next_status_type = next_state_;

  //[fenix.refactor.sm] Original ScenarioManager::execute_planning Part2
  double after_update_msgs4 = MTIME()->timestamp().sec();

  auto update_msgs_time4 = after_update_msgs4 - start4;
  // std::cout << "check_time_count: fs " << update_msgs_time4 << std::endl;
  if (world_model_->get_control_test_flag()) {
    //[fenix.refactor.sm] Temp disable control_test TODO:restore this

    // if (PlanningContext::Instance()->planning_status().scenario.status_type
    // ==
    //     StatusType::APA) {
    //   if (!PlanningContext::Instance()
    //     ->parking_behavior_planner_output().parking_lot) {
    //     onEntryApa();
    //     HybridAstarConfig::GetInstance()->planning_core = 2;
    //   }

    //   openspace_state_machine_->update();

    //   PlanningContext::Instance()
    //       ->mutable_parking_behavior_planner_output()
    //       ->is_finish =
    //       openspace_state_machine_->isActive<openspace_state_machine::Finish>();
    // } else {
    //   onLeaveApa();
    // }
  } else {
  }
  //[fenix.refactor.sm] Original ScenarioManager::execute_planning Part2
  return true;
}

bool ApaStateMachine::update_status() {
  if (current_task_ == parking_task_list_.end() ||
      (current_task_ + 1) == parking_task_list_.end()) {
    next_state_ = StatusType::WAIT;
  } else if ((current_task_ + 1)->task_type == ParkingTaskType::PARKIN) {
    next_state_ = StatusType::APA;
  } else if ((current_task_ + 1)->task_type == ParkingTaskType::PARKOUT) {
    next_state_ = StatusType::APOA;
  } else if ((current_task_ + 1)->task_type ==
             ParkingTaskType::RPA_STRAIGHT_STANDBY) {
    next_state_ = StatusType::RPA_STRAIGHT_STANDBY;
  } else if ((current_task_ + 1)->task_type == ParkingTaskType::RPA_STRAIGHT) {
    next_state_ = StatusType::RPA_STRAIGHT;
  } else {
    next_state_ = StatusType::WAIT;
  }

  PlanningContext::Instance()
      ->mutable_planning_status()
      ->planning_result.is_apa =
      (current_state_ == StatusType::APA || current_state_ == StatusType::APOA);
  return true;
}

bool ApaStateMachine::is_in_parking_slot() {
  if (world_model_->is_parking_svp()) {
    return world_model_->get_map_info().current_parking_lot_id != 0;
  } else {
    const ParkingMapInfo &parking_map_info =
        world_model_->get_parking_map_info();
    std::vector<ParkingLotDetectionInfo> parking_lots_detection_fusion_results =
        parking_map_info.parking_lots_detection_fusion_results;
    for (auto &parking_lot : parking_lots_detection_fusion_results) {
      if (parking_lot.is_car_in) {
        MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
        return true;
      }
    }
    return false;
  }
  return false;
}

void ApaStateMachine::saveParkinData() {
  if (PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.status != TaskStatusType::SUCCEEDED) {
    MSD_LOG(INFO, "Wouldn't save parking-in data as failing to park in");
    return;
  }

  // convert data to json
  auto &parking_slot_info = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->parking_slot_info;
  auto &ego_pose = world_model_->get_ego_state().ego_pose;
  auto slot_data = mjson::Json(mjson::Json::object());
  auto corner_pts = mjson::Json(mjson::Json::array());
  bool is_valid = false;
  if (parking_slot_info.corners.size() == 4) {
    is_valid = true;
    for (auto &p : parking_slot_info.corners) {
      auto corner_p = mjson::Json(mjson::Json::object());
      corner_p["x"] = mjson::Json(p.x);
      corner_p["y"] = mjson::Json(p.y);
      corner_pts.array_value().emplace_back(corner_p);
    }
  }
  slot_data["is_valid"] = mjson::Json(is_valid);
  slot_data["corner_pts"] = mjson::Json(corner_pts);
  slot_data["last_theta"] = mjson::Json(ego_pose.theta);

  auto parkin_data = mjson::Json(mjson::Json::object());
  parkin_data["slot_data"] = mjson::Json(slot_data);

  std::string config_folder =
      CarParams::GetInstance()->car_config.apoa_config.parkin_data_folder;
  std::string parkin_data_path = config_folder + "apa_parkin_data.json";
  std::string parkin_data_str = parkin_data.dump();

#ifdef USE_CONFIG_SERVICE
  parameter::Save(parkin_data_path, parkin_data_str);
  MSD_LOG(INFO, "[saveParkinData] save parkin data: %s",
          parkin_data_str.c_str());

#else
  std::ofstream ofs(parkin_data_path);
  if (!ofs.is_open()) {
    MSD_LOG(INFO, "[saveParkinData] fail to open file to save parkin data: %s",
            parkin_data_path.c_str());
    return;
  }
  ofs << parkin_data_str;
  ofs.close();
  MSD_LOG(INFO, "[saveParkinData] save parkin data locally: %s",
          parkin_data_str.c_str());
#endif
}

void ApaStateMachine::loadParkinData() {
  auto &last_parkin_data = PlanningContext::Instance()
                               ->mutable_parking_behavior_planner_output()
                               ->last_parkin_data;
  MSD_LOG(INFO, "[loadParkinData] ...");
  last_parkin_data.corner_pts.clear();
  last_parkin_data.is_valid = false;
  last_parkin_data.is_loaded = true;
  // TODO
  std::string config_folder =
      CarParams::GetInstance()->car_config.apoa_config.parkin_data_folder;
  std::string parkin_data_path = config_folder + "apa_parkin_data.json";
  std::string parkin_data_str;

#ifdef USE_CONFIG_SERVICE
  try {
    parameter::Load("apa_parkin_data", parkin_data_path);
    parkin_data_str =
        parameter::Get<std::string>("apa_parkin_data", "@apa_parkin_data");
  } catch (const std::exception &e) {
    MSD_LOG(ERROR, "[loadParkinData] failed to load apa_parkin_data from %s",
            parkin_data_path.c_str());
    return;
  }
#else
  std::ifstream ifs(parkin_data_path);
  if (!ifs.is_open()) {
    MSD_LOG(ERROR,
            "[loadParkinData] failed to load apa_parkin_data locally from %s",
            parkin_data_path.c_str());
    return;
  }
  std::string line;
  while (getline(ifs, line)) {
    parkin_data_str = parkin_data_str + line + "\n";
  }
  ifs.close();
#endif

  if (parkin_data_str.empty()) {
    MSD_LOG(ERROR, "[loadParkinData] parkin_data_str is empty");
    return;
  }

  MSD_LOG(INFO, "[loadParkinData] begin to parse parkin_data_str: %s",
          parkin_data_str.c_str());

  std::string err_code;
  auto parkin_data_parser = mjson::Json::parse(parkin_data_str, err_code);
  if (!err_code.empty()) {
    MSD_LOG(ERROR,
            "[loadParkinData] failed to parse parkin_data_str with error: %s",
            err_code.c_str());
    return;
  }

  if (!parkin_data_parser.has_key("slot_data")) {
    MSD_LOG(ERROR, "[loadParkinData] no slot_data in parkin_data_str");
    return;
  }
  auto slot_data_json = parkin_data_parser["slot_data"];

  if (!slot_data_json.has_key("is_valid")) {
    MSD_LOG(ERROR, "[loadParkinData] no is_valid in slot_data");
    return;
  }

  bool is_valid = slot_data_json["is_valid"].bool_value();
  if (!is_valid) {
    last_parkin_data.is_valid = is_valid;
    MSD_LOG(ERROR, "[loadParkinData] slot_data.is_valid = false");
    return;
  }

  if (!slot_data_json.has_key("corner_pts")) {
    MSD_LOG(ERROR, "[loadParkinData] no corner_pts in slot_data");
    return;
  }

  if (!slot_data_json.has_key("last_theta")) {
    MSD_LOG(ERROR, "[loadParkinData] no last_theta in slot_data");
    return;
  }

  auto corner_pts_json = slot_data_json["corner_pts"].array_value();
  if (corner_pts_json.size() != 4) {
    last_parkin_data.is_valid = false;
    MSD_LOG(ERROR, "[loadParkinData] corner_pts size=%u, which is not 4",
            corner_pts_json.size());
    return;
  }

  for (unsigned int i = 0; i < corner_pts_json.size(); i++) {
    auto corner_p_json = corner_pts_json[i];
    planning_math::Vec2d corner_p;
    if (!corner_p_json.has_key("x") || !corner_p_json.has_key("y")) {
      last_parkin_data.is_valid = false;
      MSD_LOG(ERROR,
              "[loadParkinData] corner_p_json has not x or y at index = %u", i);
      return;
    }
    corner_p.set_x(corner_p_json["x"].number_value());
    corner_p.set_y(corner_p_json["y"].number_value());
    last_parkin_data.corner_pts.push_back(corner_p);
  }
  MSD_LOG(INFO, "[loadParkinData] success.");
  last_parkin_data.last_theta = slot_data_json["last_theta"].number_value();
  last_parkin_data.is_valid = true;
}

} // namespace parking

} // namespace msquare
