#ifndef COMMON_CONFIG_TASK_CONFIG_H_
#define COMMON_CONFIG_TASK_CONFIG_H_

#include <boost/optional.hpp>
#include <list>
#include <string>
#include <vector>

namespace msquare {

using namespace boost;

// specific task configuration
// need to customize
typedef struct {
  std::string level{"normal"};
  bool enable_heuristic_frenet_search{false};
  double kStartLonComp_nolidar{0};
  double kLatComp_nolidar{0};
  double kPolygonLatError_nolidar{0};
  double kLatBufferComp_noLidar{0};
  double kLatBufferComp_noLidar_atten{0};
  double kLatBufferCompMax_v{4.5};
} LateralBehaviorPlannerConfig;

typedef struct {
  std::string level{"normal"};
  bool enable_heuristic_frenet_search{false};
  bool enable_crosswalk_pedestrian_stop{false};
  double lane_width_buff_sensor{0.0};
  double max_cruise_speed{60.0};
  double max_cruise_acceleration{1.5};
  double lane_change_prebrake_dis{70};
  double lane_change_prebrake_vlim{6.0};
  double lane_change_prebrake_ratio{0.15};
  double lane_change_prebrake_sensor_ratio{0.6};
  bool enable_prebrake_for_close_obs{true};
  bool enable_prebrake_for_intersection{true};
  bool enable_prebrake_for_curvature{true};
  bool enable_prebrake_for_small_curvature{true};
  bool enable_prebrake_for_lane_change{true};
  bool enable_prebrake_for_ttc{true};
  bool enable_prebrake_for_cutin_score{true};
  bool enable_prebrake_for_nbo{true};
  bool enable_prebrake_for_uncertainty{true};
  bool enable_prebrake_for_latBP{true};
  bool enable_prebrake_for_POI{true};
  double min_curvature{0.001};
  double max_centrifugal_acceleration{2.0};
  double ds_step_curvature{1.0};
  double ds_length_curvature{60.0};
  typedef struct {
    std::vector<double> a_max_ratio_vector;
    double a_max_ratio;
  } Driving_style_params;
  Driving_style_params driving_style_params{};
} SpeedBoundaryDeciderConfig;

typedef struct {
  std::string level{"normal"};
} PathDeciderConfig;

typedef struct {
  std::string level{"normal"};
} BackupPathDeciderConfig;

typedef struct {
  std::string level{"normal"};
  double time_horizon{8.0};
  double max_cruise_speed{60.0};
  double kAccCrossMin_GS{0.5};
  double kAccCrossMax_GS{1.4};
  double kJerkCrossMin_GS{0.8};
  double kJerkCrossMax_GS{2.8};
  double kAccCrossMin_TL{0.5};
  double kAccCrossMax_TL{1.4};
  double kJerkCrossMin_TL{0.8};
  double kJerkCrossMax_TL{2.8};
  double kAccCrossMin_TR{0.5};
  double kAccCrossMax_TR{1.4};
  double kJerkCrossMin_TR{0.8};
  double kJerkCrossMax_TR{2.8};
  double kAccCrossMin_UT{0.5};
  double kAccCrossMax_UT{1.0};
  double kJerkCrossMin_UT{0.5};
  double kJerkCrossMax_UT{1.5};
  double kRefDis2Cross{10.0};
  double kDeltaDis2Cross{20.0};
  double kW_v{500.0};
  double kW_acc{10.0};
  double kW_jerk{3000.0};
  double kW_s{100.0};
  double kW_v_dv{1.0};
  double kW_acc_dv{20.0};
  double kW_jerk_pow{10.0};
  double kW_s_TTC{2.0};
  double kW_s_ds{10.0};
  double kW_s_pow{30.0};
  double kDecay_vr0{0.995};
  double kDecay_vr1{0.94};
  double kDecay_acc0{0.995};
  double kDecay_acc1{0.96};
  double kDecay_jerk0{0.995};
  double kDecay_jerk1{0.98};
  double kDecay_ds0{0.995};
  double kDecay_ds1{0.94};
  double kDecay_dt_GS{0.93};
  double kDecay_dt_TR{0.94};
  double kDecay_dt_TL{0.96};
  double kDecay_dt_UT{0.97};
  double kDecay_dis_base_GS{10.0};
  double kDecay_dis_base_TR{8.0};
  double kDecay_dis_base_TL{7.0};
  double kDecay_dis_base_UT{5.0};
  double kDecay_dis_ratio{1.0};
  double kDecay_dis_buff_GS{0.005};
  double kDecay_dis_buff_TR{0.008};
  double kDecay_dis_buff_TL{0.01};
  double kDecay_dis_buff_UT{0.03};
  double kDecay_GSvsGS{1.0};
  double kDecay_GSvsTR{0.8};
  double kDecay_GSvsTL{0.3};
  double kDecay_GSvsUT{0.1};
  double kDecay_GSvsFM{1.0};
  double kDecay_TRvsGS{1.5};
  double kDecay_TRvsTR{1.0};
  double kDecay_TRvsTL{0.6};
  double kDecay_TRvsUT{1.0};
  double kDecay_TRvsFM{1.0};
  double kDecay_TLvsGS{2.0};
  double kDecay_TLvsTR{0.6};
  double kDecay_TLvsTL{1.0};
  double kDecay_TLvsUT{1.0};
  double kDecay_TLvsFM{1.0};
  double kDecay_UTvsGS{2.0};
  double kDecay_UTvsTR{1.0};
  double kDecay_UTvsTL{1.0};
  double kDecay_UTvsUT{1.0};
  double kDecay_UTvsFM{1.0};
  double kDecay_IniPos{2.0};
  double kDsRoad_GS{1.0};
  double kDsRoad_TL{2.0};
  double kDsRoad_TR{1.5};
  double kDsRoad_UT{6.0};
  double kDsPed{3.0};
  double kDsOfo{3.0};
  double kDsTruck{3.0};
  double kTTC_Lon{2.3};
  double kTTC_Buff_Sensor{0.2};
  typedef struct {
    std::vector<double> d_stop_vector;
    double d_stop;
    std::vector<double> weight_acc_ratio_vector;
    double weight_acc_ratio;
    std::vector<double> weight_jerk_ratio_vector;
    double weight_jerk_ratio;
  } Driving_style_params;
  Driving_style_params driving_style_params{};
} LongitudinalBehaviourPlannerConfig;

typedef struct {
  std::string level{"normal"};
} LongitudinalMotionPlannerConfig;

typedef struct {
  std::string level{"normal"};
} ObstacleDeciderConfig;

typedef struct {
  std::string level{"normal"};
} GeneralMotionPlannerConfig;

typedef struct {
  std::string level{"normal"};
  double kStartLonComp_nolidar{2.0};
  double kLatComp_nolidar{-0.3};
  double kPolygonLatError_nolidar{-0.2};
  bool enable_offset_smooth{false};
  double kLatBufferCompStatic_Highway{0};
  double kMaxStartBuffer_LowSpeed{6.0};
  typedef struct {
    std::vector<double> l_weight_ratio_vector;
    double l_weight_ratio;
  } Driving_style_params;
  Driving_style_params driving_style_params{};
} LateralMotionPlannerConfig;

typedef struct {
  std::string level{"normal"};
  bool execute_in_lane_change_scenario{false};
  bool execute_in_lane_merge_scenario{false};
} LaneChangeDeciderConfig;

typedef struct {
  std::string level{"normal"};
  double initial_rival_aggressiveness{1.5};
  double time_horizon{6.0};
  double time_step{1.0};
} LaneChangePOMDPPlannerConfig;

class TaskConfig {
public:
  explicit TaskConfig(){};

  ~TaskConfig() = default;

  bool has_lateral_behavior_planner_config() const {
    return !lateral_behavior_planner_config_ ? false : true;
  }
  bool has_speed_boundary_decider_config() const {
    return !speed_boundary_decider_config_ ? false : true;
  }
  bool has_path_decider_config() const {
    return !path_decider_config_ ? false : true;
  }
  bool has_backup_path_decider_config() const {
    return !backup_path_decider_config_ ? false : true;
  }
  bool has_longitudinal_behavior_planner_config() const {
    return !longitudinal_behavior_planner_config_ ? false : true;
  }
  bool has_longitudinal_motion_planner_config() const {
    return !longitudinal_motion_planner_config_ ? false : true;
  }
  bool has_lateral_motion_planner_config() const {
    return !lateral_motion_planner_config_ ? false : true;
  }
  bool has_lane_change_decider_config() const {
    return !lane_change_decider_config_ ? false : true;
  }
  bool has_obstacle_decider_config() const {
    return !obstacle_decider_config_ ? false : true;
  }
  bool has_general_motion_planner_config() const {
    return !general_motion_planner_config_ ? false : true;
  }

  LateralBehaviorPlannerConfig lateral_behavior_planner_config() const {
    return *lateral_behavior_planner_config_;
  }
  SpeedBoundaryDeciderConfig speed_boundary_decider_config() const {
    return *speed_boundary_decider_config_;
  }
  PathDeciderConfig path_decider_config() const {
    return *path_decider_config_;
  }
  BackupPathDeciderConfig backup_path_decider_config() const {
    return *backup_path_decider_config_;
  }
  LongitudinalBehaviourPlannerConfig
  longitudinal_behavior_planner_config() const {
    return *longitudinal_behavior_planner_config_;
  }
  LongitudinalMotionPlannerConfig longitudinal_motion_planner_config() const {
    return *longitudinal_motion_planner_config_;
  }
  LateralMotionPlannerConfig lateral_motion_planner_config() const {
    return *lateral_motion_planner_config_;
  }
  LaneChangeDeciderConfig lane_change_decider_config() const {
    return *lane_change_decider_config_;
  }
  ObstacleDeciderConfig obstacle_decider_config() const {
    return *obstacle_decider_config_;
  }
  GeneralMotionPlannerConfig general_motion_planner_config() const {
    return *general_motion_planner_config_;
  }

  void clear() {
    lateral_behavior_planner_config_ = none;
    speed_boundary_decider_config_ = none;
    path_decider_config_ = none;
    backup_path_decider_config_ = none;
    longitudinal_behavior_planner_config_ = none;
    longitudinal_motion_planner_config_ = none;
    lateral_motion_planner_config_ = none;
    lane_change_decider_config_ = none;
    obstacle_decider_config_ = none;
    general_motion_planner_config_ = none;
  }

  typedef enum {
    LATERAL_BEHAVIOR_PLANNER = 1,
    SPEED_BOUNDARY_DECIDER = 2,
    PATH_DECIDER = 3,        // DISCARD
    BACKUP_PATH_DECIDER = 4, // DISCARD
    LONGITUDINAL_BEHAVIOR_PLANNER = 5,
    LONGITUDINAL_MOTION_PLANNER = 6,
    LATERAL_MOTION_PLANNER = 7,
    LANE_CHANGE_DECIDER = 8,
    OBSTACLE_DECIDER = 9,
    GENERAL_MOTION_PLANNER = 10,
    TASK_LAST = 11,
  } TaskType;

  void
  set_lateral_behavior_planner_config(LateralBehaviorPlannerConfig config) {
    clear();
    lateral_behavior_planner_config_ = config;
    task_type_ = TaskType::LATERAL_BEHAVIOR_PLANNER;
  }
  void set_speed_boundary_decider_config(SpeedBoundaryDeciderConfig config) {
    clear();
    speed_boundary_decider_config_ = config;
    task_type_ = TaskType::SPEED_BOUNDARY_DECIDER;
  }
  void set_path_decider_config(PathDeciderConfig config) {
    clear();
    path_decider_config_ = config;
    task_type_ = TaskType::PATH_DECIDER;
  }
  void set_backup_path_decider_config(BackupPathDeciderConfig config) {
    clear();
    backup_path_decider_config_ = config;
    task_type_ = TaskType::BACKUP_PATH_DECIDER;
  }
  void set_longitudinal_behavior_planner_config(
      LongitudinalBehaviourPlannerConfig config) {
    clear();
    longitudinal_behavior_planner_config_ = config;
    task_type_ = TaskType::LONGITUDINAL_BEHAVIOR_PLANNER;
  }
  void set_longitudinal_motion_planner_config(
      LongitudinalMotionPlannerConfig config) {
    clear();
    longitudinal_motion_planner_config_ = config;
    task_type_ = TaskType::LONGITUDINAL_MOTION_PLANNER;
  }
  void set_lateral_motion_planner_config(LateralMotionPlannerConfig config) {
    clear();
    lateral_motion_planner_config_ = config;
    task_type_ = TaskType::LATERAL_MOTION_PLANNER;
  }
  void set_lane_change_decider_config(LaneChangeDeciderConfig config) {
    clear();
    lane_change_decider_config_ = config;
    task_type_ = TaskType::LANE_CHANGE_DECIDER;
  }
  void set_obstacle_decider_config(ObstacleDeciderConfig config) {
    clear();
    obstacle_decider_config_ = config;
    task_type_ = TaskType::OBSTACLE_DECIDER;
  }

  void set_general_motion_planner_config(GeneralMotionPlannerConfig config) {
    clear();
    general_motion_planner_config_ = config;
    task_type_ = TaskType::GENERAL_MOTION_PLANNER;
  }

  TaskType task_type() const { return task_type_; }

private:
  TaskType task_type_ = {};
  optional<LateralBehaviorPlannerConfig> lateral_behavior_planner_config_;
  optional<SpeedBoundaryDeciderConfig> speed_boundary_decider_config_;
  optional<PathDeciderConfig> path_decider_config_;
  optional<BackupPathDeciderConfig> backup_path_decider_config_;
  optional<LongitudinalBehaviourPlannerConfig>
      longitudinal_behavior_planner_config_;
  optional<LongitudinalMotionPlannerConfig> longitudinal_motion_planner_config_;
  optional<LateralMotionPlannerConfig> lateral_motion_planner_config_;
  optional<LaneChangeDeciderConfig> lane_change_decider_config_;
  optional<ObstacleDeciderConfig> obstacle_decider_config_;
  optional<GeneralMotionPlannerConfig> general_motion_planner_config_;
};

const static std::string
    TaskTypeNames[static_cast<int>(TaskConfig::TaskType::TASK_LAST) - 1] = {
        "LateralBehaviorPlanner",
        "SpeedBoundaryDecider",
        "PathDecider",
        "BackupPathDecider",
        "LongitudinalBehaviourPlanner",
        "LongitudinalMotionPlanner",
        "LateralMotionPlanner",
        "LaneChangeDecider",
        "ObstacleDecider",
        "GeneralMotionPlanner"};

} // namespace msquare

#endif // COMMON_CONFIG_TASK_CONFIG_H_
