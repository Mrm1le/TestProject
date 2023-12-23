#include <boost/program_options.hpp>
#include <chrono>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Bool.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "convert/convert.h"
#include "mjson/mjson.hpp"
#include "mlog_msg_id/mlog_msg_id_pub.hpp"
#include "mlog_publisher/publisher.h"
#include "mtime_core/mtime.h"
#include "timeline/timeline_ros.h"

#include "obstacle_hacks.hpp"
#include "param_helper.hpp"
#include "pnc.h"
#include "upload_msd_test.h"

using namespace msquare;

std::string read_file(std::string path) {
  FILE *file = fopen(path.c_str(), "r");
  if (file == nullptr) {
    ROS_ERROR("Fail to open file '%s'", path.c_str());
    return "";
  }
  std::shared_ptr<FILE> fp(file, [](FILE *file) { fclose(file); });
  fseek(fp.get(), 0, SEEK_END);
  std::vector<char> content(ftell(fp.get()));
  fseek(fp.get(), 0, SEEK_SET);
  auto read_bytes = fread(content.data(), 1, content.size(), fp.get());
  assert(read_bytes == content.size());
  (void)read_bytes;
  return std::string(content.begin(), content.end());
}

void print_stacktrace() {
  int thread_id = ::syscall(SYS_gettid);
  int child_pid = ::fork();
  if (child_pid == 0) {
    char pid_buf[32];
    sprintf(pid_buf, "0x%016x", thread_id);

    // invoke GDB
    char name_buf[512];
    name_buf[::readlink("/proc/self/exe", &name_buf[0], 511)] = 0;

    ::execl("/usr/bin/gdb", "/usr/bin/gdb", "--batch", "-n", "-ex", "bt", "-ex",
            "quit", &name_buf[0], &pid_buf[0], nullptr);
    ::exit(1); // if GDB failed to start
  } else if (child_pid == -1) {
    ::exit(1); // if forking failed
  } else {
    // make it work for non root users
    if (0 != getuid()) {
      ::prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY, 0, 0, 0);
    }
    ::waitpid(child_pid, nullptr, 0);
  }
}

extern "C" void __cxa_throw(void *thrown_exception, void *pvtinfo,
                            void (*dest)(void *)) {
  int len = 0;
  char buff[1024];

  struct std::type_info *type_info = (struct std::type_info *)pvtinfo;
  char *exception_type =
      abi::__cxa_demangle(type_info->name(), nullptr, nullptr, nullptr);

  if (exception_type) {
    len = sprintf(buff, "%s", exception_type);
    std::free(exception_type);
  } else {
    len = sprintf(buff, "%s", type_info->name());
  }

  const abi::__class_type_info *exc =
      dynamic_cast<const abi::__class_type_info *>(&typeid(std::exception));
  const abi::__class_type_info *cti =
      dynamic_cast<abi::__class_type_info *>(type_info);

  if (cti && exc) {
    std::exception *exception_info = reinterpret_cast<std::exception *>(
        abi::__dynamic_cast(thrown_exception, exc, cti, -1));
    if (exception_info) {
      sprintf(buff + len, "--%s", exception_info->what());
    }
  }

  printf("Exception [%s] thrown, current process will be aborted\n", buff);
  print_stacktrace();
  std::abort();
}

namespace msd_planning {

class MSDDecisionNode {
public:
  explicit MSDDecisionNode(ros::NodeHandle &nh) : nh_(nh) {
    ros::NodeHandle ___nh_param_reader = nh_;
    READ_STRING_PARAM_WITH_DEFAULT(config_file, "./config.json");
    READ_STRING_PARAM_WITH_DEFAULT(mtaskflow_config_file,
                                   "./mtaskflow_config.json");
    READ_STRING_PARAM_WITH_DEFAULT(config_file_dir, "./config");
    READ_INT_PARAM_WITH_DEFAULT(scene_type, 0);
    READ_GLOBAL_BOOL_PARAM_WITH_DEFAULT(use_sim_time, false);
    // universal topics
    READ_STRING_PARAM_WITH_DEFAULT(pub_planning_topic, "/msd/planning/plan");
    READ_STRING_PARAM_WITH_DEFAULT(pub_local_lat_motion_info_topic,
                                   "/msd/path_planner_input");
    READ_STRING_PARAM_WITH_DEFAULT(msd_module_status_topic,
                                   "/msd/function_module_status");
    READ_STRING_PARAM_WITH_DEFAULT(pub_nodes_status_topic,
                                   "/node_status/planning");
    READ_STRING_PARAM_WITH_DEFAULT(sub_chassis_report_topic,
                                   "/vehicle/chassis_report");
    READ_STRING_PARAM_WITH_DEFAULT(sub_wheel_report_topic,
                                   "/vehicle/wheel_report");
    READ_STRING_PARAM_WITH_DEFAULT(sub_body_report_topic,
                                   "/vehicle/body_report");
    READ_STRING_PARAM_WITH_DEFAULT(sub_imu_topic, "/sensor/imu");
    READ_STRING_PARAM_WITH_DEFAULT(sub_worldmodel_map_topic,
                                   "/worldmodel/processed_map");
    READ_STRING_PARAM_WITH_DEFAULT(sub_worldmodel_objects_topic,
                                   "/worldmodel/objects");
    READ_STRING_PARAM_WITH_DEFAULT(sub_worldmodel_parking_slots_topic,
                                   "/worldmodel/parking_slot_info");
    READ_STRING_PARAM_WITH_DEFAULT(sub_worldmodel_scene_objects_topic,
                                   "/worldmodel/scene_object");
    READ_STRING_PARAM_WITH_DEFAULT(sub_traffic_light_topic,
                                   "/worldmodel/traffic_light");
    READ_STRING_PARAM_WITH_DEFAULT(
        sub_fusion_objects_topic,
        "/perception/fusion/object_parking_environment");
    READ_STRING_PARAM_WITH_DEFAULT(sub_fusion_groundlines_topic,
                                   "/perception/fusion/ground_line");
    READ_STRING_PARAM_WITH_DEFAULT(sub_fusion_uss_groundlines_topic,
                                   "/perception/fusion/ground_line_uss");
    READ_STRING_PARAM_WITH_DEFAULT(sub_fusion_uss_topic,
                                   "/perception/fusion/distance_uss");
    READ_STRING_PARAM_WITH_DEFAULT(sub_ego_pose_topic, "/mla/egopose");
    READ_STRING_PARAM_WITH_DEFAULT(sub_prediction_topic,
                                   "/msd/prediction/prediction_result");
    READ_STRING_PARAM_WITH_DEFAULT(sub_mpc_trajecrory_topic,
                                   "/msd/control/mpc_traj");
    READ_STRING_PARAM_WITH_DEFAULT(sbp_request_topic, "/msd/sbp_request");
    READ_STRING_PARAM_WITH_DEFAULT(sbp_result_topic, "/msd/sbp_result");
    READ_STRING_PARAM_WITH_DEFAULT(
        sub_planning_control_cmd_request_topic,
        "/system_manager/planning/control_cmd_request");
    READ_STRING_PARAM_WITH_DEFAULT(
        pub_planning_control_cmd_response_topic,
        "/system_manager/planning/control_cmd_response");
    READ_STRING_PARAM_WITH_DEFAULT(sub_planning_request_topic,
                                   "/system_manager/planning/request");
    READ_STRING_PARAM_WITH_DEFAULT(pub_planning_response_topic,
                                   "/system_manager/planning/response");
    READ_STRING_PARAM_WITH_DEFAULT(pub_planning_info_topic, "/planning/info");
    READ_STRING_PARAM_WITH_DEFAULT(sub_perception_vision_lane_topic,
                                   "/perception/vision/lane");
    READ_STRING_PARAM_WITH_DEFAULT(sub_perception_radar_topic,
                                   "/perception/radar/result_front");
    READ_PARAM_END;

    try {
      std::string config_data = read_file(config_file);
      init_config_data(config_data);
      // init mlog and mtime
      mlog::PublisherSettings publisher_settings;
      publisher_settings.handle = &nh_;
      publisher_settings.config_file = config_file.c_str();
      auto publisher = mlog::MLogPublisher::make(publisher_settings);

      mlog::MLogEndpointCallback planning_callback =
          [publisher](const std::vector<mlog::MLogEntry> &entries) {
            publisher->publish(entries);
          };
      std::shared_ptr<mtime::Timeline> timeline =
          std::make_shared<mtime::TimelineRos>();

      MSDPlanning_set_log_config(config_data, &nh_);
      MSDPlanning_set_log_callback(planning_callback);
      MSDPlanning_init_log_manager();
      MTIME()->init(timeline);

      MSDPlanningConfig planning_config{(SceneType)scene_type,
                                        true,
                                        enable_ilc_,
                                        enable_alc_,
                                        ilc_limit_velocity_,
                                        ilc_limit_time_,
                                        default_cruise_velocity_,
                                        is_acc_mode_,
                                        is_ddmap_,
                                        driving_style_,
                                        config_file_dir,
                                        mtaskflow_config_file,
                                        "",
                                        init_wm_};

      planning_engine_ = PlanningEngineInterface::make(planning_config);

      planning_engine_->init();

      if (start_machine_) {
        planning_engine_->start();

        // feed start cmd
        maf_system_manager::ModuleControlCmdRequest request{};
        request.header.stamp = MTIME()->timestamp().ns();
        request.module_control_cmd.value =
            maf_system_manager::ModuleControlCmdEnum::RESUME;
        if (scene_type_ == SceneType::URBAN) {
          request.running_mode.value =
              maf_system_manager::RunningModeEnum::PILOT;
        } else {
          request.running_mode.value =
              maf_system_manager::RunningModeEnum::PARKING;
        }
        planning_engine_->feed_planning_control_cmd_request(request);
      }

      auto no_delay = ros::TransportHints().tcpNoDelay(true);

      publisher_map_[PLANNING] =
          nh_.advertise<planning_msgs::Planning>(pub_planning_topic, 1);
      // publisher_map_[MSD_MODULE_STATUS] =
      //     nh_.advertise<framework_status_msgs::ModuleStatus>(
      //         msd_module_status_topic, 1);
      publisher_map_[SBP_REQUEST] =
          nh_.advertise<planning_msgs::SBPRequest>(sbp_request_topic, 1);

      subscriber_map_[MSD_MODULE_STATUS] =
          nh_.subscribe(msd_module_status_topic, 1,
                        &MSDDecisionNode::update_module_status, this, no_delay);
      publisher_map_[NODES_STATUS] =
          nh_.advertise<framework_status_msgs::NodesStatus>(
              pub_nodes_status_topic, 1);
      publisher_map_[PLAN_INFO] =
          nh_.advertise<std_msgs::Header>(pub_planning_info_topic, 1);
      subscriber_map_[CHASSIS_REPORT] = nh_.subscribe(
          sub_chassis_report_topic, 1,
          &MSDDecisionNode::chassis_report_callback, this, no_delay);
      subscriber_map_[WHEEL_REPORT] = nh_.subscribe(
          sub_wheel_report_topic, 1, &MSDDecisionNode::wheel_report_callback,
          this, no_delay);
      subscriber_map_[BODY_REPORT] =
          nh_.subscribe(sub_body_report_topic, 1,
                        &MSDDecisionNode::body_report_callback, this, no_delay);
      subscriber_map_[IMU_REPORT] =
          nh_.subscribe(sub_imu_topic, 1, &MSDDecisionNode::imu_report_callback,
                        this, no_delay);
      subscriber_map_[WORLD_MODEL_MAP] = nh_.subscribe(
          sub_worldmodel_map_topic, 1, &MSDDecisionNode::update_world_model_map,
          this, no_delay);
      subscriber_map_[WORLD_MODEL_OBJECTS] = nh_.subscribe(
          sub_worldmodel_objects_topic, 1,
          &MSDDecisionNode::update_world_model_objecs, this, no_delay);
      subscriber_map_[WORLD_MODEL_PARKING_SLOTS] = nh_.subscribe(
          sub_worldmodel_parking_slots_topic, 1,
          &MSDDecisionNode::update_world_model_parking_slots, this, no_delay);
      subscriber_map_[WORLD_MODEL_SCENE_OBJECTS] = nh_.subscribe(
          sub_worldmodel_scene_objects_topic, 1,
          &MSDDecisionNode::update_world_model_scene_objects, this, no_delay);
      subscriber_map_[TRAFFIC_LIGHT] =
          nh_.subscribe(sub_traffic_light_topic, 1,
                        &MSDDecisionNode::update_traffic_light, this, no_delay);
      subscriber_map_[FUSION_OBJECTS] = nh_.subscribe(
          sub_fusion_objects_topic, 1, &MSDDecisionNode::update_fusion_objects,
          this, no_delay);
      subscriber_map_[FUSION_GROUNDLINES] = nh_.subscribe(
          sub_fusion_groundlines_topic, 1,
          &MSDDecisionNode::update_fusion_groundlines, this, no_delay);
      subscriber_map_[FUSION_GROUNDLINES] = nh_.subscribe(
          sub_fusion_uss_groundlines_topic, 1,
          &MSDDecisionNode::update_fusion_uss_groundlines, this, no_delay);
      subscriber_map_[FUSION_USS] =
          nh_.subscribe(sub_fusion_uss_topic, 1,
                        &MSDDecisionNode::update_fusion_uss, this, no_delay);
      subscriber_map_[EGO_POSE] =
          nh_.subscribe(sub_ego_pose_topic, 1,
                        &MSDDecisionNode::update_ego_pose, this, no_delay);
      subscriber_map_[PREDICTION] = nh_.subscribe(
          sub_prediction_topic, 1, &MSDDecisionNode::update_prediction_info,
          this, no_delay);
      subscriber_map_[MSD_CONTROL_MPC] = nh_.subscribe(
          sub_mpc_trajecrory_topic, 1, &MSDDecisionNode::update_mpc_trajectory,
          this, no_delay);
      subscriber_map_[SBP_RESULT] =
          nh_.subscribe(sbp_result_topic, 1,
                        &MSDDecisionNode::update_sbp_result, this, no_delay);
      subscriber_map_[PLAN_REQUEST] = nh_.subscribe(
          sub_planning_request_topic, 1,
          &MSDDecisionNode::planning_request_callback, this, no_delay);
      subscriber_map_[PERCEPTION_VISION_LANE] = nh_.subscribe(
          sub_perception_vision_lane_topic, 1,
          &MSDDecisionNode::perception_vision_lane_callback, this, no_delay);
      subscriber_map_[PERCEPTION_RADAR] = nh_.subscribe(
          sub_perception_radar_topic, 1,
          &MSDDecisionNode::perception_radar_callback, this, no_delay);

      auto &pub_planning = publisher_map_[PLANNING];
      planning_engine_->set_callback(
          [&pub_planning](const MSDPlanningOutputMeta &meta,
                          const maf_planning::Planning &planning_result,
                          const std::string &trigger_msg_id) {
            if (meta.succeed) {
              planning_msgs::Planning planning_msg{};
              ros_cpp_struct_convert::to_ros(planning_result, planning_msg);

              MSDPlanning_record_relation(
                  mlog::MLOG_msg_id_ros(planning_msg.header, kPlanTag),
                  trigger_msg_id);

              pub_planning.publish(planning_msg);
            }
          });

      auto &pub_planning_info = publisher_map_[PLAN_INFO];
      planning_engine_->set_callback(
          [&pub_planning_info](const maf_std::Header &planning_info) {
            std_msgs::Header planning_info_msg;
            ros_cpp_struct_convert::to_ros(planning_info, planning_info_msg);

            pub_planning_info.publish(planning_info_msg);
          });

      // auto &pub_msd_module_status = publisher_map_[MSD_MODULE_STATUS];
      // planning_engine_->set_callback(
      //     [&pub_msd_module_status](const maf_framework_status::ModuleStatus
      //                                  &planning_module_status) {
      //       framework_status_msgs::ModuleStatus status_msg{};
      //       ros_cpp_struct_convert::to_ros(planning_module_status,
      //       status_msg); pub_msd_module_status.publish(status_msg);
      //     });

      auto &pub_nodes_status = publisher_map_[NODES_STATUS];
      planning_engine_->set_callback(
          [&pub_nodes_status](
              const maf_framework_status::NodeStatus &planning_node_status) {
            maf_framework_status::NodesStatus planning_nodes_status;
            framework_status_msgs::NodesStatus status_msg{};

            planning_nodes_status.header.stamp = MTIME()->timestamp().ns();

            planning_nodes_status.nodes_status.push_back(planning_node_status);

            ros_cpp_struct_convert::to_ros(planning_nodes_status, status_msg);
            pub_nodes_status.publish(status_msg);
          });

      planning_engine_->set_callback([&](const RecognizeInfo &recognize_info) {
        UploadMsdTest().upload(recognize_info);
      });

      auto &pub_sbp_request = publisher_map_[SBP_REQUEST];
      planning_engine_->set_callback(
          [&pub_sbp_request](const maf_planning::SBPRequest &sbp_request) {
            planning_msgs::SBPRequest sbp_request_msg{};
            ros_cpp_struct_convert::to_ros(sbp_request, sbp_request_msg);
            pub_sbp_request.publish(sbp_request_msg);
          });

    } catch (std::exception &e) {
      ROS_ERROR("Error: %s", e.what());
      abort();
    }
  }
  virtual ~MSDDecisionNode() = default;

private:
  void init_config_data(std::string &config_data) {
    char *log_level = std::getenv("MSQUARE_PNC_LOG_LEVEL");
    auto config_reader = mjson::Reader(config_data.c_str());
    if (log_level) {
      std::string log_level_lower(log_level);
      transform(log_level_lower.begin(), log_level_lower.end(),
                log_level_lower.begin(), tolower);
      auto logger_reader =
          mjson::Reader(config_reader.get<mjson::Json>("logger"));
      logger_reader.override_value("level", log_level_lower);
      config_reader.override_value("logger", logger_reader.raw());
      config_data = config_reader.raw().dump().c_str();
    }
    enable_ilc_ = config_reader.get<bool>("enable_ilc", false, true);
    enable_alc_ = config_reader.get<bool>("enable_alc", false, false);
    ilc_limit_velocity_ =
        config_reader.get<double>("interactive_lc_limit_velocity", false, 0.0);
    ilc_limit_time_ =
        config_reader.get<double>("interactive_lc_limit_time", false, 10.0);
    default_cruise_velocity_ =
        config_reader.get<double>("default_cruise_velocity", false, 120.0);
    start_machine_ = config_reader.get<bool>("start_machine", false, false);
    is_acc_mode_ = config_reader.get<bool>("is_acc_mode", false, false);
    is_ddmap_ = config_reader.get<bool>("is_ddmap", false, false);
    driving_style_ = config_reader.get<int>("driving_style", false, 1);

    init_wm_ = config_reader.get<bool>("init_wm", false, false);
  }

  void update_module_status(
      const framework_status_msgs::ModuleStatus &module_status) {
    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(module_status.header, kModuleStatusTag),
        MSDPlanning_msg_tag(kModuleStatusTag), 0);
    maf_framework_status::ModuleStatus maf_module_status{};
    ros_cpp_struct_convert::from_ros(module_status, maf_module_status);
    planning_engine_->feed_module_status(maf_module_status);
  }

  void
  chassis_report_callback(const endpoint_msgs::ChassisReport &chassis_report) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(chassis_report.header, kChassisReportTag),
        MSDPlanning_msg_tag(kChassisReportTag), 0);

    auto maf_chassis_report = std::make_shared<maf_endpoint::ChassisReport>();
    ros_cpp_struct_convert::from_ros(chassis_report, *maf_chassis_report);
    planning_engine_->feed_chassis_report(maf_chassis_report);
  }

  void wheel_report_callback(const endpoint_msgs::WheelReport &wheel_report) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(wheel_report.header, kWheelReportTag),
        MSDPlanning_msg_tag(kWheelReportTag), 0);

    auto maf_wheel_report = std::make_shared<maf_endpoint::WheelReport>();
    ros_cpp_struct_convert::from_ros(wheel_report, *maf_wheel_report);
    planning_engine_->feed_wheel_report(maf_wheel_report);
  }

  void body_report_callback(const endpoint_msgs::BodyReport &body_report) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(body_report.header, kBodyReportTag),
        MSDPlanning_msg_tag(kBodyReportTag), 0);

    auto maf_body_report = std::make_shared<maf_endpoint::BodyReport>();
    ros_cpp_struct_convert::from_ros(body_report, *maf_body_report);
    planning_engine_->feed_body_report(maf_body_report);
  }

  void imu_report_callback(const gps_imu_msgs::MLAImu &imu_report) {
    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(imu_report.header, kImuReportTag),
        MSDPlanning_msg_tag(kImuReportTag), 0);

    auto maf_imu_report = std::make_shared<maf_gps_imu::MLAImu>();
    ros_cpp_struct_convert::from_ros(imu_report, *maf_imu_report);
    planning_engine_->feed_imu_report(maf_imu_report);
  }

  void
  update_world_model_map(const worldmodel_msgs::ProcessedMap &worldmodel_map) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(worldmodel_map.header, kWorldModelMapTag),
        MSDPlanning_msg_tag(kWorldModelMapTag), 0);

    auto maf_worldmodel_map = std::make_shared<maf_worldmodel::ProcessedMap>();
    ros_cpp_struct_convert::from_ros(worldmodel_map, *maf_worldmodel_map);
    planning_engine_->feed_worldmodel_map(maf_worldmodel_map);
  }

  void update_world_model_objecs(
      const worldmodel_msgs::ObjectsInterface &worldmodel_object) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(worldmodel_object.header, kWorldModelObjectTag),
        MSDPlanning_msg_tag(kWorldModelObjectTag), 0);

    auto maf_worldmodel_object =
        std::make_shared<maf_worldmodel::ObjectsInterface>();
    ros_cpp_struct_convert::from_ros(worldmodel_object, *maf_worldmodel_object);
    planning_engine_->feed_worldmodel_objects(maf_worldmodel_object);
  }

  void update_world_model_parking_slots(
      const worldmodel_msgs::FusionAPA &parking_slots) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(parking_slots.header, kWorldModelParkingSlotTag),
        MSDPlanning_msg_tag(kWorldModelParkingSlotTag), 0);

    auto maf_worldmodel_parking_slots =
        std::make_shared<maf_worldmodel::FusionAPA>();
    ros_cpp_struct_convert::from_ros(parking_slots,
                                     *maf_worldmodel_parking_slots);
    planning_engine_->feed_world_model_parking_slots(
        maf_worldmodel_parking_slots);
  }

  void update_world_model_scene_objects(
      const worldmodel_msgs::SceneObjects &scene_objects) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(scene_objects.header, kWorldModelSceneObjectTag),
        MSDPlanning_msg_tag(kWorldModelSceneObjectTag), 0);

    auto maf_worldmodel_scene_objects =
        std::make_shared<maf_worldmodel::SceneObjects>();
    ros_cpp_struct_convert::from_ros(scene_objects,
                                     *maf_worldmodel_scene_objects);
    planning_engine_->feed_world_model_scene_objects(
        maf_worldmodel_scene_objects);
  }

  void update_traffic_light(
      const perception_interface_msgs::TrafficLightPerception &traffic_light) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(traffic_light.header, kTrafficLightTag),
        MSDPlanning_msg_tag(kTrafficLightTag), 0);

    auto maf_traffic_light =
        std::make_shared<maf_perception_interface::TrafficLightPerception>();
    ros_cpp_struct_convert::from_ros(traffic_light, *maf_traffic_light);
    planning_engine_->feed_traffic_light(maf_traffic_light);
  }

  void update_fusion_objects(
      const perception_interface_msgs::PerceptionFusionObjectResult
          &fusion_objects) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(fusion_objects.header, kFusionObjectTag),
        MSDPlanning_msg_tag(kFusionObjectTag), 0);

    auto maf_fusion_objects = std::make_shared<
        maf_perception_interface::PerceptionFusionObjectResult>();
    ros_cpp_struct_convert::from_ros(fusion_objects, *maf_fusion_objects);
    planning_engine_->feed_fusion_objects(maf_fusion_objects);
  }

  void update_fusion_groundlines(
      const perception_interface_msgs::FusionGroundLineResult
          &fusion_groundlines) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(fusion_groundlines.header, kFusionGroundLineTag),
        MSDPlanning_msg_tag(kFusionGroundLineTag), 0);

    auto maf_fusion_groundlines =
        std::make_shared<maf_perception_interface::FusionGroundLineResult>();
    ros_cpp_struct_convert::from_ros(fusion_groundlines,
                                     *maf_fusion_groundlines);
    planning_engine_->feed_fusion_grond_lines(maf_fusion_groundlines);
  }

  void update_fusion_uss_groundlines(
      const perception_interface_msgs::FusionGroundLineResult
          &fusion_groundlines) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(fusion_groundlines.header, kFusionGroundLineTag),
        MSDPlanning_msg_tag(kFusionGroundLineTag), 0);

    auto maf_fusion_groundlines =
        std::make_shared<maf_perception_interface::FusionGroundLineResult>();
    ros_cpp_struct_convert::from_ros(fusion_groundlines,
                                     *maf_fusion_groundlines);
    planning_engine_->feed_fusion_uss_grond_lines(maf_fusion_groundlines);
  }

  void update_fusion_uss(
      const sensor_interface_msgs::UltrasonicUpaReport &fusion_uss) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(fusion_uss.header, kFusionUssTag),
        MSDPlanning_msg_tag(kFusionUssTag), 0);

    auto maf_fusion_uss =
        std::make_shared<maf_sensor_interface::UltrasonicUpaReport>();
    ros_cpp_struct_convert::from_ros(fusion_uss, *maf_fusion_uss);
    planning_engine_->feed_fusion_uss(maf_fusion_uss);
  }
  void update_ego_pose(const mla_localization_msgs::MLALocalization &msg) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(msg.header.stamp, kEgoPoseTag),
        MSDPlanning_msg_tag(kEgoPoseTag), 0);

    auto ego_pose = std::make_shared<maf_mla_localization::MLALocalization>();
    ros_cpp_struct_convert::from_ros(msg, *ego_pose);
    planning_engine_->feed_ego_pose(ego_pose);
  }

  void update_prediction_info(
      const worldmodel_msgs::PredictionResult &prediction_msg) {
    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(prediction_msg.header, kPredictionTag),
        MSDPlanning_msg_tag(kPredictionTag), 0);

    auto maf_prediction = std::make_shared<maf_worldmodel::PredictionResult>();
    ros_cpp_struct_convert::from_ros(prediction_msg, *maf_prediction);
    planning_engine_->feed_prediction_info(maf_prediction);
  }

  void update_mpc_trajectory(
      const planning_msgs::MpcTrajectoryResult &mpc_trajectory) {
    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(mpc_trajectory.header, kMpcTrajectorydTag),
        MSDPlanning_msg_tag(kMpcTrajectorydTag), 0);

    auto maf_mpc_trajectory =
        std::make_shared<maf_planning::MpcTrajectoryResult>();
    ros_cpp_struct_convert::from_ros(mpc_trajectory, *maf_mpc_trajectory);
    planning_engine_->feed_mpc_trajectory(maf_mpc_trajectory);
  }

  void update_sbp_result(const planning_msgs::SBPResult &msg) {
    maf_planning::SBPResult sbp_result{};
    ros_cpp_struct_convert::from_ros(msg, sbp_result);
    planning_engine_->feed_sbp_result(sbp_result);
  }

  void perception_vision_lane_callback(
      const perception_interface_msgs::RoadLinePerception
          &perception_vision_lane) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(perception_vision_lane.header,
                              kPerceptionVisionLaneTag),
        MSDPlanning_msg_tag(kPerceptionVisionLaneTag), 0);

    auto maf_perception_vision_lane =
        std::make_shared<maf_perception_interface::RoadLinePerception>();
    ros_cpp_struct_convert::from_ros(perception_vision_lane,
                                     *maf_perception_vision_lane);
    planning_engine_->feed_perception_vision_lane(maf_perception_vision_lane);
  }

  void perception_radar_callback(
      const perception_interface_msgs::RadarPerceptionResult
          &perception_radar) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(perception_radar.header, kPerceptionRararTag),
        MSDPlanning_msg_tag(kPerceptionRararTag), 0);

    auto maf_perception_radar =
        std::make_shared<maf_perception_interface::RadarPerceptionResult>();
    ros_cpp_struct_convert::from_ros(perception_radar, *maf_perception_radar);
    planning_engine_->feed_perception_radar(maf_perception_radar);
  }

  void planning_request_callback(
      const system_manager_msgs::SysPlanningRequest &planning_request) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(planning_request.header,
                              kPerceptionVisionLaneTag),
        MSDPlanning_msg_tag(kPerceptionVisionLaneTag), 0);

    maf_system_manager::SysPlanningRequest maf_planning_request;
    ros_cpp_struct_convert::from_ros(planning_request, maf_planning_request);
    planning_engine_->feed_planning_request(maf_planning_request);
  }

private:
  ros::NodeHandle nh_;
  std::shared_ptr<PlanningEngineInterface> planning_engine_;
  std::map<std::string, ros::Publisher> publisher_map_;
  std::map<std::string, ros::Subscriber> subscriber_map_;
  SceneType scene_type_ = SceneType::URBAN;
  bool enable_ilc_ = false;
  bool enable_alc_ = false;
  double ilc_limit_velocity_ = 0.0;
  double ilc_limit_time_ = 0.0;
  double default_cruise_velocity_ = 120.0;
  bool start_machine_ = false;
  bool is_acc_mode_ = false;
  bool is_ddmap_ = false;
  int driving_style_ = 1;

  // for wm
  bool init_wm_ = false;

  // universal topics
  static constexpr auto PLANNING = "planning";
  static constexpr auto MSD_MODULE_STATUS = "msd_module_status";
  static constexpr auto NODES_STATUS = "nodes_status";
  static constexpr auto CHASSIS_REPORT = "chassis_report";
  static constexpr auto WHEEL_REPORT = "wheel_report";
  static constexpr auto BODY_REPORT = "body_report";
  static constexpr auto IMU_REPORT = "imu_report";
  static constexpr auto WORLD_MODEL_MAP = "world_model_map";
  static constexpr auto WORLD_MODEL_OBJECTS = "world_model_objects";
  static constexpr auto WORLD_MODEL_SCENE_OBJECTS = "world_model_scene_objects";
  static constexpr auto WORLD_MODEL_PARKING_SLOTS = "world_model_parking_slots";
  static constexpr auto TRAFFIC_LIGHT = "traffic_light";
  static constexpr auto FUSION_OBJECTS = "fusion_objects";
  static constexpr auto FUSION_GROUNDLINES = "fusion_groundlines";
  static constexpr auto FUSION_USS = "fusion_uss";
  static constexpr auto EGO_POSE = "ego_pose";
  static constexpr auto PREDICTION = "prediciton";
  static constexpr auto MSD_CONTROL_MPC = "msd_control_mpc";
  static constexpr auto SBP_REQUEST = "msd_sbp_request";
  static constexpr auto SBP_RESULT = "msd_sbp_result";
  static constexpr auto PLAN_CONTROL_CMD_REQUEST =
      "planning_control_cmd_request";
  static constexpr auto PLAN_CONTROL_CMD_RESPONSE =
      "planning_control_cmd_response";
  static constexpr auto PLAN_REQUEST = "planning_request";
  static constexpr auto PLAN_RESPONSE = "planning_response";
  static constexpr auto PLAN_INFO = "planning_info";
  static constexpr auto PERCEPTION_VISION_LANE = "perception_vision_lane";
  static constexpr auto PERCEPTION_RADAR = "perception_radar";
};

} // namespace msd_planning

int main(int argc, char **argv) {
  ros::init(argc, argv, "decision_node");
  ros::NodeHandle nh("~");

  msd_planning::MSDDecisionNode decision_nd(nh);
  ros::spin();
  return 0;
}
