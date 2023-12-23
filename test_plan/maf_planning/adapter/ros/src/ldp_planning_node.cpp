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
#include "pnc/ldp_planning_engine_interface.h"
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

class LdpPlanningNode {
public:
  explicit LdpPlanningNode(ros::NodeHandle &nh) : nh_(nh) {
    ros::NodeHandle ___nh_param_reader = nh_;
    READ_STRING_PARAM_WITH_DEFAULT(config_file, "./config.json");
    READ_STRING_PARAM_WITH_DEFAULT(mtaskflow_config_file,
                                   "./mtaskflow_config.json");
    READ_STRING_PARAM_WITH_DEFAULT(config_file_dir, "./config");
    READ_INT_PARAM_WITH_DEFAULT(scene_type, 0);
    READ_GLOBAL_BOOL_PARAM_WITH_DEFAULT(use_sim_time, false);
    // universal topics
    READ_STRING_PARAM_WITH_DEFAULT(pub_planning_topic, "/msd/planning/plan");
    READ_STRING_PARAM_WITH_DEFAULT(pub_planning_ldp_topic,
                                   "/planning/ldp_status");
    READ_STRING_PARAM_WITH_DEFAULT(msd_module_status_topic,
                                   "/msd/function_module_status");
    READ_STRING_PARAM_WITH_DEFAULT(pub_nodes_status_topic,
                                   "/node_status/ldp_planning");
    READ_STRING_PARAM_WITH_DEFAULT(sub_chassis_report_topic,
                                   "/vehicle/chassis_report");
    READ_STRING_PARAM_WITH_DEFAULT(sub_wheel_report_topic,
                                   "/vehicle/wheel_report");
    READ_STRING_PARAM_WITH_DEFAULT(sub_body_report_topic,
                                   "/vehicle/body_report");
    READ_STRING_PARAM_WITH_DEFAULT(sub_perception_vision_lane_topic,
                                   "/perception/vision/lane");
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

      MSDLdpPlanningConfig planning_config{true, mtaskflow_config_file.c_str()};

      planning_engine_ = LdpPlanningEngineInterface::make(planning_config);

      planning_engine_->init();

      if (start_machine_) {
        planning_engine_->start();
      }

      auto no_delay = ros::TransportHints().tcpNoDelay(true);

      publisher_map_[PLANNING] =
          nh_.advertise<planning_msgs::Planning>(pub_planning_topic, 1);
      publisher_map_[LDP_STATUS] =
          nh_.advertise<std_msgs::Header>(pub_planning_ldp_topic, 1);
      publisher_map_[MSD_MODULE_STATUS] =
          nh_.advertise<framework_status_msgs::ModuleStatus>(
              msd_module_status_topic, 1);

      subscriber_map_[MSD_MODULE_STATUS] =
          nh_.subscribe(msd_module_status_topic, 1,
                        &LdpPlanningNode::update_module_status, this, no_delay);
      publisher_map_[NODES_STATUS] =
          nh_.advertise<framework_status_msgs::NodesStatus>(
              pub_nodes_status_topic, 1);
      subscriber_map_[CHASSIS_REPORT] = nh_.subscribe(
          sub_chassis_report_topic, 1,
          &LdpPlanningNode::chassis_report_callback, this, no_delay);
      subscriber_map_[WHEEL_REPORT] = nh_.subscribe(
          sub_wheel_report_topic, 1, &LdpPlanningNode::wheel_report_callback,
          this, no_delay);
      subscriber_map_[BODY_REPORT] =
          nh_.subscribe(sub_body_report_topic, 1,
                        &LdpPlanningNode::body_report_callback, this, no_delay);
      subscriber_map_[PERCEPTION_VISION_LANE] = nh_.subscribe(
          sub_perception_vision_lane_topic, 1,
          &LdpPlanningNode::perception_vision_lane_callback, this, no_delay);

      auto &pub_planning = publisher_map_[PLANNING];
      planning_engine_->set_callback(
          [&pub_planning](const MSDPlanningOutputMeta &meta,
                          const maf_planning::Planning &planning_result,
                          const std::string &trigger_msg_id) {
            if (meta.succeed) {
              planning_msgs::Planning planning_msg{};
              ros_cpp_struct_convert::to_ros(planning_result, planning_msg);

              MSDPlanning_record_relation(
                  mlog::MLOG_msg_id_ros(planning_msg.header, kLdpPlanTag),
                  trigger_msg_id);

              pub_planning.publish(planning_msg);
            }
          });

      auto &pub_ldp_status = publisher_map_[LDP_STATUS];
      planning_engine_->set_callback(
          [&pub_ldp_status](const std::string &ldp_output) {
            maf_std::Header maf_ldp_status{};
            std_msgs::Header ldp_status_msg{};
            maf_ldp_status.stamp = MTIME()->timestamp().ns();
            maf_ldp_status.frame_id = ldp_output;
            ros_cpp_struct_convert::to_ros(maf_ldp_status, ldp_status_msg);
            pub_ldp_status.publish(ldp_status_msg);
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

    } catch (std::exception &e) {
      ROS_ERROR("Error: %s", e.what());
      abort();
    }
  }
  virtual ~LdpPlanningNode() = default;

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
    start_machine_ = config_reader.get<bool>("start_machine", false, false);
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

  void perception_vision_lane_callback(
      const perception_interface_msgs::RoadLinePerception
          &perception_vision_lane) {

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id_ros(perception_vision_lane.header,
                              kPerceptionVisionLaneTag),
        MSDPlanning_msg_tag(kBodyReportTag), 0);

    auto maf_perception_vision_lane =
        std::make_shared<maf_perception_interface::RoadLinePerception>();
    ros_cpp_struct_convert::from_ros(perception_vision_lane,
                                     *maf_perception_vision_lane);
    planning_engine_->feed_perception_vision_lane(maf_perception_vision_lane);
  }

private:
  ros::NodeHandle nh_;
  std::shared_ptr<LdpPlanningEngineInterface> planning_engine_;
  std::map<std::string, ros::Publisher> publisher_map_;
  std::map<std::string, ros::Subscriber> subscriber_map_;

  bool start_machine_ = false;

  // universal topics
  static constexpr auto PLANNING = "planning";
  static constexpr auto LDP_STATUS = "ldp_status";
  static constexpr auto MSD_MODULE_STATUS = "msd_module_status";
  static constexpr auto NODES_STATUS = "node_status";
  static constexpr auto CHASSIS_REPORT = "chassis_report";
  static constexpr auto WHEEL_REPORT = "wheel_report";
  static constexpr auto BODY_REPORT = "body_report";
  static constexpr auto PERCEPTION_VISION_LANE = "perception_vision_lane";
};

} // namespace msd_planning

int main(int argc, char **argv) {
  ros::init(argc, argv, "ldp_planning_node");
  ros::NodeHandle nh("~");

  msd_planning::LdpPlanningNode ldp_planning_node(nh);
  ros::spin();
  return 0;
}
