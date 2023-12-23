#pragma once

#include "mfr/mfr.h"
#include <ros/ros.h>

#include "endpoint_msgs/ros_mfr_convert.hpp"
#include "framework_status_msgs/ros_mfr_convert.hpp"
#include "gps_imu_msgs/ros_mfr_convert.hpp"
#include "horizon_msgs/ros_mfr_convert.hpp"
#include "mla_localization_msgs/ros_mfr_convert.hpp"
#include "mlog_msgs/ros_mfr_convert.hpp"
#include "perception_interface_msgs/ros_mfr_convert.hpp"
#include "planning_msgs/ros_mfr_convert.hpp"
#include "sensor_interface_msgs/ros_mfr_convert.hpp"
#include "std_msgs/ros_mfr_convert.hpp"
#include "worldmodel_msgs/ros_mfr_convert.hpp"

static constexpr auto node_yaml = R"(
node_config:
  memory:
    total_virtual_memory_size_MB: 0
    total_shared_memory_size_MB: 0
)";

static constexpr auto machine_yaml = R"(
log:
  level: info
  enable_stderr: true
  export_frequence: 0
)";

void register_node(const mmemory::MFString &node_name) {
  mfr::MFRNodeConfig node_config{};
  node_config.node_type = node_name + "_type";
  node_config.node_name = node_name;
  node_config.node_param_yaml = node_yaml;
  mfr::MFRNodeMachine::instance().register_node(node_config);
}

void init_machine(const mmemory::MFString &mfrrpc,
                  const mmemory::MFString &machine_name) {
  mfr::MFRMachineConfig machine_config{};
  machine_config.machine_url = mfrrpc;
  machine_config.machine_name = machine_name;
  machine_config.machine_param_yaml = machine_yaml;
  mfr::MFRNodeMachine::instance().init(machine_config);
}

void run_machine() {
  mfr::MFRNodeMachine::instance().run();
  puts("[mfrmaster] Start mfrmaster success.");
}

void stop_machine() {
  mfr::MFRNodeMachine::instance().stop();
  mfr::MFRNodeMachine::instance().join();
  mfr::MFRNodeMachine::instance().reset();
  puts("Stop machine success.");
}
