#pragma once

#include "common/planning_context.h"

#include "common/utils/yaml_utils.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace msquare {

namespace parking {

class ApaStateMachineLogger {
private:
  ApaStateMachineLogger();
  ~ApaStateMachineLogger();

public:
  //[fenix.refactor.sm] moved from parking_scenario_manager.cpp
  static void log_debug_info() {
    MLOG_PROFILING("log_debug_info");

    std::string parking_planning_debug_msg;
    create_parking_planning_debug_msg(parking_planning_debug_msg);
    MSD_WRN_TAG_STR(parking_planning_debug, json_str,
                    parking_planning_debug_msg.c_str());

    std::string openspace_problem_msg;
    if (create_openspace_problem_msg(openspace_problem_msg)) {
      MSD_WRN_TAG_STR(openspace_problem, json_str,
                      openspace_problem_msg.c_str());
    }
  }

private:
  //[fenix.refactor.sm] moved from parking_scenario_manager.cpp
  static void create_parking_planning_debug_msg(std::string &msg) {
    const PlanningStatus &planning_status =
        PlanningContext::Instance()->planning_status();
    const PlanningResult &planning_result = planning_status.planning_result;
    const ParkingBehaviorPlannerOutput &parking_behavior_planner_output =
        PlanningContext::Instance()->parking_behavior_planner_output();
    const LongitudinalBehaviorPlannerOutput
        &longitudinal_behavior_planner_output =
            PlanningContext::Instance()->longitudinal_behavior_planner_output();
    string version = PlanningContext::Instance()->get_version();

    namespace rp = rapidjson;
    rp::Document json_doc;
    json_doc.SetObject();
    auto &alloc = json_doc.GetAllocator();

    // planning status
    rp::Value planning_status_msg(rp::kObjectType);
    planning_status_msg.AddMember(
        "v_limit", rp::Value().SetDouble(planning_status.v_limit), alloc);
    planning_status_msg.AddMember(
        "a_limit", rp::Value().SetDouble(planning_status.a_limit), alloc);
    planning_status_msg.AddMember(
        "version", rp::Value().SetString(version.c_str(), version.length()),
        alloc);
    planning_status_msg.AddMember(
        "blocked", rp::Value().SetBool(planning_status.blocked), alloc);
    planning_status_msg.AddMember(
        "block_timeout_duration",
        rp::Value().SetDouble(planning_status.block_timeout_duration), alloc);

    rp::Value scenario_msg(rp::kObjectType);
    scenario_msg.AddMember(
        "scenario_type",
        rp::Value().SetString(planning_status.scenario.scenario_type.c_str(),
                              planning_status.scenario.scenario_type.length()),
        alloc);
    scenario_msg.AddMember(
        "stage_type",
        rp::Value().SetString(planning_status.scenario.stage_type.c_str(),
                              planning_status.scenario.scenario_type.length()),
        alloc);
    scenario_msg.AddMember(
        "status_type", rp::Value().SetInt(planning_status.scenario.status_type),
        alloc);

    planning_status_msg.AddMember("scenario", scenario_msg, alloc);

    rp::Value planning_result_msg(rp::kObjectType);
    planning_result_msg.AddMember(
        "v_target", rp::Value().SetDouble(planning_result.v_target), alloc);
    planning_result_msg.AddMember(
        "a_target", rp::Value().SetDouble(planning_result.a_target), alloc);
    planning_result_msg.AddMember(
        "pwj_status", rp::Value().SetBool(planning_result.pwj_status), alloc);
    planning_result_msg.AddMember(
        "is_apa", rp::Value().SetBool(planning_result.is_apa), alloc);
    planning_result_msg.AddMember(
        "is_entrance", rp::Value().SetBool(planning_result.is_entrance), alloc);
    planning_result_msg.AddMember(
        "scene_avp", rp::Value().SetInt(planning_result.scene_avp), alloc);
    planning_result_msg.AddMember(
        "sidepass_type", rp::Value().SetInt(planning_result.sidepass_type),
        alloc);
    planning_result_msg.AddMember(
        "gear", rp::Value().SetInt((int)planning_result.gear), alloc);

    planning_status_msg.AddMember("planning_result", planning_result_msg,
                                  alloc);

    json_doc.AddMember("planning_status", planning_status_msg, alloc);

    rp::Value parking_behavior_planner_output_msg(rp::kObjectType);
    parking_behavior_planner_output_msg.AddMember(
        "planner_type",
        rp::Value().SetInt((int)parking_behavior_planner_output.planner_type),
        alloc);
    parking_behavior_planner_output_msg.AddMember(
        "is_move_ready",
        rp::Value().SetBool(parking_behavior_planner_output.is_move_ready),
        alloc);
    parking_behavior_planner_output_msg.AddMember(
        "is_finish",
        rp::Value().SetBool(parking_behavior_planner_output.is_finish), alloc);

    json_doc.AddMember("parking_behavior_planner_output",
                       parking_behavior_planner_output_msg, alloc);

    rp::Value lead_msg(rp::kObjectType);
    rp::Value lead_one_msg(rp::kObjectType);
    rp::Value lead_two_msg(rp::kObjectType);
    lead_one_msg.AddMember(
        "id",
        rp::Value().SetInt(
            longitudinal_behavior_planner_output.lead_cars.first.id),
        alloc);
    lead_one_msg.AddMember(
        "type",
        rp::Value().SetInt(
            (int)longitudinal_behavior_planner_output.lead_cars.first.type),
        alloc);
    lead_one_msg.AddMember(
        "d_rel",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.lead_cars.first.d_rel),
        alloc);
    lead_one_msg.AddMember(
        "d_path",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.lead_cars.first.d_path),
        alloc);
    lead_one_msg.AddMember(
        "v_lat",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.lead_cars.first.v_lat),
        alloc);
    lead_one_msg.AddMember(
        "v_lon",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.lead_cars.first.v_lon),
        alloc);
    lead_one_msg.AddMember(
        "a_lat",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.lead_cars.first.a_lat),
        alloc);
    lead_one_msg.AddMember(
        "a_lon",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.lead_cars.first.a_lon),
        alloc);
    lead_one_msg.AddMember(
        "yaw_relative_frenet",
        rp::Value().SetDouble(longitudinal_behavior_planner_output.lead_cars
                                  .first.yaw_relative_frenet),
        alloc);
    lead_one_msg.AddMember(
        "is_static",
        rp::Value().SetBool(
            longitudinal_behavior_planner_output.lead_cars.first.is_static),
        alloc);
    lead_one_msg.AddMember(
        "is_sidepass_obj",
        rp::Value().SetBool(longitudinal_behavior_planner_output.lead_cars.first
                                .is_sidepass_obj),
        alloc);

    lead_two_msg.AddMember(
        "id",
        rp::Value().SetInt(
            longitudinal_behavior_planner_output.lead_cars.second.id),
        alloc);
    lead_two_msg.AddMember(
        "type",
        rp::Value().SetInt(
            (int)longitudinal_behavior_planner_output.lead_cars.second.type),
        alloc);
    lead_two_msg.AddMember(
        "d_rel",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.lead_cars.second.d_rel),
        alloc);
    lead_two_msg.AddMember(
        "d_path",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.lead_cars.second.d_path),
        alloc);
    lead_two_msg.AddMember(
        "v_lat",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.lead_cars.second.v_lat),
        alloc);
    lead_two_msg.AddMember(
        "v_lon",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.lead_cars.second.v_lon),
        alloc);
    lead_two_msg.AddMember(
        "a_lat",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.lead_cars.second.a_lat),
        alloc);
    lead_two_msg.AddMember(
        "a_lon",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.lead_cars.second.a_lon),
        alloc);
    lead_two_msg.AddMember(
        "yaw_relative_frenet",
        rp::Value().SetDouble(longitudinal_behavior_planner_output.lead_cars
                                  .second.yaw_relative_frenet),
        alloc);
    lead_two_msg.AddMember(
        "is_static",
        rp::Value().SetBool(
            longitudinal_behavior_planner_output.lead_cars.second.is_static),
        alloc);
    lead_two_msg.AddMember(
        "is_sidepass_obj",
        rp::Value().SetBool(longitudinal_behavior_planner_output.lead_cars
                                .second.is_sidepass_obj),
        alloc);

    lead_msg.AddMember("LeadOne", lead_one_msg, alloc);
    lead_msg.AddMember("LeadTwo", lead_two_msg, alloc);
    json_doc.AddMember("Lead", lead_msg, alloc);

    rp::Value freespace_line_msg(rp::kObjectType);
    freespace_line_msg.AddMember(
        "id",
        rp::Value().SetInt(longitudinal_behavior_planner_output.fs_line.id),
        alloc);
    freespace_line_msg.AddMember(
        "d_rel",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.fs_line.d_rel),
        alloc);
    freespace_line_msg.AddMember(
        "x_start",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.fs_line.x_start),
        alloc);
    freespace_line_msg.AddMember(
        "x_end",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.fs_line.x_end),
        alloc);
    freespace_line_msg.AddMember(
        "y_start",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.fs_line.y_start),
        alloc);
    freespace_line_msg.AddMember(
        "y_end",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.fs_line.y_end),
        alloc);
    freespace_line_msg.AddMember(
        "s_start",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.fs_line.s_start),
        alloc);
    freespace_line_msg.AddMember(
        "s_end",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.fs_line.s_end),
        alloc);
    freespace_line_msg.AddMember(
        "l_start",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.fs_line.l_start),
        alloc);
    freespace_line_msg.AddMember(
        "l_end",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.fs_line.l_end),
        alloc);
    json_doc.AddMember("freespace_line", freespace_line_msg, alloc);

    rp::Value freespace_point_msg(rp::kObjectType);
    freespace_point_msg.AddMember(
        "id",
        rp::Value().SetInt(longitudinal_behavior_planner_output.free_space.id),
        alloc);
    freespace_point_msg.AddMember(
        "d_rel",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.free_space.d_rel),
        alloc);
    freespace_point_msg.AddMember(
        "x",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.free_space.x),
        alloc);
    freespace_point_msg.AddMember(
        "y",
        rp::Value().SetDouble(
            longitudinal_behavior_planner_output.free_space.y),
        alloc);
    json_doc.AddMember("freespace_point", freespace_point_msg, alloc);

    // json_doc.AddMember("ParkingBehavior",
    // rp::Value().SetString(PlanningContext::Instance()->parking_behavior_planner_output().debug_stringstream.str().c_str()),
    // alloc);

    // PlanningContext::Instance()->mutable_planning_status()->statemachine_stringstream.reset();
    // // clear
    // PlanningContext::Instance()->mutable_parking_behavior_planner_output()->debug_stringstream.str("");
    // // clear

    rp::StringBuffer buffer;
    rp::Writer<rp::StringBuffer> writer(buffer);
    (void)json_doc.Accept(writer);
    msg = std::string(buffer.GetString(), buffer.GetSize());
  }

  static bool create_openspace_problem_msg(std::string &msg) {

    YAML::Node problem_node(
        PlanningContext::Instance()->openspace_decider_output());
    if (problem_node["is_request_square_map"].as<bool>()) {
      namespace rp = rapidjson;
      rp::Document json_doc;
      json_doc.SetObject();
      auto &alloc = json_doc.GetAllocator();

      mlog::MLogDataStream problem_definition_ss;
      mlog::MLogDataStream problem_config_ss;
      problem_definition_ss << problem_node;
      problem_config_ss << HybridAstarConfig::GetInstance()->yaml_node_;

      rp::Value problem_definition;
      rp::Value problem_config;

      problem_definition.SetString(problem_definition_ss.str().c_str(),
                                   problem_definition_ss.str().length(), alloc);
      problem_config.SetString(problem_config_ss.str().c_str(),
                               problem_config_ss.str().length(), alloc);

      json_doc.AddMember("problem_definition", problem_definition, alloc);
      json_doc.AddMember("problem_config", problem_config, alloc);

      rp::StringBuffer buffer;
      rp::Writer<rp::StringBuffer> writer(buffer);
      (void)json_doc.Accept(writer);
      msg = std::string(buffer.GetString(), buffer.GetSize());
    }

    return problem_node["is_request_square_map"].as<bool>();
  }
};

} // namespace parking

} // namespace msquare
