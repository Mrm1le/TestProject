#include "convert/convert.h"
#include "pnc/search_based_planning_engine_interface.h"
#include <ros/ros.h>

namespace msquare {

class SearchBasedPlanningNode {
private:
  void onCallbackSBPRequest(const planning_msgs::SBPRequest &msg) {
    // MSD_LOG(INFO, "search_based_planning_node receives a request.");
    maf_planning::SBPRequest request{};
    ros_cpp_struct_convert::from_ros(msg, request);
    sbp_engine_->feedSBPRequest(request);
  }
  void onCallbackSbpResult(const maf_planning::SBPResult &sbp_result) {
    planning_msgs::SBPResult msg{};
    ros_cpp_struct_convert::to_ros(sbp_result, msg);
    result_pub_.publish(msg);
  }
  std::shared_ptr<SearchBasedPlanningEngineInterface> sbp_engine_;
  ros::NodeHandle nh_;
  ros::Subscriber request_sub_;
  ros::Publisher result_pub_;

public:
  explicit SearchBasedPlanningNode(ros::NodeHandle &nh);
  ~SearchBasedPlanningNode();
};

SearchBasedPlanningNode::SearchBasedPlanningNode(ros::NodeHandle &nh)
    : nh_(nh) {
  std::string vehicle_calib_file;
  std::string mtaskflow_config_file;
  nh_.getParam("vehicle_calib_file", vehicle_calib_file);
  nh_.getParam("mtaskflow_config_file", mtaskflow_config_file);
  bool enable_dump_file;
  nh_.getParam("enable_dump_file", enable_dump_file);
  auto no_delay = ros::TransportHints().tcpNoDelay(true);
  request_sub_ = nh_.subscribe("/msd/sbp_request", 1,
                               &SearchBasedPlanningNode::onCallbackSBPRequest,
                               this, no_delay);
  result_pub_ = nh_.advertise<planning_msgs::SBPResult>("/msd/sbp_result", 1);

  sbp_engine_ = SearchBasedPlanningEngineInterface::make(
      vehicle_calib_file.c_str(), mtaskflow_config_file.c_str());
  sbp_engine_->setEnableDumpFile(enable_dump_file);
  sbp_engine_->setCallback(
      std::bind(&SearchBasedPlanningNode::onCallbackSbpResult, this,
                std::placeholders::_1));
}

SearchBasedPlanningNode::~SearchBasedPlanningNode() {}

} // namespace msquare

int main(int argc, char **argv) {
  ros::init(argc, argv, "search_based_planning_node");
  ros::NodeHandle nh("~");

  msquare::SearchBasedPlanningNode sbp_nd(nh);
  ros::spin();
  return 0;
}