#pragma once

#ifdef USE_INFERJAM
#include "infer_jam/infer_jam.h"
using namespace infer_jam;
#endif

#include "data_driven_planner/common/basic_types.h"
#include "data_driven_planner/common/ddp_config.h"

namespace msquare {
namespace ddp {

enum class VectorType : int {
  EGO = 0,
  CAR = 1,
  OFO = 2,
  HUMAN = 3,
  CONE = 4,
  ROUTE_ON_ROUTE = 5,
  ROUTE_OFF_ROUTE = 6,
  ROUTE_BUS = 7,
  LANE_PROPOSAL_ON_ROUTE = 8,
  LANE_PROPOSAL_OFF_ROUTE = 9,
  LANE_PROPOSAL_BUS = 10,
  SOLID_LANE_BOUNDARY = 11,
  DASHED_LANE_BOUNDARY = 12,
  SOLID_ROAD_BOUNDARY = 13,
  DASHED_ROAD_BOUNDARY = 14,
  STATIC_OBSTACLE = 15,
  JUNCTION_LINE = 16,
  OTHER_LINE = 17,
  UNKNOWN = 18,
  INVALID = 19,
};

enum class ObjFeatMeaningType : int {
  X_START = 0,
  Y_START = 1,
  X_END = 2,
  Y_END = 3,
  VX = 4,
  VY = 5,
  YAW = 6,
  LENGTH = 7,
  WIDTH = 8,
  TYPE = 9,
  THETA = 10,
};

enum class StaticFeatMeaning : int {
  X_START = 0,
  Y_START = 1,
  X_END = 2,
  Y_END = 3,
  TYPE = 4,
  ON_ROUTE_DIST = 5,
};

struct DdpModelInputInfo {
  // for simplify
  int num_agent_actual = 0;
  int num_map_polyline_actual = 0;
  // for ts + egopose
  double fusion_object_info_ts = 0;
  double vision_lane_ts = 0;
  double lane_proposals_ts = 0;
  // for necessary timestamp
  std::unordered_map<int, std::map<double, std::vector<double>>>
      fusion_obj_id_ts_info;
};

#ifdef USE_INFERJAM
class DdpModel {
public:
  explicit DdpModel(const std::shared_ptr<DdpConfigBuilder> &config_builder);
  virtual ~DdpModel() = default;

  bool run(double timestamp, DdpTrajectorys &ddp_trajectorys,
           std::string &ddp_model_input, bool is_replan);

protected:
  // initialize
  void load_model();
  bool run_model(double timestamp, DdpTrajectorys &ddp_trajectorys,
                 bool is_replan, std::string &ddp_model_input);
  std::string get_model_platform();

  // input
  bool get_ego_pose(double timestamp, EgoPose &ego_pose, bool is_replan);
  void get_object_input_tensor(double timestamp, const EgoPose &ego_pose,
                               float *feature_obj_ptr, float *mask_obj_ptr,
                               float *rotate_matrix_ptr);
  void get_map_input_tensor(double timestamp, const EgoPose &ego_pose,
                            float *feature_static_ptr, float *mask_static_ptr,
                            DdpTrajectorys &ddp_trajectorys);
  void get_traffic_light_tensor(const EgoPose &ego_pose,
                                float *traffic_light_status_ptr,
                                float *traffic_light_extra_ptr);

  // output
  void process_output(const EgoPose &ego_pose, const float *lane_logits_ptr,
                      const float *plan_trajs_ptr, DdpTrajectorys &trajectorys);

  // utils
  float parse_object_type(const ObjectType type);

  // debug
  template <typename Arr, typename Ptr>
  void emplace_back_model_input_completely(const int array_size, Arr &arr,
                                           const Ptr ptr);
  template <typename Arr, typename Ptr>
  void
  emplace_back_model_input_simplify(const int num_outside, const int num_medium,
                                    const int num_inner, const int num_feature,
                                    Arr &arr_sim, const Ptr ptr);
  void set_ddp_model_input_info_default();
  void get_ddp_model_input(
      double timestamp, const EgoPose *ego_pose_plan_ptr,
      const float *feature_obj_ptr, const float *mask_obj_ptr,
      const float *feature_static_ptr, const float *mask_static_ptr,
      const float *traffic_light_status_ptr,
      const float *traffic_light_extra_ptr, const float *speed_limit_ptr,
      const float *style_feature_ptr, const float *rotate_matrix_ptr,
      std::string &ddp_model_input);
  void save_model_input(double timestamp, const float *feature_obj_ptr,
                        const float *mask_obj_ptr,
                        const float *feature_static_ptr,
                        const float *mask_static_ptr,
                        const float *traffic_light_status_ptr,
                        const float *traffic_light_extra_ptr,
                        const float *rotate_matrix_ptr);
  void save_model_output(double timestamp, const float *lane_logits_ptr,
                         const float *plan_trajs_ptr,
                         const float *pred_traj_ptr);

  void draw_debug_image(double timestamp);

private:
  DdpModelConfig config_;
  std::shared_ptr<NetManager> net_manager_;
  std::shared_ptr<Plan> plan_;
  bool has_load_model_completed_ = false;
  DdpModelInputInfo ddp_model_input_info_;
};

#else

class DdpModel {
public:
  explicit DdpModel(const std::shared_ptr<DdpConfigBuilder> &config_builder);
  virtual ~DdpModel() = default;

  bool run(double timestamp, DdpTrajectorys &ddp_trajectorys,
           std::string &ddp_model_input, bool is_replan);

private:
  DdpModelConfig config_;
};

#endif

} // namespace ddp
} // namespace msquare
