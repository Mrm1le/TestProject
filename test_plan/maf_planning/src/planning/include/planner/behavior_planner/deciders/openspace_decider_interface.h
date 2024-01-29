#pragma once
#include "common/math/map_line.h"
#include "common/math/math_utils.h"
#include "common/parking_world_model.h"

namespace msquare {
namespace parking {

class BaseOpenspaceDecider {
public:
  BaseOpenspaceDecider(const std::shared_ptr<WorldModel> &world_model);
  virtual ~BaseOpenspaceDecider() = default;
  virtual void requestSquareMapping() = 0;
  virtual void process() = 0;
  virtual void reset();

  // TODO: unify
  virtual void set_init_and_target(const TrajectoryPoint &init_state,
                                   const TrajectoryPoint &target_state) {}
  virtual TrajectoryPoint get_init() { return TrajectoryPoint(); }
  bool is_target_valid();
  // void dump_to_file(const std::string file_prefix, const
  // OpenspaceDeciderOutput& context);
  void feedOutput();
  virtual bool isReverseSearchRequired();
  virtual bool isNarrowChannelScenario() { return false; }

protected:
  std::shared_ptr<WorldModel> world_model_;
  TrajectoryPoint target_state_;
  TrajectoryPoint ego_state_;
  planning_math::Box2d map_boundary_;
  std::vector<ObstacleLine> lines_;
  std::vector<ObstacleBox> boxes_;
  std::vector<planning_math::Vec2d> points_;
  std::vector<planning_math::Vec2d> step_points_;
  std::string log_file_prefix_;
};

} // namespace parking
} // namespace msquare
