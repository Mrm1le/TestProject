#ifndef MSQUARE_DECISION_PLANNING_PLANNER_PLANNER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_PLANNER_H_

#include <memory>

#include "common/parking_world_model.h"

namespace msquare {
namespace parking {

class WorldModel;
class Planner {
public:
  Planner(const std::shared_ptr<WorldModel> &world_model)
      : world_model_(world_model) {}
  virtual ~Planner() = 0;

  virtual bool calculate() = 0;

  std::shared_ptr<WorldModel> get_world_model() { return world_model_; }

protected:
  std::shared_ptr<WorldModel> world_model_;
};

inline Planner::~Planner() {}
} // namespace parking
} // namespace msquare

#endif
