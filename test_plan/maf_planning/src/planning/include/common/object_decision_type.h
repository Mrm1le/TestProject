#ifndef MODULES_PLANNING_OPTIMIZERS_OBJECT_DECISION_TYPE_H_
#define MODULES_PLANNING_OPTIMIZERS_OBJECT_DECISION_TYPE_H_

#include "common/math/vec2d.h"
#include "planner/message_type.h"
#include <boost/optional.hpp>
#include <string>

namespace msquare {

using namespace boost;

typedef struct {
  std::string info;
} ObjectIgnore;

typedef struct {
  planning_math::Vec2d stop_point;
  double distance_s;
  double stop_heading;
} ObjectStop;

typedef struct {
  enum Type { LEFT_NUDGE, RIGHT_NUDGE, NO_NUDGE };
  Type type;
  bool is_longitunidal_ignored;
  double distance_l;
  double start_time;
  double time_buffer;
  int priority = 1;
} ObjectNudge;

typedef struct {
  planning_math::Vec2d fence_point;
  double distance_s;
  double fence_heading;
  double start_time;
  double time_buffer;
} ObjectYield;

typedef struct {
  planning_math::Vec2d fence_point;
  double distance_s;
  double fence_heading;
  double start_time;
  double time_buffer;
  int ObsTag;
  bool in_origin_lane;
} ObjectFollow;

typedef struct {
  planning_math::Vec2d fence_point;
  double distance_s;
  double fence_heading;
  double start_time;
  double time_buffer;
  int ObsTag;
} ObjectOvertake;

typedef struct {
  enum Type { LEFT, RIGHT };
  Type type;
} ObjectSidePass;

typedef struct {
  int level;
} ObjectAvoid;

typedef struct {
  double distance_s;
  double min_distance;
  bool is_collision;
  ObjectType type;
} ObjectLead;

typedef struct {
  int ID;
  double TTC_pwj;
  double TTC_ego;
  double s_prediction;
  double decay_rate;
  ObjectType type;
} OtherCarsYield;

class ObjectDecisionType {
public:
  explicit ObjectDecisionType() { object_tag_case_ = OBJECT_TAG_NOT_SET; }

  ~ObjectDecisionType() = default;

  bool has_stop() const { return !stop_ ? false : true; }
  bool has_nudge() const { return !nudge_ ? false : true; }
  bool has_yield() const { return !yield_ ? false : true; }
  bool has_follow() const { return !follow_ ? false : true; }
  bool has_overtake() const { return !overtake_ ? false : true; }
  bool has_avoid() const { return !avoid_ ? false : true; }
  bool has_ignore() const { return !ignore_ ? false : true; }
  bool has_lead() const { return !lead_ ? false : true; }
  bool has_sidepass() const { return !sidepass_ ? false : true; }
  bool has_other_cars_yield() const {
    return !other_cars_yield_ ? false : true;
  }

  ObjectStop *mutable_stop() {
    if (has_stop()) {
      return stop_.get_ptr();
    }
    ObjectStop stop = {};
    setStopDecision(stop);
    return stop_.get_ptr();
  }
  ObjectNudge *mutable_nudge() {
    if (has_nudge()) {
      return nudge_.get_ptr();
    }
    ObjectNudge nudge = {};
    setNudgeDecision(nudge);
    return nudge_.get_ptr();
  }
  ObjectYield *mutable_yield() {
    if (has_yield()) {
      return yield_.get_ptr();
    }
    ObjectYield yield = {};
    setYieldDecision(yield);
    return yield_.get_ptr();
  }
  ObjectFollow *mutable_follow() {
    if (has_follow()) {
      return follow_.get_ptr();
    }
    ObjectFollow follow = {};
    setFollowDecision(follow);
    return follow_.get_ptr();
  }
  ObjectOvertake *mutable_overtake() {
    if (has_overtake()) {
      return overtake_.get_ptr();
    }
    ObjectOvertake overtake = {};
    setOvertakeDecision(overtake);
    return overtake_.get_ptr();
  }
  ObjectAvoid *mutable_avoid() {
    if (has_avoid()) {
      return avoid_.get_ptr();
    }
    setAvoidDecision(ObjectAvoid());
    return avoid_.get_ptr();
  }
  ObjectIgnore *mutable_ignore() {
    if (has_ignore()) {
      return ignore_.get_ptr();
    }
    setIgnoreDecision(ObjectIgnore());
    return ignore_.get_ptr();
  }
  ObjectLead *mutable_lead() {
    setLeadDecision(ObjectLead());
    return lead_.get_ptr();
  }
  ObjectSidePass *mutable_sidepass() {
    setSidepassDecision(ObjectSidePass());
    return sidepass_.get_ptr();
  }
  OtherCarsYield *mutable_other_cars_yield() {
    setOtherCarsYieldDecision(OtherCarsYield());
    return other_cars_yield_.get_ptr();
  }

  ObjectStop stop() const { return *stop_; }
  ObjectNudge nudge() const { return *nudge_; }
  ObjectYield yield() const { return *yield_; }
  ObjectFollow follow() const { return *follow_; }
  ObjectOvertake overtake() const { return *overtake_; }
  ObjectAvoid avoid() const { return *avoid_; }
  ObjectIgnore ignore() const { return *ignore_; }
  std::string decision_source() const { return decision_source_; }
  ObjectLead lead() const { return *lead_; }
  ObjectSidePass sidepass() const { return *sidepass_; }
  OtherCarsYield other_cars_yield() const { return *other_cars_yield_; }

  void clear() {
    stop_ = none;
    nudge_ = none;
    yield_ = none;
    follow_ = none;
    overtake_ = none;
    avoid_ = none;
    ignore_ = none;
    lead_ = none;
    sidepass_ = none;
    other_cars_yield_ = none;
  }

  void setStopDecision(ObjectStop stop) {
    clear();
    stop_ = stop;
    object_tag_case_ = kStop;
  }
  void setNudgeDecision(ObjectNudge nudge) {
    clear();
    nudge_ = nudge;
    object_tag_case_ = kNudge;
  }
  void setYieldDecision(ObjectYield yield) {
    clear();
    yield_ = yield;
    object_tag_case_ = kYield;
  }
  void setFollowDecision(ObjectFollow follow) {
    clear();
    follow_ = follow;
    object_tag_case_ = kFollow;
  }
  void setOvertakeDecision(ObjectOvertake overtake) {
    clear();
    overtake_ = overtake;
    object_tag_case_ = kOvertake;
  }
  void setAvoidDecision(ObjectAvoid avoid) {
    clear();
    avoid_ = avoid;
    object_tag_case_ = kAvoid;
  }
  void setIgnoreDecision(ObjectIgnore ignore) {
    clear();
    ignore_ = ignore;
    object_tag_case_ = kIgnore;
  }
  void setLeadDecision(ObjectLead lead) {
    clear();
    lead_ = lead;
    object_tag_case_ = kLead;
  }

  void setSidepassDecision(ObjectSidePass sidepass) {
    clear();
    sidepass_ = sidepass;
    object_tag_case_ = kSidepass;
  }

  void setOtherCarsYieldDecision(OtherCarsYield other_cars_yield) {
    clear();
    other_cars_yield_ = other_cars_yield;
    object_tag_case_ = kOtherCarsYield;
  }

  void setDecisionSource(std::string module_name) {
    decision_source_ = module_name;
  }

  enum ObjectTagCase {
    kIgnore,
    kOvertake,
    kFollow,
    kYield,
    kStop,
    kNudge,
    kAvoid,
    kLead,
    kSidepass,
    kOtherCarsYield,
    OBJECT_TAG_NOT_SET
  };

  ObjectTagCase object_tag_case() const { return object_tag_case_; }

private:
  optional<ObjectStop> stop_;
  optional<ObjectNudge> nudge_;
  optional<ObjectYield> yield_;
  optional<ObjectFollow> follow_;
  optional<ObjectOvertake> overtake_;
  optional<ObjectAvoid> avoid_;
  optional<ObjectIgnore> ignore_;
  optional<ObjectLead> lead_;
  optional<ObjectSidePass> sidepass_;
  optional<OtherCarsYield> other_cars_yield_;
  ObjectTagCase object_tag_case_;
  std::string decision_source_;
};

} // namespace msquare
#endif /* MODULES_PLANNING_OPTIMIZERS_OBJECT_DECISION_TYPE_H_ */
