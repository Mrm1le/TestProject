#include "common/obstacle_decision.h"
#include "planning/common/common.h"

namespace msquare {

const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         ObstacleDecision::ObjectTagCaseHash>
    ObstacleDecision::s_longitudinal_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0},
        {ObjectDecisionType::kOvertake, 100},
        {ObjectDecisionType::kFollow, 200},
        {ObjectDecisionType::kYield, 300},
        {ObjectDecisionType::kStop, 400}};

const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         ObstacleDecision::ObjectTagCaseHash>
    ObstacleDecision::s_lateral_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0}, {ObjectDecisionType::kNudge, 100}};

void ObstacleDecision::update(const ObstacleDecision &obstacle_decision) {
  id_ = obstacle_decision.id_;
  decisions_ = obstacle_decision.decisions_;
  decider_tags_ = obstacle_decision.decider_tags_;
  lat_decider_tags_ = obstacle_decision.lat_decider_tags_;
  lon_decider_tags_ = obstacle_decision.lon_decider_tags_;
  lateral_decision_ = obstacle_decision.lateral_decision_;
  longitudinal_decision_ = obstacle_decision.longitudinal_decision_;
}

const std::vector<std::string> &ObstacleDecision::decider_tags() const {
  return decider_tags_;
}

const std::vector<std::string> &ObstacleDecision::lat_decider_tags() const {
  return lat_decider_tags_;
}

const std::vector<std::string> &ObstacleDecision::lon_decider_tags() const {
  return lon_decider_tags_;
}

const std::vector<ObjectDecisionType> &ObstacleDecision::decisions() const {
  return decisions_;
}

bool ObstacleDecision::IsLateralDecision(const ObjectDecisionType &decision) {
  return decision.has_ignore() || decision.has_nudge();
}

bool ObstacleDecision::IsLongitudinalDecision(
    const ObjectDecisionType &decision) {
  return decision.has_ignore() || decision.has_stop() || decision.has_yield() ||
         decision.has_follow() || decision.has_overtake();
}

const ObjectDecisionType &ObstacleDecision::LongitudinalDecision() const {
  return longitudinal_decision_;
}

ObjectDecisionType &ObstacleDecision::MutableLongitudinalDecision() {
  return longitudinal_decision_;
}

const ObjectDecisionType &ObstacleDecision::LateralDecision() const {
  return lateral_decision_;
}

ObjectDecisionType &ObstacleDecision::MutableLateralDecision() {
  return lateral_decision_;
}

ObjectDecisionType
ObstacleDecision::MergeLongitudinalDecision(const ObjectDecisionType &lhs,
                                            const ObjectDecisionType &rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  const auto lhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, lhs.object_tag_case());
  const auto rhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, rhs.object_tag_case());
  if (lhs_val < rhs_val) {
    return rhs;
  } else if (lhs_val > rhs_val) {
    return lhs;
  } else {
    if (lhs.has_ignore()) {
      return rhs;
    } else if (lhs.has_stop()) {
      return lhs.stop().distance_s < rhs.stop().distance_s ? lhs : rhs;
    } else if (lhs.has_yield()) {
      return lhs.yield().distance_s < rhs.yield().distance_s ? lhs : rhs;
    } else if (lhs.has_follow()) {
      return lhs.follow().distance_s < rhs.follow().distance_s ? lhs : rhs;
    } else if (lhs.has_overtake()) {
      return lhs.overtake().distance_s > rhs.overtake().distance_s ? lhs : rhs;
    } else {
      MSD_LOG(INFO, "Unknown decision");
    }
  }
  return lhs; // stop compiler complaining
}

bool ObstacleDecision::IsIgnore() const {
  return IsLongitudinalIgnore() && IsLateralIgnore();
}

bool ObstacleDecision::IsLongitudinalIgnore() const {
  return longitudinal_decision_.has_ignore();
}

bool ObstacleDecision::IsLateralIgnore() const {
  return lateral_decision_.has_ignore();
}

ObjectDecisionType
ObstacleDecision::MergeLateralDecision(const ObjectDecisionType &lhs,
                                       const ObjectDecisionType &rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  const auto lhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, lhs.object_tag_case());
  const auto rhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, rhs.object_tag_case());
  if (lhs_val < rhs_val) {
    return rhs;
  } else if (lhs_val > rhs_val) {
    return lhs;
  } else {
    if (lhs.has_ignore()) {
      return rhs;
    } else if (lhs.has_nudge()) {
      mph_assert(lhs.nudge().type == rhs.nudge().type);
      if (!rhs.nudge().is_longitunidal_ignored) {
        return rhs;
      } else if (!lhs.nudge().is_longitunidal_ignored) {
        return lhs;
      }
      return std::fabs(lhs.nudge().distance_l) >
                     std::fabs(rhs.nudge().distance_l)
                 ? lhs
                 : rhs;
    }
  }
  MSD_LOG(INFO, "Does not have rule to merge decision: ");
  return lhs;
}

bool ObstacleDecision::HasLateralDecision() const {
  return lateral_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

bool ObstacleDecision::HasLongitudinalDecision() const {
  return longitudinal_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

bool ObstacleDecision::HasNonIgnoreDecision() const {
  return (HasLateralDecision() && !IsLateralIgnore()) ||
         (HasLongitudinalDecision() && !IsLongitudinalIgnore());
}

void ObstacleDecision::AddLongitudinalDecision(
    const std::string &decider_tag, const ObjectDecisionType &decision) {
  mph_assert(IsLongitudinalDecision(decision));
  longitudinal_decision_ =
      MergeLongitudinalDecision(longitudinal_decision_, decision);
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
  lon_decider_tags_.push_back(decider_tag);
}

void ObstacleDecision::ReplaceLongitudinalDecision(
    const std::string &decider_tag, const ObjectDecisionType &decision) {
  mph_assert(IsLongitudinalDecision(decision));
  longitudinal_decision_ = decision;
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
  lon_decider_tags_.push_back(decider_tag);
}

void ObstacleDecision::AddLateralDecision(const std::string &decider_tag,
                                          const ObjectDecisionType &decision) {
  mph_assert(IsLateralDecision(decision));
  lateral_decision_ = MergeLateralDecision(lateral_decision_, decision);
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
  lat_decider_tags_.push_back(decider_tag);
}

void ObstacleDecision::ClearLongitudinalDecision() {
  ObjectDecisionType none_decision;
  longitudinal_decision_ = none_decision;
}

void ObstacleDecision::ClearLateralDecision() {
  ObjectDecisionType none_decision;
  lateral_decision_ = none_decision;
}

} // namespace msquare
