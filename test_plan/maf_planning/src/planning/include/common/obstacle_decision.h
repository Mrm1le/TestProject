#ifndef MSQUARE_DECISION_PLANNING_COMMON_OBSTACLE_DECISION_H_
#define MSQUARE_DECISION_PLANNING_COMMON_OBSTACLE_DECISION_H_

#include "common/object_decision_type.h"
#include "common/utils/index_list.h"
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace msquare {

class ObstacleDecision {
public:
  explicit ObstacleDecision(int id) : id_(id) {}

  void update(const ObstacleDecision &obstacle_decision);

  int Id() const { return id_; }
  /**
   * return the merged lateral decision
   * Lateral decision is one of {Nudge, Ignore}
   **/
  const ObjectDecisionType &LateralDecision() const;
  ObjectDecisionType &MutableLateralDecision();

  /**
   * @brief return the merged longitudinal decision
   * Longitudinal decision is one of {Stop, Yield, Follow, Overtake, Ignore}
   **/
  const ObjectDecisionType &LongitudinalDecision() const;
  ObjectDecisionType &MutableLongitudinalDecision();

  const std::vector<std::string> &decider_tags() const;

  const std::vector<std::string> &lat_decider_tags() const;

  const std::vector<std::string> &lon_decider_tags() const;

  const std::vector<ObjectDecisionType> &decisions() const;

  void AddLongitudinalDecision(const std::string &decider_tag,
                               const ObjectDecisionType &decision);

  void ReplaceLongitudinalDecision(const std::string &decider_tag,
                                   const ObjectDecisionType &decision);

  void AddLateralDecision(const std::string &decider_tag,
                          const ObjectDecisionType &decision);

  void ClearLongitudinalDecision();

  void ClearLateralDecision();

  bool HasLateralDecision() const;
  bool HasLongitudinalDecision() const;

  bool HasNonIgnoreDecision() const;

  /**
   * @brief Check if this object can be safely ignored.
   * The object will be ignored if the lateral decision is ignore and the
   * longitudinal decision is ignore
   *  return longitudinal_decision_ == ignore && lateral_decision == ignore.
   */
  bool IsIgnore() const;
  bool IsLongitudinalIgnore() const;
  bool IsLateralIgnore() const;
  /**
   * @brief check if an ObjectDecisionType is a longitudinal decision.
   */
  static bool IsLongitudinalDecision(const ObjectDecisionType &decision);

  /**
   * @brief check if an ObjectDecisionType is a lateral decision.
   */
  static bool IsLateralDecision(const ObjectDecisionType &decision);

private:
  static ObjectDecisionType
  MergeLongitudinalDecision(const ObjectDecisionType &lhs,
                            const ObjectDecisionType &rhs);
  static ObjectDecisionType MergeLateralDecision(const ObjectDecisionType &lhs,
                                                 const ObjectDecisionType &rhs);

private:
  int id_;
  std::vector<ObjectDecisionType> decisions_;
  std::vector<std::string> decider_tags_;
  std::vector<std::string> lat_decider_tags_;
  std::vector<std::string> lon_decider_tags_;
  ObjectDecisionType lateral_decision_;
  ObjectDecisionType longitudinal_decision_;

  struct ObjectTagCaseHash {
    size_t operator()(const ObjectDecisionType::ObjectTagCase tag) const {
      return static_cast<size_t>(tag);
    }
  };

  static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>
      s_lateral_decision_safety_sorter_;
  static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>
      s_longitudinal_decision_safety_sorter_;
};

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_COMMON_OBSTACLE_DECISION_H_
