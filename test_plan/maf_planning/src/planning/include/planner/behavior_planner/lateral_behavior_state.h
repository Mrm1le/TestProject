#ifndef MSQUARE_DECISION_PLANNING_PLANNER_LATERAL_BEHAVIOR_STATE_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_LATERAL_BEHAVIOR_STATE_H_

#include "common/hfsm/machine_single.hpp"
#include "common/world_model.h"
#include "planning/common/common.h"
#include <array>
#include <iostream>

namespace msquare {

typedef enum {
  ROAD_NONE = 0,
  ROAD_LC_LWAIT,
  ROAD_LC_RWAIT,
  ROAD_LC_LCHANGE,
  ROAD_LC_RCHANGE,
  ROAD_LC_LBACK,
  ROAD_LC_RBACK,

  ROAD_LB_LBORROW,
  ROAD_LB_RBORROW,
  ROAD_LB_LBACK,
  ROAD_LB_RBACK,
  ROAD_LB_LRETURN,
  ROAD_LB_RRETURN,
  ROAD_LB_LSUSPEND,
  ROAD_LB_RSUSPEND,

  INTER_GS_NONE,
  INTER_GS_LC_LWAIT,
  INTER_GS_LC_RWAIT,
  INTER_GS_LC_LCHANGE,
  INTER_GS_LC_RCHANGE,
  INTER_GS_LC_LBACK,
  INTER_GS_LC_RBACK,

  INTER_GS_LB_LBORROW,
  INTER_GS_LB_RBORROW,
  INTER_GS_LB_LBACK,
  INTER_GS_LB_RBACK,
  INTER_GS_LB_LRETURN,
  INTER_GS_LB_RRETURN,
  INTER_GS_LB_LSUSPEND,
  INTER_GS_LB_RSUSPEND,

  INTER_TR_NONE,
  INTER_TR_LC_LWAIT,
  INTER_TR_LC_RWAIT,
  INTER_TR_LC_LCHANGE,
  INTER_TR_LC_RCHANGE,
  INTER_TR_LC_LBACK,
  INTER_TR_LC_RBACK,

  INTER_TR_LB_LBORROW,
  INTER_TR_LB_RBORROW,
  INTER_TR_LB_LBACK,
  INTER_TR_LB_RBACK,
  INTER_TR_LB_LRETURN,
  INTER_TR_LB_RRETURN,
  INTER_TR_LB_LSUSPEND,
  INTER_TR_LB_RSUSPEND,

  INTER_TL_NONE,
  INTER_TL_LC_LWAIT,
  INTER_TL_LC_RWAIT,
  INTER_TL_LC_LCHANGE,
  INTER_TL_LC_RCHANGE,
  INTER_TL_LC_LBACK,
  INTER_TL_LC_RBACK,

  INTER_TL_LB_LBORROW,
  INTER_TL_LB_RBORROW,
  INTER_TL_LB_LBACK,
  INTER_TL_LB_RBACK,
  INTER_TL_LB_LRETURN,
  INTER_TL_LB_RRETURN,
  INTER_TL_LB_LSUSPEND,
  INTER_TL_LB_RSUSPEND,

  INTER_UT_NONE
} LateralStateEnum;

struct FsmContext {
  int state;
  bool external;
  void *user_data;
  std::shared_ptr<WorldModel> world_model;
  std::string name;
  double entry_time;
};

using M = hfsm::Machine<FsmContext>;

template <typename T> struct type2int {};

template <typename T> struct type2name {};

struct StateBase : M::Base {
  virtual ~StateBase() = default;
  void transition(Control &control, FsmContext &context) {
    if (context.external == true) {
      return;
    }

    callback(control, context);
  }

  virtual void callback(Control &control, FsmContext &context) {}

  template <typename T>
  static void change_state(Control &control, FsmContext &context) {
    if (context.state == type2int<T>::value) {
      return;
    }

    MSD_LOG(INFO, "change_state from [%s] to [%s]", context.name.c_str(),
            type2name<T>::name);

    control.changeTo<T>();
    context.state = type2int<T>::value;
    context.name = type2name<T>::name;
    context.entry_time = get_system_time();
  }
};

} // namespace msquare

#endif
