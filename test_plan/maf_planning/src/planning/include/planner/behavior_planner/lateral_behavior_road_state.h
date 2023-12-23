#ifndef MSQUARE_DECISION_PLANNING_PLANNER_LATERAL_BEHAVIOR_ROAD_STATE_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_LATERAL_BEHAVIOR_ROAD_STATE_H_

#include "planner/behavior_planner/lateral_behavior_state.h"

namespace msquare {

struct RoadState : StateBase {
  struct None : StateBase {
    void callback(Control &control, FsmContext &context);
  };

  struct LC : StateBase {
    static void process_wait(Control &control, FsmContext &context);
    static void process_change(Control &control, FsmContext &context);
    static void process_back(Control &control, FsmContext &context);

    struct LWait : StateBase {
      void callback(Control &control, FsmContext &context);
    };

    struct RWait : StateBase {
      void callback(Control &control, FsmContext &context);
    };

    struct LChange : StateBase {
      void callback(Control &control, FsmContext &context);
    };

    struct RChange : StateBase {
      void callback(Control &control, FsmContext &context);
    };

    struct LBack : StateBase {
      void callback(Control &control, FsmContext &context);
    };

    struct RBack : StateBase {
      void callback(Control &control, FsmContext &context);
    };
  };

  struct LB : StateBase {
    static void process_borrow(const Control &control,
                               const FsmContext &context);
    static void process_back(const Control &control, const FsmContext &context);
    static void process_return(const Control &control,
                               const FsmContext &context);
    static void process_suspend(const Control &control,
                                const FsmContext &context);

    struct LBorrow : StateBase {
      void callback(Control &control, FsmContext &context);
    };

    struct RBorrow : StateBase {
      void callback(Control &control, FsmContext &context);
    };

    struct LBack : StateBase {
      void callback(Control &control, FsmContext &context);
    };

    struct RBack : StateBase {
      void callback(Control &control, FsmContext &context);
    };

    struct LReturn : StateBase {
      void callback(Control &control, FsmContext &context);
    };

    struct RReturn : StateBase {
      void callback(Control &control, FsmContext &context);
    };

    struct LSuspend : StateBase {
      void callback(Control &control, FsmContext &context);
    };

    struct RSuspend : StateBase {
      void callback(Control &control, FsmContext &context);
    };
  };
};

template <> struct type2int<RoadState::None> {
  enum { value = ROAD_NONE };
};

template <> struct type2int<RoadState::LC::LWait> {
  enum { value = ROAD_LC_LWAIT };
};

template <> struct type2int<RoadState::LC::RWait> {
  enum { value = ROAD_LC_RWAIT };
};

template <> struct type2int<RoadState::LC::LChange> {
  enum { value = ROAD_LC_LCHANGE };
};

template <> struct type2int<RoadState::LC::RChange> {
  enum { value = ROAD_LC_RCHANGE };
};

template <> struct type2int<RoadState::LC::LBack> {
  enum { value = ROAD_LC_LBACK };
};

template <> struct type2int<RoadState::LC::RBack> {
  enum { value = ROAD_LC_RBACK };
};

template <> struct type2int<RoadState::LB::LBorrow> {
  enum { value = ROAD_LB_LBORROW };
};

template <> struct type2int<RoadState::LB::RBorrow> {
  enum { value = ROAD_LB_RBORROW };
};

template <> struct type2int<RoadState::LB::LBack> {
  enum { value = ROAD_LB_LBACK };
};

template <> struct type2int<RoadState::LB::RBack> {
  enum { value = ROAD_LB_RBACK };
};

template <> struct type2int<RoadState::LB::LReturn> {
  enum { value = ROAD_LB_LRETURN };
};

template <> struct type2int<RoadState::LB::RReturn> {
  enum { value = ROAD_LB_RRETURN };
};

template <> struct type2int<RoadState::LB::LSuspend> {
  enum { value = ROAD_LB_LSUSPEND };
};

template <> struct type2int<RoadState::LB::RSuspend> {
  enum { value = ROAD_LB_RSUSPEND };
};

template <> struct type2name<RoadState::None> {
  static constexpr auto name = "ROAD_NONE";
};

template <> struct type2name<RoadState::LC::LWait> {
  static constexpr auto name = "ROAD_LC_LWAIT";
};

template <> struct type2name<RoadState::LC::RWait> {
  static constexpr auto name = "ROAD_LC_RWAIT";
};

template <> struct type2name<RoadState::LC::LChange> {
  static constexpr auto name = "ROAD_LC_LCHANGE";
};

template <> struct type2name<RoadState::LC::RChange> {
  static constexpr auto name = "ROAD_LC_RCHANGE";
};

template <> struct type2name<RoadState::LC::LBack> {
  static constexpr auto name = "ROAD_LC_LBACK";
};

template <> struct type2name<RoadState::LC::RBack> {
  static constexpr auto name = "ROAD_LC_RBACK";
};

template <> struct type2name<RoadState::LB::LBorrow> {
  static constexpr auto name = "ROAD_LB_LBORROW";
};

template <> struct type2name<RoadState::LB::RBorrow> {
  static constexpr auto name = "ROAD_LB_RBORROW";
};

template <> struct type2name<RoadState::LB::LBack> {
  static constexpr auto name = "ROAD_LB_LBACK";
};

template <> struct type2name<RoadState::LB::RBack> {
  static constexpr auto name = "ROAD_LB_RBACK";
};

template <> struct type2name<RoadState::LB::LReturn> {
  static constexpr auto name = "ROAD_LB_LRETURN";
};

template <> struct type2name<RoadState::LB::RReturn> {
  static constexpr auto name = "ROAD_LB_RRETURN";
};

template <> struct type2name<RoadState::LB::LSuspend> {
  static constexpr auto name = "ROAD_LB_LSUSPEND";
};

template <> struct type2name<RoadState::LB::RSuspend> {
  static constexpr auto name = "ROAD_LB_RSUSPEND";
};

} // namespace msquare

#endif