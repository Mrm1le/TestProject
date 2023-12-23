#ifndef MSQUARE_DECISION_PLANNING_PLANNER_LATERAL_BEHAVIOR_INTERSECT_STATE_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_LATERAL_BEHAVIOR_INTERSECT_STATE_H_

#include "planner/behavior_planner/lateral_behavior_state.h"

namespace msquare {

struct InterState : StateBase {
  struct GS : StateBase {
    struct None : StateBase {
      void callback(Control &control, FsmContext &context);
    };

    struct LC : StateBase {
      static void process_wait(const Control &control,
                               const FsmContext &context);
      static void process_change(const Control &control,
                                 const FsmContext &context);
      static void process_back(const Control &control,
                               const FsmContext &context);

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
      struct LBorrow : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct RBorrow : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct LBack : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct RBack : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct LReturn : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct RReturn : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct LSuspend : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct RSuspend : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };
    };
  };

  struct TR : StateBase {
    struct None : StateBase {
      void callback(Control &control, FsmContext &context);
    };

    struct LC : StateBase {
      static void process_wait(const Control &control,
                               const FsmContext &context);
      static void process_change(const Control &control,
                                 const FsmContext &context);
      static void process_back(const Control &control,
                               const FsmContext &context);

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
      struct LBorrow : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct RBorrow : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct LBack : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct RBack : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct LReturn : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct RReturn : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct LSuspend : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct RSuspend : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };
    };
  };

  struct TL : StateBase {
    struct None : StateBase {
      void callback(Control &control, FsmContext &context);
    };

    struct LC : StateBase {
      static void process_wait(const Control &control,
                               const FsmContext &context);
      static void process_change(const Control &control,
                                 const FsmContext &context);
      static void process_back(const Control &control,
                               const FsmContext &context);

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
      struct LBorrow : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct RBorrow : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct LBack : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct RBack : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct LReturn : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct RReturn : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct LSuspend : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };

      struct RSuspend : StateBase {
        void callback(Control &control, FsmContext &context) {}
      };
    };
  };

  struct UT : StateBase {
    struct None : StateBase {
      void callback(Control &control, FsmContext &context);
    };
  };
};

template <> struct type2int<InterState::GS::None> {
  enum { value = INTER_GS_NONE };
};

template <> struct type2int<InterState::GS::LC::LWait> {
  enum { value = INTER_GS_LC_LWAIT };
};

template <> struct type2int<InterState::GS::LC::RWait> {
  enum { value = INTER_GS_LC_RWAIT };
};

template <> struct type2int<InterState::GS::LC::LChange> {
  enum { value = INTER_GS_LC_LCHANGE };
};

template <> struct type2int<InterState::GS::LC::RChange> {
  enum { value = INTER_GS_LC_RCHANGE };
};

template <> struct type2int<InterState::GS::LC::LBack> {
  enum { value = INTER_GS_LC_LBACK };
};

template <> struct type2int<InterState::GS::LC::RBack> {
  enum { value = INTER_GS_LC_RBACK };
};

template <> struct type2int<InterState::TR::None> {
  enum { value = INTER_TR_NONE };
};

template <> struct type2int<InterState::TR::LC::LWait> {
  enum { value = INTER_TR_LC_LWAIT };
};

template <> struct type2int<InterState::TR::LC::RWait> {
  enum { value = INTER_TR_LC_RWAIT };
};

template <> struct type2int<InterState::TR::LC::LChange> {
  enum { value = INTER_TR_LC_LCHANGE };
};

template <> struct type2int<InterState::TR::LC::RChange> {
  enum { value = INTER_TR_LC_RCHANGE };
};

template <> struct type2int<InterState::TR::LC::LBack> {
  enum { value = INTER_TR_LC_LBACK };
};

template <> struct type2int<InterState::TR::LC::RBack> {
  enum { value = INTER_TR_LC_RBACK };
};

template <> struct type2int<InterState::TL::None> {
  enum { value = INTER_TL_NONE };
};

template <> struct type2int<InterState::TL::LC::LWait> {
  enum { value = INTER_TL_LC_LWAIT };
};

template <> struct type2int<InterState::TL::LC::RWait> {
  enum { value = INTER_TL_LC_RWAIT };
};

template <> struct type2int<InterState::TL::LC::LChange> {
  enum { value = INTER_TL_LC_LCHANGE };
};

template <> struct type2int<InterState::TL::LC::RChange> {
  enum { value = INTER_TL_LC_RCHANGE };
};

template <> struct type2int<InterState::TL::LC::LBack> {
  enum { value = INTER_TL_LC_LBACK };
};

template <> struct type2int<InterState::TL::LC::RBack> {
  enum { value = INTER_TL_LC_RBACK };
};

template <> struct type2int<InterState::UT::None> {
  enum { value = INTER_UT_NONE };
};

template <> struct type2name<InterState::GS::None> {
  static constexpr auto name = "INTER_GS_NONE";
};

template <> struct type2name<InterState::GS::LC::LWait> {
  static constexpr auto name = "INTER_GS_LC_LWAIT";
};

template <> struct type2name<InterState::GS::LC::RWait> {
  static constexpr auto name = "INTER_GS_LC_RWAIT";
};

template <> struct type2name<InterState::GS::LC::LChange> {
  static constexpr auto name = "INTER_GS_LC_LCHANGE";
};

template <> struct type2name<InterState::GS::LC::RChange> {
  static constexpr auto name = "INTER_GS_LC_RCHANGE";
};

template <> struct type2name<InterState::GS::LC::LBack> {
  static constexpr auto name = "INTER_GS_LC_LBACK";
};

template <> struct type2name<InterState::GS::LC::RBack> {
  static constexpr auto name = "INTER_GS_LC_RBACK";
};

template <> struct type2name<InterState::TR::None> {
  static constexpr auto name = "INTER_TR_NONE";
};

template <> struct type2name<InterState::TR::LC::LWait> {
  static constexpr auto name = "INTER_TR_LC_LWAIT";
};

template <> struct type2name<InterState::TR::LC::RWait> {
  static constexpr auto name = "INTER_TR_LC_RWAIT";
};

template <> struct type2name<InterState::TR::LC::LChange> {
  static constexpr auto name = "INTER_TR_LC_LCHANGE";
};

template <> struct type2name<InterState::TR::LC::RChange> {
  static constexpr auto name = "INTER_TR_LC_RCHANGE";
};

template <> struct type2name<InterState::TR::LC::LBack> {
  static constexpr auto name = "INTER_TR_LC_LBACK";
};

template <> struct type2name<InterState::TR::LC::RBack> {
  static constexpr auto name = "INTER_TR_LC_RBACK";
};

template <> struct type2name<InterState::TL::None> {
  static constexpr auto name = "INTER_TL_NONE";
};

template <> struct type2name<InterState::TL::LC::LWait> {
  static constexpr auto name = "INTER_TL_LC_LWAIT";
};

template <> struct type2name<InterState::TL::LC::RWait> {
  static constexpr auto name = "INTER_TL_LC_RWAIT";
};

template <> struct type2name<InterState::TL::LC::LChange> {
  static constexpr auto name = "INTER_TL_LC_LCHANGE";
};

template <> struct type2name<InterState::TL::LC::RChange> {
  static constexpr auto name = "INTER_TL_LC_RCHANGE";
};

template <> struct type2name<InterState::TL::LC::LBack> {
  static constexpr auto name = "INTER_TL_LC_LBACK";
};

template <> struct type2name<InterState::TL::LC::RBack> {
  static constexpr auto name = "INTER_TL_LC_RBACK";
};

template <> struct type2name<InterState::UT::None> {
  static constexpr auto name = "INTER_UT_NONE";
};

} // namespace msquare

#endif
