#pragma once

// // enable logger functionality
// #define HFSM_ENABLE_LOG_INTERFACE

// // optional: enable FSM structure report in debugger
// #define HFSM_ENABLE_STRUCTURE_REPORT

#include "common/hfsm/machine_single.hpp"
#include "common/parking_world_model.h"
#include "planner/message_type.h"
#include "zigzag_path.h"
#include <map>
#include <stdio.h>
#include <vector>

namespace msquare {
namespace parking {
namespace path_sampler_statemachine {

enum class SamplerFsmStateEnum {
  INVALID = 0,
  INIT,
  DRIVE,
  REVERSE,
  DRIVE2REVERSE,
  REVERSE2DRIVE,
  FINISH,
  STATE_NUM,
};

// data shared between FSM states and outside code
struct Context {
  bool is_vehicle_static;
  ZigzagPath path;
  std::vector<DirTrajectoryPoint>::const_iterator path_iter;
  maf_vehicle_status::GearType gear{};

  Context(const ZigzagPath &path) : is_vehicle_static(false), path(path) {
    path_iter = path.get_points().begin();
  }
  Context() : is_vehicle_static(false) {}
};

// event for react method
struct FinishEvent {};

// convenience typedef
using M = hfsm::Machine<Context>;

struct Follow;
struct Finish;

struct Follow : M::Base {
  void enter(const Context &context) {}
  void update(const Context &context) {}
  void transition(Control &control, const Context &context) {
    if (context.path_iter == context.path.get_points().end() - 1) {
      control.changeTo<Finish>();
    }
  }
  template <typename TEvent> void react(const TEvent &, Control &, Context &) {}

  struct Drive;
  struct Reverse;
  struct Drive2Reverse;
  struct Reverse2Drive;

  struct Drive : M::Base {
    void enter(const Context &context) {
      if (context.path_iter->direction != 1) {
        // throw std::logic_error("path point direction doesn't match gear!");
      }
    }
    void transition(Control &control, const Context &context) {
      if (context.path_iter ==
          context.path.get_stage_upper(context.path_iter) - 1) {
        control.changeTo<Drive2Reverse>();
      }
    }
    template <typename TEvent>
    void react(const TEvent &, Control &, Context &) {}
  };

  struct Reverse : M::Base {
    void enter(const Context &context) {
      if (context.path_iter->direction != -1) {
        // throw std::logic_error("path point direction doesn't match gear!");
      }
    }
    void transition(Control &control, const Context &context) {
      if (context.path_iter ==
          context.path.get_stage_upper(context.path_iter) - 1) {
        control.changeTo<Reverse2Drive>();
      }
    }
    template <typename TEvent>
    void react(const TEvent &, Control &, Context &) {}
  };

  struct Drive2Reverse : M::Base {
    void enter(const Context &context) {}
    void transition(Control &control, Context &context) {
      if (context.is_vehicle_static) {
        control.changeTo<Reverse>();
        context.path_iter = context.path.get_stage_upper(context.path_iter);
      }
    }
    template <typename TEvent>
    void react(const TEvent &, Control &, Context &) {}
  };

  struct Reverse2Drive : M::Base {
    void enter(const Context &context) {}
    void transition(Control &control, Context &context) {
      if (context.is_vehicle_static) {
        control.changeTo<Drive>();
        context.path_iter = context.path.get_stage_upper(context.path_iter);
      }
    }
    template <typename TEvent>
    void react(const TEvent &, Control &, Context &) {}
  };
};

struct Init : M::Base {
  void enter(const Context &context) {}
  void transition(Control &control, const Context &context) {
    if (context.path_iter->direction == 1) {
      if (context.is_vehicle_static ||
          context.gear.value == maf_vehicle_status::GearType::DRIVE) {
        control.changeTo<Follow::Drive>();
      }
    } else if (context.path_iter->direction == -1) {
      if (context.is_vehicle_static ||
          context.gear.value == maf_vehicle_status::GearType::REVERSE) {
        control.changeTo<Follow::Reverse>();
      }
    } else {
      // throw std::logic_error("Invalid direction of path");
    }
  }
  template <typename TEvent>
  void react(const TEvent &, Control &control, Context &) {}
};

struct Finish : M::Base {
  void enter(const Context &context) {}
  template <typename TEvent> void react(const TEvent &, Control &, Context &) {}
};

using MPeerRoot = hfsm::Machine<Context>::PeerRoot<
    Init,
    M::Composite<Follow, Follow::Drive, Follow::Reverse, Follow::Drive2Reverse,
                 Follow::Reverse2Drive>,
    Finish>;

} // namespace path_sampler_statemachine
} // namespace parking
} // namespace msquare
