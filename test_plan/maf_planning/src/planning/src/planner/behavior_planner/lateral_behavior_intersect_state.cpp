#include "planner/behavior_planner/lateral_behavior_intersect_state.h"
#include "planner/arbitrators/arbitrator_cost_strategy.h"
#include "planner/arbitrators/merit_based_arbitrator.h"
#include "planner/arbitrators/priority_based_arbitrator.h"
#include "planner/arbitrators/sequence_arbitrator.h"
#include "planner/arbitrators/single_scenario_arbitrator.h"
#include "planner/behavior_planner/lateral_behavior_planner.h"
#include "planner/behavior_planner/lateral_behavior_state_machine.h"
#include "planner/scenarios/scenario_factory.h"

namespace msquare {

void InterState::GS::None::callback(Control &control, FsmContext &context) {}

void InterState::GS::LC::process_wait(const Control &control,
                                      const FsmContext &context) {}

void InterState::GS::LC::process_change(const Control &control,
                                        const FsmContext &context) {}

void InterState::GS::LC::process_back(const Control &control,
                                      const FsmContext &context) {}

void InterState::GS::LC::LWait::callback(Control &control,
                                         FsmContext &context) {
  process_wait(control, context);
}

void InterState::GS::LC::RWait::callback(Control &control,
                                         FsmContext &context) {
  process_wait(control, context);
}

void InterState::GS::LC::LChange::callback(Control &control,
                                           FsmContext &context) {
  process_change(control, context);
}

void InterState::GS::LC::RChange::callback(Control &control,
                                           FsmContext &context) {
  process_change(control, context);
}

void InterState::GS::LC::LBack::callback(Control &control,
                                         FsmContext &context) {
  process_back(control, context);
}

void InterState::GS::LC::RBack::callback(Control &control,
                                         FsmContext &context) {
  process_back(control, context);
}

void InterState::TR::None::callback(Control &control, FsmContext &context) {}

void InterState::TR::LC::process_wait(const Control &control,
                                      const FsmContext &context) {}

void InterState::TR::LC::process_change(const Control &control,
                                        const FsmContext &context) {}

void InterState::TR::LC::process_back(const Control &control,
                                      const FsmContext &context) {}

void InterState::TR::LC::LWait::callback(Control &control,
                                         FsmContext &context) {
  process_wait(control, context);
}

void InterState::TR::LC::RWait::callback(Control &control,
                                         FsmContext &context) {
  process_wait(control, context);
}

void InterState::TR::LC::LChange::callback(Control &control,
                                           FsmContext &context) {
  process_wait(control, context);
}

void InterState::TR::LC::RChange::callback(Control &control,
                                           FsmContext &context) {
  process_wait(control, context);
}

void InterState::TR::LC::LBack::callback(Control &control,
                                         FsmContext &context) {
  process_wait(control, context);
}

void InterState::TR::LC::RBack::callback(Control &control,
                                         FsmContext &context) {
  process_wait(control, context);
}

void InterState::TL::None::callback(Control &control, FsmContext &context) {}

void InterState::TL::LC::process_wait(const Control &control,
                                      const FsmContext &context) {}

void InterState::TL::LC::process_change(const Control &control,
                                        const FsmContext &context) {}

void InterState::TL::LC::process_back(const Control &control,
                                      const FsmContext &context) {}

void InterState::TL::LC::LWait::callback(Control &control,
                                         FsmContext &context) {
  process_wait(control, context);
}

void InterState::TL::LC::RWait::callback(Control &control,
                                         FsmContext &context) {
  process_wait(control, context);
}

void InterState::TL::LC::LChange::callback(Control &control,
                                           FsmContext &context) {
  process_change(control, context);
}

void InterState::TL::LC::RChange::callback(Control &control,
                                           FsmContext &context) {
  process_change(control, context);
}

void InterState::TL::LC::LBack::callback(Control &control,
                                         FsmContext &context) {
  process_back(control, context);
}

void InterState::TL::LC::RBack::callback(Control &control,
                                         FsmContext &context) {
  process_back(control, context);
}

void InterState::UT::None::callback(Control &control, FsmContext &context) {}

} // namespace msquare
