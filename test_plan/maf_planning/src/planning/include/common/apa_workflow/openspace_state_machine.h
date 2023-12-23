#pragma once

#include "common/parking_world_model.h"
#include "common/utils/timer.h"
#include "openspace_state_machine_typedef.h"
#include "planner/behavior_planner/deciders/openspace_decider_interface.h"
#include "planner/motion_planner/openspace_motion_planner/path_sampler.h"

namespace msquare {
namespace parking {
namespace openspace_state_machine {

class OpenspaceStateMachine {
public:
  enum Type {
    PARKIN,
    PARKOUT,

  };

public:
  OpenspaceStateMachine(const std::shared_ptr<WorldModel> &world_model,
                        const std::shared_ptr<PathSampler> &path_sampler);
  ~OpenspaceStateMachine();

public:
  bool createMachineParkIn();
  bool createMachineParkOut();
  bool createMachineRpaStraight();

  bool destroyMachine();

  bool update();
  bool changeToStandby();

  openspace_state_machine::OpenspaceStateEnum getCurrentState();

private:
  // statemachine context and machine
  openspace_state_machine::Context openspace_state_machine_ctx_;
  std::unique_ptr<hfsm::Machine<openspace_state_machine::Context>::PeerRoot<
      openspace_state_machine::COMPOSITE>>
      openspace_state_machine_;

  // shared (external) modules
  std::shared_ptr<WorldModel> world_model_;
  std::shared_ptr<PathSampler> path_sampler_;

  // internal moudles

  std::unique_ptr<BaseOpenspaceDecider> openspace_decider_;

  const double PLANNING_RUN_DURATION = 3.0;
  Timer planning_run_timer_;
  const double WLC_DAMPING_DURATION = 2.0;
  Timer wlc_damping_timer_;

  void setupCallbackFunctions();

  //[fenix.refactor] (former) openspace statemachine callback functions

  // entry callback
  void onEntryOpenspaceStandby();
  void onEntryOpenspacePlanning();
  void onEntryOpenspaceFallback();
  void onEntryOpenspaceRunning();
  void onEntryOpenspaceFinish();

  // update callback
  void onUpdateOpenspaceStandby();
  void onUpdateOpenspacePlanning();
  void onUpdateOpenspaceFallback(){};
  void onUpdateOpenspaceRunning();
  void onUpdateOpenspaceFinish(){};

  // transition callback
  void onTransitionOpenspaceStandby(
      hfsm::Machine<openspace_state_machine::Context>::Control &control);
  void onTransitionOpenspacePlanning(
      hfsm::Machine<openspace_state_machine::Context>::Control &control);
  void onTransitionOpenspaceFallback(
      const hfsm::Machine<openspace_state_machine::Context>::Control &control);
  void onTransitionOpenspaceRunning(
      hfsm::Machine<openspace_state_machine::Context>::Control &control);
  void onTransitionOpenspaceFinish(
      hfsm::Machine<openspace_state_machine::Context>::Control &control);

  // leave callback
  void onLeaveOpenspaceStandby();
  void onLeaveOpenspacePlanning();
  void onLeaveOpenspaceFallback(){};
  void onLeaveOpenspaceRunning(){};
  void onLeaveOpenspaceFinish(){};

  //[fenix.refactor] end of (former) openspace statemachine callback functions
private:
  void runTransitionOpenspaceRunning(hfsm::Machine<openspace_state_machine::Context>::Control &control);
  void runTransitionOpenspaceRunningSOP(hfsm::Machine<openspace_state_machine::Context>::Control &control);

private:
  bool can_follow_plan_traj(double dist_threshold,
                            bool use_average_diff = false);
};

} // namespace openspace_state_machine
} // namespace parking
} // namespace msquare
