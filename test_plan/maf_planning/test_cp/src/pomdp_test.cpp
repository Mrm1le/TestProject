#include "planner/tasks/pomdp_planner/pomdp_planner.h"

using namespace msquare;
using namespace despot;

int main(int argc, char* argv[]) {
  TaskConfig config;
  LaneChangePOMDPPlannerConfig conf;
  conf.initial_rival_aggressiveness = 1.5;
  conf.time_horizon = 6.0;
  conf.time_step = 1.0;
  config.set_lane_change_pomdp_planner_config(conf);
  LCMDPPlanner myplanner(config);
  return myplanner.RunEvaluation(argc, argv);
  //   Globals::config.search_depth = 25;
  //   Globals::config.sim_len = 30;
  //   Globals::config.num_scenarios = 10;
  //   Globals::config.pruning_constant = 0.01;
  //   Globals::config.max_policy_sim_len = 20;
  //   Globals::config.discount = 0.999;
  //   DSPOMDP* model = NULL;
  //   std::array<double, 5> lat_range_info{1.9, -1.9, 0.2, 1.9, -5.7};
  //   std::array<double, 3> lon_range_info{0.0, 100.0, 1.0};
  //   model = new LCEnv(lat_range_info, lon_range_info);
  //   History history;
  //   ACT_TYPE action;
  //   OBS_TYPE obs;
  //   double step_reward;
  //   // action = 2665;
  //   action = 2;
  //   auto state = model->CreateStartState();
  //   LCAgentState& lcstate = static_cast<LCAgentState&>(*state);
  //   lcstate.time = 9.0;
  //   lcstate.ego_lat_vel = -0.5;
  //   lcstate.ego_lon_acc = 0.0;
  //   lcstate.ego_lon_vel = 15.0;
  //   lcstate.rival_lon_acc = 0.5;
  //   lcstate.rival_lon_vel = 15.0;
  //   lcstate.rival_pos.x = 126.5;
  //   lcstate.rival_pos.y = 0.0;
  //   lcstate.ego_pos.x = 143;
  //   lcstate.ego_pos.y = -0.05;
  //   std::cout << "zzd1 state " << state->text() << std::endl;
  //   model->PrintAction(action);
  //   // std::vector<State*> particles;
  //   // particles.resize(1);
  //   // particles[0] = state;
  //   // bool terminal = model->Step(*state, 0.1, action,
	// 	// 	step_reward, obs);
  //   // auto action_sets = model->GetFeasibleActions(obs, particles, history);
  //   // for (auto act : action_sets) {
  //   //   std::cout << "zzd1 action " << act << std::endl;
  //   //   model->PrintAction(act);
  //   // }
	// bool terminal = model->Step(*state, 0.1, action,
	// 		step_reward, obs);
	// std::cout << "zzd2 state " << state->text() << " terminal " << terminal << " reward: " << step_reward << " obs: " << obs << std::endl;
  //   return 0;
}

