#ifndef MODULES_PLANNING_OPTIMIZERS_OBS_MANAGER_CONFIG_H_
#define MODULES_PLANNING_OPTIMIZERS_OBS_MANAGER_CONFIG_H_

#include <vector>
namespace msquare {

// s horizon
const double FLAGS_speed_lon_decision_horizon = 200.0;
// t horizon
const double FLAGS_speed_lon_decision_time_horizon = 5.0;
// t resolution
const double FLAGS_speed_lon_decision_time_resolution = 0.2;

const int STATIC_CNT_CAR = 5;
const int MOVING_CNT_CAR = 5;
const int STATIC_CNT_CAR_MAX = 10;
const int MOVING_CNT_CAR_MAX = 10;

const int STATIC_CNT_PED = 5;
const int MOVING_CNT_PED = 5;
const int STATIC_CNT_PED_MAX = 10;
const int MOVING_CNT_PED_MAX = 10;

const int DIRCTION_CNT_MAX = 10;

const double STATIC_VTHRE = 3.0;

const int PREDICTION_SIZE = 30;
// set of params for SVP objects
const std::vector<double> _V_EGO_AVP_CAR_SVP{0.0, 0.1, 1.0, 1.7, 3.0};
const std::vector<double> _V_THRES_AVP_CAR_SVP{0.3, 0.3, 0.4, 0.6, 0.8};

const std::vector<double> _V_EGO_APA_CAR_SVP{0.0, 0.1, 0.7};
const std::vector<double> _V_THRES_APA_CAR_SVP{0.3, 0.3, 0.5};

const std::vector<double> _V_EGO_TURN_CAR_SVP{0.0, 0.1, 1.0, 1.7, 3.0};
const std::vector<double> _V_THRES_TURN_CAR_SVP{0.4, 0.4, 0.6, 0.8, 1.0};

const std::vector<double> _V_EGO_AVP_HUMAN_SVP{0.0, 0.1, 0.5, 1.0};
const std::vector<double> _V_THRES_AVP_HUMAN_SVP{0.1, 0.1, 0.2, 0.3};

const std::vector<double> _V_EGO_TURN_HUMAN_SVP{0.0, 0.1, 0.5, 1.0};
const std::vector<double> _V_THRES_TURN_HUMAN_SVP{0.2, 0.2, 0.4, 0.5};

// set of params for FCP objects
// const std::vector<double>  _V_EGO_AVP_CAR_FCP {0.0, 0.1, 1.0, 1.7};
// const std::vector<double>  _V_THRES_AVP_CAR_FCP {0.3, 0.3, 0.6, 1.0};

// const std::vector<double>  _V_EGO_APA_CAR_FCP {0.0, 0.1, 0.7};
// const std::vector<double>  _V_THRES_APA_CAR_FCP {0.3, 0.3, 0.5};

// const std::vector<double>  _V_EGO_TURN_CAR_FCP {0.0, 0.1, 1.0, 1.7};
// const std::vector<double>  _V_THRES_TURN_CAR_FCP {0.4, 0.4, 0.8, 1.2};

// const std::vector<double>  _V_EGO_AVP_HUMAN_FCP {0.0, 0.1, 1.0, 1.7};
// const std::vector<double>  _V_THRES_AVP_HUMAN_FCP {0.2, 0.2, 0.5, 0.8};

// const std::vector<double>  _V_EGO_TURN_HUMAN_FCP {0.0, 0.1, 1.0, 1.7};
// const std::vector<double>  _V_THRES_TURN_HUMAN_FCP {0.3, 0.3, 0.7, 1.0};
} // namespace msquare

#endif /* MODULES_PLANNING_OPTIMIZERS_OBS_MANAGER_CONFIG_H_ */
