#include "planner/motion_planner/optimizers/openspace_optimizer/State.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"


namespace msquare
{

class NextStateGenerator
{
public:
    int next_node_num_;
    int step_direction_;
    double step_size_;
    double xy_grid_resolution_;
    double phi_grid_resolution_;

    double wheel_base_;                 //轴距
    double max_delta_angle_;            //最大前轮转角（deg）
    double max_delta_angle_rate_;       //最大前轮转速（deg/s）
    bool enable_multiple_steer_modes_;  //支持后轮转向


public:
    //forward and backward are currently symmetrical.
    //  for better future extensibilty 
    //  (forward rotation radius might differ from backward rotation radius)
    //  here defines seperatly
    std::vector<std::shared_ptr<SearchNode>> origin_next_states_forward_; //next states generated by origin(0,0,0)
    std::vector<std::shared_ptr<SearchNode>> origin_next_states_backward_; //next states generated by origin(0,0,0)
    
public:
    NextStateGenerator();
    ~NextStateGenerator();

    void setParamsFromConfig();
    
    void printAllParams();    
    bool checkParamsConsistencyConfig();

    void generateAllOriginNextStates();

    std::vector<std::shared_ptr<SearchNode>> 
    getNextStateByLookup(const std::shared_ptr<SearchNode> &current);

    std::vector<std::shared_ptr<SearchNode>>
    getNextStateByCalculation(const std::shared_ptr<SearchNode> &current);
    
};



}