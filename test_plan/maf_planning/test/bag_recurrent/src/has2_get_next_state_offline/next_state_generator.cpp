#include "has2_get_next_state_offline/next_state_generator.h"
#include "common/math/math_utils.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "slog_macro_impl.h"

namespace msquare
{


NextStateGenerator::NextStateGenerator()
{
    
}

NextStateGenerator::~NextStateGenerator()
{
    //do nothing
}

void NextStateGenerator::setParamsFromConfig()
{
    //HybridAstarConfig
    next_node_num_ = HybridAstarConfig::GetInstance()->next_node_num;
    step_direction_ = HybridAstarConfig::GetInstance()->step_direction;
    step_size_ = HybridAstarConfig::GetInstance()->step_size;
    xy_grid_resolution_ = HybridAstarConfig::GetInstance()->xy_grid_resolution;
    phi_grid_resolution_ = HybridAstarConfig::GetInstance()->phi_grid_resolution;

    //CarParams
    enable_multiple_steer_modes_ = CarParams::GetInstance()->enable_multiple_steer_modes;
    max_delta_angle_ = CarParams::GetInstance()->max_delta_angle;
    wheel_base_ = CarParams::GetInstance()->wheel_base;
    max_delta_angle_rate_ = CarParams::GetInstance()->max_delta_angle_rate;

}


bool NextStateGenerator::checkParamsConsistencyConfig()
{
    
//  macro explanation: check if PARAM1 == param 2, if fails, print a message using FORMAT and set RESULT_VAR to false
#define CHECK_PARAM_CONSISTENCY(PARAM1, PARAM2, FORMAT, RESULT_VAR)                                                                       \
    if(PARAM1 != PARAM2){                                                                                                   \
        SLOG_WARN("Param Consistency : "#PARAM1" = "#FORMAT", "#PARAM2" = "#FORMAT", ", PARAM1, PARAM2);       \
        RESULT_VAR =  false;                                                                                                     \
    }

    bool check_result = true;

    //HybridAstarConfig
    CHECK_PARAM_CONSISTENCY(next_node_num_, HybridAstarConfig::GetInstance()->next_node_num, %d, check_result)
    CHECK_PARAM_CONSISTENCY(step_direction_, HybridAstarConfig::GetInstance()->step_direction, %d, check_result)
    CHECK_PARAM_CONSISTENCY(step_size_, HybridAstarConfig::GetInstance()->step_size, %f, check_result)
    CHECK_PARAM_CONSISTENCY(xy_grid_resolution_, HybridAstarConfig::GetInstance()->xy_grid_resolution, %f, check_result)
    CHECK_PARAM_CONSISTENCY(phi_grid_resolution_, HybridAstarConfig::GetInstance()->phi_grid_resolution, %f, check_result)

    //CarParams
    CHECK_PARAM_CONSISTENCY(wheel_base_, CarParams::GetInstance()->wheel_base, %f, check_result)
    CHECK_PARAM_CONSISTENCY(max_delta_angle_, CarParams::GetInstance()->max_delta_angle, %f, check_result)
    CHECK_PARAM_CONSISTENCY(max_delta_angle_rate_, CarParams::GetInstance()->max_delta_angle_rate, %f, check_result)
    CHECK_PARAM_CONSISTENCY(enable_multiple_steer_modes_, CarParams::GetInstance()->enable_multiple_steer_modes, %d, check_result)

    return check_result;
}
void NextStateGenerator::printAllParams()
{
    SLOG_INFO("[NextStateGenerator]Param next_node_num_ = %d", next_node_num_);
    SLOG_INFO("[NextStateGenerator]Param step_direction_ = %d", step_direction_);
    SLOG_INFO("[NextStateGenerator]Param step_size_ = %f", step_size_);
    SLOG_INFO("[NextStateGenerator]Param xy_grid_resolution_ = %f", xy_grid_resolution_);
    SLOG_INFO("[NextStateGenerator]Param enable_multiple_steer_modes_ = %d", enable_multiple_steer_modes_);
    SLOG_INFO("[NextStateGenerator]Param max_delta_angle_ = %f", max_delta_angle_);
    SLOG_INFO("[NextStateGenerator]Param wheel_base_ = %f", wheel_base_);
    SLOG_INFO("[NextStateGenerator]Param max_delta_angle_rate_ = %f", max_delta_angle_rate_);
}
    

void NextStateGenerator::generateAllOriginNextStates()
{
    std::shared_ptr<SearchNode> origin_forward = std::shared_ptr<SearchNode>(
            new SearchNode(0, 0, 0, step_size_, 0));

    std::shared_ptr<SearchNode> origin_backward = std::shared_ptr<SearchNode>(
        new SearchNode(0, 0, 0, -step_size_, 0));



    origin_next_states_forward_ = getNextStateByCalculation(origin_forward);
    origin_next_states_backward_ = getNextStateByCalculation(origin_backward);
    
}

std::vector<std::shared_ptr<SearchNode>> 
NextStateGenerator::getNextStateByLookup(const std::shared_ptr<SearchNode> &current)
{
    std::vector<std::shared_ptr<SearchNode>> result;
    const std::vector<std::shared_ptr<SearchNode>>& lut = current->vel >=0? 
        origin_next_states_forward_ : origin_next_states_backward_;  //lookup table set according to vel

    result.resize(lut.size());
    for(int i=0; i<(int)result.size(); i++)
    {
        result[i]=std::shared_ptr<SearchNode>(
            new SearchNode(current->x + lut[i]->x, current->y + lut[i]->y, 
                current->theta + lut[i]->theta, lut[i]->vel, lut[i]->delta));
    }
    
    return result;
}

std::vector<std::shared_ptr<SearchNode>>
NextStateGenerator::getNextStateByCalculation(const std::shared_ptr<SearchNode> &current) 
{
    
    if (enable_multiple_steer_modes_) {
        SLOG_ERROR("invalid rear-wheel steering option. This implementation has removed RWS support!");
        throw std::invalid_argument("invalid rear-wheel steering option. This implementation has removed RWS support!");
    }

    if (next_node_num_ % 2 == 0 || next_node_num_ < 2) {
        SLOG_ERROR("invalid next_node_num!");
        throw std::invalid_argument("invalid next_node_num!");
    }

    if (step_direction_ != 0) {
        SLOG_ERROR("invalid step_direction option(must be 0) This implementation only support Bi-directional search");
        throw std::invalid_argument("invalid step_direction option(must be 0) This implementation only support Bi-directional search");
    } 

    std::vector<std::shared_ptr<SearchNode>> next;
    double next_x, next_y, next_theta;
    double alpha, beta, R;

    double x, y, theta;

    double deg2rad = M_PI / 180.0;

    // double wheel_base_offset = wheel_base_offset_options_[i];  // wheel_base_offset = constant zero
    double max_delta_angle_apply = max_delta_angle_;
    double initial_travel_step = -step_size_;
    double terminal_travel_step = step_size_;
    for (double traveled_distance = initial_travel_step; traveled_distance < terminal_travel_step + 0.0001; traveled_distance += step_size_ * 2.0 / 16) {
        //TODO: understand this equation (what does max_delta_rate_qauivalent mean)

        // double max_delta_rate_equivalent =
        //     CarParams::GetInstance()->max_delta_angle_rate *
        //     (wheel_base_ + wheel_base_offset) *
        //     std::cos(current->delta * deg2rad) *
        //     std::cos(current->delta * deg2rad) / wheel_base_;

        //when wheel_base_offset is constant zero, this is equal 
        double max_delta_rate_equivalent = max_delta_angle_rate_ * std::cos(current->delta * deg2rad) * std::cos(current->delta * deg2rad);

        double max_delta_rate_apply = max_delta_rate_equivalent; 

        double steer_lower, steer_upper;


        // if (traveled_distance * current->vel < 0) {
        //   steer_lower = -max_delta_angle_apply;
        //   steer_upper = max_delta_angle_apply;
        // } else {
        //   steer_lower = std::max(current->delta - max_delta_rate_apply,
        //                               -max_delta_angle_apply);
        //   steer_upper = std::min(current->delta + max_delta_rate_apply,
        //                               max_delta_angle_apply);
        // }
        
        //no delta considered
        steer_lower = -max_delta_angle_apply;
        steer_upper = max_delta_angle_apply;


        double delta_step = (steer_upper - steer_lower) / (double)(next_node_num_ - 1);
    
        // transform the state w.r.t virtual wheel base
        x = current->x;
        y = current->y;
        theta = current->theta;

        for (alpha = steer_lower; alpha < steer_upper + 0.001; alpha += delta_step) {
        
            if (std::abs(alpha) < 1e-6) {
                alpha = alpha > 0 // parasoft-suppress AUTOSAR-M6_5_3 "f-drop"
                            ? 1e-6
                            : -1e-6;
            }

            R = (wheel_base_) / std::tan(alpha * deg2rad);
            beta = traveled_distance / R;
            next_theta = planning_math::NormalizeAngle(theta + beta);
            next_x = x + R * (std::cos(theta) * std::sin(beta) -
                                std::sin(theta) * (1 - std::cos(beta)));
            next_y = y + R * (std::sin(theta) * std::sin(beta) +
                                std::cos(theta) * (1 - std::cos(beta)));
            //discrete
            //SLOG_DEBUG("     before discrete: %f %f %f ", next_x, next_y, next_theta);
            // next_x = xy_grid_resolution_ * std::round(next_x/xy_grid_resolution_);
            // next_y = xy_grid_resolution_ * std::round(next_y/xy_grid_resolution_);
            // next_theta = phi_grid_resolution_ * std::round(next_theta/phi_grid_resolution_);
            //SLOG_DEBUG("     after discrete: %f %f %f ", next_x, next_y, next_theta);

            std::shared_ptr<SearchNode> next_node = std::shared_ptr<SearchNode>(
              new SearchNode(next_x, next_y, next_theta, traveled_distance, alpha));
            next.push_back(next_node);
        }   
    }

    return next;
}



}