#include "core/vehicle.hpp"
#include "worker/StrightLine1.h"
#define MAX_SEARCH_CNT 200
namespace im_ddtool
{
    bool StrightLine1::RunOnce()
    {
        if (vehicle_model_->CalculateRSP(current_node_))
            return true; // 调用RS曲线生成函数判断是否当前是否可为接管点

        while (++search_direction < 2) // 遍历搜索前进\后退
        {
            while (++search_cnt < 71) // 遍历搜索71个type
            {
                vehicle_model_->SetCurrentSteerReq(steer_req_angle); // 设置当前转角
                while (++step < MAX_SEARCH_CNT &&
                       (vehicle_model_->GetReedSheppPath()->total_length > 8 ||
                        std::abs(vehicle_model_->GetReedSheppPath()->total_length - 0.0f) < math::EPS))
                {
                    auto self_position_ = vehicle_model_->GetSelfPosition();
                    vehicle_model_->UserOperationPreProcess(search_direction);
                    vehicle_model_->TrajectoryGenerator();
                    vehicle_model_->CalNextPosition();
                    current_node_ = std::shared_ptr<SearchNode>(
                        new SearchNode(self_position_.x, self_position_.y,
                                       self_position_.yaw, 0, 0, 0)); // 当前点更新
                    Pose2D trajectory_point;
                    trajectory_point.x = self_position_.x;
                    trajectory_point.y = self_position_.y;
                    trajectory_point.theta = self_position_.yaw;
                    trajectory_.push_back(trajectory_point); //轨迹更新
                    vehicle_model_->CalculateRSP(current_node_); // 计算当前点是否为满足接观点(使用RS范式完成泊入)

                    auto this_score = scorer_->RunWorkerScore(trajectory_); // 计算当前轨迹得分

                    if (this_score > best_score)
                    {
                        best_trajectory_.clear();
                        best_RS_path = *vehicle_model_->GetReedSheppPath(); // 当前为最高分，存储最优RS轨迹
                        best_score = this_score;
                        best_trajectory_ = trajectory_; //当前最优轨迹序列, 输出1！！！！！！！！！！！！！
                    }
                }
            }
            // vehicle_model_->CalBehindCost();
            steer_req_angle += 1.0f;
            vehicle_model_->Reset();
            trajectory_.clear();
            vehicle_model_->InitSelfPosition(init_position_);
            vehicle_model_->InitSlotPosition(init_slot_position_, slot_type_); // 继续下一次循环
        }
    }

    if (best_score > 0.0f) //获取最高分
    {
        vehicle_model_->SetReedSheppPath(&best_RS_path);
        auto break_point0 = std::make_tuple<int, int>(0, best_search_direction * 71 + 70 - best_search_cnt); // 折点1
        printf("break_point0[%d][%d]\n ", std::get<0>(break_point0), std::get<1>(break_point0));
        auto break_point1 = std::make_tuple<int, int>(best_trajectory_.size(), best_search_direction * 71 + 70 - best_search_cnt); // 折点2
        printf("break_point1[%d][%d]\n ", std::get<0>(break_point1), std::get<1>(break_point1));
        break_points_.push_back(break_point0);
        break_points_.push_back(break_point1); // 折点序列，输出2！！！！！！！！！！！！！
        return true;
    }
    return false;
}
