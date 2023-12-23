#ifndef PARKING_SCENARIO_POSE_ADJUSTER
#define PARKING_SCENARIO_POSE_ADJUSTER

#include "parking_scenario/parking_scenario.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/hybrid_a_star_2.h"

namespace parking_scenario {

class PoseAdjuster{
public:
    PoseAdjuster(const PlanParams& params);
    void reset(const PlanParams& params);

    ~PoseAdjuster(){}

    std::vector<std::string> planOnce(const Point2d& point, msquare::parking::SearchProcessDebug *sp_debug = nullptr);

    double getMinX(){ return min_x_; }
    double getMinY(){ return min_y_; }
    double getMaxX(){ return max_x_; }
    double getMaxY(){ return max_y_; }
    std::string getOdo();

private:
    void generateParallelScenario(const PlanParams &params,
                              msquare::parking::OpenspaceDeciderOutput &odo);
    void generateVerticalScenario(const PlanParams &params,
                              msquare::parking::OpenspaceDeciderOutput &odo);
    void generateVerticalScenario2(const PlanParams &params,
                              msquare::parking::OpenspaceDeciderOutput &odo);
    void generateObliqueScenario(const PlanParams &params,
                              msquare::parking::OpenspaceDeciderOutput &odo);
    msquare::parking::OpenspaceDeciderOutput odo_;
    const PlanParams params_;
    double min_x_;
    double min_y_;
    double max_x_;
    double max_y_;
    double channel_width_;
    bool out_;

    std::shared_ptr<msquare::hybrid_a_star_2::HybridAstar2> astar2_solver_;
};

}   // end namespace parking_scenario

#endif