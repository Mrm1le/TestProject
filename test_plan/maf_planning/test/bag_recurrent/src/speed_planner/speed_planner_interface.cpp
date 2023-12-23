#include "planner_interface.h"
#include "speed_planner/speed_planner_interface.h"
#include "planner/motion_planner/optimizers/parking_speed_optimizer/st_pieceewise_jerk_speed_optimizer.h"



namespace speed_planner_interface{
bool getReversStatus(parking_scenario::Point2d pt,parking_scenario::Point2d pt_next){
    double dx=pt_next.x-pt.x;
    double dy=pt_next.y-pt.y;
    bool flag=(dx*cos(pt.theta)+dy*sin(pt.theta)>0 ? false:true);
    return flag;
}

MarginDebugWithDebugPara getSpeedMarginByParas(std::vector<double>& xs, 
                                const std::vector<double > ys ,std::vector<double>& thetas,
                                std::vector<parking_scenario::Point2d> obs_pts,
                                std::vector<double> curvatures,double dt,const std::string& para_str){
    
    // nlohmann::json input_json = nlohmann::json::parse(ods_str);
    // msquare::parking::OpenspaceDeciderOutput odo = input_json;
    
    std::vector<Pose2D> res_pose;
    for (int i=0;i<xs.size();++i){
        res_pose.emplace_back(xs[i],ys[i], thetas[i]);
    }

    std::vector<msquare::planning_math::Vec2d> obs_vec2d;
    for(auto obs_p:obs_pts){
        obs_vec2d.emplace_back(obs_p.x, obs_p.y);
    }
    parking_scenario::Point2d pt,pt_next;
    pt.x=xs[0];pt.y=ys[0];pt.theta=thetas[0];
    pt_next.x=xs[1];pt_next.y=ys[2];pt.theta=thetas[1];
    bool is_reverse=getReversStatus(pt,pt_next);
    std::cout<<"Is_rverse:"<<is_reverse<<std::endl;

    msquare::parking::SpeedMarginPara speed_margin_para;
    speed_margin_para.init(is_reverse, true,
        std::vector<msquare::planning_math::Box2d>{}, 
        obs_vec2d,
        std::vector<msquare::planning_math::LineSegment2d>{});
    
    speed_margin_para.is_apa_not_parallel = (msquare::HybridAstarConfig::GetInstance()->planning_core != 3);
    
    msquare::parking::SpeedMarginLimiter speed_margin_limiter(speed_margin_para, res_pose, curvatures, true);
    MarginDebugWithDebugPara debug_info;
    debug_info.debug_vec_sv.clear();
    if (para_str!=""){
        msquare::parking::SpeedMarginDebug speed_margin_debug=msquare::parking::SpeedMarginLimiter::generateFromString(para_str);
        speed_margin_para.UpdateByDebug(speed_margin_debug);
        debug_info.debug_vec_sv=speed_margin_debug.vec_sv;
        }
    speed_margin_limiter.filterSV();
    debug_info.vec_sv = speed_margin_limiter.getVecSV();
    std::vector<std::vector<msquare::planning_math::Vec2d>> polygons_2d = speed_margin_limiter.getTrajDebug();
    debug_info.polygons.reserve(polygons_2d.size());
    for(auto p2d:polygons_2d){
        std::vector<parking_scenario::Point2d> p2d_cur;
        for(auto p:p2d){
            p2d_cur.emplace_back(p.x(), p.y());
        }
        debug_info.polygons.emplace_back(p2d_cur);
    }
    return debug_info;
}


std::vector<msquare::parking::VecSV> marginVelocityLimit(std::string ods_str, std::vector<double> curvatures){
    std::cout<<"marginVelocityLimit...............ENTER"<<std::endl;
    nlohmann::json input_json = nlohmann::json::parse(ods_str);

    msquare::parking::OpenspaceDeciderOutput odo = input_json;
    msquare::SbpResult planner_res = planner_interface::planInterface(odo);
    std::reverse(planner_res.x.begin(), planner_res.x.end());
    std::reverse(planner_res.y.begin(), planner_res.y.end());
    std::reverse(planner_res.phi.begin(), planner_res.phi.end());

    std::vector<Pose2D> res_pose;
    int debug_num = msquare::CarParams::GetInstance()->car_config.common_config.debug_num;
    debug_num = std::min(int(planner_res.x.size()), debug_num);
    for(int i=0;i<debug_num /*planner_res.x.size()*/;i++){
    // for(int i=0;i<planner_res.x.size();i++){
        res_pose.emplace_back(planner_res.x[i], planner_res.y[i], planner_res.phi[i]);
    }
    std::cout<<"marginVelocityLimit:............... plan complete"<<std::endl;

    msquare::parking::SpeedMarginPara speed_margin_para;
    speed_margin_para.init(false, false, odo.obstacle_boxs, odo.points, odo.obstacle_lines);
    speed_margin_para.is_apa_not_parallel = (msquare::HybridAstarConfig::GetInstance()->planning_core != 3);
    
    msquare::parking::SpeedMarginLimiter speed_margin_limiter(speed_margin_para, res_pose, curvatures, true);
    speed_margin_limiter.filterSV();

    std::vector<msquare::parking::VecSV> vec_sv = speed_margin_limiter.getVecSV();
    if(vec_sv.empty()){
        return vec_sv;
    }

    double all_length = vec_sv[0].front().first + 0.5;
    int seg_num = 500;
    double seg_step = all_length / seg_num;
    msquare::parking::VecSV linear_inter;
    msquare::parking::SpeedRes temp_res;
    for(int i=0;i<seg_num+1;i++){
        double v;
        speed_margin_limiter.getV(i*seg_step, v, temp_res);
        linear_inter.emplace_back(i*seg_step, v);
    }
    vec_sv.push_back(linear_inter);
    return vec_sv;
}


std::vector<msquare::parking::VecSV> marginVelocityLimitDynamic(std::string ods_str, std::vector<double> curvatures, std::vector<double> last_res){
    std::cout<<"marginVelocityLimit...............ENTER"<<std::endl;
    nlohmann::json input_json = nlohmann::json::parse(ods_str);

    msquare::parking::OpenspaceDeciderOutput odo = input_json;
    msquare::SbpResult planner_res = planner_interface::planInterface(odo);
    std::reverse(planner_res.x.begin(), planner_res.x.end());
    std::reverse(planner_res.y.begin(), planner_res.y.end());
    std::reverse(planner_res.phi.begin(), planner_res.phi.end());

    std::vector<Pose2D> res_pose;
    int debug_num = msquare::CarParams::GetInstance()->car_config.common_config.debug_num;
    debug_num = std::min(int(planner_res.x.size()), debug_num);
    for(int i=0;i<debug_num /*planner_res.x.size()*/;i++){
    // for(int i=0;i<planner_res.x.size();i++){
        res_pose.emplace_back(planner_res.x[i], planner_res.y[i], planner_res.phi[i]);
    }
    std::cout<<"marginVelocityLimit:............... plan complete"<<std::endl;

    msquare::parking::SpeedMarginPara speed_margin_para;
    speed_margin_para.init(false,true, odo.obstacle_boxs, odo.points, odo.obstacle_lines);
    speed_margin_para.is_apa_not_parallel = (msquare::HybridAstarConfig::GetInstance()->planning_core != 3);

    speed_margin_para.setDynamicPlanning(last_res[0], last_res[0], true);
    msquare::parking::SpeedRes speed_res;
    switch(int(last_res[1])){
        case 0:
            speed_res.type = msquare::parking::SpeedType::ACC;
            std::cout<<"acc"<<std::endl;
            break;
        case 1:
            speed_res.type = msquare::parking::SpeedType::CONST_SPEED;
            std::cout<<"const speed"<<std::endl;
            break;
        case 2:
            speed_res.type = msquare::parking::SpeedType::DEC;
            std::cout<<"dec"<<std::endl;
            break;
        default:
            break;

    }
    speed_res.travel_s = last_res[2];
    speed_margin_para.last_res = speed_res;
    
    msquare::parking::SpeedMarginLimiter speed_margin_limiter(speed_margin_para, res_pose, curvatures, true);
    speed_margin_limiter.filterSV();

    std::vector<msquare::parking::VecSV> vec_sv = speed_margin_limiter.getVecSV();
    if(vec_sv.empty()){
        return vec_sv;
    }

    double all_length = vec_sv[0].front().first + 0.5;
    int seg_num = 500;
    double seg_step = all_length / seg_num;
    msquare::parking::VecSV linear_inter;
    msquare::parking::SpeedRes temp_res;
    for(int i=0;i<seg_num+1;i++){
        double v;
        speed_margin_limiter.getV(i*seg_step, v, temp_res);
        // std::cout<<i*seg_step<<" "<<temp_res.type<<" "<<temp_res.travel_s<<std::endl;
        linear_inter.emplace_back(i*seg_step, v);
    }
    vec_sv.push_back(linear_inter);
    return vec_sv;
}

std::vector<std::vector<double> > getSVByT(std::string ods_str, std::vector<double> curvatures, double dt){
    std::cout<<"marginVelocityLimit...............ENTER"<<std::endl;
    nlohmann::json input_json = nlohmann::json::parse(ods_str);

    msquare::parking::OpenspaceDeciderOutput odo = input_json;
    msquare::SbpResult planner_res = planner_interface::planInterface(odo);
    std::reverse(planner_res.x.begin(), planner_res.x.end());
    std::reverse(planner_res.y.begin(), planner_res.y.end());
    std::reverse(planner_res.phi.begin(), planner_res.phi.end());

    std::vector<Pose2D> res_pose;
    int debug_num = msquare::CarParams::GetInstance()->car_config.common_config.debug_num;
    debug_num = std::min(int(planner_res.x.size()), debug_num);
    for(int i=0;i<debug_num /*planner_res.x.size()*/;i++){
    // for(int i=0;i<planner_res.x.size();i++){
        res_pose.emplace_back(planner_res.x[i], planner_res.y[i], planner_res.phi[i]);
    }
    std::cout<<"marginVelocityLimit:............... plan complete"<<std::endl;

    msquare::parking::SpeedMarginPara speed_margin_para;
    speed_margin_para.init(false, false, odo.obstacle_boxs, odo.points, odo.obstacle_lines);
    speed_margin_para.is_apa_not_parallel = (msquare::HybridAstarConfig::GetInstance()->planning_core != 3);
    
    msquare::parking::SpeedMarginLimiter speed_margin_limiter(speed_margin_para, res_pose, curvatures, true);
    speed_margin_limiter.filterSV();

    msquare::parking::VecST vec_st;
    bool is_valid = speed_margin_limiter.getSVByT(vec_st, dt);
    std::cout<<"getSVByT, is_valid:"<<is_valid<<std::endl;

    std::vector<std::vector<double> > res;
    for(auto& st:vec_st){
        res.push_back({st.t, st.s, st.v, st.a});
    }
    return res;
}

std::vector<std::vector<double>> solveSpeed(const std::vector<std::vector<double> >& vec_st, 
            double dt, double ego_speed,const double init_a){
    msquare::parking::SpeedPlannerCofig config(dt, true);
    std::vector<std::vector<double>> path;
    
    msquare::parking::VecST vec_st_real;
    std::cout << "the config is: " << config.s_w <<"  "
    << config.v_w << " " << config.a_w  << " " << config.j_w
    << " " << config.s_ref_w << " " << config.v_ref_w
    << " " << config.dt  << " " << config.a_lo 
    << " " << config.a_up << " " << config.v_lo
    << " " << config.v_up << " " << config.j_lo
    << " " << config.j_up << std::endl;   // weight for v
    // config.a_w = config_vector[2];   // weight for a
    // config.j_w = config_vector[3];   // weight for jerk
    // config.s_ref_w = config_vector[4];   // weight for s_ref
    // config.v_ref_w = config_vector[5];   // weight for v_ref
    // config.dt = config_vector[6];    // delta t
    // config.a_lo = config_vector[7];  // for acc
    // config.a_up = config_vector[8];
    // config.v_lo = config_vector[9];  // for velocity
    // config.v_up = config_vector[10];
    // config.j_lo = config_vector[11];  // for jerk
    // config.j_up = config_vector[12];
    vec_st_real.reserve(vec_st.size());
    for(auto& v: vec_st){
        vec_st_real.emplace_back(v[0], v[1], v[2], v[3]);
    }
    msquare::parking::PiecewiseJerkSpeedOptimizer piece_wise_optimizer(config); 
    msquare::parking::VecST first_st_obs;
    msquare::parking::VecST second_st_obs;

    piece_wise_optimizer.makeOptimize(std::fabs(ego_speed), init_a, &path, 
                vec_st_real, first_st_obs, second_st_obs);

    std::cout << "path size is===========:" << path.size() << std::endl;
    return path;
}

std::vector<std::vector<double>> solveSpeedByConfig(const std::vector<std::vector<double> > &vec_st,
                        const std::vector<std::vector<double> > &first_obs_vec_st,
            const std::vector<std::vector<double> > &second_obs_vec_st,
            double dt, double ego_speed, const double init_a, std::vector<double> config_vector) {
    std::cout << "Start QP Speed " << std::endl;
    std::vector<std::vector<double>> path;
    msquare::parking::SpeedPlannerCofig config;
    // for (const auto data : config_vector) {
    //     std::cout << "!!!!!!!" << data << std::endl;
    // }
    if (vec_st.empty()) {
        return path;
    }
    if (config_vector.size() < 13) {
        return path;
    }
    config.dt = dt;
    // config.is_reverse = true;
    config.s_w =  config_vector[0];  // weight for s
    config.v_w = config_vector[1];   // weight for v
    config.a_w = config_vector[2];   // weight for a
    config.j_w = config_vector[3];   // weight for jerk
    config.s_ref_w = config_vector[4];   // weight for s_ref
    config.v_ref_w = config_vector[5];   // weight for v_ref
    config.dt = config_vector[6];    // delta t
    config.a_lo = config_vector[7];  // for acc
    config.a_up = config_vector[8];
    config.v_lo = config_vector[9];  // for velocity
    config.v_up = config_vector[10];
    config.j_lo = config_vector[11];  // for jerk
    config.j_up = config_vector[12];
    
    msquare::parking::VecST vec_st_real;
    vec_st_real.reserve(vec_st.size());
    for(auto& v: vec_st){
        vec_st_real.emplace_back(v[0], v[1], v[2], v[3]);
    }
    msquare::parking::PiecewiseJerkSpeedOptimizer piece_wise_optimizer(config);
    msquare::parking::VecST first_st_obs;
    msquare::parking::VecST second_st_obs;
    first_st_obs.reserve(first_obs_vec_st.size());
    for(auto& v: first_obs_vec_st){
        first_st_obs.emplace_back(v[0], v[1], v[2], v[3]);
    }
    second_st_obs.reserve(second_obs_vec_st.size());
    for(auto& v: second_obs_vec_st){
        second_st_obs.emplace_back(v[0], v[1], v[2], v[3]);
    }
    std::cout << "the first obs size is:" << first_st_obs.size()
              << " second obs size is: " << second_st_obs.size()
              << std::endl;
    piece_wise_optimizer.makeOptimize(std::fabs(ego_speed), init_a, 
            &path, vec_st_real, first_st_obs, second_st_obs);

    std::cout << "path size is ---------->:" << path.size() << std::endl;
    return path;
}

}
