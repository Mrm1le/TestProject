#ifndef MSQUARE_PARKING_TIME_HYBRID
#define MSQUARE_PARKING_TIME_HYBRID

#include "common/planning_context.h"
#include "common/sbp_strategy.h"
#include "common/priority_obs.h"
#include "common/utils/yaml_utils.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/openspace_optimizer.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/State.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/openspace_footprint_model.h"
#include "common/footprint_model.h"
#include "planner/message_type.h"

#include "common/sbp_obstacle_box.h"
#include "common/sbp_obstacle_line.h"
#include "common/sbp_obstacle_point.h"
#include "common/utils/trajectory_point_utils.h"

#include "matplotlibcpp.h"

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <dirent.h>
#include <chrono>
#include <fstream>
#include <sys/stat.h>
#include <iomanip>　

using namespace std;
using namespace msquare;
using namespace msquare::parking;
using namespace msquare::planning_math;

namespace plt = matplotlibcpp;

void plotPoint(const std::vector<Vec2d>& ps, double scale,  const std::map<std::string, std::string>& keywords);
void plotLine(LineSegment2d& line, string color);
void plotBox(Box2d& box, string color);
void plotTrajectory(std::vector<TrajectoryPoint>& traj, string color);
void plotCar(TrajectoryPoint& p, string color);
void plotOSD(OpenspaceDeciderOutput& osd);
void GetFileNames(string path,vector<string>& filenames);
bool isDirectory(string path);
void initConfig();

void GetFileNames(string path,vector<string>& filenames);
void testMulti(int argc, const char**argv);
double runPoint(int iter_times, int split_num, Pose2D& start_pose, FootprintModelPtr& car_model_, std::vector<Vec2d>& points);

void testPlanner(int argc,const char**argv){
    if(argc < 3){
        std::cout<<"exe -p file/folder {-s}"<<std::endl;
        return ;
    }

    string path=argv[2];
    bool is_show = true;
    if(argc > 3){
        is_show = bool(strcmp(argv[3], "-s"));
    }

    vector<string> filenames;
    if(isDirectory(path)){
        GetFileNames(path, filenames);
    }else{
        filenames.push_back(path);
    }
    for(int i=0; i<filenames.size();i++){
        std::cout<<filenames[i]<<std::endl;
    }


    vector<double> time_used;
    double time_all=0;
    string time_log_file = "time_log.txt";
    for(int i=0;i<filenames.size();i++){
        std::cout<<"TEST FILE:"<<filenames[i]<<std::endl;
        YAML::Node node = YAML::LoadFile(filenames[i]);
        OpenspaceDeciderOutput osd = node.as<OpenspaceDeciderOutput>();
        auto &target_state = osd.target_state;
        Pose2D target_pose(target_state.path_point.x, target_state.path_point.y,
                        target_state.path_point.theta);
        msquare::SearchBasedPlannerPtr planner = std::make_shared<msquare::HybridAstar>(target_pose, 0, osd.map_boundary);
        auto t1=std::chrono::steady_clock::now();
        std::vector<TrajectoryPoint> traj = planWithStrategy(planner, msquare::StrategyParams::GetInstance(), osd);
        
        auto t2=std::chrono::steady_clock::now();
        double chrono_ms=std::chrono::duration<double, std::milli>(t2-t1).count();
        time_used.push_back(chrono_ms);
        time_all += chrono_ms;
        
        if(!is_show){
            continue;
        }
        plt::figure_size(1200, 800);
        plt::set_aspect_equal();
        plotTrajectory(traj, "y--");
        plotOSD(osd);
        plt::show();

    }
    std::cout<<"\n************************Result***************"<<std::endl;
    std::cout<<"AVG:"<<time_all/filenames.size()<<"\tTIME:";
    for(int i=0;i<time_used.size();i++){
        std::cout<<"\t"<<time_used[i];
    }
    std::cout<<std::endl;
    // std::cout << "Time(ms) all:"<<chrono_ms<<"  avg:"<<chrono_ms/iter_num<<std::endl;

    ofstream time_file(time_log_file, ios::out|ios::app);
    time_file<<"\n************************Result";
    time_file<< (HybridAstarConfig::GetInstance()->use_t_line_ ? " use grid": " no  grid");
    time_file<<"***************"<<std::endl;
    time_file<<"AVG:"<<time_all/filenames.size()<<"\tTIME:";
    for(int i=0;i<time_used.size();i++){
        time_file<<"\t"<<time_used[i];
    }
    time_file<<std::endl;
    time_file.close();
    std::cout<<"save to file:"<< time_log_file <<std::endl;

}

void testPointSplit(int argc, char const *argv[]){
    if(argc != 3){
        std::cout<<argc<<"argc"<<std::endl;
        std::cout<<"exe. iter_num config_file "<<std::endl;
        return ;
    }
    YAML::Node node = YAML::LoadFile(argv[2]);
    OpenspaceDeciderOutput osd = node.as<OpenspaceDeciderOutput>();
    std::vector<SbpObstaclePtr> sbp_obs_ptrs;
    sbp_obs_ptrs.push_back(std::make_shared<SbpObstacleLine>(osd.obstacle_lines));
    sbp_obs_ptrs.push_back(std::make_shared<SbpObstaclePoint>(osd.points));
    sbp_obs_ptrs.push_back(std::make_shared<SbpMapLine>(osd.lines));
    sbp_obs_ptrs.push_back(std::make_shared<SbpObstacleBox>(osd.obstacle_boxs));
     
    // create foorprint model
    FootprintModelPtr box_model = std::make_shared<BoxFootprintModel>(
        VehicleParam::Instance(), 
        CarParams::GetInstance()->lat_inflation(),
        CarParams::GetInstance()->shrink_ratio_for_lines_
    );
    FootprintModelPtr circle_model = std::make_shared<CircleFootprintModel>(
        VehicleParam::Instance(),
        CarParams::GetInstance()->inflation_rearview_mirror,
        CarParams::GetInstance()->shrink_ratio_for_lines_
    );
    FootprintModelPtr car_model = std::make_shared<CompositeFootprintModel>(
            std::vector<FootprintModelPtr>({box_model, circle_model})
    );

    Pose2D start_pose(osd.init_state.path_point.x, osd.init_state.path_point.y, osd.init_state.path_point.theta);
    std::shared_ptr<SearchNode> current_state = std::make_shared<SearchNode>(
        osd.init_state.path_point.x,
        osd.init_state.path_point.y,
        osd.init_state.path_point.theta
    );
    int iter_times = atoi(argv[1]);
    std::vector<double> time_res;
    std::vector<int> time_split;
    double time_usage=0;
    int split_num = 1;
    time_usage = runPoint(iter_times, split_num,start_pose, car_model, osd.points);
    time_res.push_back(time_usage);
    time_split.push_back(1);

    split_num = 2;
    time_usage = runPoint(iter_times, split_num,start_pose, car_model, osd.points);
    time_res.push_back(time_usage);
    time_split.push_back(2);

    split_num = 5;
    time_usage = runPoint(iter_times, split_num,start_pose, car_model, osd.points);
    time_res.push_back(time_usage);
    time_split.push_back(5);


    split_num = 10;
    time_usage = runPoint(iter_times, split_num,start_pose, car_model, osd.points);
    time_res.push_back(time_usage);
    time_split.push_back(10);


    std::cout<<"\n************************Result***************"<<std::endl;
    std::cout<<"SPLIT_NUM:";
    for(int i=0;i<time_res.size();i++){
        std::cout<<"\t"<<time_split[i];
    }
    std::cout<<endl;

    std::cout<<"     TIME:";
    for(int i=0;i<time_res.size();i++){
        std::cout<<"\t"<<time_res[i];
    }
    std::cout<<"\n"<<endl;
}

std::vector<double>  runBoxPoint(int iter_times,Pose2D& start_pose, FootprintModelPtr& car_model, OpenspaceDeciderOutput& osd){
    std::vector<SbpObstaclePtr> obs_ptrs_p, obs_ptrs_l,obs_ptrs_mapl, obs_ptrs_b;
    obs_ptrs_l.push_back(std::make_shared<SbpObstacleLine>(osd.obstacle_lines));
    obs_ptrs_mapl.push_back(std::make_shared<SbpMapLine>(osd.lines));
    obs_ptrs_p.push_back(std::make_shared<SbpObstaclePoint>(osd.points));

    for(auto& b:osd.obstacle_boxs){
        obs_ptrs_b.push_back(std::make_shared<SbpObstacleBox>(b));
    }

    std::vector<double> num_time={osd.points.size(),osd.obstacle_lines.size(),osd.lines.size(), osd.obstacle_boxs.size(),0,0,0,0};
    for(int i=0; i< iter_times;i++){
        std::shared_ptr<SearchNode> current_state = std::make_shared<SearchNode>(
            start_pose.x+0.01*i,
            start_pose.y+0.01*i,
            start_pose.theta
        );

        current_state->previous=current_state;
        auto t1=std::chrono::steady_clock::now();
        for(const SbpObstaclePtr &obs_ptr: obs_ptrs_p){
            obs_ptr->checkCollision(current_state, car_model);
        }
        auto t2=std::chrono::steady_clock::now();
        for(const SbpObstaclePtr &obs_ptr: obs_ptrs_l){
            obs_ptr->checkCollision(current_state, car_model);
        }
        auto t3=std::chrono::steady_clock::now();
        for(const SbpObstaclePtr &obs_ptr: obs_ptrs_mapl){
            obs_ptr->checkCollision(current_state, car_model);
        }
        auto t4=std::chrono::steady_clock::now();
        for(const SbpObstaclePtr &obs_ptr: obs_ptrs_b){
            obs_ptr->checkCollision(current_state, car_model);
        }
        auto t5=std::chrono::steady_clock::now();
        num_time[4] += std::chrono::duration<double, std::milli>(t2-t1).count();
        num_time[5] += std::chrono::duration<double, std::milli>(t3-t2).count();
        num_time[6] += std::chrono::duration<double, std::milli>(t4-t3).count();
        num_time[7] += std::chrono::duration<double, std::milli>(t5-t4).count();
    }

    return num_time;
}

void testPLB(int argc,const char**argv){
    if(argc < 3){
        std::cout<<"exe. config_file num path"<<std::endl;
        return ;
    }

    string path=argv[1];
    vector<string> filenames;
    if(isDirectory(path)){
        GetFileNames(path, filenames);
    }else{
        filenames.push_back(path);
    }

    int iter_times = atoi(argv[2]);


    ofstream time_file("time_plb.txt", ios::out|ios::app);
    time_file<<"\n************************ ITER NUMBER - ";
    time_file<< iter_times;
    time_file<<"***********************"<<std::endl;
    time_file<<"obstacle: point\tline\tmap_l\tbox\t | time_ratio = time / all_time \t | time_single = time / count "<<std::endl;
    for(int i=0; i<filenames.size();i++){
        std::cout<<filenames[i]<<std::endl;
        YAML::Node node = YAML::LoadFile(filenames[i]);
        OpenspaceDeciderOutput osd = node.as<OpenspaceDeciderOutput>();
     
        // create foorprint model
        FootprintModelPtr box_model = std::make_shared<BoxFootprintModel>(
            VehicleParam::Instance(), 
            CarParams::GetInstance()->lat_inflation(),
            CarParams::GetInstance()->shrink_ratio_for_lines_
        );
        FootprintModelPtr circle_model = std::make_shared<CircleFootprintModel>(
            VehicleParam::Instance(),
            CarParams::GetInstance()->inflation_rearview_mirror,
            CarParams::GetInstance()->shrink_ratio_for_lines_
        );
        FootprintModelPtr car_model = std::make_shared<CompositeFootprintModel>(
                std::vector<FootprintModelPtr>({box_model, circle_model})
        );

        Pose2D start_pose(osd.init_state.path_point.x, osd.init_state.path_point.y, osd.init_state.path_point.theta);

        std::vector<double> num_time= runBoxPoint(iter_times,start_pose, car_model, osd);

       
        double t_all = num_time[4] + num_time[5] +num_time[6] +num_time[7];
        time_file << "----------------------------------------------------------------------------" <<std::endl;
        time_file<<"count: " << num_time[0] << "\t" << num_time[1] <<"\t"<< num_time[2]<<"\t"<< num_time[3] << std::endl;
        time_file<<"t(ms): "
                 << setprecision(4) << num_time[4] << "\t" << num_time[5] <<"\t"<< num_time[6] <<"\t"<< num_time[7] << "\t | "
                 << setprecision(2) << num_time[4]/t_all << "\t" << num_time[5]/t_all << "\t"<< num_time[6]/t_all << "\t"<< num_time[7]/t_all << "\t|"
                 << setprecision(4) << (num_time[0] == 0 ? 0: num_time[4]/num_time[0] ) << "\t"
                 << setprecision(4) << (num_time[1] == 0 ? 0: num_time[5]/num_time[1] ) << "\t"
                 << setprecision(4) << (num_time[2] == 0 ? 0: num_time[6]/num_time[2] ) << "\t"
                 << setprecision(4) << (num_time[3] == 0 ? 0: num_time[7]/num_time[3] ) << "\t"
                 << std::endl;

    }
    time_file.close();

}

double runPoint(int iter_times,int split_num, Pose2D& start_pose, FootprintModelPtr& car_model, std::vector<Vec2d>&  points){
    int point_size=points.size();
    if(split_num<1 || point_size<1){
        std::cout<<"ERROR: split_num must > 0"<<std::endl;
    }

    std::cout<<"\n************************Test Point***************"<<std::endl;
    int shang=point_size / split_num;
    int yushu=point_size % split_num;

    std::cout<<"SPLIT NUM:"<<split_num;
    std::cout<<"\tPOINT SIZE:"<<point_size<<std::endl;
    std::vector<SbpObstaclePtr>  obs_pss(split_num);
    int last_index=0;
    int add_num=0;
    for(int i=0;i<split_num;i++){
        if(yushu > 0){
            add_num =shang+1;
            yushu --;
        }else{
            add_num = shang;
        }
        std::vector<Vec2d> pts;

        pts.insert(pts.end(), points.begin()+last_index, points.begin()+last_index+add_num);
        obs_pss[i]=std::make_shared<SbpObstaclePoint>(pts);
        last_index +=add_num;
    }

    auto t1=std::chrono::steady_clock::now();
    for(int i=0; i< iter_times;i++){
        std::shared_ptr<SearchNode> current_state = std::make_shared<SearchNode>(
            start_pose.x+0.01*i,
            start_pose.y+0.01*i,
            start_pose.theta
        );
        current_state->previous=current_state;
        for(const SbpObstaclePtr &obs_ptr: obs_pss){
            obs_ptr->checkCollision(current_state, car_model);
        }
    }
    auto t2=std::chrono::steady_clock::now();
    double chrono_ms=std::chrono::duration<double, std::milli>(t2-t1).count();
    std::cout<<"time usage:"<<chrono_ms<<std::endl;
    return chrono_ms;
}

void displayScene(int argc, const char ** argv){
    if(argc < 3){
        std::cout<<"exe.  config_file "<<std::endl;
        return ;
    }

    string path=argv[2];
    vector<string> filenames;
    if(isDirectory(path)){
        GetFileNames(path, filenames);
    }else{
        filenames.push_back(path);
    }
    for(string file_name:filenames){
        std::cout<<"TEST FILE:"<<file_name<<std::endl;
        YAML::Node node = YAML::LoadFile(file_name);
        OpenspaceDeciderOutput osd = node.as<OpenspaceDeciderOutput>();
        Vec2d start_point(osd.init_state.path_point.x,  osd.init_state.path_point.y);
        Vec2d target_point(osd.target_state.path_point.x,  osd.target_state.path_point.y);

        Vec2d pp(0,0);
        Box2d bb=planning_math::Box2d(pp,0,2,3);
        GridObsManager manager(osd);

        Box2d planning_space = osd.map_boundary;
        std::cout<<"length:"<<planning_space.length()<<"\twidth:"<<planning_space.width()<<"\theading:"<<planning_space.heading()<<std::endl;
        std::vector<Vec2d> corners = planning_space.GetAllCorners();
        Vec2d local_point(corners[3].x(), corners[3].y());

        std::vector<LineSegment2d> lines=manager.getRefLine(osd);
        // plt::figure();
        plt::figure_size(1300,900);
        plotOSD(osd);
        // plotPoint({start_point},2.0, {{"color","r"},{"marker", "star"}});
        // plotPoint({local_point},2.0, {{"color","r"},{"marker", "circle"}});
        plotPoint({start_point, target_point},5.0, {{"color","r"}});
        plotPoint({local_point},10.0, {{"color","r"}});

        for(auto l:lines){
            plotLine(l,"g--");
        }
        plt::show();
    }

}

void initConfig(){
    std::string car_param_file = "/home/ros/catkin_ws/src/maf_planning/resource/"
                            "config/scenario_configs_json/parking/"
                            "vehicle_param.yaml";
    std::string config_file_name = "/home/ros/catkin_ws/src/maf_planning/"
                                "resource/config/scenario_configs_json/"
                                "parking/apa.yaml";

    if (!msquare::HybridAstarConfig::GetInstance()->loadFile(config_file_name)) {
        cout<<"HybridAstarConfig load file failed."<<endl;
    }
    if (!msquare::TrajectoryOptimizerConfig::GetInstance()->loadFile(
            config_file_name)) {
        cout<<"TrajectoryOptimizerConfig load file failed."<<endl;
    }
    if (!msquare::CarParams::GetInstance()->loadFile(car_param_file)) {
        cout<<"CarParams load file failed."<<endl;
    }
    if (!msquare::CarParams::GetInstance()->loadFile4Plan(config_file_name)) {
        cout<<"CarParams load file4Plan failed."<<endl;
    }
    if (!msquare::StrategyParams::GetInstance()->loadFile(config_file_name)) {
        cout<<"StrategyParams load file failed."<<endl;
    }
    if (!msquare::VehicleParam::Instance()->loadFile(car_param_file)) {
        cout<<"VehicleParam load file failed."<<endl;
    }

}


void plotBox(Box2d& box, string color){
    vector<double> sx, sy;
    std::vector<Vec2d> corners=box.GetAllCorners();
    for(auto& p : corners){
        sx.push_back(p.x());
        sy.push_back(p.y());
    }
    sx.push_back(corners[0].x());
    sy.push_back(corners[0].y());
    plt::plot(sx, sy, color);
}

void plotLine(LineSegment2d& line, string color){
    vector<double> sx, sy;
    
    sx.push_back(line.start().x());
    sy.push_back(line.start().y());
    sx.push_back(line.end().x());
    sy.push_back(line.end().y());
    plt::plot(sx, sy, color);
}

void plotPoint(const std::vector<Vec2d>& ps, double scale=1.0,  const std::map<std::string, std::string>& keywords = {}){
    vector<double> sx, sy;
    
    for(auto&p: ps){
        sx.push_back(p.x());
        sy.push_back(p.y());
    }
    plt::scatter(sx, sy,scale, keywords);
}

void plotOSD(OpenspaceDeciderOutput& osd){
    plotBox(osd.map_boundary, "b");
    for(auto& b : osd.obstacle_boxs){
        plotBox(b, "g");
    }

    for(auto& l : osd.obstacle_lines){
        plotLine(l, "r");
    }

    for(auto& l : osd.lines){
        plotLine(l, "r--");
    }

    plotPoint(osd.points,1.0, {{"color","r"}});
}

void plotTrajectory(std::vector<TrajectoryPoint>& traj, string color){
    std::vector<double > sx, sy;
    for(auto& t: traj){
        sx.push_back(t.path_point.x);
        sy.push_back(t.path_point.y);
        plotCar(t, color);
    }

    // path
    plt::plot(sx, sy, "g");

}

void plotCar(TrajectoryPoint& p, string color){
  double  car_x = p.path_point.x;
  double  car_y = p.path_point.y;
  double car_theta = p.path_point.theta;
  double car_front_length = msquare::VehicleParam::Instance()->front_edge_to_center;
  double car_back_length = msquare::VehicleParam::Instance()->back_edge_to_center;
  double car_width = msquare::VehicleParam::Instance()->width;
  
  std::vector<double > sx, sy;
  std::vector<double> ss={
        car_front_length, car_width / 2,
        -car_back_length, car_width / 2,
        -car_back_length, -car_width / 2,
        car_front_length, -car_width / 2,
        car_front_length, car_width / 2
  };
  
  for(int i=0;i<5;i++){
      sx.push_back(ss[2*i+0] * cos(car_theta) - ss[2*i+1] * sin(car_theta) + car_x);
      sy.push_back(ss[2*i+0] * sin(car_theta) + ss[2*i+1] * cos(car_theta) + car_y);
  }

  plt::plot(sx, sy, color);
}

bool isDirectory(string path){
    struct stat s;
    if ( stat ( path.c_str(), &s ) == 0 ) {
        if ( s.st_mode & S_IFDIR ) {
            return true;
        }
    }
    return false;
        // ( s.st_mode & S_IFREG ) 

}
void GetFileNames(string path,vector<string>& filenames)
{
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str()))){
        cout<<"Folder doesn't Exist!"<<endl;
        return;
    }
    while((ptr = readdir(pDir))!=0) {
            if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
                filenames.push_back(path + "/" + ptr->d_name);
        }
    }
    closedir(pDir);
}

#endif
