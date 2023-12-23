#include "diff_between_sides.h"
#include "trend.h"
#include "topsis.h"
#include <string>
#include <yaml-cpp/yaml.h>

// struct Pose_v{
//     Pose2D pose;
//     double vel;
//     Pose_v(Pose2D p, double v): pose(p), vel(v){}
// };

class Eva{
public:
    double evaTraj(const char* file_path_){
        YAML::Node file;
        try{
            //std::string rinima = "/home/ros/Downloads/bak/create_gmb_trajectory/medium_traj/opensapce_decider2020-09-23_12-23-32553939.yaml";
            file = YAML::LoadFile(file_path_);
        }catch(YAML::BadFile &e){
            std::cout<<"read error!"<<std::endl;
            return -1;
        }

        //build trajectory
        int traj_size = file["result"]["points"].size();
        if(traj_size <= 0) {
            std::cout << "The path is empty !"<< std::endl;
            return -1;
        }
        std::vector<msquare::Pose_v> traj;
        for(int i = 0; i < traj_size; ++i){
            Pose2D temp_point(file["result"]["points"][i]["x"].as<double>(), file["result"]["points"][i]["y"].as<double>(), file["result"]["points"][i]["theta"].as<double>());
            msquare::Pose_v tmp_p_v(temp_point, file["result"]["points"][i]["v"].as<double>());
            traj.emplace_back(tmp_p_v);
        }

        std::vector<msquare::planning_math::Vec2d> uss_points;
        int uss_size = file["points"].size();
        for(int i = 0; i < uss_size; ++i){
            msquare::planning_math::Vec2d temp_vec(file["points"][i]["x"].as<double>(), file["points"][i]["y"].as<double>());
            uss_points.emplace_back(temp_vec);
        }

        //build obstacles
        std::vector<msquare::planning_math::LineSegment2d> obstacles;
        for(int i = 0; i < file["obstacle_lines"].size(); ++i){
            msquare::planning_math::Vec2d obs_start(file["obstacle_lines"][i]["start_x"].as<double>(), file["obstacle_lines"][i]["start_y"].as<double>());
            msquare::planning_math::Vec2d obs_end(file["obstacle_lines"][i]["end_x"].as<double>(), file["obstacle_lines"][i]["end_y"].as<double>());
            msquare::planning_math::LineSegment2d temp_obs(obs_start, obs_end);
            obstacles.emplace_back(temp_obs);
        }

        //init DifBetSides, compute diff betweent each side
        msquare::DifBetSides DBS(msquare::VehicleParam::Instance(), traj, obstacles, uss_points);
        double ave_diff = DBS.calculate_diff();
        std::cout << "The average diff of the whole traj is " << ave_diff <<std::endl;
        double diffest_value = DBS.get_diffest_value();
        std::cout << "Diff's worst_value is " << diffest_value <<std::endl;
        double diff_bad_rate = DBS.get_bad_rate();
        std::cout << "Diff's Bad_rate is " << diff_bad_rate <<std::endl;

        std::vector<double> ave_vec {2.01634, 1.15775, 2.36872, 1.7406, 2.54148, 2.43566, 0.0, 4.25091, 4.30496, 4.71923};
        std::vector<double> wor_vec {4.40134, 1.98199, 2.88463, 1.8153, 2.89252, 2.51829, 0.0, 4.70567, 4.75405, 4.76909};
        std::vector<double> rat_vec {0.419355, 0.162791, 0.411765, 0.34, 0.473684, 0.355556, 0.0, 0.0888889, 0.285714, 0.301887};
        ave_vec.emplace_back(ave_diff);
        wor_vec.emplace_back(diffest_value);
        rat_vec.emplace_back(diff_bad_rate);
        Tiny2Great(ave_vec);
        Tiny2Great(wor_vec);
        Tiny2Great(rat_vec);
        std::vector<std::vector<double>> matrix;
        matrix.emplace_back(ave_vec);
        matrix.emplace_back(wor_vec);
        matrix.emplace_back(rat_vec);
        Standard(matrix);
        double Score = Normalized(matrix);
        std::cout << "This trajectory' s points score the " << Score << "points !"<<std::endl;
        return Score;
    }
};

