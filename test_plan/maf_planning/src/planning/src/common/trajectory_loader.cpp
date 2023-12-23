#include "common/trajectory_loader.h"
#include "common/planning_config.h"

namespace msquare {

TrajectoryLoader::TrajectoryLoader() {}

bool TrajectoryLoader::LoadTrajectory() {
  trajectory.clear();
  int track_id = 0;
  std::string trajectory_file_path =
      PlanningConfig::Instance()->config_files().trajectory_file;
  try {
    std::ifstream trajectory_input(trajectory_file_path);
    std::string trajectory_line;
    if (trajectory_input.fail()) {
      // std::cerr << "[Error]Trajectory Loader: reading trajectory file failed!
      // Path is " << trajectory_file_path << '\n';
      return false;
    }
    while (getline(trajectory_input, trajectory_line)) {
      std::istringstream trajectory_point_data(trajectory_line);
      RefTrajectoryPoint trajectory_point;
      trajectory_point_data >> trajectory_point.timestamp;
      trajectory_point_data >> trajectory_point.quaternion.x;
      trajectory_point_data >> trajectory_point.quaternion.y;
      trajectory_point_data >> trajectory_point.quaternion.z;
      trajectory_point_data >> trajectory_point.quaternion.w;
      trajectory_point_data >> trajectory_point.x;
      trajectory_point_data >> trajectory_point.y;
      trajectory_point_data >> trajectory_point.z;
      trajectory_point_data >> trajectory_point.yaw;
      trajectory.emplace_back(trajectory_point);
    }
    // for (auto point : trajectory ) {
    //   std::cout << "Point: "<<std::endl;
    //   std::cout << point.timestamp <<std::endl;
    //   std::cout << point.quaternion.x <<std::endl;
    //   std::cout << point.quaternion.y <<std::endl;
    //   std::cout << point.quaternion.z <<std::endl;
    //   std::cout << point.quaternion.w <<std::endl;
    //   std::cout << point.x <<std::endl;
    //   std::cout << point.y <<std::endl;
    //   std::cout << point.z <<std::endl;
    //   std::cout << point.yaw <<std::endl;
    // }
  } catch (const std::exception &e) {
    // std::cerr << "[Error]Trajectory Loader: " << e.what() << '\n';
    return false;
  }
  return true;
}
} // namespace msquare
