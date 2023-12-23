/*
 * @Author: your name
 * @Date: 2021-10-21 19:26:44
 * @LastEditTime: 2021-10-21 19:42:17
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /maf_planning/test/openspace_optimizer/planner_wrapper.h
 */
#pragma once

extern "C" {

typedef struct {
  double x_;
  double y_;
  double theta_;
  double s_;
  double kappa_;
} PathPoint;

typedef struct {
  PathPoint path_point_;
  double v_;
  double a_;
  double da_; // jerk
  double steer_;
  double relative_time_;
  double wheel_base_offset_;
} TrajectoryPoint;

typedef struct {
  float x;
  float y;
} Point2d;

typedef struct {
  float x;
  float y;
  float theta;
} Pose2d;

typedef struct {
  Point2d point[2];
} Line2d;

typedef struct {
  Point2d center;
  float heading;
  float length;
  float width;
} Box2d;

int construct_planner(const char *config_file_name, const char *arg_stra_odca_file, const char *car_param_file,
                      TrajectoryPoint target_state, Box2d map_boundary);
int destruct_planner(void);
double plan(const char *arg_stra_odca_file);
int get_result_size();
void get_result(TrajectoryPoint *trajectory, int *num_segments, int *iteration_times, char *debug_string);
int get_search_size();
// int get_edge_size();
void get_search_progress(Pose2d *Search_points);
}