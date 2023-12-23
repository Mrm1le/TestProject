#ifndef __UTILS__POSE2D_UTILS_HPP__
#define __UTILS__POSE2D_UTILS_HPP__

#include <string>
#include <time.h>
#include <tuple>
#include <vector>

#include "common/utils/cartesian_coordinate_system.h"

using namespace std;

template <class T> typename std::underlying_type<T>::type enumToInteger(T t) {
  return static_cast<typename std::underlying_type<T>::type>(t);
}

// dis
double get_dis(const Pose2D &p1, const Pose2D &p2);

double get_dis(const Pose2D &p0, const Pose2D &p1, const Pose2D &p2);
double get_dis(const vector<Pose2D> &pose_array);
double get_dis(const vector<Pose2D> &pose_array, int start_ind, int end_ind);
double get_dis(const vector<Pose2D> &pose_array1,
               const vector<Pose2D> &pose_array2);
double get_dis(const Pose2D &pose, const vector<Pose2D> &pose_array);
double get_dis_from(const vector<Pose2D> &pose_array, int from);
double get_dis_to(const vector<Pose2D> &pose_array1,
                  const vector<Pose2D> &pose_array2, double to);

int get_nearest_index(const Pose2D &pose, const vector<Pose2D> &pose_array);
int get_nearest_index(const Pose2D &pose, const vector<Pose2D> &pose_array,
                      int from, int to);
tuple<int, int> get_nearest_index2(const Pose2D &pose,
                                   const vector<Pose2D> &pose_array);

double get_sig_dis(const Pose2D &p1, const Pose2D &p2);
double get_sig_dis(const Pose2D &pose, const vector<Pose2D> &pose_array);
double get_sig_dis(const vector<Pose2D> &pose_array1,
                   const vector<Pose2D> &pose_array2);
double get_sig_dis_to(const vector<Pose2D> &pose_array1,
                      const vector<Pose2D> &pose_array2, double to);

Point2D convert_pose2point(const Pose2D &pose);

// theta
double format_theta(double theta);
double get_theta(const Pose2D &p0, const Pose2D &p1);
bool get_theta(vector<Pose2D> &pose_array);
bool get_theta(const vector<Pose2D> &in_array, vector<Pose2D> &out_array);
double sub_theta(double t1, double t2);
double go2target(double in, double step, double target);
// double go2target(const double &in, const double &step, const double &target,
// double &preview_dis);

// planning
int get_preview_index(const vector<Pose2D> &pose_array, const Pose2D &pose,
                      double preview_dis);
int get_preview_index_4planning(const vector<Pose2D> &pose_array,
                                const Pose2D &pose, double preview_dis);
void shift_pose(Pose2D &pose, double w);
void shift_pose_array(vector<Pose2D> &pose_array, double w);
void forward_pose(Pose2D &pose, double l);
bool sim_pose_array(const vector<Pose2D> &in_array, vector<Pose2D> &out_array,
                    const Pose2D &car_pose, double car_vel);

// decision
bool concat(double dis1, double dis2, const vector<Pose2D> &pose_array1,
            const vector<Pose2D> &pose_array2, vector<Pose2D> &pose_array3,
            const Pose2D &car_pose);

// controller
/* interpolation when x is a scalor,
TODO: interpolation for both vector and scalor.
orignaly it is for both vector and scalor,
but not that convinient for C++, function overloading is required*/
double interp(double x, const vector<double> &xp, const vector<double> &fp);
double interp(double x1, double x2, const vector<double> &xp1,
              const vector<double> &xp2, const vector<double> &fp);

template <class T>
T interpolation(T x, const vector<T> &xp, const vector<T> &fp) {
  const size_t N = xp.size();
  size_t hi;
  for (hi = 0; hi < N && x > xp[hi]; hi++)
    ;

  if (hi == 0)
    return fp[0];

  if (hi == N && x > xp[N - 1])
    return fp[N - 1];

  const size_t low = hi - 1;
  const T xp_diff = xp[hi] - xp[low];
  if (xp_diff < static_cast<T>(1e-5f) && xp_diff > static_cast<T>(-1e-5f))
    return fp[low];

  return (x - xp[low]) * (fp[hi] - fp[low]) / xp_diff + fp[low];
}

template <class T> T clip(T x, T max_x, T min_x) {
  if (x > max_x)
    return max_x;
  if (x < min_x)
    return min_x;
  return x;
}

// ADRC public tools
template <typename T> int sign(T val);

double fhan(double x1, double x2, double r, double h);
double fal(double error, double alpha, double delta);

#endif
