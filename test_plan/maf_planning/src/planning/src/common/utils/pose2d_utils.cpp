#include "common/utils/pose2d_utils.hpp"
#include "common/utils/utils_math.hpp"

#include <limits>

template <class T> T dist_squared(T x1, T y1, T z1, T x2, T y2, T z2) {
  const T dx = x2 - x1, dy = y2 - y1, dz = z2 - z1;
  return dx * dx + dy * dy + dz * dz;
}

template <class T> T dist_squared(T x1, T y1, T x2, T y2) {
  const T dx = x2 - x1, dy = y2 - y1;
  return dx * dx + dy * dy;
}

template <class T> T dist(T x1, T y1, T z1, T x2, T y2, T z2) {
  return std::sqrt(dist_squared(x1, y1, z1, x2, y2, z2));
}

template <class T> T dist(T x1, T y1, T x2, T y2) {
  return std::sqrt(dist_squared(x1, y1, x2, y2));
}

double get_dis(const Pose2D &p1, const Pose2D &p2) {
  return dist(p1.x, p1.y, p2.x, p2.y);
}

double get_dis(const Pose2D &p0, const Pose2D &p1, const Pose2D &p2) {
  const double A = p1.y - p2.y;
  const double B = p2.x - p1.x;
  const double C = -A * p1.x - B * p1.y;
  return (A * p0.x + B * p0.y + C) / sqrt(A * A + B * B);
}

double get_dis(const vector<Pose2D> &pose_array) {
  double sum_dis = 0.0f;
  for (size_t i = 1; i < pose_array.size(); i++) {
    sum_dis += get_dis(pose_array[i - 1], pose_array[i]);
  }
  return sum_dis;
}

double get_dis_from(const vector<Pose2D> &pose_array, int from) {
  double sum_dis = 0.0f;
  for (size_t i = (size_t)max(1, from); i < pose_array.size(); i++) {
    sum_dis += get_dis(pose_array[i - 1], pose_array[i]);
  }
  return sum_dis;
}

double get_dis(const Pose2D &pose, const vector<Pose2D> &pose_array) {
  int index = get_nearest_index(pose, pose_array);
  return get_dis(pose, pose_array[index]);
}

double get_dis(const vector<Pose2D> &pose_array1,
               const vector<Pose2D> &pose_array2) {
  if (pose_array1.size() == 0 || pose_array2.size() == 0) {
    return (numeric_limits<double>::max)();
  }
  double sum_dis = 0.0f;
  for (size_t i = 0; i < pose_array1.size(); i++) {
    sum_dis += get_dis(pose_array1[i], pose_array2);
  }
  return sum_dis / pose_array1.size();
}

double get_dis(const vector<Pose2D> &pose_array, int start_ind, int end_ind) {
  double sum_dis = 0.0f;
  for (int i = max(start_ind + 1, 1);
       i <= (min(end_ind, (int)pose_array.size() - 1)); i++) {
    sum_dis += get_dis(pose_array[i - 1], pose_array[i]);
  }
  return sum_dis;
}

int get_nearest_index(const Pose2D &pose, const vector<Pose2D> &pose_array) {
  int index = -1;
  double min_dis = (numeric_limits<double>::max)();
  for (size_t i = 0; i < pose_array.size(); i++) {
    double dis = get_dis(pose, pose_array[i]);
    if (dis < min_dis) {
      min_dis = dis;
      index = i;
    }
  }
  return index;
}

int get_nearest_index(const Pose2D &pose, const vector<Pose2D> &pose_array,
                      int from, int to) {
  int index = -1;
  double min_dis = (numeric_limits<double>::max)();
  for (size_t i = max(0, from); i < min(pose_array.size(), (size_t)to); i++) {
    double dis = get_dis(pose, pose_array[i]);
    if (dis < min_dis) {
      min_dis = dis;
      index = i;
    }
  }
  return index;
}

tuple<int, int> get_nearest_index2(const Pose2D &pose,
                                   const vector<Pose2D> &pose_array) {
  int index = get_nearest_index(pose, pose_array);
  double dist1, dist2;
  int index2;

  if (0 < index && index < (int)pose_array.size() - 1) {
    dist1 = get_dis(pose, pose_array[index - 1]);
    dist2 = get_dis(pose, pose_array[index + 1]);

    /*if (dist1 < dist2) {
        index2 = index - 1;
    } else {
        index2 = index + 1;
    }*/
    index2 = dist1 < dist2 ? index - 1 : index + 1;
  } else if (index == 0) {
    index2 = 1;
  } else {
    index2 = pose_array.size() - 2;
  }
  return make_tuple(index, index2);
}

double get_dis_to(const vector<Pose2D> &pose_array1,
                  const vector<Pose2D> &pose_array2, double to) {
  if (pose_array1.size() == 0 || pose_array2.size() == 0) {
    return (numeric_limits<double>::max)();
  }
  double sum = 0.0f;
  double sum_dis = 0.0f;
  int cnt = 0;
  for (size_t i = 0; i < pose_array1.size() && sum_dis < to; i++) {
    if (i > 0) {
      sum_dis += get_dis(pose_array1[i - 1], pose_array1[i]);
    }
    sum += get_dis(pose_array1[i], pose_array2);
    cnt++;
  }
  if (cnt == 0) {
    return (numeric_limits<double>::max)();
  }
  return sum / cnt;
}

double get_sig_dis(const Pose2D &p1, const Pose2D &p2) {
  const double theta = p2.theta;
  double dist = cos(theta) * (p1.x - p2.x) + sin(theta) * (p2.y - p1.y);
  return dist;
}

double get_sig_dis(const Pose2D &pose, const vector<Pose2D> &pose_array) {
  if (pose_array.size() == 0) {
    return (numeric_limits<double>::max)();
  }
  const int index = get_nearest_index(pose, pose_array);
  return get_sig_dis(pose, pose_array[index]);
}

double get_sig_dis(const vector<Pose2D> &pose_array1,
                   const vector<Pose2D> &pose_array2) {
  if (pose_array1.size() == 0 || pose_array2.size() == 0) {
    return (numeric_limits<double>::max)();
  }
  double sum = 0;
  for (size_t i = 0; i < pose_array1.size(); i++) {
    sum += fabs(get_sig_dis(pose_array1[i], pose_array2));
  }
  return sum / pose_array1.size();
}

double get_sig_dis_to(const vector<Pose2D> &pose_array1,
                      const vector<Pose2D> &pose_array2, double to) {
  if (pose_array1.size() == 0 || pose_array2.size() == 0) {
    return (numeric_limits<double>::max)();
  }
  double sum = 0.0f;
  double sum_dis = 0.0f;
  int cnt = 0;
  for (size_t i = 0; i < pose_array1.size() && sum_dis < to; i++) {
    if (i > 0) {
      sum_dis += get_dis(pose_array1[i - 1], pose_array1[i]);
    }
    sum += fabs(get_sig_dis(pose_array1[i], pose_array2));
    cnt++;
  }
  if (cnt == 0) {
    return (numeric_limits<double>::max)();
  }
  return sum / cnt;
}

Point2D convert_pose2point(const Pose2D &pose) {
  Point2D pt;
  pt.x = pose.x;
  pt.y = pose.y;
  return pt;
}

bool get_theta(vector<Pose2D> &pose_array) {
  const size_t size = pose_array.size();
  if (size < 7) {
    pose_array.clear();
    return false;
  }

  size_t i = 0;
  double theta = get_theta(pose_array[0], pose_array[6]);
  for (; i < 3; i++) {
    pose_array[i].theta = theta;
  }
  for (; i < size - 3; i++) {
    pose_array[i].theta = get_theta(pose_array[i - 3], pose_array[i + 3]);
  }
  theta = get_theta(pose_array[size - 7], pose_array[size - 1]);
  for (; i < size; i++) {
    pose_array[i].theta = theta;
  }
  return true;
}

bool get_theta(const vector<Pose2D> &in_array, vector<Pose2D> &out_array) {
  out_array.clear();
  const size_t size = in_array.size();
  if (size < 7) {
    return false;
  }

  out_array.resize(size);

  size_t i = 0;
  double theta = get_theta(in_array[0], in_array[6]);
  for (; i < size; i++) {
    out_array[i].x = in_array[i].x;
    out_array[i].y = in_array[i].y;
    out_array[i].theta = theta;
  }
  for (; i < size - 3; i++) {
    out_array[i].x = in_array[i].x;
    out_array[i].y = in_array[i].y;
    out_array[i].theta = get_theta(in_array[i - 3], in_array[i + 3]);
  }
  theta = get_theta(in_array[size - 7], in_array[size - 1]);
  for (; i < size; i++) {
    out_array[i].x = in_array[i].x;
    out_array[i].y = in_array[i].y;
    out_array[i].theta = theta;
  }
  return true;
}

double get_theta(const Pose2D &p0, const Pose2D &p1) {
  double theta = M_PI_2 - atan2(p1.y - p0.y, p1.x - p0.x);
  return format_theta(theta);
}

double format_theta(double theta) {
  double ret = theta;
  while (ret > M_PI) {
    ret -= 2 * M_PI;
  }
  while (ret <= -M_PI) {
    ret += 2 * M_PI;
  }
  return ret;
}

double sub_theta(double t1, double t2) { return format_theta(t1 - t2); }

double go2target(double in, double step, double target) {
  const double dtheta = sub_theta(target, in);
  if (dtheta > 0) {
    if (dtheta < step) {
      return format_theta(in + dtheta);
    } else {
      return format_theta(in + step);
    }
  } else {
    if (dtheta > -step) {
      return format_theta(in + dtheta);
    } else {
      return format_theta(in - step);
    }
  }
}

int get_preview_index(const vector<Pose2D> &pose_array, const Pose2D &pose,
                      double preview_dis) {
  if (pose_array.size() == 0)
    return -1;

  int start_index = get_nearest_index(pose, pose_array);
  double sum_dis = get_dis(pose, pose_array[start_index]);
  for (size_t i = start_index + 1; i < pose_array.size(); i++) {
    if (i > 0) {
      sum_dis += get_dis(pose_array[i - 1], pose_array[i]);
    }
    if (sum_dis >= preview_dis) {
      return i;
    }
  }
  return (int)pose_array.size() - 1;
}

int get_preview_index_4planning(const vector<Pose2D> &pose_array,
                                const Pose2D &pose, double preview_dis) {
  if (pose_array.size() == 0)
    return -1;
  int start_index = get_nearest_index(pose, pose_array);
  double ldis = get_dis(pose, pose_array[start_index]);
  double pdis = preview_dis;
  double sum_dis = 0.0f;

  for (size_t i = start_index + 1; i < pose_array.size(); i++) {
    sum_dis += get_dis(pose_array[i - 1], pose_array[i]);
    // line
    if (sum_dis >= ldis + pdis) {
      return i;
    }
    // curve
    double dtheta = 1.5f / 180.0f * M_PI;
    double dtheta_ = fabs(pose_array[i].theta - pose_array[start_index].theta);
    if (dtheta_ > dtheta && dtheta_ < 2.0f * dtheta) {
      if (sum_dis >= ldis + 0.5f * pdis) {
        return i;
      }
    } else if (dtheta_ > 2.0f * dtheta) {
      if (sum_dis >= ldis + 0.25f * pdis) {
        return i;
      }
    }
  }

  return (int)pose_array.size() - 1;
}

void shift_pose(Pose2D &pose, double w) {
  pose.x += w * cos(pose.theta);
  pose.y -= w * sin(pose.theta);
}

void shift_pose_array(vector<Pose2D> &pose_array, double w) {
  for (size_t i = 0; i < pose_array.size(); i++) {
    shift_pose(pose_array[i], w);
  }
}

void forward_pose(Pose2D &pose, double l) {
  pose.x += l * sin(pose.theta);
  pose.y += l * cos(pose.theta);
}

bool sim_pose_array(const vector<Pose2D> &in_array, vector<Pose2D> &out_array,
                    const Pose2D &car_pose, double car_vel) {
  out_array.clear();
  if (in_array.size() == 0) {
    return false;
  }
  const double theta =
      abs(in_array.back().theta - in_array.front().theta) / 80.0;
  const double dtheta = max(theta, (double)(0.3 / 180 * M_PI));
  const double dl = 0.3f;

  //    const int index = get_nearest_index(car_pose, in_array);
  //    const double in_dis = get_dis_from(in_array, index);
  double sum_dis = 0.0f;
  Pose2D sim_pose = car_pose;
  out_array.push_back(sim_pose);
  while (sum_dis < 80.0f) {
    const double preview_dis = car_vel + 4.0f;
    const int preview_index =
        get_preview_index_4planning(in_array, sim_pose, preview_dis);

    if (preview_index == -1) {
      break;
    }

    if (preview_index == (int)in_array.size() - 1) {
      sim_pose.theta = sim_pose.theta;
      forward_pose(sim_pose, dl);
      out_array.push_back(sim_pose);
      sum_dis += dl;
      break;
    }

    double steer_angle = get_theta(sim_pose, in_array[preview_index]);
    sim_pose.theta = go2target(sim_pose.theta, dtheta, steer_angle);
    forward_pose(sim_pose, dl);
    out_array.push_back(sim_pose);
    sum_dis += dl;
  }
  return true;
}

bool concat(double dis1, double dis2, const vector<Pose2D> &pose_array1,
            const vector<Pose2D> &pose_array2, vector<Pose2D> &pose_array3,
            const Pose2D &car_pose) {
  const int start1 = get_nearest_index(car_pose, pose_array1);
  if (start1 == -1)
    return false;

  const int start2 = get_preview_index(pose_array2, car_pose, dis2);
  if (start2 == -1)
    return false;

  const int end1 = get_preview_index(pose_array1, car_pose, dis1);
  if (end1 == -1)
    return false;

  for (int i = start1; i < end1; i++) {
    pose_array3.push_back(pose_array1[i]);
  }
  Pose2D start = pose_array1[end1];
  Pose2D end = pose_array2[start2];
  const double dis_10 = get_dis(start, end) * 10.0f;
  const double c1 = (end.x - start.x) / dis_10;
  const double c2 = (end.y - start.y) / dis_10;
  for (int i = 0; i < dis_10; i++) {
    Pose2D pose;
    pose.x = start.x + c1 * i;
    pose.y = start.y + c2 * i;
    pose_array3.push_back(pose);
  }
  for (int i = start2; i < (int)pose_array2.size(); i++) {
    pose_array3.push_back(pose_array2[i]);
  }

  return true;
}

/* Look up table, given x in xp, find fp */
double interp(double x, const vector<double> &xp, const vector<double> &fp) {
  const size_t N = xp.size();
  size_t hi;
  for (hi = 0; hi < N && x > xp[hi]; hi++)
    ;

  if (hi == 0)
    return fp[0];

  if (hi == N && x > xp[N - 1])
    return fp[N - 1];

  const size_t low = hi - 1;
  const double xp_diff = xp[hi] - xp[low];
  if (xp_diff < 1e-5f && xp_diff > -1e-5f)
    return fp[low];

  return (x - xp[low]) * (fp[hi] - fp[low]) / xp_diff + fp[low];
}

/* sign function */
template <typename T> int sign(T val) { return (T(0) < val) - (val < T(0)); }
template int sign<double>(double);

/* fhan */
double fhan(double x1, double x2, double r, double h) {
  const double d = r * h * h;
  const double a0 = h * x2;
  const double y = x1 + a0;
  const double a1 = sqrt(d * (d + 8.0f * fabs(y)));
  const double a2 = a0 + sign(y) * (a1 - d) / 2.0f;
  const double sy = (sign(y + d) - sign(y - d)) / 2.0f;
  const double a = (a0 + y - a2) * sy + a2;
  const double sa = (sign(a + d) - sign(a - d)) / 2.0f;
  const double u = -r * (a / d - sign(a)) * sa - r * sign(a);
  return u;
}

/* fal */
double fal(double error, double alpha, double delta) {
  double sed = (sign(error + delta) - sign(error - delta)) / 2.0f;
  double fal_val = error * sed / pow(delta, 1.0f - alpha) +
                   pow(fabs(error), alpha) * sign(error) * (1.0f - sed);
  return fal_val;
}
