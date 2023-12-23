import os
import subprocess
import ctypes.util

from cffi import FFI

lib_hobca_planner_dir = os.path.dirname(os.path.abspath(__file__))
lib_hobca_planner_fn = os.path.join(lib_hobca_planner_dir, "../libopenspace_optimizer.so")
# subprocess.check_output(["make", "-j4"], cwd=lib_hobca_planner_dir)

ffi = FFI()
ffi.cdef("""
typedef struct
{
  double x_;
  double y_;
  double theta_;
  double s_;
  double kappa_;
} PathPoint;

typedef struct
{
  PathPoint path_point_;
  double v_;
  double a_;
  double da_; // jerk
  double steer_;
  double relative_time_;
  double wheel_base_offset_;
} TrajectoryPoint;

typedef struct
{
  float x;
  float y;
} Point2d;

typedef struct
{
  float x;
  float y;
  float theta;
} Pose2d;

typedef struct
{
  Point2d point[2];
} Line2d;

typedef struct
{
  Point2d center;
  float heading;
  float length;
  float width;
} Box2d;

int construct_planner(const char* config_file_name, const char *arg_stra_odca_file, const char *car_param_file, TrajectoryPoint target_state, Box2d map_boundary);
int destruct_planner(void);
double plan(const char *arg_stra_odca_file);
int get_result_size();
void get_result(TrajectoryPoint *trajectory, int *num_segments, int *iteration_times, char *debug_string);
int get_search_size();
void get_search_progress(Pose2d  *search_points);
""")

lib_hobca_planner = ffi.dlopen(lib_hobca_planner_fn, flags=256) # RTLD_GLOBAL = 256
