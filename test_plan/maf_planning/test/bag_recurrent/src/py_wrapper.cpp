#include <dlfcn.h>

#include <pybind11/chrono.h>
#include <pybind11/complex.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;

typedef void (*type_c_apa_initSingletonParams)(void *car_params_file,
                                               void *config_param_file);

typedef void (*type_c_apa_planInterfaceSerialize)(void *ods_str, void *out);

typedef double (*type_c_apa_getEgoMinObsDistance)(double x, double y,
                                                  double theta, void *odo_str);

static type_c_apa_initSingletonParams c_apa_initSingletonParams = 0;
static type_c_apa_planInterfaceSerialize c_apa_planInterfaceSerialize = 0;
static type_c_apa_getEgoMinObsDistance c_apa_getEgoMinObsDistance = 0;
static void *dl_handler = 0;

static int py_wrapper_init(const char *libpath) {
  printf("--------------------\r");
  dl_handler = dlopen(libpath, RTLD_LAZY);
  if (dl_handler == NULL) {
    printf("ERROR:%s:dlopen\n", dlerror());
    return -1;
  }

  c_apa_initSingletonParams = (type_c_apa_initSingletonParams)dlsym(
      dl_handler, "c_apa_initSingletonParams");
  if (c_apa_initSingletonParams == NULL) {
    printf("ERROR:%s:dlsym\n", dlerror());
    return -1;
  }

  c_apa_planInterfaceSerialize = (type_c_apa_planInterfaceSerialize)dlsym(
      dl_handler, "c_apa_planInterfaceSerialize");
  if (c_apa_planInterfaceSerialize == NULL) {
    printf("ERROR:%s:dlsym\n", dlerror());
    return -1;
  }

  c_apa_getEgoMinObsDistance = (type_c_apa_getEgoMinObsDistance)dlsym(
      dl_handler, "c_apa_getEgoMinObsDistance");
  if (c_apa_getEgoMinObsDistance == NULL) {
    printf("ERROR:%s:dlsym\n", dlerror());
    return -1;
  }

  return 0;
}

static void py_wrapper_uninit() {
  if (dl_handler) {
    dlclose(dl_handler);
  }
}

static void initSingletonParams(const std::string &car_params_file,
                                const std::string &config_param_file) {
  c_apa_initSingletonParams((void *)&car_params_file,
                            (void *)&config_param_file);
}

static std::string planInterfaceSerialize(std::string ods_str) {
  std::string result;
  c_apa_planInterfaceSerialize(&ods_str, &result);
  return result;
}

struct PyWrapperPoint2d {
  double x;
  double y;
  double theta;
  PyWrapperPoint2d() : x(0.0), y(0.0), theta(0.0) {}
  PyWrapperPoint2d(double _x, double _y, double _theta)
      : x(_x), y(_y), theta(_theta) {}
};

static double getEgoMinObsDistance(const PyWrapperPoint2d &ego_pose,
                                   const std::string &odo_str) {
  return c_apa_getEgoMinObsDistance(ego_pose.x, ego_pose.y, ego_pose.theta,
                                    (void *)&odo_str);
}

// python binding module
PYBIND11_MODULE(parking_plotter_cpython_wrapper, m) {
  m.def("py_wrapper_init", &py_wrapper_init);
  m.def("py_wrapper_uninit", &py_wrapper_uninit);
  m.def("initSingletonParams", &initSingletonParams);
  m.def("planInterfaceSerialize", &planInterfaceSerialize);
  m.def("getEgoMinObsDistance", &getEgoMinObsDistance);

  // scenario perf frontier
  py::class_<PyWrapperPoint2d>(m, "Point2d")
      .def(py::init<>())
      .def(py::init<double, double, double>())
      .def_readwrite("x", &PyWrapperPoint2d::x)
      .def_readwrite("y", &PyWrapperPoint2d::y)
      .def_readwrite("theta", &PyWrapperPoint2d::theta);
}

