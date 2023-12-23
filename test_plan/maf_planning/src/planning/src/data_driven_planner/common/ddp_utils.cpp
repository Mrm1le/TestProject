#include "data_driven_planner/common/ddp_utils.h"

#include <array>
#include <iomanip>
#include <iostream>

#include "common/math/math_utils.h"
#include "common/mdebug_context.h"
#include "data_driven_planner/common/ddp_context.h"
#include "data_driven_planner/common/ddp_debug_logger.h"
// #include "data_driven_planner/common/ego_state.h"
// #include "data_driven_planner/common/reference_path.h"
#include "mjson/mjson.hpp"

namespace msquare {
namespace ddp {
double interpolate(double x1, double y1, double x2, double y2, double x) {
  auto ratio = (x - x1) / (x2 - x1);
  auto y = (1 - ratio) * y1 + ratio * y2;
  return y;
}

double interpolate(double y1, double y2, double ratio) {
  auto y = (1 - ratio) * y1 + ratio * y2;
  return y;
}

double interpolate_angle(double x1, double y1, double x2, double y2, double x) {
  auto ratio = (x - x1) / (x2 - x1);
  auto y = interpolate_angle(y1, y2, ratio);
  return y;
}

double interpolate_angle(double y1, double y2, double ratio) {
  using namespace planning_math;
  y1 = NormalizeAngle(y1);
  y2 = NormalizeAngle(y2);

  if (y1 - y2 > M_PI) {
    y2 += M_PI * 2;
  } else if (y2 - y1 > M_PI) {
    y1 += M_PI * 2;
  }

  auto y = interpolate(y1, y2, ratio);
  y = NormalizeAngle(y);

  return y;
}

// void debug_info(const std::shared_ptr<ReferencePath> reference_path) {
//   auto &points = reference_path->get_points();
//   auto ego_s = reference_path->get_frenet_ego_state().s();
//   for (auto &pt : points) {
//     if ((ego_s - 50) <= pt.frenet_point.x and
//         pt.frenet_point.x <= (ego_s + 100)) {
//       MDEBUG_JSON_BEGIN_OBJECT(object)
//       MDEBUG_JSON_ADD_ITEM(x, pt.enu_point.x, object)
//       MDEBUG_JSON_ADD_ITEM(y, pt.enu_point.y, object)
//       MDEBUG_JSON_ADD_ITEM(z, pt.enu_point.z, object)
//       MDEBUG_JSON_END_OBJECT(object)
//     }
//   }
// }

void debug_info(const TrajectoryPoints &trajectory_points) {
  for (auto &pt : trajectory_points) {
    MDEBUG_JSON_BEGIN_OBJECT(object)
    MDEBUG_JSON_ADD_ITEM(x, pt.x, object)
    MDEBUG_JSON_ADD_ITEM(y, pt.y, object)
    MDEBUG_JSON_END_OBJECT(object)
  }
}

static bool toggle_on_from_env(const std::string &env_var_name) {
  bool result = true;
  auto env_var_value = std::getenv(env_var_name.c_str());
  if (env_var_value != nullptr) {
    result = std::stoi(env_var_value) == 1;
  }
  return result;
}
static std::tuple<int, int, int> get_rgb_from_env(std::string const &id, int r,
                                                  int g, int b) {
  const char *env_r = std::getenv((id + "_r").c_str());
  const char *env_g = std::getenv((id + "_g").c_str());
  const char *env_b = std::getenv((id + "_b").c_str());

  if (env_r != nullptr) {
    r = std::atoi(env_r);
  }
  if (env_g != nullptr) {
    g = std::atoi(env_g);
  }
  if (env_b != nullptr) {
    b = std::atoi(env_b);
  }

  return std::make_tuple(r, g, b);
}
static std::tuple<int, int, int, double>
get_rgb_width_from_env(std::string const &id, int r, int g, int b,
                       double width) {
  const char *env_width = std::getenv((id + "_width").c_str());
  if (env_width != nullptr) {
    width = std::atof(env_width);
  }

  std::tie(r, g, b) = get_rgb_from_env(id, r, g, b);

  return std::make_tuple(r, g, b, width);
}
static std::tuple<int, int, int, double, double>
get_rgb_radius_width_from_env(std::string const &id, int r, int g, int b,
                              double radius, double width) {
  const char *env_radius = std::getenv((id + "_radius").c_str());
  if (env_radius != nullptr) {
    radius = std::atof(env_radius);
  }

  std::tie(r, g, b, width) = get_rgb_width_from_env(id, r, g, b, width);

  return std::make_tuple(r, g, b, radius, width);
}

static const std::string ddp_utils_category = "ddp_trajectory";
static constexpr size_t segment_num = 4U;
static constexpr std::array<int, segment_num> mdebug_alphas = {255, 191, 95,
                                                               47};
// TODO: 应该在mdebug_context中提供易用的接口
static void mdebug_traj_helper(const TrajectoryPoints &trajectory_points,
                               double z, int r, int g, int b, double width,
                               double radius, const std::string &id,
                               const MdebugContext::ENTRY_TYPE type,
                               const std::vector<std::string> &tags) {
  if (trajectory_points.size() == 0) {
    return;
  }
  if ((type != MdebugContext::ENTRY_TYPE::POLYLINE) &&
      (type != MdebugContext::ENTRY_TYPE::POINT)) {
    return;
  }
  if (!toggle_on_from_env(id)) {
    return;
  }

  auto tags_json = mjson::Json::array{};
  for (auto &tag : tags) {
    tags_json.emplace_back(tag);
  }

  std::tie(r, g, b, width) = get_rgb_width_from_env(id, r, g, b, width);
  // 数据分段，通过alpha通道控制透明度来区分不同轨迹段
  size_t segment_length = (trajectory_points.size() - 1U) / segment_num;
  for (size_t i = 0U; i < segment_num; ++i) {
    mjson::Json data_arr = mjson::Json(mjson::Json::array());
    if (i == 0U) {
      // 第一个点特殊处理
      data_arr.array_value().emplace_back(mjson::Json::array{
          mjson::Json(trajectory_points[0].x),
          mjson::Json(trajectory_points[0].y), mjson::Json(z)});
    }
    for (size_t j = segment_length * i + 1;
         j <= segment_length * (i + 1U) && j < trajectory_points.size(); ++j) {
      auto &pt = trajectory_points[j];
      data_arr.array_value().emplace_back(mjson::Json::array{
          mjson::Json(pt.x), mjson::Json(pt.y), mjson::Json(z)});
    }

    // mdebug data部分
    mjson::Json mdebug_data = mjson::Json(mjson::Json::object());
    mdebug_data["tags"] = mjson::Json(tags_json);
    if (type == MdebugContext::ENTRY_TYPE::POLYLINE) {
      mdebug_data["polyline"] = data_arr;
    } else if (type == MdebugContext::ENTRY_TYPE::POINT) {
      mdebug_data["point"] = data_arr;
    }

    // mdebug style部分
    mdebug_data["overWriteStyle"] = mjson::Json(mjson::Json::object());
    auto color = mjson::Json(mjson::Json::array{mjson::Json(r), mjson::Json(g),
                                                mjson::Json(b),
                                                mjson::Json(mdebug_alphas[1])});
    mdebug_data["overWriteStyle"]["stroke_color"] = color;
    mdebug_data["overWriteStyle"]["stroke_width"] = mjson::Json(width);
    mdebug_data["id"] = mjson::Json(id);

    // mdebug entry部分
    mjson::Json mdebug_entry = mjson::Json(mjson::Json::object());
    mdebug_entry["data"] = mdebug_data;
    mdebug_entry["type"] = mjson::Json(static_cast<int>(type));
    mdebug_entry["coordinate"] =
        mjson::Json(static_cast<int>(MdebugContext::COORDINATE::BOOT));
    mdebug_entry["category"] = mjson::Json(ddp_utils_category);
    MdebugContext::Instance()->add_entry(mdebug_entry);
  }
}

void dump_mdebug_planning_failed_msg(const std::string &planning_failed_msg) {
  mjson::Json text = mjson::Json(mjson::Json::object());
  constexpr int ENTRY_TYPE_TEXT = 5;
  constexpr int COORDINATE_EGO = 1;
  constexpr int CAMERA_SOURCE_DEFAULT = 0;
  text["type"] = mjson::Json(static_cast<int>(ENTRY_TYPE_TEXT));
  text["coordinate"] = mjson::Json(static_cast<int>(COORDINATE_EGO));
  text["camera_source"] = mjson::Json(static_cast<int>(CAMERA_SOURCE_DEFAULT));
  text["category"] = mjson::Json("np_planning");

  mjson::Json yellow = mjson::Json(
      mjson::Json::array{mjson::Json(255), mjson::Json(255), mjson::Json(0)});
  mjson::Json over_write_style = mjson::Json(mjson::Json::object());
  over_write_style["text_size"] = 18;
  over_write_style["fill_color"] = yellow;
  over_write_style["text_anchor"] = mjson::Json("MIDDLE");
  over_write_style["text_baseline"] = mjson::Json("CENTER");
  mjson::Json text_data = mjson::Json(mjson::Json::object());
  text_data["overWriteStyle"] = over_write_style;
  text_data["id"] = mjson::Json("planning_failed_msg");
  text_data["position"] = mjson::Json(
      mjson::Json::array{mjson::Json(30), mjson::Json(0), mjson::Json(0)});
  text_data["text"] = mjson::Json(planning_failed_msg);
  text["data"] = text_data;
  MdebugContext::Instance()->add_entry(text);
}

void dump_mdebug_polyline(const TrajectoryPoints &trajectory_points, double z,
                          int r, int g, int b, double width,
                          const std::string &id,
                          const std::vector<std::string> &tags) {
  mdebug_traj_helper(trajectory_points, z, r, g, b, width, 0, id,
                     MdebugContext::ENTRY_TYPE::POLYLINE, tags);
}

void dump_mdebug_point(const TrajectoryPoints &trajectory_points, double z,
                       int r, int g, int b, double width,
                       const std::string &id) {
  mdebug_traj_helper(trajectory_points, z, r, g, b, width, 0, id,
                     MdebugContext::ENTRY_TYPE::POINT, {});
}

static void mdebug_circle_helper(const TrajectoryPoint &pt, const double z,
                                 const int r, const int g, const int b,
                                 const int alpha, const double radius,
                                 const double width, const std::string &id) {
  mjson::Json circle_data = mjson::Json(mjson::Json::object());
  circle_data["position"] =
      mjson::Json::array{mjson::Json(pt.x), mjson::Json(pt.y), mjson::Json(z)};
  circle_data["radius"] = mjson::Json(radius);
  circle_data["overWriteStyle"] = mjson::Json(mjson::Json::object());
  auto color = mjson::Json(mjson::Json::array{
      mjson::Json(r), mjson::Json(g), mjson::Json(b), mjson::Json(alpha)});
  circle_data["overWriteStyle"]["fill_color"] = color;
  circle_data["overWriteStyle"]["filled"] = mjson::Json(true);
  circle_data["overWriteStyle"]["stroke_color"] = color;
  circle_data["overWriteStyle"]["stroked"] = mjson::Json(true);
  circle_data["overWriteStyle"]["stroke_width"] = mjson::Json(width);
  circle_data["id"] = mjson::Json(id);

  mjson::Json circle_entry = mjson::Json(mjson::Json::object());
  circle_entry["data"] = circle_data;
  circle_entry["type"] =
      mjson::Json(static_cast<int>(MdebugContext::ENTRY_TYPE::CIRCLE));
  circle_entry["coordinate"] =
      mjson::Json(static_cast<int>(MdebugContext::COORDINATE::BOOT));
  circle_entry["category"] = mjson::Json(ddp_utils_category);
  MdebugContext::Instance()->add_entry(circle_entry);
}

void dump_mdebug_circle(const TrajectoryPoints &trajectory_points, double z,
                        int r, int g, int b, double radius, double width,
                        const std::string &id) {
  if (trajectory_points.size() == 0) {
    return;
  }
  if (!toggle_on_from_env(id)) {
    return;
  }

  std::tie(r, g, b, radius, width) =
      get_rgb_radius_width_from_env(id, r, g, b, radius, width);
  // 数据分段，通过alpha通道控制透明度来区分不同轨迹段
  size_t segment_length = (trajectory_points.size() - 1U) / segment_num;
  for (size_t i = 0; i < segment_num; ++i) {
    // 第一个点特殊处理
    if (i == 0) {
      mdebug_circle_helper(trajectory_points[0], z, r, g, b, mdebug_alphas[0],
                           radius, width * 2.5, id);
    }
    for (size_t j = segment_length * i + 1;
         j <= segment_length * (i + 1U) && j < trajectory_points.size(); ++j) {
      auto &pt = trajectory_points[j];
      if (j == segment_length * (i + 1U)) {
        mdebug_circle_helper(pt, z, r, g, b, mdebug_alphas[0], radius,
                             width * 2.5, id);
      } else {
        mdebug_circle_helper(pt, z, r, g, b, mdebug_alphas[1], radius, width,
                             id);
      }
    }
  }
}

// bool compare_obstacle_s_descend(const FrenetObstacle &o1,
//                                 const FrenetObstacle &o2) {
//   return (o1.frenet_s() > o2.frenet_s());
// }

// bool compare_obstacle_s_ascend(const FrenetObstacle &o1,
//                                const FrenetObstacle &o2) {
//   return (o1.frenet_s() < o2.frenet_s());
// }

std::string expand_environment_variable(const std::string &origin_string) {
  std::string converted_string;
  std::string raw_str = origin_string;
  std::smatch match{};
  const std::regex pattern("\\$\\{(\\w+)\\}");

  std::map<std::string, std::string> variable_dict;
  while (std::regex_search(raw_str, match, pattern)) {
    converted_string += match.prefix().str();
    assert(match.size() >= 1);
    const auto environment_variable = match[1].str();

    const auto env = std::getenv(environment_variable.c_str());
    const auto value = env == nullptr ? "" : env;
    variable_dict[environment_variable] = value;
    converted_string += value;
    raw_str = match.suffix().str();
  }
  converted_string += raw_str;

  return converted_string;
}

} // namespace ddp
} // namespace msquare
