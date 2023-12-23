#include "common/mdebug_context.h"

namespace msquare {

static const std::string mdebug_entry_category = "np_planning";

void MdebugContext::begin_entry(const ENTRY_TYPE type,
                                const COORDINATE coordinate,
                                const CAMERA_SOURCE camera_source,
                                const std::string &category) {
  auto entry = mjson::Json(mjson::Json::object());
  entry["type"] = mjson::Json(static_cast<int>(type));
  entry["coordinate"] = mjson::Json(static_cast<int>(coordinate));
  entry["camera_source"] = mjson::Json(static_cast<int>(camera_source));
  entry["category"] = mjson::Json(category);
  entry["data"] = mjson::Json(mjson::Json::object());
  mdebug_log_json_.array_value().emplace_back(std::move(entry));
  current_node_ =
      std::make_unique<Node>(type, mdebug_log_json_.array_value().back());
}

void MdebugContext::add_json_entry(mjson::Json const &json_object) {
  begin_entry(ENTRY_TYPE::JSON, COORDINATE::BOOT, CAMERA_SOURCE::DEFAULT,
              mdebug_entry_category);
  mdebug_log_json_.array_value().back()["data"] = json_object;
}

/**
 * # type = TABLE
 * "data": {
 *   "table_name": string, //unique for each table
 *   "table_description": string,
 *   "columns": [{
 *     "name": string,
 *     "type": string, // string | number | boolean
 *     "id": number // unused currently
 *   }],
 *   "rows": [{
 *     "data": []
 *   }]
 * }
 */

void MdebugContext::begin_table_entry(const std::string &table_name) {
  if (table_name.empty()) {
    return;
  }

  std::string table_name_with_num = table_name;

  if (table_cnt_map_.find(table_name) == table_cnt_map_.end()) {
    table_cnt_map_[table_name] = 1U;
  } else {
    table_cnt_map_[table_name] += 1U;
    table_name_with_num += std::to_string(table_cnt_map_[table_name]);
  }

  begin_entry(ENTRY_TYPE::TABLE, COORDINATE::BOOT, CAMERA_SOURCE::DEFAULT,
              mdebug_entry_category);

  auto &table_data = current_node_->json_node()["data"];

  table_data["table_name"] = mjson::Json(table_name_with_num);
  table_data["description"] = mjson::Json("");
  table_data["columns"] = mjson::Json(mjson::Json::array());
  table_data["rows"] = mjson::Json(mjson::Json::array());
}

void MdebugContext::add_table_column(const std::string &col_name,
                                     const std::string &col_type) {
  if (current_node_.get() == nullptr ||
      current_node_->type() != ENTRY_TYPE::TABLE) {
    return;
  }

  auto column = mjson::Json(mjson::Json::object());
  column["id"] = mjson::Json(current_node_->table_column_id());
  column["name"] = mjson::Json(col_name);
  column["type"] = mjson::Json(col_type);

  auto &table_data = current_node_->json_node()["data"];
  table_data["columns"].array_value().emplace_back(std::move(column));
  ++current_node_->multable_table_column_id();
}

/**
 * rows: [{ data: ['abc', 10, true] }, { data: ['def', 0.5, true] }, ...]
 */
void MdebugContext::begin_table_row() {
  if (current_node_.get() == nullptr ||
      current_node_->type() != ENTRY_TYPE::TABLE) {
    return;
  }
  auto row_data = mjson::Json(mjson::Json::object());
  row_data["data"] = mjson::Json(mjson::Json::array());
  row_data["data"].array_value().reserve(current_node_->table_column_id());

  auto &table_data = current_node_->json_node()["data"];
  table_data["rows"].array_value().emplace_back(std::move(row_data));
}

void MdebugContext::add_table_row_data(const double data) {
  if (current_node_.get() == nullptr ||
      current_node_->type() != ENTRY_TYPE::TABLE) {
    return;
  }
  auto &table_data = current_node_->json_node()["data"];
  table_data["rows"].array_value().back()["data"].array_value().emplace_back(
      data);
}

void MdebugContext::add_table_row_data(const std::string &data) {
  if (current_node_.get() == nullptr ||
      current_node_->type() != ENTRY_TYPE::TABLE) {
    return;
  }
  auto &table_data = current_node_->json_node()["data"];
  table_data["rows"].array_value().back()["data"].array_value().emplace_back(
      data);
}

void MdebugContext::add_ego_info_entry(const double x, const double y,
                                       const double z) {
  const std::string id = "ego_info";

  auto read_env = [&id](const std::string &s, const double v) -> double {
    const char *env_v = std::getenv((id + "_" + s).c_str());
    if (env_v != nullptr) {
      return std::atof(env_v);
    }
    return v;
  };

  const int r = static_cast<int>(read_env("r", 150));
  const int g = static_cast<int>(read_env("g", 150));
  const int b = static_cast<int>(read_env("b", 150));
  const int alpha = static_cast<int>(read_env("alpha", 255));
  const double radius = read_env("radius", 0.2);
  const double width = read_env("width", 0.3);

  mjson::Json circle_data = mjson::Json(mjson::Json::object());
  circle_data["position"] =
      mjson::Json::array{mjson::Json(x), mjson::Json(y), mjson::Json(z)};
  circle_data["radius"] = mjson::Json(radius);
  circle_data["overWriteStyle"] = mjson::Json(mjson::Json::object());
  auto color = mjson::Json(mjson::Json::array{
      mjson::Json(r), mjson::Json(g), mjson::Json(b), mjson::Json(alpha)});
  circle_data["overWriteStyle"]["fill_color"] = color;
  circle_data["overWriteStyle"]["filled"] = mjson::Json(true);
  circle_data["overWriteStyle"]["stroke_color"] = color;
  circle_data["overWriteStyle"]["stroked"] = mjson::Json(true);
  circle_data["overWriteStyle"]["stroke_width"] = mjson::Json(width);
  circle_data["id"] = mjson::Json("ego_info");
  auto tags_json = mjson::Json::array{};
  for (auto &info : ego_infos_) {
    tags_json.emplace_back(info);
  }
  circle_data["tags"] = tags_json;

  mjson::Json circle_entry = mjson::Json(mjson::Json::object());
  circle_entry["data"] = circle_data;
  circle_entry["type"] =
      mjson::Json(static_cast<int>(MdebugContext::ENTRY_TYPE::CIRCLE));
  circle_entry["coordinate"] =
      mjson::Json(static_cast<int>(MdebugContext::COORDINATE::BOOT));
  circle_entry["category"] = mjson::Json(mdebug_entry_category);
  MdebugContext::Instance()->add_entry(circle_entry);
  ego_infos_.clear();
}

} // namespace msquare