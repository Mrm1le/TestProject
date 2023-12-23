#include "argparse.hpp"
#include "msd/planning.h"
#include "pnc/planning_engine_interface.h"

#include <boost/program_options.hpp>

using namespace msd_planning;
using namespace msquare;

namespace {
inline std::string read_file(std::string path) {
  std::shared_ptr<FILE> fp(fopen(path.c_str(), "r"),
                           [](FILE *file) { fclose(file); });
  if (fp == nullptr)
    return "";
  fseek(fp.get(), 0, SEEK_END);
  std::vector<char> content(ftell(fp.get()));
  fseek(fp.get(), 0, SEEK_SET);
  auto read_bytes = fread(content.data(), 1, content.size(), fp.get());
  assert(read_bytes == content.size());
  return std::string(content.begin(), content.end());
}

} // namespace

int main(int argc, char *argv[]) {
  argparse::ArgumentParser argument_parser{};
  argument_parser.short_name("-c")
      .long_name("--config")
      .default_value("../resource/config/msquare_planning.json")
      .done();
  auto args = argument_parser.parse_args_any_type(argc, argv);

  std::string json_data = read_file(args.get_value<std::string>("config"));
  MSDPlanning_set_log_config(json_data);

  //  SceneType scene_type = SceneType::URBAN;
  //
  //  // TODO:
  //  auto planning_engine = PlanningEngineInterface::make(scene_type);
}
