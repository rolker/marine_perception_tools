// Copyright 2026 Roland Arsenault
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <QApplication>

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>

#include "bag_loader.hpp"
#include "main_window.hpp"
#include "resim_engine.hpp"

namespace
{

double arg_double(int argc, char ** argv, const std::string & flag, double def)
{
  for (int i = 1; i + 1 < argc; ++i) {
    if (flag == argv[i]) {return std::atof(argv[i + 1]);}
  }
  return def;
}

}  // namespace

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);

  if (argc < 2 || argv[1][0] == '-') {
    std::fprintf(stderr,
      "usage: %s <bag_uri> [--start-s S] [--end-s S] [--window-m 120] [--res 0.25]\n"
      "          [--max-range 150] [--min-grazing-deg 0]\n"
      "Replays the oak_forward segmentation from <bag_uri> and tunes the\n"
      "segmentation->costmap marking interactively. --start-s/--end-s window the\n"
      "replay (end<0 == to end); the buffer accumulates from --start-s.\n",
      argv[0]);
    return 1;
  }

  marine_perception_tools::BagLoadOptions opts;
  opts.start_s = arg_double(argc, argv, "--start-s", 0.0);
  opts.end_s = arg_double(argc, argv, "--end-s", -1.0);
  const double window_m = arg_double(argc, argv, "--window-m", 120.0);
  const double res = arg_double(argc, argv, "--res", 0.25);
  const double max_range = arg_double(argc, argv, "--max-range", 150.0);
  const double min_grazing = arg_double(argc, argv, "--min-grazing-deg", 0.0);

  if (!(window_m > 0.0) || !(res > 0.0) || !(max_range > 0.0)) {
    std::fprintf(stderr, "error: --window-m, --res, and --max-range must be > 0\n");
    return 1;
  }
  if (!(min_grazing >= 0.0 && min_grazing < 90.0)) {
    std::fprintf(stderr, "error: --min-grazing-deg must be in [0, 90)\n");
    return 1;
  }

  marine_perception_tools::LoadedBag bag;
  try {
    std::fprintf(stderr, "loading %s ...\n", argv[1]);
    bag = marine_perception_tools::load_forward_camera(argv[1], opts);
    std::fprintf(stderr, "loaded %zu frames\n", bag.frames.size());
  } catch (const std::exception & e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }

  marine_perception_tools::ReSimEngine engine(
    bag, window_m, res, max_range,
    sea_surface_segmentation::OccupancyParams{}, min_grazing);

  marine_perception_tools::MainWindow window(engine);
  window.resize(1280, 720);
  window.show();
  return app.exec();
}
