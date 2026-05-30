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
#include <QString>

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>

#include "bag_loader.hpp"
#include "main_window.hpp"

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

  // A leading flag (or no args) means "open with no bag" — use File -> Open Bag…
  // A positional first arg is treated as a bag URI and opened on startup.
  const bool has_bag = (argc >= 2 && argv[1][0] != '-');
  if (!has_bag && argc >= 2 && std::string(argv[1]) == "--help") {
    std::fprintf(stderr,
      "usage: %s [bag_uri] [--start-s S] [--end-s S] [--window-m 120] [--res 0.25]\n"
      "          [--max-range 150] [--min-grazing-deg 0] [--probe]\n"
      "Replays all four OAK cameras' segmentation and tunes the segmentation->\n"
      "costmap marking interactively. With no bag_uri the window opens empty — use\n"
      "File -> Open Bag…. --start-s/--end-s window the replay (end<0 == to end).\n"
      "--probe loads the bag, prints frame/rgb/costmap counts, and exits (headless).\n",
      argv[0]);
    return 0;
  }

  bool probe = false;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--probe") {probe = true;}
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

  // Headless probe: load the bag, report counts, exit — for verifying the loader
  // (incl. H.265 decode) against a real bag without a display. Run with
  // QT_QPA_PLATFORM=offscreen so QApplication needs no X server.
  if (probe) {
    if (!has_bag) {
      std::fprintf(stderr, "error: --probe requires a bag_uri\n");
      return 1;
    }
    try {
      const auto bag = marine_perception_tools::load_bag(argv[1], opts);
      std::fprintf(stderr,
        "probe: %zu seg frames, %zu rgb frames, %zu recorded costmaps, compressed_seg=%d\n",
        bag.frames.size(), bag.rgb_frames.size(), bag.costmaps.size(),
        static_cast<int>(bag.used_compressed_segmentation));
      if (!bag.rgb_frames.empty()) {
        const cv::Mat & m = bag.rgb_frames.front().bgr;
        const cv::Scalar mean = cv::mean(m);
        std::fprintf(stderr,
          "  first rgb: cam %d, %dx%d, BGR mean (%.1f, %.1f, %.1f)\n",
          bag.rgb_frames.front().cam, m.cols, m.rows, mean[0], mean[1], mean[2]);
      }
      return 0;
    } catch (const std::exception & e) {
      std::fprintf(stderr, "probe error: %s\n", e.what());
      return 1;
    }
  }

  marine_perception_tools::MainWindow window(opts, window_m, res, max_range, min_grazing);
  window.resize(1280, 720);
  window.show();
  if (has_bag) {
    window.openBag(QString::fromUtf8(argv[1]));  // errors surface in the status bar
  }
  return app.exec();
}
