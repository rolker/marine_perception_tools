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
    if (flag != argv[i]) {continue;}
    const char * text = argv[i + 1];
    char * end = nullptr;
    const double value = std::strtod(text, &end);
    if (end == text || end == nullptr || *end != '\0') {
      std::fprintf(stderr, "error: invalid value for %s: '%s' (expected a number)\n",
        flag.c_str(), text);
      std::exit(2);
    }
    return value;
  }
  return def;
}

}  // namespace

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);

  // A leading flag (or no args) means "open with no bag" â€” use File -> Open Bagâ€¦
  // A positional first arg is treated as a bag URI and opened on startup.
  const bool has_bag = (argc >= 2 && argv[1][0] != '-');
  if (!has_bag && argc >= 2 && std::string(argv[1]) == "--help") {
    std::fprintf(stderr,
      "usage: %s [bag_uri] [--start-s S] [--end-s S] [--window-m 120] [--res 0.25]\n"
      "          [--max-range 150] [--min-grazing-deg 0]\n"
      "          [--integration-halflives 1] [--margin-s 10] [--retention-s 120]\n"
      "          [--probe]\n"
      "Replays all four OAK cameras' segmentation and tunes the segmentation->\n"
      "costmap marking interactively. With no bag_uri the window opens empty â€” use\n"
      "File -> Open Bagâ€¦. File->Open buffers a window around the scrub point rather\n"
      "than the whole bag: integration-halflives x decay_half_life_s of warm-up,\n"
      "+/- margin-s of reload-free scrub slack, keeping retention-s of extra\n"
      "already-read frames. --start-s/--end-s optionally clamp the session to a\n"
      "sub-range (end<0 == to end). --probe loads [start,end], prints frame/rgb/\n"
      "costmap counts, and exits (headless).\n",
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
  // Windowed-buffer policy knobs (Milestone D4): window = halflives x
  // decay_half_life_s of warm-up, +/- margin of scrub slack, + retention extra.
  const double integ_hl = arg_double(argc, argv, "--integration-halflives", 1.0);
  const double margin_s = arg_double(argc, argv, "--margin-s", 10.0);
  const double retention_s = arg_double(argc, argv, "--retention-s", 120.0);

  if (!(window_m > 0.0) || !(res > 0.0) || !(max_range > 0.0)) {
    std::fprintf(stderr, "error: --window-m, --res, and --max-range must be > 0\n");
    return 1;
  }
  if (!(min_grazing >= 0.0 && min_grazing < 90.0)) {
    std::fprintf(stderr, "error: --min-grazing-deg must be in [0, 90)\n");
    return 1;
  }
  if (!(integ_hl > 0.0) || !(margin_s >= 0.0) || !(retention_s >= 0.0)) {
    std::fprintf(stderr,
      "error: --integration-halflives must be > 0; --margin-s/--retention-s >= 0\n");
    return 1;
  }

  // Headless probe: load the bag, report counts, exit â€” for verifying the loader
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
        "probe: %zu seg frames, %zu rgb frames, %zu recorded costmaps, "
        "compressed_seg=%d, tf_skipped=%zu\n",
        bag.frames.size(), bag.rgb_frames.size(), bag.costmaps.size(),
        static_cast<int>(bag.used_compressed_segmentation), bag.frames_skipped_no_tf);
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

  marine_perception_tools::MainWindow window(
    opts, window_m, res, max_range, min_grazing, integ_hl, margin_s, retention_s);
  window.resize(1280, 720);
  window.show();
  if (has_bag) {
    window.openBag(QString::fromUtf8(argv[1]));  // errors surface in the status bar
  }
  return app.exec();
}
