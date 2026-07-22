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

#ifndef MBES_PASS_LOADER_HPP_
#define MBES_PASS_LOADER_HPP_

// The multi-pass MBES cloud loader (#21, moved out of MbesCloudWindow for the
// integrated explorer, #24): windowed bag reads via read_mbes_window, with
// cross-bag soundings reprojected through the earth anchor into the FIRST
// pass's world frame. Widget-free (QStringList only) so it runs on a worker
// thread and unit-tests without a display.

#include <QStringList>

#include <cstdint>
#include <string>
#include <vector>

#include "mbes_geometry.hpp"

namespace marine_perception_tools
{

// One pass to load into the cloud: the bag it lives in, its time window (from
// the survey index), and a human label for the legend.
struct CloudPassInfo
{
  std::string bag_path;
  std::string label;
  std::int64_t t_start_ns = 0;
  std::int64_t t_end_ns = 0;
};

// Everything the worker thread hands back to the UI thread in one piece.
struct CloudLoadOutcome
{
  std::vector<std::vector<MbesSounding>> pass_clouds;  // reference world frame
  std::vector<int> sounding_counts;   // per input pass (0 = empty/skipped)
  QStringList notes;                  // per-pass problems, human-readable
  int skipped_passes = 0;             // passes dropped (frame not resolvable)
  int skipped_pings = 0;              // pings dropped (no TF), summed
};

// Load every pass's soundings into the first loadable pass's world frame.
// Never throws: a failing bag costs only its own pass (skipped + noted).
CloudLoadOutcome load_cloud_passes(const std::vector<CloudPassInfo> & passes);

}  // namespace marine_perception_tools

#endif  // MBES_PASS_LOADER_HPP_
