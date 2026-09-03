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

#ifndef MBES_GEOMETRY_HPP_
#define MBES_GEOMETRY_HPP_

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "marine_acoustic_msgs/msg/sonar_detections.hpp"

// Qt-free MBES (M3) beam geometry for the offline viewer's 3D point cloud + the
// backscatter waterfall. This is a QC-grade projection: a single sound speed, no
// CUBE error model. The per-beam formula mirrors
// cube_bathymetry/include/cube_bathymetry/sounding.h exactly so the viewer and
// the production importer agree on geometry:
//   range = two_way_travel_times[i] * ping_info.sound_speed / 2
//   x = range * -sin(tx_angle)              (along-track, sensor frame)
//   y = range *  sin(rx_angle)              (across-track)
//   z = range *  cos(tx_angle)*cos(rx_angle) (down)
// Soundings are in the sensor frame (bizzy/m3); the session lifts them to the
// world frame with the static m3->base and per-ping base->world transforms.

namespace marine_perception_tools
{

// One projected MBES sounding in the sensor frame (metres), plus its backscatter
// and the per-beam geometry cube's angular-response correction consumes (#27):
// the receive/steering angle and the slant range, kept bound to the intensity
// they measured (the same {raw intensity, angle} pairing as cube::Sounding).
struct MbesSounding
{
  double x = 0.0;          // along-track (sensor frame)
  double y = 0.0;          // across-track
  double z = 0.0;          // down
  float intensity = 0.0f;  // backscatter (dB), from detections.intensities[i]
  float beam_angle = std::numeric_limits<float>::quiet_NaN();   // rx, radians
  float slant_range = std::numeric_limits<float>::quiet_NaN();  // metres
};

// Project a single beam to a sensor-frame point. `tx_angle`/`rx_angle` in radians,
// `twtt` in seconds, `sound_speed` in m/s. Matches cube::Sounding's formula.
inline MbesSounding project_beam(
  double twtt, double tx_angle, double rx_angle, double sound_speed)
{
  const double range = twtt * sound_speed / 2.0;
  MbesSounding s;
  s.x = range * -std::sin(tx_angle);
  s.y = range * std::sin(rx_angle);
  s.z = range * std::cos(tx_angle) * std::cos(rx_angle);
  s.beam_angle = static_cast<float>(rx_angle);
  s.slant_range = static_cast<float>(range);
  return s;
}

// Project every beam of a SonarDetections ping to sensor-frame soundings. A beam
// is skipped when its travel time is non-positive (no bottom detection). Missing
// tx_angle defaults to 0 (nadir along-track), matching cube::Sounding; rx_angle
// and intensity are read per beam when present.
inline std::vector<MbesSounding> project_detections(
  const marine_acoustic_msgs::msg::SonarDetections & d)
{
  const double sound_speed = d.ping_info.sound_speed;
  const std::size_t n = d.two_way_travel_times.size();
  std::vector<MbesSounding> out;
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double twtt = d.two_way_travel_times[i];
    if (!(twtt > 0.0) || !(sound_speed > 0.0)) {
      continue;  // no detection / unusable scale on this beam
    }
    const double tx = (i < d.tx_angles.size()) ? d.tx_angles[i] : 0.0;
    const double rx = (i < d.rx_angles.size()) ? d.rx_angles[i] : 0.0;
    MbesSounding s = project_beam(twtt, tx, rx, sound_speed);
    if (i < d.intensities.size()) {
      s.intensity = d.intensities[i];
    }
    out.push_back(s);
  }
  return out;
}

}  // namespace marine_perception_tools

#endif  // MBES_GEOMETRY_HPP_
