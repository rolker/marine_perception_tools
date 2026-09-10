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

#ifndef TF_LIFT_HPP_
#define TF_LIFT_HPP_

#include <tf2/buffer_core.h>

#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mbes_geometry.hpp"

namespace marine_perception_tools
{

// Shared TF-capture + sensor→world lift helpers, used by SidescanBagSession's
// full-bag load pass and by read_mbes_window()'s one-shot windowed read (#21).
// Header-only and deterministic — unit-tested in test_mbes_window_reader.

// Look up target<-source at `stamp`, falling back to the latest available
// transform on an extrapolation throw (mirrors cube::DetectionsProjector). Used
// to capture the full world<-m3 transform at a detections ping's stamp, while
// the bounded TF cache still brackets that stamp.
inline bool lookup_at_or_latest(
  const tf2::BufferCore & tf, const std::string & target, const std::string & source,
  const tf2::TimePoint & stamp, geometry_msgs::msg::TransformStamped & out)
{
  try {
    out = tf.lookupTransform(target, source, stamp);
    return true;
  } catch (const tf2::ExtrapolationException &) {
    try {
      out = tf.lookupTransform(target, source, tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  } catch (const tf2::TransformException &) {
    return false;
  }
}

// Rotate vector v by quaternion q (q assumed normalized): v' = v + 2 q_w (u x v)
// + 2 u x (u x v), with u = q.xyz. Used to lift sensor-frame soundings to world.
inline void rotate_by_quat(
  double qx, double qy, double qz, double qw,
  double vx, double vy, double vz, double & ox, double & oy, double & oz)
{
  // t = 2 * (u x v)
  const double tx = 2.0 * (qy * vz - qz * vy);
  const double ty = 2.0 * (qz * vx - qx * vz);
  const double tz = 2.0 * (qx * vy - qy * vx);
  // v' = v + qw * t + u x t
  ox = vx + qw * tx + (qy * tz - qz * ty);
  oy = vy + qw * ty + (qz * tx - qx * tz);
  oz = vz + qw * tz + (qx * ty - qy * tx);
}

// Rigidly lift one sensor-frame sounding into the world frame: rotate its
// position by `q`, translate by `t`, and CARRY EVERY OTHER FIELD UNCHANGED.
//
// The copy-then-overwrite shape is the point. A rigid transform moves the
// position and nothing else — the beam angle and slant range are measured in
// the sensor frame and are the same numbers in world, and the intensity is
// not geometry at all. Building the world sounding field by field instead
// silently drops whatever the author forgot, which is exactly how
// `beam_angle` and `slant_range` came to be NaN on every bag-loaded sounding
// (#42): both the ARA/TL correction (#27) and the angle-aware per-sounding
// uncertainty (#49) read them off the world sounding, and the latter now
// DROPS a sounding whose angle is not finite. Any field added to
// MbesSounding rides along here for free, in both callers at once.
inline MbesSounding lift_sounding_to_world(
  const MbesSounding & s,
  double tx, double ty, double tz,
  double qx, double qy, double qz, double qw)
{
  double rx = 0.0;
  double ry = 0.0;
  double rz = 0.0;
  rotate_by_quat(qx, qy, qz, qw, s.x, s.y, s.z, rx, ry, rz);
  MbesSounding w = s;
  w.x = tx + rx;
  w.y = ty + ry;
  w.z = tz + rz;
  return w;
}

}  // namespace marine_perception_tools

#endif  // TF_LIFT_HPP_
