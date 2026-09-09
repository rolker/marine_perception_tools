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

#ifndef SOUNDING_UNCERTAINTY_HPP_
#define SOUNDING_UNCERTAINTY_HPP_

#include <cmath>

// Qt-free, angle-aware per-sounding uncertainty for the CUBE lab (#49).
//
// STILL A PLACEHOLDER. The real budget is cube::ErrorModel, which needs the raw
// detections, the platform attitude and the vessel offsets — none of which the
// explorer's cloud path carries (mpt#27 follow-up). What it replaces was worse
// than a placeholder: a depth-only formula
//
//     v_std = 0.1 + 0.007 * depth,  h_std = 0.2 + 0.01 * depth
//
// with no beam-angle term at all, so a beam at the swath edge was handed to the
// estimator with exactly the confidence of one directly under the transducer.
// CUBE weights soundings by their uncertainty, so it had no basis to prefer a
// pass's clean near-nadir coverage over another pass's noisy outer beams — the
// operator's report of 2026-09-09: the refraction smile's outer beams spoiling
// an otherwise clean surface.
//
// This does NOT model refraction. The smile is a SYSTEMATIC error; no
// uncertainty model removes it. Correcting it is the ray-traced re-projection
// in mpt#28. What an angular term does do is stop the noisy outer beams from
// outvoting clean near-nadir ones where passes overlap.
//
// THE MODEL — first-order propagation of a range error and an angular error
// through the beam geometry the cloud path already retains (MbesSounding's
// `beam_angle` and `slant_range`). For a beam at angle t from nadir at slant
// range R:
//
//     sigma_z^2 = (sigma_R * cos t)^2 + (R * sigma_theta * sin t)^2
//     sigma_y^2 = (sigma_R * sin t)^2 + (R * sigma_theta * cos t)^2 + sigma_pos^2
//
// where sigma_pos is the inherited horizontal positioning stand-in (see
// kPositionStdFixedM — the horizontal half of the old placeholder, kept
// because navigation, not beam geometry, is what puts a sounding on the
// seabed, and because CUBE spends sigma_y on the influence radius).
//
// The `R * sigma_theta * sin t` term is the one that matters: exactly zero at
// nadir and growing with both range and angle. Each propagated component is
// then floored at the device's 0.05 m so no beam claims zero error — see
// sounding_uncertainty() for why the floor lands there and not on sigma_R.
//
// Compared at a fixed DEPTH — two passes over one node, the comparison CUBE
// makes — sigma_z rises with angle: ~1.4x nadir at 60 deg, ~1.9x at 70 deg. In
// CUBE's inverse-variance weighting that is a 60-deg beam counting about half
// of a nadir one.
//
// CONSTANTS come from cube::Device's own documented defaults
// (cube_bathymetry/include/cube_bathymetry/error_model.h) rather than being
// invented here, so this stand-in and the real model are seeded alike.

namespace marine_perception_tools
{

/// Range error as a fraction of the measured range — cube::Device::
/// range_error_percent (0.005 == 0.5%).
constexpr double kRangeErrorPercent = 0.005;

/// Absolute floor on the error, m — cube::Device::range_error_floor_m. The
/// library calls it a floor on the RANGE error, "prevents a zero error near
/// the surface"; here it is applied to each PROPAGATED component instead of to
/// the range before projection. See sounding_uncertainty() for why.
constexpr double kRangeErrorFloorM = 0.05;

/// Across-track (receive) beamwidth, degrees — cube::Device::
/// across_track_beamwidth.
constexpr double kAcrossTrackBeamwidthDeg = 2.0;

/// POSITIONING STAND-IN, horizontal only: std = fixed + percent * depth, m.
///
/// These two are the horizontal half of the depth-only placeholder this model
/// replaced (0.2 m + 1% of depth), kept deliberately rather than dropped. A
/// sounding's horizontal uncertainty in the WORLD is dominated by navigation —
/// GPS, attitude, latency, lever arms — none of which the cloud path carries,
/// and all of which are metres of budget next to the centimetres of beam
/// geometry below. Dropping them would have this file assert ~3 cm positioning,
/// which nothing here has measured.
///
/// It is also what CUBE spends the horizontal error on: it sets the RADIUS a
/// sounding spreads over (Parameters::influenceRadius caps the radius at
/// 2.56 * sigma_y). At beam-geometry-only sigma_y that cap falls below one cell
/// for any normal grid, which pins every radius to the cell size and makes the
/// operator's uncertainty-budget control (#45) inert — a silent loss of a knob,
/// from a change that is supposed to be about beam angle. Keeping the incumbent
/// term keeps this change to exactly what it claims to be.
///
/// Unsourced, and inherited: the real numbers belong to cube::Vessel
/// (gps_drms and friends), which arrives with the real ErrorModel (mpt#27).
constexpr double kPositionStdFixedM = 0.2;
constexpr double kPositionStdPercentOfDepth = 0.01;

/// BEAMWIDTH-TO-SIGMA CONVENTION: sigma_theta = beamwidth / 12.
///
/// A beamwidth is not a standard deviation, so the divisor is a modelling
/// decision. This one is NOT invented here: it is the convention
/// cube::ErrorModel already uses. In ErrorModel::swath_angle_error
/// (cube_bathymetry/src/error_model.cpp) the angular measurement sigma is
///
///     ang_meas = rx_beamwidths[i] * (M_PI / 180.0) / 12.0
///
/// i.e. the -3 dB beamwidth in radians over twelve. Following it means the two
/// places in this codebase that turn a beamwidth into a sigma agree; inventing
/// a second divisor here (a uniform distribution's sqrt(12), say) would leave
/// one quantity defined two ways, which is worse than either choice alone.
///
/// One caveat when comparing the two: ErrorModel's FALLBACK line, taken when a
/// ping reports no per-beam beamwidths, divides Device::across_track_beamwidth
/// by 12 WITHOUT converting it from degrees
/// (`ang_meas = device_.across_track_beamwidth / 12.0`), while the per-beam
/// line above does convert. We follow the per-beam line — the radian form, the
/// one applied whenever the sonar reports beamwidths — and so deliberately do
/// not reproduce that fallback's unit mismatch.
constexpr double kBeamwidthToSigmaDivisor = 12.0;

/// Angular uncertainty of one beam, radians. See kBeamwidthToSigmaDivisor.
/// 2 deg / 12 == 0.167 deg == 2.909e-3 rad.
inline double beam_angle_sigma_rad()
{
  return (kAcrossTrackBeamwidthDeg * M_PI / 180.0) / kBeamwidthToSigmaDivisor;
}

/// Range uncertainty at slant range `slant_range_m`, metres — the device
/// fraction of the range, unfloored (the floor lands on the propagated
/// components instead).
///
/// cube::ErrorModel::range_error expresses this fraction against DEPTH; here it
/// is applied to the SLANT RANGE, which is what the sonar actually measured and
/// what the propagation below is expressed in. This is not cosmetic: with the
/// fraction on range, the range term's contribution to the VERTICAL error is
/// sigma_R*cos(t) == percent * depth for every beam of a ping, so the whole of
/// a ping's angle dependence lives in the angular term, where it belongs. Put
/// the fraction on depth instead and the vertical range term shrinks as cos(t)
/// across the swath, which drags sigma_z back DOWN over the inner half of the
/// swath before the angular term overtakes it.
inline double range_sigma_m(double slant_range_m)
{
  return std::fabs(slant_range_m) * kRangeErrorPercent;
}

/// One sounding's uncertainty, as the VARIANCES cube::Sounding stores
/// (vertical_error / horizontal_error are variances, m^2 — not standard
/// deviations).
struct SoundingUncertainty
{
  double vertical_variance = 0.0;    // m^2, sigma_z^2
  double horizontal_variance = 0.0;  // m^2, sigma_y^2
};

/// Uncertainty for a beam at `beam_angle_rad` from nadir and slant range
/// `slant_range_m`.
///
/// WHERE THE FLOOR GOES, and why it is a decision. cube::Device documents its
/// 0.05 m as a floor on the range error. Applied there — max(percent*R, floor)
/// before projection — a floored beam's VERTICAL error is floor*cos(t), which
/// FALLS as the beam steers outboard. In water shallow enough for the floor to
/// bind (roughly < 10 m for these constants) that inverts the very ordering
/// this model exists to establish: the swath edge would come out more certain
/// than nadir, and CUBE would prefer exactly the beams the operator is trying
/// to demote. So the floor is applied to each propagated component instead,
/// which preserves its stated purpose (no beam ever claims zero error) without
/// the inversion. At nadir the two placements agree exactly: sigma_z there is
/// max(percent*R, floor), which is cube::ErrorModel::range_error's own value.
///
/// WHAT THIS MODEL DOES NOT CLAIM. Compare beams at a fixed DEPTH — two passes
/// over one node, which is the comparison CUBE actually makes — and sigma_z
/// rises monotonically with angle, by ~1.4x at 60 deg and ~1.9x at 70 deg.
/// Compare them at a fixed SLANT RANGE and it FALLS, because an outer beam at
/// the same range is measuring shallower water and its range error projects
/// mostly sideways. That is a property of the geometry, not a bug, but it means
/// "outer beams are always less certain" is not a statement this model makes.
/// The other absentees are the terms that make real outer beams bad: roll,
/// SVP and refraction. Refraction in particular is SYSTEMATIC and no
/// uncertainty model removes it (mpt#28).
///
/// Returns false — writing nothing — for geometry that cannot produce a
/// meaningful answer: a non-finite angle or range, or a negative range. A
/// sounding whose geometry is unknown must be DROPPED by the caller, not
/// handed to the estimator with a fabricated confidence.
inline bool sounding_uncertainty(
  double beam_angle_rad, double slant_range_m, SoundingUncertainty * out)
{
  if (out == nullptr) {
    return false;
  }
  if (!std::isfinite(beam_angle_rad) || !std::isfinite(slant_range_m) ||
    slant_range_m < 0.0)
  {
    return false;
  }
  const double sigma_r = range_sigma_m(slant_range_m);
  const double angular = slant_range_m * beam_angle_sigma_rad();
  const double cos_t = std::cos(beam_angle_rad);
  const double sin_t = std::sin(beam_angle_rad);

  // Horizontal carries the positioning stand-in in quadrature; vertical does
  // not, deliberately — a platform-scale vertical term would swamp the angular
  // one this issue exists to introduce (see kPositionStdFixedM).
  const double depth = slant_range_m * std::fabs(cos_t);
  const double sigma_pos = kPositionStdFixedM + kPositionStdPercentOfDepth * depth;

  const double sigma_z = std::fmax(
    std::hypot(sigma_r * cos_t, angular * sin_t), kRangeErrorFloorM);
  const double sigma_y = std::fmax(
    std::hypot(std::hypot(sigma_r * sin_t, angular * cos_t), sigma_pos),
    kRangeErrorFloorM);

  out->vertical_variance = sigma_z * sigma_z;
  out->horizontal_variance = sigma_y * sigma_y;
  return true;
}

/// The one-line description of the model in force, for the UI caveat and any
/// other surface that must not drift out of step with the formula above.
inline const char * sounding_uncertainty_caveat()
{
  return
    "Compared against a PLACEHOLDER per-sounding error (mpt#49): first-order "
    "propagation of a 0.5%-of-range error and a 2 deg / 12 beam angular error "
    "through each beam's own angle and slant range (each component floored at "
    "0.05 m), so an outer beam is less trusted than a nadir beam over the same "
    "depth — about half the weight at 60 deg. The explorer's cloud path does not "
    "carry the detections, attitude and vessel offsets the real CUBE error "
    "model needs, so these thresholds are still applied against synthetic "
    "uncertainty (mpt#27 follow-up). It does not correct the refraction smile "
    "— that is a systematic error (mpt#28).";
}

}  // namespace marine_perception_tools

#endif  // SOUNDING_UNCERTAINTY_HPP_
