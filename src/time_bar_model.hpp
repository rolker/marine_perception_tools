// Copyright 2026 Roland Arsenault
//
// Portions ported from GeoZui4D's TimeControl (Roland Arsenault, 2002), used
// here under the author's personal copyright.
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

#ifndef TIME_BAR_MODEL_HPP_
#define TIME_BAR_MODEL_HPP_

// Pure tick-ladder math for the GeoZui-style time bar (#24): a continuous
// multi-resolution ladder (milliseconds -> deciseconds -> seconds -> minutes
// -> hours -> dated days -> months -> years) where each level draws
// only while its tick spacing is in [3 px, 100k px), tick height grows
// smoothly with spacing, and labels appear as room allows. Ported from
// GeoZui4D's TimeControl::drawTimeScale/drawTicks (rja, 2002). Times are
// UNIX ns (UTC, like bag stamps); the calendar decomposition can be shifted
// into a display timezone via `utc_offset_s` (#26). Qt-free and
// unit-testable; the widget draws what this computes.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <string>
#include <utility>
#include <vector>

namespace marine_perception_tools
{

struct TimeTick
{
  double x_px = 0.0;      // pixel offset from the window's left edge
  std::string label;      // empty = tick mark only
  bool minor_label = false;   // label under the crowded-spacing rule (smaller font)
};

struct TickRow
{
  double interval_s = 0.0;
  double px_interval = 0.0;
  double tick_frac = 0.0;     // tick height as a fraction of the row height
  std::vector<TimeTick> ticks;
};

namespace time_bar_detail
{

constexpr double kDaySeconds = 3600.0 * 24.0;

inline const char * dayName(int wday)
{
  static const char * kDays[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
  return kDays[((wday % 7) + 7) % 7];
}

inline const char * monthName(int mon)   // NEXT month's 0-based index (tm_mon+1)
{
  static const char * kMonths[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  return kMonths[((mon % 12) + 12) % 12];
}

inline bool isLeap(int year)
{
  return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

inline int daysInMonth(int mon0, int year)   // mon0 = 0-based month
{
  static const int kDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  return (mon0 == 1 && isLeap(year)) ? 29 : kDays[((mon0 % 12) + 12) % 12];
}

// One ladder level: ticks stepping `interval_s` from `start_px`, the running
// value rolling over at `loop_limit` (0 = no rollover). `fmt` is either a
// printf format with one %i (numeric levels) or the single character 'd'/'m'
// selecting day/month names. `sub_interval` labels every Nth tick when full
// labelling is too crowded (0 = never). Mirrors drawTicks' visibility rules.
inline TickRow makeRow(
  double interval_s, double start_px, int start_value, int loop_limit,
  const char * fmt, int sub_interval, double width_px, double spp)
{
  TickRow row;
  row.interval_s = interval_s;
  const double px = interval_s / spp;
  row.px_interval = px;
  if (px < 3.0 || px >= 100000.0 || width_px <= 0.0) {
    return row;   // level invisible at this zoom
  }
  row.tick_frac = std::min(1.0, 0.8 * (1.0 - 3.0 / px));
  const bool named = fmt[1] == '\0';
  int value = start_value;
  for (double x = start_px; x < width_px; x += px, ++value) {
    if (loop_limit && value >= loop_limit) {
      value = 0;
    }
    if (x <= 0.0) {
      continue;
    }
    TimeTick tick;
    tick.x_px = x;
    const bool sub_labeled = sub_interval > 0 && sub_interval * px > 50.0 &&
      !named && value % sub_interval == 0;
    if (px > 50.0) {
      if (named) {
        tick.label = (fmt[0] == 'd') ? dayName(value) : monthName(value);
      } else {
        char buf[32];
        std::snprintf(buf, sizeof(buf), fmt, value);
        tick.label = buf;
      }
    } else if (sub_labeled) {
      char buf[32];
      std::snprintf(buf, sizeof(buf), fmt, value);
      tick.label = buf;
      tick.minor_label = true;
    }
    row.ticks.push_back(std::move(tick));
  }
  return row;
}

// Advance a civil date (0-based month) by one day, rolling weekday, month
// and year.
inline void advanceDay(int & wday, int & mday, int & mon, int & year)
{
  wday = (wday + 1) % 7;
  if (++mday > daysInMonth(mon, year)) {
    mday = 1;
    if (++mon == 12) {
      mon = 0;
      ++year;
    }
  }
}

// The day-level row walks the civil calendar so labels can carry the date,
// not just the weekday. `wday/mday/mon/year` describe the window's LEFT
// edge; the first tick is the next midnight. Labels grow with the room a
// tick has: "Mon 22" past the ladder's 50 px labelling threshold,
// "Mon Jun 22" once ticks are 90 px apart.
inline TickRow makeDayRow(
  double start_px, int wday, int mday, int mon, int year,
  double width_px, double spp)
{
  TickRow row;
  row.interval_s = kDaySeconds;
  const double px = kDaySeconds / spp;
  row.px_interval = px;
  if (px < 3.0 || px >= 100000.0 || width_px <= 0.0) {
    return row;   // level invisible at this zoom
  }
  row.tick_frac = std::min(1.0, 0.8 * (1.0 - 3.0 / px));
  for (double x = start_px; x < width_px; x += px) {
    advanceDay(wday, mday, mon, year);
    if (x <= 0.0) {
      continue;
    }
    TimeTick tick;
    tick.x_px = x;
    if (px > 90.0) {
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%s %s %i", dayName(wday), monthName(mon), mday);
      tick.label = buf;
    } else if (px > 50.0) {
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%s %i", dayName(wday), mday);
      tick.label = buf;
    }
    row.ticks.push_back(std::move(tick));
  }
  return row;
}

}  // namespace time_bar_detail

// A nanosecond count computed in double, saturated just inside int64 range —
// an out-of-range float->int cast is UB, not a clamp.
inline std::int64_t saturateNs(double t_ns)
{
  constexpr double kMaxNs = 9.2e18;   // inside int64 range after the cast
  return static_cast<std::int64_t>(std::clamp(t_ns, -kMaxNs, kMaxNs));
}

// `base_ns` shifted by a pixel span at `spp` seconds per pixel. At kMaxSpp a
// span the width of a large window is ~1e19 ns — past int64 range — so both
// the raw float->int cast and the subsequent addition would be UB; do the sum
// in double and saturate instead.
inline std::int64_t offsetTimeNs(std::int64_t base_ns, double span_px, double spp)
{
  return saturateNs(static_cast<double>(base_ns) + span_px * spp * 1e9);
}

// The full ladder for a window whose LEFT edge is `left_ns` (UNIX ns, UTC)
// at `spp` seconds per pixel over `width_px` pixels. Rows whose level is
// invisible at this zoom come back with no ticks. Month/year stepping uses
// the original's average-length approximation past the first (calendar-
// derived) tick — display-grade, like the source design.
// `utc_offset_s` shifts the calendar decomposition into a display timezone
// (e.g. -14400 for EDT). This moves the ticks themselves, not just their
// labels: the ladder is placed on that zone's calendar boundaries, so a day
// tick sits at LOCAL midnight — a different real instant from UTC midnight.
// The time axis itself is unchanged; it is the set of instants chosen for
// ticks that follows the zone. The offset is a single value for the whole
// window — display-grade across a DST change inside one view (the caller
// re-derives it per repaint).
inline std::vector<TickRow> computeTickLadder(
  std::int64_t left_ns, double spp, double width_px, int utc_offset_s = 0)
{
  using time_bar_detail::makeRow;

  std::vector<TickRow> rows;
  if (!(spp > 0.0) || width_px <= 0.0) {
    return rows;
  }

  // The shifted value only feeds the civil-field decomposition below; sum in
  // double and saturate, as a left edge already near the int64 rails plus an
  // offset would overflow (UB) in int64.
  const std::int64_t disp_ns = saturateNs(
    static_cast<double>(left_ns) + static_cast<double>(utc_offset_s) * 1e9);

  // Floor-divide: '/' and '%' truncate toward zero, so a pre-epoch left edge
  // (reachable by panning or zooming left of 1970) would get a negative frac
  // and a whole second off by one, misplacing every tick in the ladder.
  std::int64_t whole = disp_ns / 1000000000LL;
  std::int64_t rem = disp_ns % 1000000000LL;
  if (rem < 0) {
    --whole;
    rem += 1000000000LL;
  }
  const auto whole_s = static_cast<time_t>(whole);
  const double frac = static_cast<double>(rem) / 1e9;
  std::tm g{};
  gmtime_r(&whole_s, &g);
  const double sec = g.tm_sec + frac;
  const int min = g.tm_min;
  const int hour = g.tm_hour;
  const int mday = g.tm_mday;
  const int mon = g.tm_mon;
  const int year = g.tm_year + 1900;
  const int wday = g.tm_wday;
  const int yday = g.tm_yday;

  constexpr double kDayS = time_bar_detail::kDaySeconds;
  constexpr double kYearS = kDayS * 365.25;   // approximate, as in the original
  const double seconds_into_day = hour * 3600.0 + min * 60.0 + sec;
  const double seconds_into_year = yday * kDayS + seconds_into_day;

  // milliseconds
  double temp = sec * 1000.0;
  rows.push_back(makeRow(0.001, (0.001 - (temp - std::floor(temp)) / 1000.0) / spp,
    static_cast<int>(std::ceil(temp)) % 1000, 1000, "%ims", 50, width_px, spp));
  // deciseconds
  temp = sec * 10.0;
  rows.push_back(makeRow(0.1, (0.1 - (temp - std::floor(temp)) / 10.0) / spp,
    static_cast<int>(std::ceil(temp)) % 10, 10, "%ids", 5, width_px, spp));
  // seconds
  rows.push_back(makeRow(1.0, (1.0 - (sec - std::floor(sec))) / spp,
    static_cast<int>(std::ceil(sec)), 60, "%is", 15, width_px, spp));
  // minutes
  rows.push_back(makeRow(60.0, (60.0 - sec) / spp, min + 1, 60, "%im", 15,
    width_px, spp));
  // hours
  rows.push_back(makeRow(3600.0, (3600.0 - (min * 60.0 + sec)) / spp, hour + 1,
    24, "%ih", 6, width_px, spp));
  // days (weekday name + date, walking the civil calendar)
  rows.push_back(time_bar_detail::makeDayRow(
      (kDayS - seconds_into_day) / spp, wday, mday, mon, year, width_px, spp));
  // months (names; average-month stepping past the calendar-true first tick)
  const double dim = time_bar_detail::daysInMonth(mon, year) * kDayS;
  rows.push_back(makeRow(kYearS / 12.0,
    (dim - ((mday - 1) * kDayS + seconds_into_day)) / spp, mon + 1, 12, "m", 0,
    width_px, spp));
  // years
  rows.push_back(makeRow(kYearS, (kYearS - seconds_into_year) / spp, year + 1,
    0, "y%i", 25, width_px, spp));
  return rows;
}

}  // namespace marine_perception_tools

#endif  // TIME_BAR_MODEL_HPP_
