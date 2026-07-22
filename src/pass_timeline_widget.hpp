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

#ifndef PASS_TIMELINE_WIDGET_HPP_
#define PASS_TIMELINE_WIDGET_HPP_

#include <QString>
#include <QWidget>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "pass_timeline_model.hpp"

namespace marine_perception_tools
{

// One bar on the timeline: a coalesced pass over the tile selection plus its
// display payload. `color_index` >= 0 draws the bar in that golden-angle
// pass colour — the SAME index the cloud legend uses for this pass, so cloud
// and timeline correlate at a glance; -1 (a pass that is not in the cloud,
// e.g. sidescan) draws neutral.
struct TimelinePassInfo
{
  std::string bag_path;
  std::string sensor_type;
  std::int64_t t_start_ns = 0;
  std::int64_t t_end_ns = 0;
  std::int64_t ping_count = 0;
  int color_index = -1;
};

// The pass timeline pane (#24, replaces the stage-2 pass list): the tile
// selection's coalesced pass intervals on a gap-compressed UTC axis
// (pass_timeline_model), one lane per sensor type. Clicking a bar activates
// the pass — the window cues the single-pass waterfall through the existing
// bag-open + jump-to-pass machinery. Tooltips carry the detail the retired
// pass list showed (bag, UTC interval, duration, ping count).
class PassTimelineWidget : public QWidget
{
  Q_OBJECT

public:
  explicit PassTimelineWidget(QWidget * parent = nullptr);

  // Replaces the bars (lanes are re-derived from the sensor types present).
  void setPasses(std::vector<TimelinePassInfo> passes);
  void clearPasses();
  int passCount() const {return static_cast<int>(infos_.size());}

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override {return sizeHint();}

signals:
  // A bar was clicked: cue the viewer to this pass (bag + UNIX-ns interval).
  void passActivated(const QString & bag_path, qlonglong t_start_ns, qlonglong t_end_ns);

protected:
  void paintEvent(QPaintEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void leaveEvent(QEvent * event) override;

private:
  // Axis-fraction x and lane row under a widget point; row -1 = off-lane.
  std::pair<double, int> hitCoords(double px, double py) const;
  int laneOf(const std::string & sensor_type) const;
  QRectF barRect(std::size_t i) const;

  std::vector<TimelinePassInfo> infos_;
  std::vector<TimelinePass> model_passes_;   // parallel to infos_
  TimelineLayout layout_;
  std::vector<std::string> lanes_;           // lane index -> sensor type
  int hover_ = -1;                           // hovered bar, -1 = none
};

}  // namespace marine_perception_tools

#endif  // PASS_TIMELINE_WIDGET_HPP_
