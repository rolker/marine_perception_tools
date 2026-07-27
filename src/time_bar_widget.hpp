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

#ifndef TIME_BAR_WIDGET_HPP_
#define TIME_BAR_WIDGET_HPP_

#include <QString>
#include <QTimer>
#include <QWidget>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "time_bar_model.hpp"

namespace marine_perception_tools
{

// One pass drawn on the time bar: a coalesced pass over the tile selection
// plus its display payload. `color_index` >= 0 draws the bar in that
// golden-angle pass colour — the SAME index the cloud legend uses for this
// pass, so cloud and time bar correlate; -1 (not in the cloud, e.g.
// sidescan) draws neutral.
struct TimelinePassInfo
{
  std::string bag_path;
  std::string sensor_type;
  std::int64_t t_start_ns = 0;
  std::int64_t t_end_ns = 0;
  std::int64_t ping_count = 0;
  int color_index = -1;
};

// The GeoZui-style time bar (#24; ported from GeoZui4D's TimeControl, rja
// 2002, navigation-only — no playback transport yet). Two rows:
//  - TAPE (top): a zoomable time axis with the multi-resolution tick ladder
//    (time_bar_model), the current time pinned at a centre cursor. Left-drag
//    pans (grab-and-slide; pulling vertically past 100 px also stretches the
//    scale, as in the original); wheel zooms (ctrl = coarse, shift = fine);
//    double- or middle-click animates a 0.25 s jump to the clicked time.
//    Out-of-extent regions wash red. The selection's pass bars draw on the
//    tape in per-sensor sub-lanes; clicking one activates that pass.
//  - SCROLLBAR (bottom): the whole extent with its own tick scale, a
//    translucent green thumb showing the tape's visible window (min 30 px).
//    Thumb-drag scrubs absolutely; a click in the trough pages by a
//    screenful and autorepeats.
// The centre time is committed (timeSelected) on release / jump end / page,
// not continuously — the viewer cues bags from it, which is not a per-frame
// operation.
class TimeBarWidget : public QWidget
{
  Q_OBJECT

public:
  explicit TimeBarWidget(QWidget * parent = nullptr);

  // Replaces the pass bars; the extent grows to cover them and the view
  // re-fits when the user hasn't taken it over.
  void setPasses(std::vector<TimelinePassInfo> passes);
  // Drops the pass bars AND resets the extent (to empty) and the user's
  // view-override flag — i.e. it clears the setExtent state too, not just the
  // passes. A caller that wants to keep an explicit extent must re-apply it
  // after (as exitSelectionCloud does when returning to bag mode).
  void clearPasses();
  int passCount() const {return static_cast<int>(passes_.size());}

  // Extent independent of passes (bag mode: the open bag's time span).
  void setExtent(std::int64_t t0_ns, std::int64_t t1_ns);
  bool hasExtent() const {return extent_t1_ > extent_t0_;}

  // Move the centre cursor to the viewer's current time (scrub round-trip).
  // Ignored mid-interaction/animation so the operator's hand wins.
  void setCurrentTime(std::int64_t t_ns);
  std::int64_t currentTime() const {return center_ns_;}

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override {return sizeHint();}

signals:
  // The centre time moved — fires CONTINUOUSLY (every drag step, animation
  // frame, page, scrub round-trip). For cheap live followers only (the map
  // position arrow); anything that loads data listens to timeSelected.
  void centerTimeChanged(qlonglong t_ns);
  // The operator committed a new centre time (drag release / jump / page).
  void timeSelected(qlonglong t_ns);
  // A pass bar was clicked: cue the viewer to this pass.
  void passActivated(const QString & bag_path, qlonglong t_start_ns, qlonglong t_end_ns);

protected:
  void paintEvent(QPaintEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;
  void mouseDoubleClickEvent(QMouseEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;
  void leaveEvent(QEvent * event) override;

private:
  QRectF tapeRect() const;
  QRectF scrollRect() const;
  double xOfTime(std::int64_t t_ns) const;          // tape px
  std::int64_t timeOfX(double x_px) const;          // tape px -> time
  int laneOf(const std::string & sensor_type) const;
  QRectF passBarRect(std::size_t i) const;          // on the tape
  int barAt(const QPointF & pos) const;
  void fitExtent();                                 // view spans the extent
  void startJump(std::int64_t target_ns);
  void commitTime();                                // emit timeSelected(center)
  void pageBy(int direction);
  std::pair<double, double> thumbSpan() const;      // scrollbar thumb x0..x1

  std::vector<TimelinePassInfo> passes_;
  std::vector<std::string> lanes_;                  // sensor type -> sub-lane
  std::int64_t extent_t0_ = 0;
  std::int64_t extent_t1_ = 0;

  std::int64_t center_ns_ = 0;                      // time at the centre cursor
  double spp_ = 1.0;                                // seconds per pixel (zoom)
  bool user_adjusted_ = false;
  // A pre-layout setExtent must not fit against the unlaid widget width (the
  // canvas's stage-2 lesson): fits stay pending and apply at paint time,
  // re-applying on resize until the user takes the view over.
  bool fit_pending_ = false;

  // Tape drag (pan + vertical stretch).
  bool dragging_tape_ = false;
  bool drag_moved_ = false;
  QPointF drag_start_pos_;
  std::int64_t drag_start_center_ = 0;
  double drag_start_spp_ = 1.0;

  // Scrollbar thumb drag / trough paging.
  bool dragging_thumb_ = false;
  double thumb_grab_x_ = 0.0;
  std::int64_t thumb_start_center_ = 0;
  QTimer page_timer_;
  int page_direction_ = 0;

  // Animated jump (0.25 s, like the original's middle-click translate).
  QTimer anim_timer_;
  std::int64_t anim_from_ns_ = 0;
  std::int64_t anim_to_ns_ = 0;
  double anim_progress_ = 0.0;

  int hover_bar_ = -1;
};

}  // namespace marine_perception_tools

#endif  // TIME_BAR_WIDGET_HPP_
