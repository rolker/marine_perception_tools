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

#include "time_bar_widget.hpp"

#include <QColor>
#include <QDateTime>
#include <QFileInfo>
#include <QFont>
#include <QFontMetrics>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QTimeZone>
#include <QToolTip>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "point_cloud_view.hpp"   // pass_color (shared with the cloud legend)

namespace marine_perception_tools
{

namespace
{

constexpr double kTapeFraction = 0.62;    // of the widget height; rest = scrollbar
constexpr double kPassLaneHeight = 5.0;   // px per sensor sub-lane on the tape
constexpr double kMinThumbPx = 30.0;
constexpr int kPageRepeatMs = 500;        // original pages every 0.5 s while held
constexpr int kAnimStepMs = 16;
constexpr double kAnimDurationS = 0.25;
// Zoom clamp: ~0.1 ms/px (ping scale) to ~1 month/px (multi-campaign scale).
constexpr double kMinSpp = 1e-4;
constexpr double kMaxSpp = 2.7e6;

const char * kPreferredLanes[] = {"mbes-bathy", "sidescan-port", "sidescan-starboard"};

QString isoUtc(std::int64_t t_ns)
{
  return QDateTime::fromMSecsSinceEpoch(t_ns / 1000000LL, QTimeZone::utc())
         .toString("yyyy-MM-dd HH:mm:ss");
}

QColor barColor(const TimelinePassInfo & info)
{
  if (info.color_index >= 0) {
    float r = 1.0f;
    float g = 1.0f;
    float b = 1.0f;
    pass_color(info.color_index, r, g, b);
    return QColor::fromRgbF(r, g, b);
  }
  return QColor(150, 160, 170);
}

}  // namespace

TimeBarWidget::TimeBarWidget(QWidget * parent)
: QWidget(parent)
{
  setMouseTracking(true);
  page_timer_.setInterval(kPageRepeatMs);
  connect(&page_timer_, &QTimer::timeout, this, [this]() {pageBy(page_direction_);});
  anim_timer_.setInterval(kAnimStepMs);
  connect(&anim_timer_, &QTimer::timeout, this, [this]() {
      anim_progress_ += kAnimStepMs / 1000.0 / kAnimDurationS;
      if (anim_progress_ >= 1.0) {
        anim_timer_.stop();
        center_ns_ = anim_to_ns_;
        update();
        commitTime();
        return;
      }
      center_ns_ = anim_from_ns_ + static_cast<std::int64_t>(
        static_cast<double>(anim_to_ns_ - anim_from_ns_) * anim_progress_);
      update();
    });
}

QSize TimeBarWidget::sizeHint() const
{
  return QSize(500, 64);
}

void TimeBarWidget::setPasses(std::vector<TimelinePassInfo> passes)
{
  passes_ = std::move(passes);
  hover_bar_ = -1;
  lanes_.clear();
  for (const char * sensor : kPreferredLanes) {
    if (std::any_of(passes_.begin(), passes_.end(),
      [sensor](const TimelinePassInfo & p) {return p.sensor_type == sensor;}))
    {
      lanes_.emplace_back(sensor);
    }
  }
  for (const auto & p : passes_) {
    if (laneOf(p.sensor_type) < 0) {
      lanes_.push_back(p.sensor_type);
    }
  }
  if (!passes_.empty()) {
    std::int64_t t0 = passes_.front().t_start_ns;
    std::int64_t t1 = passes_.front().t_end_ns;
    for (const auto & p : passes_) {
      t0 = std::min(t0, p.t_start_ns);
      t1 = std::max(t1, p.t_end_ns);
    }
    setExtent(t0, t1);
  }
  update();
}

void TimeBarWidget::clearPasses()
{
  passes_.clear();
  lanes_.clear();
  hover_bar_ = -1;
  extent_t0_ = extent_t1_ = 0;
  user_adjusted_ = false;
  update();
}

void TimeBarWidget::setExtent(std::int64_t t0_ns, std::int64_t t1_ns)
{
  if (t1_ns <= t0_ns) {
    return;
  }
  extent_t0_ = t0_ns;
  extent_t1_ = t1_ns;
  if (!user_adjusted_) {
    fit_pending_ = true;   // applied at paint time, when the width is real
  }
  update();
}

void TimeBarWidget::fitExtent()
{
  if (!hasExtent()) {
    return;
  }
  const double dur_s = static_cast<double>(extent_t1_ - extent_t0_) / 1e9;
  const double w = std::max(50.0, static_cast<double>(width()) - 10.0);
  spp_ = std::clamp(dur_s * 1.1 / w, kMinSpp, kMaxSpp);
  // Stay pending: refit on each resize until the user takes the view over —
  // only re-derive the zoom here, so a scrub-tracking centre survives.
  if (center_ns_ < extent_t0_ || center_ns_ > extent_t1_) {
    center_ns_ = extent_t0_ + (extent_t1_ - extent_t0_) / 2;
  }
}

void TimeBarWidget::setCurrentTime(std::int64_t t_ns)
{
  if (dragging_tape_ || dragging_thumb_ || anim_timer_.isActive() ||
    page_timer_.isActive())
  {
    return;   // the operator's hand wins
  }
  center_ns_ = t_ns;   // recentre only; zoom (and any pending fit) unchanged
  update();
}

QRectF TimeBarWidget::tapeRect() const
{
  return QRectF(0.0, 0.0, width(), height() * kTapeFraction);
}

QRectF TimeBarWidget::scrollRect() const
{
  const double top = height() * kTapeFraction;
  return QRectF(0.0, top, width(), height() - top);
}

double TimeBarWidget::xOfTime(std::int64_t t_ns) const
{
  return width() * 0.5 +
         static_cast<double>(t_ns - center_ns_) / 1e9 / spp_;
}

std::int64_t TimeBarWidget::timeOfX(double x_px) const
{
  return center_ns_ + static_cast<std::int64_t>(
    (x_px - width() * 0.5) * spp_ * 1e9);
}

int TimeBarWidget::laneOf(const std::string & sensor_type) const
{
  for (std::size_t i = 0; i < lanes_.size(); ++i) {
    if (lanes_[i] == sensor_type) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

QRectF TimeBarWidget::passBarRect(std::size_t i) const
{
  const auto & p = passes_[i];
  const double x0 = xOfTime(p.t_start_ns);
  const double x1 = std::max(xOfTime(p.t_end_ns), x0 + 3.0);   // stay clickable
  const int lane = laneOf(p.sensor_type);
  const double bottom = tapeRect().bottom() - 1.0;
  const double y = bottom - (lane + 1) * (kPassLaneHeight + 1.0);
  return QRectF(x0, y, x1 - x0, kPassLaneHeight);
}

int TimeBarWidget::barAt(const QPointF & pos) const
{
  // Drawn-rect hit test, narrowest wins (a short pass under a long one must
  // stay reachable) — same rule as the retired gap-compressed timeline.
  int best = -1;
  double best_w = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < passes_.size(); ++i) {
    const QRectF r = passBarRect(i);
    if (!r.contains(pos)) {
      continue;
    }
    if (r.width() < best_w) {
      best_w = r.width();
      best = static_cast<int>(i);
    }
  }
  return best;
}

std::pair<double, double> TimeBarWidget::thumbSpan() const
{
  const QRectF sr = scrollRect();
  const double dur = static_cast<double>(extent_t1_ - extent_t0_);
  const std::int64_t left_ns = timeOfX(0.0);
  const std::int64_t right_ns = timeOfX(width());
  double x0 = sr.left() + sr.width() *
    static_cast<double>(left_ns - extent_t0_) / dur;
  double x1 = sr.left() + sr.width() *
    static_cast<double>(right_ns - extent_t0_) / dur;
  if (x1 - x0 < kMinThumbPx) {
    const double pad = (kMinThumbPx - (x1 - x0)) / 2.0;
    x0 -= pad;
    x1 += pad;
  }
  x0 = std::clamp(x0, sr.left(), sr.right());
  x1 = std::clamp(x1, sr.left(), sr.right());
  return {x0, x1};
}

void TimeBarWidget::paintEvent(QPaintEvent * event)
{
  Q_UNUSED(event);
  QPainter painter(this);
  if (fit_pending_ && !user_adjusted_ && width() > 0) {
    fitExtent();   // now the width is the laid-out one
  }
  const QRectF tape = tapeRect();
  const QRectF sr = scrollRect();
  painter.fillRect(tape, QColor(26, 26, 51));   // the original's dark blue

  if (!hasExtent()) {
    painter.setPen(QColor(120, 130, 140));
    painter.drawText(rect(), Qt::AlignCenter,
      "Open a bag or select index tiles to navigate time here.");
    return;
  }

  QFont big = painter.font();
  big.setPointSize(8);
  QFont small = painter.font();
  small.setPointSize(7);

  // --- TAPE ----------------------------------------------------------------
  // Tick ladder, minor levels first so major ticks/labels draw over them.
  const std::int64_t left_ns = timeOfX(0.0);
  const auto ladder = computeTickLadder(left_ns, spp_, width());
  painter.setFont(big);
  for (const auto & row : ladder) {
    for (const auto & tick : row.ticks) {
      painter.setPen(QColor(0, 230, 0));
      painter.drawLine(
        QPointF(tick.x_px, tape.bottom()),
        QPointF(tick.x_px, tape.bottom() - row.tick_frac * tape.height()));
      if (tick.label.empty()) {
        continue;
      }
      const QString label = QString::fromStdString(tick.label);
      painter.setFont(tick.minor_label ? small : big);
      if (!tick.minor_label) {
        const double lw = painter.fontMetrics().horizontalAdvance(label);
        painter.fillRect(
          QRectF(tick.x_px + 1.0, tape.top() + 2.0, lw + 5.0,
            tape.height() * 0.5),
          QColor(26, 26, 51));
      }
      painter.setPen(QColor(255, 255, 255));
      painter.drawText(
        QPointF(tick.x_px + 3.0, tape.top() + tape.height() * 0.45), label);
      painter.setFont(big);
    }
  }

  // Out-of-extent wash (translucent red), as in the original.
  const double ex0 = xOfTime(extent_t0_);
  const double ex1 = xOfTime(extent_t1_);
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(230, 77, 77, 100));
  if (ex0 > tape.left()) {
    painter.drawRect(QRectF(tape.left(), tape.top(), ex0 - tape.left(), tape.height()));
  }
  if (ex1 < tape.right()) {
    painter.drawRect(QRectF(ex1, tape.top(), tape.right() - ex1, tape.height()));
  }

  // Pass bars in per-sensor sub-lanes along the tape bottom.
  for (std::size_t i = 0; i < passes_.size(); ++i) {
    const QRectF r = passBarRect(i);
    if (r.right() < 0 || r.left() > width()) {
      continue;
    }
    QColor c = barColor(passes_[i]);
    if (static_cast<int>(i) == hover_bar_) {
      c = c.lighter(130);
    }
    painter.setPen(QPen(QColor(0, 0, 0, 120), 0.5));
    painter.setBrush(c);
    painter.drawRect(r);
  }

  // Centre cursor (the current time), blue like the original.
  painter.setPen(QPen(QColor(77, 77, 230), 3.0));
  painter.drawLine(
    QPointF(width() * 0.5, tape.top()), QPointF(width() * 0.5, tape.bottom()));

  // Current-time readout, top-right.
  painter.setFont(big);
  painter.setPen(QColor(255, 255, 255));
  const QString readout = isoUtc(center_ns_) + " UTC";
  const double rw = painter.fontMetrics().horizontalAdvance(readout);
  painter.fillRect(
    QRectF(width() - rw - 8.0, tape.top() + 1.0, rw + 8.0, 13.0), QColor(26, 26, 51));
  painter.drawText(QPointF(width() - rw - 4.0, tape.top() + 11.0), readout);

  // --- SCROLLBAR -----------------------------------------------------------
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(255, 255, 255));
  painter.drawRect(sr);
  // Its own tick scale over the whole extent (dark text on white).
  const double sb_spp = static_cast<double>(extent_t1_ - extent_t0_) / 1e9 /
    std::max(1.0, sr.width());
  painter.setFont(small);
  for (const auto & row : computeTickLadder(extent_t0_, sb_spp, sr.width())) {
    for (const auto & tick : row.ticks) {
      painter.setPen(QColor(0, 160, 0));
      painter.drawLine(
        QPointF(tick.x_px, sr.bottom()),
        QPointF(tick.x_px, sr.bottom() - row.tick_frac * sr.height()));
      if (!tick.label.empty()) {
        painter.setPen(QColor(26, 26, 51));
        painter.drawText(QPointF(tick.x_px + 2.0, sr.top() + 9.0),
          QString::fromStdString(tick.label));
      }
    }
  }
  // Pass markers on the scrollbar.
  for (const auto & p : passes_) {
    const double x0 = sr.left() + sr.width() *
      static_cast<double>(p.t_start_ns - extent_t0_) /
      static_cast<double>(extent_t1_ - extent_t0_);
    const double x1 = std::max(x0 + 1.0, sr.left() + sr.width() *
        static_cast<double>(p.t_end_ns - extent_t0_) /
        static_cast<double>(extent_t1_ - extent_t0_));
    QColor c = barColor(p);
    c.setAlpha(170);
    painter.setPen(Qt::NoPen);
    painter.setBrush(c);
    painter.drawRect(QRectF(x0, sr.top() + sr.height() * 0.55, x1 - x0,
      sr.height() * 0.4));
  }
  // Thumb (translucent green) + centre line.
  const auto [tx0, tx1] = thumbSpan();
  painter.setBrush(QColor(0, 230, 0, 128));
  painter.drawRect(QRectF(tx0, sr.top(), tx1 - tx0, sr.height()));
  painter.setPen(QPen(QColor(255, 255, 255, 180), 1.0));
  painter.drawLine(
    QPointF((tx0 + tx1) / 2.0, sr.top()), QPointF((tx0 + tx1) / 2.0, sr.bottom()));
}

void TimeBarWidget::mousePressEvent(QMouseEvent * event)
{
  if (!hasExtent()) {
    return;
  }
  const QPointF pos = event->pos();
  if (event->button() == Qt::MiddleButton && tapeRect().contains(pos)) {
    startJump(timeOfX(pos.x()));
    return;
  }
  if (event->button() != Qt::LeftButton) {
    return;
  }
  if (tapeRect().contains(pos)) {
    dragging_tape_ = true;
    drag_moved_ = false;
    drag_start_pos_ = pos;
    drag_start_center_ = center_ns_;
    drag_start_spp_ = spp_;
    return;
  }
  if (scrollRect().contains(pos)) {
    const auto [tx0, tx1] = thumbSpan();
    if (pos.x() >= tx0 && pos.x() <= tx1) {
      dragging_thumb_ = true;
      thumb_grab_x_ = pos.x();
      thumb_start_center_ = center_ns_;
    } else {
      page_direction_ = (pos.x() < tx0) ? -1 : 1;
      pageBy(page_direction_);
      page_timer_.start();
    }
  }
}

void TimeBarWidget::mouseMoveEvent(QMouseEvent * event)
{
  const QPointF pos = event->pos();
  if (dragging_tape_) {
    const double dx = pos.x() - drag_start_pos_.x();
    const double dy = std::abs(pos.y() - drag_start_pos_.y());
    // Pulling vertically past 100 px stretches the scale while panning — the
    // original's combined pan+zoom gesture.
    const double stretch = (dy > 100.0) ? 1.0 + (dy - 100.0) / 3.0 : 1.0;
    spp_ = std::clamp(drag_start_spp_ * stretch, kMinSpp, kMaxSpp);
    center_ns_ = drag_start_center_ -
      static_cast<std::int64_t>(dx * spp_ * 1e9);
    if (std::abs(dx) > 3.0 || dy > 3.0) {
      drag_moved_ = true;
      user_adjusted_ = true;
    }
    update();
    return;
  }
  if (dragging_thumb_) {
    const QRectF sr = scrollRect();
    const double per_px = static_cast<double>(extent_t1_ - extent_t0_) /
      std::max(1.0, sr.width());
    center_ns_ = thumb_start_center_ + static_cast<std::int64_t>(
      (pos.x() - thumb_grab_x_) * per_px);
    user_adjusted_ = true;
    update();
    return;
  }
  const int hit = tapeRect().contains(pos) ? barAt(pos) : -1;
  if (hit != hover_bar_) {
    hover_bar_ = hit;
    setCursor(hit >= 0 ? Qt::PointingHandCursor : Qt::ArrowCursor);
    update();
  }
  if (hover_bar_ >= 0) {
    const auto & p = passes_[static_cast<std::size_t>(hover_bar_)];
    const double dur_s = static_cast<double>(p.t_end_ns - p.t_start_ns) / 1e9;
    QToolTip::showText(event->globalPos(), QString("%1\n%2 — %3\n%4 s, %5 pings")
      .arg(QFileInfo(QString::fromStdString(p.bag_path)).fileName())
      .arg(isoUtc(p.t_start_ns)).arg(isoUtc(p.t_end_ns))
      .arg(dur_s, 0, 'f', 1).arg(p.ping_count), this);
  } else {
    QToolTip::hideText();
  }
}

void TimeBarWidget::mouseReleaseEvent(QMouseEvent * event)
{
  if (event->button() != Qt::LeftButton) {
    return;
  }
  if (dragging_tape_) {
    dragging_tape_ = false;
    if (!drag_moved_) {
      const int hit = barAt(event->pos());
      if (hit >= 0) {
        const auto & p = passes_[static_cast<std::size_t>(hit)];
        emit passActivated(
          QString::fromStdString(p.bag_path),
          static_cast<qlonglong>(p.t_start_ns),
          static_cast<qlonglong>(p.t_end_ns));
        return;
      }
    } else {
      commitTime();
    }
    return;
  }
  if (dragging_thumb_) {
    dragging_thumb_ = false;
    commitTime();
    return;
  }
  if (page_timer_.isActive()) {
    page_timer_.stop();
    commitTime();
  }
}

void TimeBarWidget::mouseDoubleClickEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton && hasExtent() &&
    tapeRect().contains(event->pos()) && barAt(event->pos()) < 0)
  {
    startJump(timeOfX(event->pos().x()));
  }
}

void TimeBarWidget::wheelEvent(QWheelEvent * event)
{
  if (!hasExtent()) {
    return;
  }
  const double steps = event->angleDelta().y() / 120.0;
  if (steps == 0.0) {
    return;
  }
  // ctrl = coarse, shift = fine, as in the original.
  double factor = 0.8;
  if (event->modifiers() & Qt::ControlModifier) {
    factor = 0.6;
  } else if (event->modifiers() & Qt::ShiftModifier) {
    factor = 0.95;
  }
  spp_ = std::clamp(
    (steps > 0.0) ? spp_ * std::pow(factor, steps) : spp_ / std::pow(factor, -steps),
    kMinSpp, kMaxSpp);
  user_adjusted_ = true;
  update();
}

void TimeBarWidget::leaveEvent(QEvent * event)
{
  Q_UNUSED(event);
  if (hover_bar_ != -1) {
    hover_bar_ = -1;
    setCursor(Qt::ArrowCursor);
    update();
  }
}

void TimeBarWidget::startJump(std::int64_t target_ns)
{
  anim_from_ns_ = center_ns_;
  anim_to_ns_ = target_ns;
  anim_progress_ = 0.0;
  user_adjusted_ = true;
  anim_timer_.start();
}

void TimeBarWidget::commitTime()
{
  emit timeSelected(static_cast<qlonglong>(center_ns_));
}

void TimeBarWidget::pageBy(int direction)
{
  center_ns_ += static_cast<std::int64_t>(direction * width() * spp_ * 1e9);
  user_adjusted_ = true;
  update();
}

}  // namespace marine_perception_tools
