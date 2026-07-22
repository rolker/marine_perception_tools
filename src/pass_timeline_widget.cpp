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

#include "pass_timeline_widget.hpp"

#include <QColor>
#include <QDateTime>
#include <QFileInfo>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QTimeZone>
#include <QToolTip>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "point_cloud_view.hpp"   // pass_color (shared with the cloud legend)

namespace marine_perception_tools
{

namespace
{

// Layout metrics (px).
constexpr double kLabelMargin = 96.0;   // lane labels left of the axis
constexpr double kRightMargin = 6.0;
constexpr double kLaneHeight = 16.0;
constexpr double kLaneGap = 3.0;
constexpr double kAxisHeight = 14.0;    // span time labels under the lanes
constexpr double kTopMargin = 3.0;

// Coalesced passes across adjacent tiles sit back-to-back; pad the intervals
// so one transit's segments merge into one covered span (same 5 s scale the
// pass coalescing uses).
constexpr std::int64_t kSpanPadNs = 5LL * 1000000000LL;

// Lane order: the cloud's sensor first, then sidescan port/starboard; any
// other sensor types append in the order encountered.
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
  return QColor(140, 150, 160);   // not in the cloud (e.g. sidescan): neutral
}

}  // namespace

PassTimelineWidget::PassTimelineWidget(QWidget * parent)
: QWidget(parent)
{
  setMouseTracking(true);   // hover highlight + tooltips without a button
}

void PassTimelineWidget::setPasses(std::vector<TimelinePassInfo> passes)
{
  infos_ = std::move(passes);
  hover_ = -1;

  // Lanes: preferred sensors first (only if present), then any others.
  lanes_.clear();
  for (const char * sensor : kPreferredLanes) {
    if (std::any_of(infos_.begin(), infos_.end(),
      [sensor](const TimelinePassInfo & p) {return p.sensor_type == sensor;}))
    {
      lanes_.emplace_back(sensor);
    }
  }
  for (const auto & info : infos_) {
    if (laneOf(info.sensor_type) < 0) {
      lanes_.push_back(info.sensor_type);
    }
  }

  model_passes_.clear();
  model_passes_.reserve(infos_.size());
  for (const auto & info : infos_) {
    model_passes_.push_back(TimelinePass{
        info.t_start_ns, info.t_end_ns, laneOf(info.sensor_type)});
  }
  layout_ = layoutTimeline(model_passes_, kSpanPadNs);
  updateGeometry();
  update();
}

void PassTimelineWidget::clearPasses()
{
  infos_.clear();
  model_passes_.clear();
  layout_ = TimelineLayout{};
  lanes_.clear();
  hover_ = -1;
  updateGeometry();
  update();
}

QSize PassTimelineWidget::sizeHint() const
{
  const int n_lanes = std::max<std::size_t>(1, lanes_.size());
  const int h = static_cast<int>(kTopMargin + kAxisHeight +
    n_lanes * (kLaneHeight + kLaneGap));
  return QSize(400, h);
}

int PassTimelineWidget::laneOf(const std::string & sensor_type) const
{
  for (std::size_t i = 0; i < lanes_.size(); ++i) {
    if (lanes_[i] == sensor_type) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

QRectF PassTimelineWidget::barRect(std::size_t i) const
{
  const auto & [x0, x1] = layout_.pass_x[i];
  const double axis_w = std::max(1.0, width() - kLabelMargin - kRightMargin);
  const double px0 = kLabelMargin + x0 * axis_w;
  // A short pass must stay clickable: floor the bar at 3 px.
  const double px1 = std::max(kLabelMargin + x1 * axis_w, px0 + 3.0);
  const double y = kTopMargin +
    model_passes_[i].row * (kLaneHeight + kLaneGap);
  return QRectF(px0, y, px1 - px0, kLaneHeight);
}

int PassTimelineWidget::barAt(const QPointF & pos) const
{
  // Hit-test against the drawn rects, not the raw axis fractions: barRect
  // floors a short pass's bar at 3 px, and a fraction-space test (hitPass)
  // would leave most of that visible bar dead — a zero-duration pass would
  // be un-clickable entirely. Narrowest-wins keeps a small bar reachable
  // under a big one, same rule as the pure model's hitPass.
  int best = -1;
  double best_w = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < infos_.size() && i < layout_.pass_x.size(); ++i) {
    const QRectF r = barRect(i);
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

void PassTimelineWidget::paintEvent(QPaintEvent * event)
{
  Q_UNUSED(event);
  QPainter painter(this);
  painter.fillRect(rect(), QColor(24, 28, 32));
  if (infos_.empty()) {
    painter.setPen(QColor(120, 130, 140));
    painter.drawText(rect(), Qt::AlignCenter,
      "Select index tiles to see their passes here.");
    return;
  }

  QFont small = painter.font();
  small.setPointSize(7);
  painter.setFont(small);

  // Lane labels.
  painter.setPen(QColor(170, 180, 190));
  for (std::size_t lane = 0; lane < lanes_.size(); ++lane) {
    const double y = kTopMargin + lane * (kLaneHeight + kLaneGap);
    painter.drawText(
      QRectF(2.0, y, kLabelMargin - 6.0, kLaneHeight),
      Qt::AlignVCenter | Qt::AlignRight,
      QString::fromStdString(lanes_[lane]));
  }

  const double axis_w = std::max(1.0, width() - kLabelMargin - kRightMargin);
  const double lanes_bottom = kTopMargin + lanes_.size() * (kLaneHeight + kLaneGap);

  // Compressed-gap break markers: a dashed vertical pair between spans.
  QPen break_pen(QColor(90, 100, 110));
  break_pen.setStyle(Qt::DashLine);
  for (std::size_t s = 1; s < layout_.spans.size(); ++s) {
    const double gx0 = kLabelMargin + layout_.spans[s - 1].x1 * axis_w;
    const double gx1 = kLabelMargin + layout_.spans[s].x0 * axis_w;
    painter.setPen(break_pen);
    painter.drawLine(QPointF(gx0, kTopMargin), QPointF(gx0, lanes_bottom));
    painter.drawLine(QPointF(gx1, kTopMargin), QPointF(gx1, lanes_bottom));
  }

  // Span start labels along the bottom axis, skipping overlaps.
  painter.setPen(QColor(150, 160, 170));
  double last_label_end = -1.0;
  for (const auto & span : layout_.spans) {
    const QString label = QDateTime::fromMSecsSinceEpoch(
      span.t0 / 1000000LL, QTimeZone::utc()).toString("MM-dd HH:mm");
    const double x = kLabelMargin + span.x0 * axis_w;
    const double w = painter.fontMetrics().horizontalAdvance(label) + 6.0;
    if (x < last_label_end) {
      continue;
    }
    painter.drawText(QPointF(x, lanes_bottom + kAxisHeight - 3.0), label);
    last_label_end = x + w;
  }

  // Bars, hovered one on top with a highlight border.
  for (std::size_t i = 0; i < infos_.size(); ++i) {
    if (static_cast<int>(i) == hover_) {
      continue;
    }
    painter.setPen(QPen(QColor(0, 0, 0, 120), 0.5));
    painter.setBrush(barColor(infos_[i]));
    painter.drawRect(barRect(i));
  }
  if (hover_ >= 0 && hover_ < static_cast<int>(infos_.size())) {
    painter.setPen(QPen(QColor(255, 255, 255), 1.2));
    painter.setBrush(barColor(infos_[hover_]).lighter(120));
    painter.drawRect(barRect(hover_));
  }
}

void PassTimelineWidget::mousePressEvent(QMouseEvent * event)
{
  if (event->button() != Qt::LeftButton || infos_.empty()) {
    return;
  }
  const int hit = barAt(event->pos());
  if (hit < 0) {
    return;
  }
  const auto & info = infos_[static_cast<std::size_t>(hit)];
  emit passActivated(
    QString::fromStdString(info.bag_path),
    static_cast<qlonglong>(info.t_start_ns),
    static_cast<qlonglong>(info.t_end_ns));
}

void PassTimelineWidget::mouseMoveEvent(QMouseEvent * event)
{
  if (infos_.empty()) {
    return;
  }
  const int hit = barAt(event->pos());
  if (hit != hover_) {
    hover_ = hit;
    setCursor(hover_ >= 0 ? Qt::PointingHandCursor : Qt::ArrowCursor);
    update();
  }
  if (hover_ >= 0) {
    const auto & info = infos_[static_cast<std::size_t>(hover_)];
    const double dur_s =
      static_cast<double>(info.t_end_ns - info.t_start_ns) / 1e9;
    QToolTip::showText(event->globalPos(), QString("%1\n%2 — %3\n%4 s, %5 pings")
      .arg(QFileInfo(QString::fromStdString(info.bag_path)).fileName())
      .arg(isoUtc(info.t_start_ns)).arg(isoUtc(info.t_end_ns))
      .arg(dur_s, 0, 'f', 1).arg(info.ping_count), this);
  } else {
    QToolTip::hideText();
  }
}

void PassTimelineWidget::leaveEvent(QEvent * event)
{
  Q_UNUSED(event);
  if (hover_ != -1) {
    hover_ = -1;
    setCursor(Qt::ArrowCursor);
    update();
  }
}

}  // namespace marine_perception_tools
