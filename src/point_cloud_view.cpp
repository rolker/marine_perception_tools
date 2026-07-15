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

#include "point_cloud_view.hpp"

#include <QColor>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QPointF>
#include <QString>
#include <QSurfaceFormat>
#include <QVector3D>
#include <QVector4D>
#include <QWheelEvent>
#include <QtMath>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "marine_colormap/colormap.hpp"
#include "marine_colormap/palette.hpp"
#include "marine_colormap/transfer.hpp"

namespace marine_perception_tools
{

namespace
{
constexpr char kVertexShader[] =
  R"(#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_col;
uniform mat4 u_mvp;
uniform float u_point_size;
out vec3 v_col;
void main()
{
  gl_Position = u_mvp * vec4(a_pos, 1.0);
  gl_PointSize = u_point_size;
  v_col = a_col;
}
)";

constexpr char kFragmentShader[] =
  R"(#version 330 core
in vec3 v_col;
out vec4 frag_color;
void main()
{
  frag_color = vec4(v_col, 1.0);
}
)";
}  // namespace

PointCloudView::PointCloudView(QWidget * parent)
: QOpenGLWidget(parent)
{
  setMinimumSize(256, 256);
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);   // hover reporting (hoverWorld) needs moves without a button
  // Request a 3.3 context with a depth buffer (the cloud depth-tests). Without
  // this the widget can get a default context that lacks the 3.3 entry points and
  // QOpenGLFunctions_3_3_Core::initializeOpenGLFunctions() fails (null gl* -> crash).
  QSurfaceFormat fmt = format();
  fmt.setVersion(3, 3);
  fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
  fmt.setDepthBufferSize(24);
  setFormat(fmt);
}

PointCloudView::~PointCloudView()
{
  if (gl_ready_) {
    makeCurrent();
    pos_vbo_.destroy();
    col_vbo_.destroy();
    arrow_vbo_.destroy();
    arrow_vao_.destroy();
    vao_.destroy();
    doneCurrent();
  }
}

void PointCloudView::setPoints(const std::vector<MbesSounding> & world_soundings)
{
  pts_.clear();
  depth_.clear();
  intensity_.clear();
  // Single-pass entry: every point belongs to pass 0. setMultiPassPoints
  // overwrites this with the real per-pass ids after delegating here.
  pass_of_point_.assign(world_soundings.size(), 0);
  if (world_soundings.empty()) {
    colors_.clear();
    buffers_dirty_ = true;
    update();
    return;
  }

  // Centroid (recentre so float precision + the camera target stay conditioned).
  double sx = 0.0;
  double sy = 0.0;
  double sz = 0.0;
  for (const auto & s : world_soundings) {
    sx += s.x;
    sy += s.y;
    sz += s.z;
  }
  const double n = static_cast<double>(world_soundings.size());
  center_x_ = static_cast<float>(sx / n);
  center_y_ = static_cast<float>(sy / n);
  center_z_ = static_cast<float>(sz / n);

  pts_.reserve(world_soundings.size());
  depth_.reserve(world_soundings.size());
  intensity_.reserve(world_soundings.size());
  float max_r2 = 0.0f;
  for (const auto & s : world_soundings) {
    const float rx = static_cast<float>(s.x) - center_x_;
    const float ry = static_cast<float>(s.y) - center_y_;
    const float rz = static_cast<float>(s.z) - center_z_;
    pts_.emplace_back(rx, ry, rz);
    depth_.push_back(-rz);            // positive = below the centroid (deeper)
    intensity_.push_back(s.intensity);
    max_r2 = std::max(max_r2, rx * rx + ry * ry + rz * rz);
  }
  radius_ = std::max(1.0f, std::sqrt(max_r2));
  // Frame the camera distance only once (first cloud after a resetView). On a scrub
  // the cloud is recentred but the operator's zoom/orientation is preserved.
  if (!framed_) {
    distance_ = radius_ * 2.5f;       // frame the whole cloud
    framed_ = true;
  }

  rebuild_colors();
  buffers_dirty_ = true;
  update();
}

void PointCloudView::setMultiPassPoints(
  const std::vector<std::vector<MbesSounding>> & passes)
{
  std::size_t total = 0;
  for (const auto & pass : passes) {
    total += pass.size();
  }
  std::vector<MbesSounding> all;
  std::vector<int> ids;
  all.reserve(total);
  ids.reserve(total);
  for (std::size_t i = 0; i < passes.size(); ++i) {
    all.insert(all.end(), passes[i].begin(), passes[i].end());
    ids.insert(ids.end(), passes[i].size(), static_cast<int>(i));
  }
  setPoints(all);   // recentre/frame; assigns pass 0 everywhere
  pass_of_point_ = std::move(ids);
  rebuild_colors();  // now with the real per-pass ids
  buffers_dirty_ = true;
  update();
}

void PointCloudView::clear()
{
  setPoints({});
}

void PointCloudView::resetView()
{
  // Restore the default orbit and re-arm auto-framing so the next setPoints frames
  // the cloud. Used on a new bag / a Fit View action — not on a scrub.
  azimuth_deg_ = 0.0f;
  elevation_deg_ = 35.0f;
  framed_ = false;
  if (radius_ > 0.0f) {distance_ = radius_ * 2.5f;}
  update();
}

void PointCloudView::setColorMode(ColorMode mode)
{
  if (mode_ == mode) {return;}
  mode_ = mode;
  rebuild_colors();
  buffers_dirty_ = true;
  update();
}

void PointCloudView::setZExaggeration(float z)
{
  zexag_ = std::max(1.0f, z);
  update();
}

void PointCloudView::setColorMap(int palette_index)
{
  palette_index_ = palette_index;
  rebuild_colors();
  buffers_dirty_ = true;
  update();
}

void PointCloudView::setPointSize(float px)
{
  point_size_ = std::max(1.0f, px);
  update();
}

void PointCloudView::setBoat(
  double wx, double wy, double wz, double heading_rad, bool valid)
{
  boat_valid_ = valid;
  boat_x_ = static_cast<float>(wx);
  boat_y_ = static_cast<float>(wy);
  boat_z_ = static_cast<float>(wz);
  boat_heading_ = static_cast<float>(heading_rad);
  arrow_dirty_ = true;
  update();
}

void PointCloudView::build_arrow()
{
  arrow_dirty_ = false;
  arrow_verts_ = 0;
  if (!gl_ready_ || !boat_valid_) {return;}

  // Local arrow (x forward, y left), ~2.4 m long (-1.2..1.2) x 1.0 m wide head:
  // a 0.5 m-wide shaft quad + a triangular head. Filled triangles, interleaved
  // [x, y, z, r, g, b]; orange so it reads against the cloud.
  struct V {float x; float y;};
  const std::array<V, 9> local = {{
    {-1.2f, -0.25f}, {0.3f, -0.25f}, {0.3f, 0.25f},   // shaft tri 1
    {-1.2f, -0.25f}, {0.3f, 0.25f}, {-1.2f, 0.25f},   // shaft tri 2
    {1.2f, 0.0f}, {0.3f, 0.5f}, {0.3f, -0.5f}}};      // head
  constexpr float kR = 1.0f;
  constexpr float kG = 0.55f;
  constexpr float kB = 0.0f;
  const float ch = std::cos(boat_heading_);
  const float sh = std::sin(boat_heading_);
  const float bx = boat_x_ - center_x_;
  const float by = boat_y_ - center_y_;
  const float bz = boat_z_ - center_z_;
  std::vector<float> data;
  data.reserve(local.size() * 6);
  for (const auto & p : local) {
    // forward (x) along heading; left (y) is (-sin, cos).
    const float wx = bx + p.x * ch - p.y * sh;
    const float wy = by + p.x * sh + p.y * ch;
    data.push_back(wx);
    data.push_back(wy);
    data.push_back(bz);
    data.push_back(kR);
    data.push_back(kG);
    data.push_back(kB);
  }
  arrow_vao_.bind();
  arrow_vbo_.bind();
  arrow_vbo_.allocate(data.data(), static_cast<int>(data.size() * sizeof(float)));
  program_.enableAttributeArray(0);
  program_.setAttributeBuffer(0, GL_FLOAT, 0, 3, 6 * sizeof(float));
  program_.enableAttributeArray(1);
  program_.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 3, 6 * sizeof(float));
  arrow_vao_.release();
  arrow_verts_ = static_cast<int>(local.size());
}

void PointCloudView::rebuild_colors()
{
  colors_.clear();
  if (pts_.empty()) {return;}
  if (mode_ == ColorMode::Pass) {
    // Pass identity, not a scalar ramp: one golden-angle hue per pass index.
    colors_.reserve(pts_.size() * 3);
    for (std::size_t i = 0; i < pts_.size(); ++i) {
      const int pass = (i < pass_of_point_.size()) ? pass_of_point_[i] : 0;
      float r = 1.0f;
      float g = 1.0f;
      float b = 1.0f;
      pass_color(pass, r, g, b);
      colors_.push_back(r);
      colors_.push_back(g);
      colors_.push_back(b);
    }
    return;
  }
  const std::vector<float> & scalar = (mode_ == ColorMode::Depth) ? depth_ : intensity_;

  float lo = std::numeric_limits<float>::max();
  float hi = std::numeric_limits<float>::lowest();
  for (const float v : scalar) {
    lo = std::min(lo, v);
    hi = std::max(hi, v);
  }
  const float span = (hi > lo) ? (hi - lo) : 1.0f;

  const std::size_t n_pal = marine_colormap::palette_count();
  const std::size_t idx = (n_pal > 0) ?
    static_cast<std::size_t>(std::clamp(palette_index_, 0, static_cast<int>(n_pal - 1))) : 0;
  const auto lut = marine_colormap::bake_lut(
    marine_colormap::palette(idx), marine_colormap::TransferParams{}, 256);

  colors_.reserve(pts_.size() * 3);
  for (const float v : scalar) {
    const float t = std::clamp((v - lo) / span, 0.0f, 1.0f);
    const std::size_t li = static_cast<std::size_t>(t * (lut.size() - 1) + 0.5f);
    const auto & c = lut[std::min(li, lut.size() - 1)];
    colors_.push_back(c.r / 255.0f);
    colors_.push_back(c.g / 255.0f);
    colors_.push_back(c.b / 255.0f);
  }
}

void PointCloudView::initializeGL()
{
  if (!initializeOpenGLFunctions()) {
    // No 3.3 entry points on this context — leave gl_ready_ false so paintGL is a
    // no-op rather than dereferencing null GL function pointers.
    return;
  }
  glClearColor(20.0f / 255.0f, 20.0f / 255.0f, 24.0f / 255.0f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_PROGRAM_POINT_SIZE);

  program_.addShaderFromSourceCode(QOpenGLShader::Vertex, kVertexShader);
  program_.addShaderFromSourceCode(QOpenGLShader::Fragment, kFragmentShader);
  program_.link();

  vao_.create();
  pos_vbo_.create();
  col_vbo_.create();
  arrow_vao_.create();
  arrow_vbo_.create();
  gl_ready_ = true;
  if (!pts_.empty()) {upload();}
  if (boat_valid_) {build_arrow();}
}

void PointCloudView::upload()
{
  if (!gl_ready_ || pts_.empty()) {return;}
  vao_.bind();

  pos_vbo_.bind();
  pos_vbo_.allocate(
    pts_.data(), static_cast<int>(pts_.size() * sizeof(QVector3D)));
  program_.enableAttributeArray(0);
  program_.setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(QVector3D));

  col_vbo_.bind();
  col_vbo_.allocate(
    colors_.data(), static_cast<int>(colors_.size() * sizeof(float)));
  program_.enableAttributeArray(1);
  program_.setAttributeBuffer(1, GL_FLOAT, 0, 3, 3 * sizeof(float));

  vao_.release();
  buffers_dirty_ = false;
}

void PointCloudView::resizeGL(int w, int h)
{
  glViewport(0, 0, w, std::max(1, h));
}

void PointCloudView::paintGL()
{
  if (!gl_ready_) {return;}
  // Re-assert depth state every frame. The 2D QPainter overlay (draw_overlay) leaves
  // the GL paint engine's state behind — depth test disabled, depth mask + blend
  // changed — so without this the cloud renders without depth testing after the first
  // frame, and a left-disabled depth mask would even defeat the depth clear below.
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glDisable(GL_BLEND);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (pts_.empty()) {return;}
  if (buffers_dirty_) {upload();}

  const float aspect =
    static_cast<float>(width()) / static_cast<float>(std::max(1, height()));
  QMatrix4x4 proj;
  const float far_plane = distance_ + radius_ * zexag_ * 4.0f + 10.0f;
  proj.perspective(45.0f, aspect, std::max(0.05f, distance_ * 0.01f), far_plane);

  const float az = qDegreesToRadians(azimuth_deg_);
  const float el = qDegreesToRadians(elevation_deg_);
  const QVector3D eye(
    distance_ * std::cos(el) * std::cos(az),
    distance_ * std::cos(el) * std::sin(az),
    distance_ * std::sin(el));
  QMatrix4x4 view;
  view.lookAt(eye, QVector3D(0, 0, 0), QVector3D(0, 0, 1));

  QMatrix4x4 model;
  model.scale(1.0f, 1.0f, zexag_);   // stretch depth about the centroid

  mvp_ = proj * view * model;
  program_.bind();
  program_.setUniformValue("u_mvp", mvp_);
  program_.setUniformValue("u_point_size", point_size_);
  vao_.bind();
  glDrawArrays(GL_POINTS, 0, static_cast<int>(pts_.size()));
  vao_.release();

  // Boat-context arrow (same MVP; flat in the horizontal plane at the boat z).
  if (arrow_dirty_) {build_arrow();}
  if (boat_valid_ && arrow_verts_ > 0) {
    arrow_vao_.bind();
    glDrawArrays(GL_TRIANGLES, 0, arrow_verts_);
    arrow_vao_.release();
  }
  program_.release();

  // 2D overlay (scale bar + orientation axes) on top of the GL scene. Horizontal
  // metres-per-pixel at the camera target from the perspective FOV.
  const float fovy = qDegreesToRadians(45.0f);
  const float mpp = (2.0f * distance_ * std::tan(fovy * 0.5f)) /
    static_cast<float>(std::max(1, height()));
  draw_overlay(view, mpp);
}

void PointCloudView::draw_overlay(const QMatrix4x4 & view, float metres_per_pixel)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  const int h = height();

  // --- Scale bar (bottom-left): a round length nearest ~120 px. ---
  if (metres_per_pixel > 0.0f && std::isfinite(metres_per_pixel)) {
    const double target_m = 120.0 * metres_per_pixel;
    const double mag = std::pow(10.0, std::floor(std::log10(target_m)));
    double nice = 10.0 * mag;
    for (double m : {1.0, 2.0, 5.0}) {
      if (m * mag >= target_m) {nice = m * mag; break;}
    }
    const int bar_px = static_cast<int>(nice / metres_per_pixel);
    const int x0 = 16;
    const int y0 = h - 20;
    QPen pen(QColor(235, 235, 235));
    pen.setWidth(2);
    painter.setPen(pen);
    painter.drawLine(x0, y0, x0 + bar_px, y0);
    painter.drawLine(x0, y0 - 4, x0, y0 + 4);
    painter.drawLine(x0 + bar_px, y0 - 4, x0 + bar_px, y0 + 4);
    const QString label = (nice >= 1.0) ?
      QString::number(nice, 'f', 0) + " m" : QString::number(nice, 'f', 2) + " m";
    painter.drawText(x0, y0 - 8, label);
  }

  // --- Orientation axes (top-left): project world E/N/Up into screen directions. ---
  const float ox = 34.0f;
  const float oy = 34.0f;
  const float len = 22.0f;
  struct Axis {QVector3D dir; QColor col; const char * lbl;};
  const std::array<Axis, 3> axes = {{
    {QVector3D(1, 0, 0), QColor(255, 90, 90), "E"},
    {QVector3D(0, 1, 0), QColor(90, 220, 90), "N"},
    {QVector3D(0, 0, 1), QColor(120, 160, 255), "Up"}}};
  for (const auto & a : axes) {
    const QVector3D d = view.mapVector(a.dir);   // eye-space direction (rotation only)
    const float sx = d.x();
    const float sy = -d.y();                     // screen y is down
    const float n = std::hypot(sx, sy);
    const float ux = (n > 1e-6f) ? sx / n : 0.0f;
    const float uy = (n > 1e-6f) ? sy / n : 0.0f;
    QPen pen(a.col);
    pen.setWidth(2);
    painter.setPen(pen);
    painter.drawLine(QPointF(ox, oy), QPointF(ox + ux * len, oy + uy * len));
    painter.drawText(QPointF(ox + ux * (len + 7) - 4, oy + uy * (len + 7) + 4), a.lbl);
  }

  // --- Linked cursor: project the shared map point (at the centroid depth) ---
  if (cursor_valid_) {
    const QVector3D rc(cursor_x_ - center_x_, cursor_y_ - center_y_, 0.0f);
    const QVector4D clip = mvp_ * QVector4D(rc, 1.0f);
    if (clip.w() > 1e-6f) {
      const float sx = (clip.x() / clip.w() * 0.5f + 0.5f) * static_cast<float>(width());
      const float sy =
        (1.0f - (clip.y() / clip.w() * 0.5f + 0.5f)) * static_cast<float>(height());
      QPen pen(QColor(0, 255, 255));
      pen.setWidthF(1.5);
      painter.setPen(pen);
      const double s = 7.0;
      painter.drawLine(QPointF(sx - s, sy), QPointF(sx + s, sy));
      painter.drawLine(QPointF(sx, sy - s), QPointF(sx, sy + s));
    }
  }
  painter.end();
}

void PointCloudView::setCursorWorld(double world_x, double world_y, bool valid)
{
  cursor_valid_ = valid;
  cursor_x_ = static_cast<float>(world_x);
  cursor_y_ = static_cast<float>(world_y);
  update();
}

bool PointCloudView::unproject_ground(
  const QPoint & px, double & world_x, double & world_y) const
{
  if (width() <= 0 || height() <= 0) {return false;}
  bool ok = false;
  const QMatrix4x4 inv = mvp_.inverted(&ok);
  if (!ok) {return false;}
  const float ndcx = 2.0f * static_cast<float>(px.x()) / static_cast<float>(width()) - 1.0f;
  const float ndcy = 1.0f - 2.0f * static_cast<float>(px.y()) / static_cast<float>(height());
  const QVector4D n4 = inv * QVector4D(ndcx, ndcy, -1.0f, 1.0f);
  const QVector4D f4 = inv * QVector4D(ndcx, ndcy, 1.0f, 1.0f);
  if (std::abs(n4.w()) < 1e-9f || std::abs(f4.w()) < 1e-9f) {return false;}
  const QVector3D p0 = n4.toVector3DAffine();
  const QVector3D p1 = f4.toVector3DAffine();
  const QVector3D dir = p1 - p0;
  if (std::abs(dir.z()) < 1e-9f) {return false;}  // ray parallel to the ground plane
  const float t = -p0.z() / dir.z();              // intersect the recentred z=0 plane
  const QVector3D hit = p0 + t * dir;
  world_x = static_cast<double>(hit.x()) + center_x_;
  world_y = static_cast<double>(hit.y()) + center_y_;
  return true;
}

void PointCloudView::mousePressEvent(QMouseEvent * event)
{
  if (event->button() == Qt::MiddleButton) {
    double wx = 0.0;
    double wy = 0.0;
    if (unproject_ground(event->pos(), wx, wy)) {Q_EMIT seekWorld(wx, wy);}
    return;
  }
  last_mouse_ = event->pos();
}

void PointCloudView::mouseMoveEvent(QMouseEvent * event)
{
  if (!(event->buttons() & Qt::LeftButton)) {
    // Hover: report the ground-plane pick under the cursor for the linked cursor.
    double wx = 0.0;
    double wy = 0.0;
    const bool ok = unproject_ground(event->pos(), wx, wy);
    Q_EMIT hoverWorld(wx, wy, ok);
    QOpenGLWidget::mouseMoveEvent(event);
    return;
  }
  const QPoint d = event->pos() - last_mouse_;
  last_mouse_ = event->pos();
  azimuth_deg_ -= 0.4f * static_cast<float>(d.x());
  elevation_deg_ = std::clamp(
    elevation_deg_ + 0.4f * static_cast<float>(d.y()), -89.0f, 89.0f);
  update();
}

void PointCloudView::wheelEvent(QWheelEvent * event)
{
  const float steps = static_cast<float>(event->angleDelta().y()) / 120.0f;
  distance_ *= std::pow(0.85f, steps);
  distance_ = std::clamp(distance_, 0.1f, radius_ * 50.0f + 100.0f);
  update();
}

}  // namespace marine_perception_tools
