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

#include <QMouseEvent>
#include <QSurfaceFormat>
#include <QWheelEvent>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <limits>

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
    vao_.destroy();
    doneCurrent();
  }
}

void PointCloudView::setPoints(const std::vector<MbesSounding> & world_soundings)
{
  pts_.clear();
  depth_.clear();
  intensity_.clear();
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

void PointCloudView::rebuild_colors()
{
  colors_.clear();
  if (pts_.empty()) {return;}
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
  gl_ready_ = true;
  if (!pts_.empty()) {upload();}
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

  program_.bind();
  program_.setUniformValue("u_mvp", proj * view * model);
  program_.setUniformValue("u_point_size", point_size_);
  vao_.bind();
  glDrawArrays(GL_POINTS, 0, static_cast<int>(pts_.size()));
  vao_.release();
  program_.release();
}

void PointCloudView::mousePressEvent(QMouseEvent * event)
{
  last_mouse_ = event->pos();
}

void PointCloudView::mouseMoveEvent(QMouseEvent * event)
{
  if (!(event->buttons() & Qt::LeftButton)) {
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
