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

#ifndef POINT_CLOUD_VIEW_HPP_
#define POINT_CLOUD_VIEW_HPP_

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>
#include <QVector3D>

#include <vector>

#include "mbes_geometry.hpp"

namespace marine_perception_tools
{

// A view-only 3D point-cloud viewport for the windowed MBES soundings. GeoZui-style
// orbit: left-drag rotates (azimuth + elevation), wheel zooms; a Z-exaggeration
// factor stretches depth so subtle relief reads. Points are recentred on the
// window centroid so float precision and the camera target stay well-conditioned.
// Coloured by depth (default) or backscatter via the shared marine_colormap LUT.
class PointCloudView : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
  Q_OBJECT

public:
  explicit PointCloudView(QWidget * parent = nullptr);
  ~PointCloudView() override;

  enum class ColorMode { Depth, Backscatter };

  // Replace the displayed cloud with this window's world-frame soundings. Recentres
  // on their centroid so the cloud stays in view, but preserves the operator's
  // zoom/orientation across scrubs — the camera distance is only auto-framed on the
  // first cloud after a resetView() (new bag / Fit View), not on every update.
  void setPoints(const std::vector<MbesSounding> & world_soundings);
  void clear();

  // Re-frame the camera (default orbit + auto distance) on the next setPoints. Call
  // on a new bag or from a "Fit View" action; scrubbing does not call this, so the
  // zoom set by the operator is kept.
  void resetView();

  void setColorMode(ColorMode mode);
  void setZExaggeration(float z);          // >= 1; stretches depth
  void setColorMap(int palette_index);     // marine_colormap palette index
  void setPointSize(float px);             // GL point size in pixels (>= 1)

protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;

private:
  void rebuild_colors();   // recompute the per-point colour buffer for the mode
  void upload();           // (re)upload position + colour buffers (GL-current)

  QOpenGLShaderProgram program_;
  QOpenGLVertexArrayObject vao_;
  QOpenGLBuffer pos_vbo_{QOpenGLBuffer::VertexBuffer};
  QOpenGLBuffer col_vbo_{QOpenGLBuffer::VertexBuffer};
  bool gl_ready_ = false;
  bool buffers_dirty_ = false;

  // Recentred geometry + the per-point scalars used for colouring.
  std::vector<QVector3D> pts_;     // world soundings minus centroid
  std::vector<float> depth_;       // -z (positive down) for the depth ramp
  std::vector<float> intensity_;   // backscatter dB
  std::vector<float> colors_;      // rgb triples, size == 3 * pts_.size()
  float center_x_ = 0.0f;
  float center_y_ = 0.0f;
  float center_z_ = 0.0f;
  float radius_ = 1.0f;            // bounding radius (for default camera distance)

  // Camera (orbit around the recentred origin).
  float azimuth_deg_ = 0.0f;
  float elevation_deg_ = 35.0f;
  float distance_ = 10.0f;
  float zexag_ = 1.0f;             // no vertical exaggeration by default
  float point_size_ = 2.5f;        // GL point size (pixels)
  bool framed_ = false;            // true once the camera distance has been auto-framed
  ColorMode mode_ = ColorMode::Depth;
  int palette_index_ = 0;
  QPoint last_mouse_;
};

}  // namespace marine_perception_tools

#endif  // POINT_CLOUD_VIEW_HPP_
