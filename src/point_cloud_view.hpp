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
#include <QTimer>
#include <QVector3D>

#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "mbes_geometry.hpp"

namespace marine_perception_tools
{

// Colour for a pass index in ColorMode::Pass: golden-angle hue steps (137.508°)
// maximise perceptual separation for the first ~10 passes; constant saturation/
// value keep the colours vivid. Pure (no Qt/GL) — pinned in test_point_cloud_view.
inline void pass_color(int pass_index, float & r, float & g, float & b)
{
  const float h = std::fmod(static_cast<float>(pass_index) * 137.508f, 360.0f);
  constexpr float s = 0.85f;
  constexpr float v = 0.9f;
  const float c = v * s;
  const float hp = h / 60.0f;
  const float x = c * (1.0f - std::abs(std::fmod(hp, 2.0f) - 1.0f));
  float r1 = 0.0f;
  float g1 = 0.0f;
  float b1 = 0.0f;
  switch (static_cast<int>(hp)) {
    case 0: r1 = c; g1 = x; break;
    case 1: r1 = x; g1 = c; break;
    case 2: g1 = c; b1 = x; break;
    case 3: g1 = x; b1 = c; break;
    case 4: r1 = x; b1 = c; break;
    default: r1 = c; b1 = x; break;
  }
  const float m = v - c;
  r = r1 + m;
  g = g1 + m;
  b = b1 + m;
}

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

  enum class ColorMode { Depth, Backscatter, Pass };

  // Replace the displayed cloud with this window's world-frame soundings. Recentres
  // on their centroid so the cloud stays in view, but preserves the operator's
  // zoom/orientation across scrubs — the camera distance is only auto-framed on the
  // first cloud after a resetView() (new bag / Fit View), not on every update.
  void setPoints(const std::vector<MbesSounding> & world_soundings);

  // Replace the cloud with several passes' soundings at once (all in ONE common
  // world frame — the caller reprojects first, #21). In ColorMode::Pass each
  // pass keeps a distinct golden-angle hue (pass_color) so repeat-pass
  // agreement/disagreement over a spot is visible.
  void setMultiPassPoints(const std::vector<std::vector<MbesSounding>> & passes);
  void clear();

  // How many passes the current cloud was loaded as: 0 for a setPoints cloud
  // (one undifferentiated set of points), otherwise the setMultiPassPoints
  // count — including empty passes, whose legend rows still exist. The colour
  // vocabulary reads this to say whether ColorMode::Pass has anything to
  // distinguish (#36).
  int passCount() const {return pass_count_;}

  // Re-frame the camera (default orbit + auto distance) on the next setPoints. Call
  // on a new bag or from a "Fit View" action; scrubbing does not call this, so the
  // zoom set by the operator is kept.
  void resetView();

  void setColorMode(ColorMode mode);

  // 1 = full density; N > 1 = the last upload exceeded the render budget and
  // shows every Nth sounding (see set_points_impl) — surface it, never cap
  // silently.
  int decimationStride() const {return decimation_stride_;}
  void setZExaggeration(float z);          // >= 1; stretches depth
  void setColorMap(int palette_index);     // marine_colormap palette index
  // Manual colour range for the scalar modes (Depth/Backscatter), in the
  // active scalar's units; nullopt (default) auto-scales to the data extent.
  void setScalarRange(const std::optional<std::pair<float, float>> & range);

  // CUBE surface layer (#27): a triangulated heightmap in the SAME world
  // frame as the current points (it recentres by the cloud's centroid, so
  // set points from the same load first). xyz/rgb triples + triangle indices.
  void setSurface(
    std::vector<float> positions_xyz, std::vector<float> colors_rgb,
    std::vector<std::uint32_t> indices);
  void clearSurface();
  void setSurfaceVisible(bool on);
  void setSurfaceAlpha(float alpha);   // clamped to [0.05, 1]
  // Hide/show the point cloud itself (e.g. to view the CUBE surface alone).
  void setPointsVisible(bool on);
  void setPointSize(float px);             // GL point size in pixels (>= 1)

  // Place a forward-pointing boat arrow (~2.4 m x 1 m) at a world position for
  // 3D context. `heading_rad` is the ENU heading (CCW from +x/east). valid=false
  // hides it. World coords (recentred internally like the cloud).
  void setBoat(
    double world_x, double world_y, double world_z, double heading_rad,
    bool valid);

  // Cross-pane linked cursor: draw a cross at a world map point (drawn at the cloud
  // centroid depth). valid=false clears it.
  void setCursorWorld(double world_x, double world_y, bool valid);

Q_SIGNALS:
  // Hovered / middle-clicked world position from a ground-plane pick under the
  // mouse (for the linked cursor + click-to-seek). hoverWorld valid=false when the
  // pick fails (e.g. the view ray is parallel to the ground plane).
  void hoverWorld(double world_x, double world_y, bool valid);
  void seekWorld(double world_x, double world_y);

protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;

private:
  // Shared body of setPoints/setMultiPassPoints: recentre, frame, colour —
  // one colour rebuild with the final per-point pass ids.
  void set_points_impl(
    const std::vector<MbesSounding> & world_soundings, std::vector<int> pass_ids);
  void rebuild_colors();   // recompute the per-point colour buffer for the mode
  void upload();           // (re)upload position + colour buffers (GL-current)
  void upload_surface();   // (re)upload the CUBE surface mesh (GL-current)
  // Nearest point (recentred coords) within a pixel radius of `px`, closest
  // to the camera on a tie — the middle-click examine pick.
  bool pick_point(const QPoint & px, QVector3D & out) const;
  void build_arrow();      // (re)build the boat-arrow vertices (GL-current)
  // Draw the 2D overlay (scale bar + E/N/Up orientation axes + linked cursor).
  void draw_overlay(const QMatrix4x4 & view, float metres_per_pixel);
  // Pick a world (x,y) on the cloud-centroid ground plane under a widget pixel.
  bool unproject_ground(const QPoint & px, double & world_x, double & world_y) const;

  QOpenGLShaderProgram program_;
  QOpenGLVertexArrayObject vao_;
  QOpenGLBuffer pos_vbo_{QOpenGLBuffer::VertexBuffer};
  QOpenGLBuffer col_vbo_{QOpenGLBuffer::VertexBuffer};
  // Boat-arrow overlay (interleaved pos+col, drawn as triangles with the same program).
  QOpenGLVertexArrayObject arrow_vao_;
  QOpenGLBuffer arrow_vbo_{QOpenGLBuffer::VertexBuffer};
  int arrow_verts_ = 0;
  bool arrow_dirty_ = false;
  bool boat_valid_ = false;
  float boat_x_ = 0.0f;
  float boat_y_ = 0.0f;
  float boat_z_ = 0.0f;
  float boat_heading_ = 0.0f;
  bool gl_ready_ = false;
  bool buffers_dirty_ = false;

  // CUBE surface layer (#27).
  QOpenGLVertexArrayObject surface_vao_;
  QOpenGLBuffer surface_pos_vbo_{QOpenGLBuffer::VertexBuffer};
  QOpenGLBuffer surface_col_vbo_{QOpenGLBuffer::VertexBuffer};
  QOpenGLBuffer surface_ibo_{QOpenGLBuffer::IndexBuffer};
  std::vector<float> surface_pos_;      // world frame; recentred at upload
  std::vector<float> surface_col_;
  std::vector<std::uint32_t> surface_idx_;
  int surface_index_count_ = 0;
  bool surface_dirty_ = false;
  bool surface_visible_ = true;
  float surface_alpha_ = 1.0f;
  bool points_visible_ = true;

  // GeoZui4D-style examine pivot (#27 follow-up): middle-click animates the
  // picked point to the view centre and the orbit rotates about it. Stored
  // UNSCALED in recentred coordinates; z-exaggeration applies at view time
  // so a later Zx change keeps the pivot on the point.
  QVector3D pivot_{0.0f, 0.0f, 0.0f};
  QTimer pivot_timer_;
  QVector3D pivot_from_;
  QVector3D pivot_to_;
  float pivot_progress_ = 1.0f;

  // Recentred geometry + the per-point scalars used for colouring.
  std::vector<QVector3D> pts_;     // world soundings minus centroid
  std::vector<float> depth_;       // -z (positive down) for the depth ramp
  std::vector<float> intensity_;   // backscatter dB
  std::vector<int> pass_of_point_;  // pass index per point (ColorMode::Pass)
  int pass_count_ = 0;              // passes in the last multi-pass load (0 = none)
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

  QMatrix4x4 mvp_;                  // last frame's MVP, for project/unproject (cursor)
  bool cursor_valid_ = false;      // linked-cursor world point present
  float cursor_x_ = 0.0f;
  float cursor_y_ = 0.0f;
  ColorMode mode_ = ColorMode::Depth;
  std::optional<std::pair<float, float>> scalar_range_;   // manual colour range
  int decimation_stride_ = 1;
  int palette_index_ = 0;
  QPoint last_mouse_;
};

}  // namespace marine_perception_tools

#endif  // POINT_CLOUD_VIEW_HPP_
