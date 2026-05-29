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

#ifndef CV_QT_HPP_
#define CV_QT_HPP_

#include <QImage>

#include <opencv2/core.hpp>

namespace marine_perception_tools
{

// Convert a CV_8UC3 image to a self-owning QImage. `bgr` true → input is in
// OpenCV's BGR order (e.g. the costmap render) and channels are swapped to RGB;
// false → input is already rgb8 (e.g. the decoded segmentation mask). Both
// branches return a deep copy that owns its pixels (the source `mat` may be
// transient), so the result is safe to hand to Qt.
inline QImage cvMatToQImage(const cv::Mat & mat, bool bgr)
{
  if (mat.empty() || mat.type() != CV_8UC3) {
    return QImage();
  }
  const QImage view(
    mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_RGB888);
  return bgr ? view.rgbSwapped() : view.copy();
}

}  // namespace marine_perception_tools

#endif  // CV_QT_HPP_
