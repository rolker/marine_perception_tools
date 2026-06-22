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

#ifndef SIDESCAN_WATERFALL_HPP_
#define SIDESCAN_WATERFALL_HPP_

#include <QImage>
#include <QWidget>

namespace marine_perception_tools
{

// The classic uncorrected (slant-range) sidescan waterfall for the current scrub
// window: each row is a ping cycle, port samples on the left (far range outward)
// and starboard on the right, intensity = backscatter, newest ping at the top
// (matching the live rqt waterfall plugin). No georeferencing or slant-to-ground
// correction — the raw display analysts read. Stretched to fill the widget.
class SidescanWaterfall : public QWidget
{
  Q_OBJECT

public:
  explicit SidescanWaterfall(QWidget * parent = nullptr);

  void setImage(const QImage & image);

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  QImage image_;
};

}  // namespace marine_perception_tools

#endif  // SIDESCAN_WATERFALL_HPP_
