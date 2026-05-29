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

#ifndef MAIN_WINDOW_HPP_
#define MAIN_WINDOW_HPP_

#include <QMainWindow>

namespace marine_perception_tools
{

// Top-level window for the sea_surface_tuner. A placeholder shell at this
// stage; the bag-replay timeline, camera/segmentation/costmap panes, and
// parameter widgets described in marine_perception_tools#1 are layered in by
// the MVP that follows this skeleton.
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget * parent = nullptr);
};

}  // namespace marine_perception_tools

#endif  // MAIN_WINDOW_HPP_
