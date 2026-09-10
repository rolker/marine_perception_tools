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

#ifndef MAP_GESTURE_HPP_
#define MAP_GESTURE_HPP_

// Click-versus-drag on the index map (#42).
//
// Both mouse buttons carry two meanings that are only told apart by whether
// the pointer travelled: left click clears the region while left drag draws
// it, and middle click centres the view while middle drag pans. Getting the
// threshold wrong silently swaps a destructive action for a constructive one
// — a shaky hand clearing a region it meant to draw — so the rule lives here,
// Qt-free and tested, rather than as a bare literal in an event handler.

#include <cstdlib>

namespace marine_perception_tools
{

/// Manhattan pixels the pointer may travel and still count as a click.
/// Matches the tolerance the contact-marking drag has always used.
constexpr int kClickSlopPx = 4;

/// True when a press-release pair is a click rather than a drag.
/// Deliberately inclusive at the threshold: exactly kClickSlopPx of travel is
/// still a click, so the boundary favours the gesture the operator aimed at
/// over an accidental one-pixel drag.
inline bool gestureIsClick(int dx, int dy, int slop_px = kClickSlopPx)
{
  return std::abs(dx) + std::abs(dy) <= slop_px;
}

}  // namespace marine_perception_tools

#endif  // MAP_GESTURE_HPP_
