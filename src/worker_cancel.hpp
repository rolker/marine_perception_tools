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

#ifndef WORKER_CANCEL_HPP_
#define WORKER_CANCEL_HPP_

#include <atomic>
#include <memory>

namespace marine_perception_tools
{

/// Cancel the background job currently holding `token`, and hand back a fresh
/// token for the job replacing it.
///
/// TWO MEANINGS, TWO TOKENS (#42 review). A teardown token says "this window is
/// going away" and must never be reset, so it cannot also carry "this job was
/// superseded" — a reset would tell a closing window's workers to keep going.
/// Supersede is this function: the old token is set so the abandoned worker
/// stops reading its bag, and the caller captures the returned one.
///
/// Without it, a superseded load ran to completion with its result thrown away.
/// That is not merely wasted work: each abandoned job keeps its whole pass set
/// resident, and occupies a slot in the GLOBAL QThreadPool that the scrub
/// render, the basemap loader and the next real job all queue behind. Holding a
/// spin box's arrow is enough to start one per valueChanged.
///
/// The worker captures the returned shared_ptr BY VALUE, so it keeps the token
/// alive and reads the same flag the next supersede sets.
inline std::shared_ptr<std::atomic<bool>> supersede_token(
  std::shared_ptr<std::atomic<bool>> & token)
{
  if (token) {
    token->store(true);
  }
  token = std::make_shared<std::atomic<bool>>(false);
  return token;
}

}  // namespace marine_perception_tools

#endif  // WORKER_CANCEL_HPP_
