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

// supersede_token(): the mechanism that makes a REPLACED background job stop
// (#42). Before it, cancellation covered teardown only, so a superseded cloud
// or CUBE or drape load ran its whole bag read to completion with the result
// discarded — while holding its pass set in memory and a slot in the global
// thread pool.

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <vector>

#include "worker_cancel.hpp"

using marine_perception_tools::supersede_token;

// The first dispatch has nothing to cancel and must not fault on a null token.
TEST(WorkerCancel, FirstDispatchStartsUncancelled)
{
  std::shared_ptr<std::atomic<bool>> token;   // no job has run yet
  const auto first = supersede_token(token);

  ASSERT_NE(nullptr, first);
  EXPECT_FALSE(first->load());
  EXPECT_EQ(first, token);
}

// THE REGRESSION. Dispatching again must cancel the job already in flight, and
// hand the new job a token of its own that is NOT already cancelled.
TEST(WorkerCancel, SupersedingCancelsThePreviousJobOnly)
{
  std::shared_ptr<std::atomic<bool>> token;
  const auto first = supersede_token(token);
  const auto second = supersede_token(token);

  EXPECT_TRUE(first->load()) << "the superseded job was never told to stop";
  EXPECT_FALSE(second->load()) << "the replacement job started cancelled";
  EXPECT_NE(first, second) << "both jobs share one flag";
  EXPECT_EQ(second, token);
}

// The worker holds its token by value, so it keeps reading the same flag after
// the member has moved on — that is what lets a later supersede reach it.
TEST(WorkerCancel, AWorkerHoldingItsTokenStillSeesItsOwnCancel)
{
  std::shared_ptr<std::atomic<bool>> token;
  const auto held_by_worker = supersede_token(token);
  ASSERT_FALSE(held_by_worker->load());

  supersede_token(token);   // a newer job replaces it

  EXPECT_TRUE(held_by_worker->load());
  EXPECT_EQ(1, held_by_worker.use_count()) << "only the worker's copy remains";
}

// Rapid re-dispatch — the spin-box-held-down case — cancels every abandoned
// job, not just the most recent one.
TEST(WorkerCancel, EveryAbandonedJobIsCancelledNotJustTheLast)
{
  std::shared_ptr<std::atomic<bool>> token;
  std::vector<std::shared_ptr<std::atomic<bool>>> issued;
  for (int i = 0; i < 20; ++i) {
    issued.push_back(supersede_token(token));
  }

  for (std::size_t i = 0; i + 1 < issued.size(); ++i) {
    EXPECT_TRUE(issued[i]->load()) << "job " << i << " was left running";
  }
  EXPECT_FALSE(issued.back()->load());
}

// A token already cancelled by teardown is REPLACED by a supersede, not
// preserved — which is why the dispatchers check the teardown token
// (`worker_cancel_`, which never resets) rather than relying on this one. The
// asymmetry is deliberate and pinned here so nobody 'fixes' it by making
// supersede sticky: a sticky token would hand every later job a cancelled flag
// and the window would silently stop working after any close that was
// cancelled.
TEST(WorkerCancel, SupersedingReplacesEvenATokenTeardownAlreadySet)
{
  std::shared_ptr<std::atomic<bool>> token;
  const auto live = supersede_token(token);
  token->store(true);   // what cancelWorkers() does

  const auto next = supersede_token(token);

  EXPECT_TRUE(live->load()) << "the job running at teardown must still stop";
  EXPECT_FALSE(next->load());
}
