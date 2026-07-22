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

// Back-compat shim (#22/#24): sidescan_target_viewer grew into the survey
// explorer. The old executable name keeps working — scripts and muscle
// memory included — by exec'ing the sibling `survey_explorer` binary with
// the same arguments.

#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

int main(int argc, char ** argv)
{
  std::error_code ec;
  std::filesystem::path self = std::filesystem::read_symlink("/proc/self/exe", ec);
  if (ec && argc > 0) {
    self = std::filesystem::path(argv[0]);
  }
  const std::string target = (self.parent_path() / "survey_explorer").string();

  std::string arg0 = target;
  std::vector<char *> args;
  args.reserve(static_cast<std::size_t>(argc) + 1);
  args.push_back(arg0.data());
  for (int i = 1; i < argc; ++i) {
    args.push_back(argv[i]);
  }
  args.push_back(nullptr);
  execv(target.c_str(), args.data());
  std::perror(("sidescan_target_viewer: exec " + target).c_str());
  return 127;
}
