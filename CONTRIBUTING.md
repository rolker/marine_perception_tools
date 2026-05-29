# Contributing to marine_perception_tools

This repo is developed as part of the
[ROS 2 Agent Workspace](https://github.com/rolker/ros2_agent_workspace). When
checked out inside that workspace, follow the workflow rules in the workspace
`AGENTS.md` (worktree isolation, branch naming, AI signatures, pre-commit
hooks). See [`.agents/README.md`](.agents/README.md) for the per-repo agent
guide.

## Quick rules

- All changes land via Pull Requests against the `jazzy` default branch.
- Run pre-commit hooks before committing (`pre-commit run --all-files`); never
  bypass with `--no-verify`.
- New source files carry the Apache-2.0 copyright header (see existing files in
  `src/`).
- One logical change per commit.

## License

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~
