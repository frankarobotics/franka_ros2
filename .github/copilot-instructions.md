# Copilot Instructions — franka_ros2

> **Living document.** When you discover an important, reusable insight (e.g. a
> non-obvious build quirk, a naming convention not yet captured, a hard-won
> debugging lesson), add it to the appropriate section below. Before adding,
> re-validate existing entries so nothing becomes stale. Only record things that
> will save meaningful time in the future — skip trivia and one-off facts.
>
> **Small, surgical edits only.** Never rewrite this file wholesale. Use targeted
> insertions or replacements to add or update individual entries. This keeps diffs
> reviewable and avoids accidentally dropping existing content.

## Project Overview

ROS 2 Jazzy workspace for Franka Robotics research robots, built on `libfranka`.
Supports FR3 single-arm, FR3 Duo dual-arm, and Mobile FR3 Duo configurations.
Uses `ros2_control` for hardware abstraction and MoveIt for motion planning.

## Workspace Layout

- Source lives under `/ros2_ws/src/`
- `franka_*` packages are the project's own code
- `libfranka/` and `franka_description/` are **separate repos** with their own `AGENTS.md` — see those files for repo-specific conventions
- `ros2_control/`, `realtime_tools/`, `gz_ros2_control/`, `serial/`, `zed_description/`, `ros2_robotiq_gripper/`, `olvx_descriptions_module/` are vendored dependencies — avoid modifying these unless patching
- `dependency.repos` defines external dependency versions (imported via `vcs`)
- `patches/` contains patches applied on top of vendored deps (e.g., `manage_overruns.patch` for `ros2_control/hardware_interface`)

## Build & Test

```bash
# Source ROS 2 environment
. /opt/ros/$ROS_DISTRO/setup.sh

# Build everything
colcon build --base-paths src \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
               -DCHECK_TIDY=ON \
               -DBUILD_TESTING=ON

# Run unit tests (excludes hardware tests)
. install/setup.sh
colcon test --base-paths src \
  --packages-select-regex '^franka_(?!bringup$|gripper$)' \
  --event-handlers console_direct+ \
  --ctest-args --exclude-regex test_hardware
colcon test-result --verbose
```

- Working directory for colcon commands is `/ros2_ws` (one level above `src/`)
- Always source `install/setup.sh` before running tests
- Hardware tests require a real robot and run separately via `franka_ros2/scripts/run_hardware_tests.sh <robot_ip>`

## C++ Conventions

- **Standard**: C++20 (some packages still on C++17)
- **Style**: Chromium-based (`.clang-format`), 100-column limit, sorted includes
- **Compiler flags**: `-Wall -Wextra -Wpedantic`
- **Naming** (enforced by `.clang-tidy`):
  - Namespaces: `lower_case`
  - Classes / Structs / Enums / Typedefs: `CamelCase`
  - Functions / Methods: `camelBack`
  - Variables / Parameters: `lower_case`
  - Private member variables: `lower_case_` (trailing underscore)
  - Constants: `kCamelCase`
- **Build system**: `ament_cmake` with `pluginlib` for plugin export
- **Testing**: Google Test / Google Mock via `ament_add_gmock()`
- **Linting**: clang-format, clang-tidy, ament_cppcheck, ament_cpplint, ament_lint_cmake

## Python Conventions

- **Formatter / Linter**: Ruff (configured in `pyproject.toml`)
- **Line length**: 99
- **Quotes**: single quotes
- **Import order**: Google-style (future → stdlib → third-party), managed by `ruff lint --select I`

## Commits

All commits must include a `Signed-off-by:` line (DCO requirement). Use `git commit -s`.

## Docker / Dev Container

- Base image: `ros:jazzy-ros-base`
- Dev container config in `.devcontainer/`
- `docker-compose.yml` runs with `--network host`, real-time ulimits, and privileged mode for robot communication
- `franka_entrypoint.sh` imports dependencies via `vcs` and applies patches

## License

Apache 2.0 — see `LICENSE` and `NOTICE`.
