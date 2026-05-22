---
name: backport-to-humble
description: 'Backport a Jazzy branch to ROS 2 Humble. Use when: backport, port to humble, humble branch, create humble version, ROS 2 Humble port, cherry-pick to humble, adapt for humble.'
---

# Backport to Humble

## When to Use

- User wants to port the current Jazzy branch to ROS 2 Humble
- Creating a Humble-compatible version of a feature branch
- Adapting controllers, hardware interfaces, or launch files for Humble

## Architecture

The project uses **separate git worktrees** on the host for Jazzy and Humble:

| Container | Host path | Mount | Base branch |
|-----------|-----------|-------|-------------|
| `franka_ros2_jazzy` | `~/git/jazzy/franka_ros2` | `/ros2_ws/src` | `origin/jazzy` |
| `franka_ros2_humble` | `~/git/humble/franka_ros2` | `/ros2_ws/src` | `origin/humble` |

Because they are separate directories with independent git state, switching branches
in one container does **not** affect the other. The backport workflow uses the Jazzy
container to prepare changes, pushes to the remote, then uses `docker exec` into the
Humble container to fetch, checkout, build, and test.

## Branch Naming

The backport branch is created from the **current branch name** with a `-humble` suffix.

Example: if on `PRCUN-1234-my-feature`, the backport branch is `PRCUN-1234-my-feature-humble`.

## Procedure

### 0. Pre-flight: Humble Container Detection

Before starting, verify the Humble container is available:

```bash
# Check if container is running
if docker ps --format '{{.Names}}' | grep -q '^franka_ros2_humble$'; then
  echo "✓ Humble container is running"
# Check if container exists but is stopped
elif docker ps -a --format '{{.Names}}' | grep -q '^franka_ros2_humble$'; then
  echo "⚠ Humble container exists but is stopped — starting it"
  docker start franka_ros2_humble
# Check if image exists
elif docker images --format '{{.Repository}}' | grep -q 'franka_ros2.*humble'; then
  echo "⚠ Humble image exists but no container — inform user to build the dev container"
  echo "  User should open the humble worktree in VS Code and 'Reopen in Container'"
  exit 1
else
  echo "✗ No Humble container or image found."
  echo "  The user must first build the Humble dev container:"
  echo "  1. Clone the repo into a separate directory (e.g., ~/git/humble/franka_ros2)"
  echo "  2. Checkout the 'humble' branch"
  echo "  3. Open in VS Code and 'Reopen in Container'"
  exit 1
fi
```

### 0b. Sanity Checks on Humble Container

```bash
# Check current branch in the Humble container
HUMBLE_BRANCH=$(docker exec franka_ros2_humble bash -c "cd /ros2_ws/src && git branch --show-current")
HUMBLE_STATUS=$(docker exec franka_ros2_humble bash -c "cd /ros2_ws/src && git status --porcelain")

echo "Humble container is on branch: $HUMBLE_BRANCH"

if [[ -n "$HUMBLE_STATUS" ]]; then
  echo "⚠ Humble worktree has uncommitted changes:"
  echo "$HUMBLE_STATUS"
  echo "  Ask user whether to proceed or clean up first"
fi

# Check if already on the target backport branch
CURRENT_BRANCH=$(git branch --show-current)
TARGET_HUMBLE_BRANCH="${CURRENT_BRANCH}-humble"
if [[ "$HUMBLE_BRANCH" == "$TARGET_HUMBLE_BRANCH" ]]; then
  echo "⚠ Humble container is already on '$TARGET_HUMBLE_BRANCH'"
  echo "  Ask user whether to reset/continue from current state"
fi
```

### 1. Determine Scope

The entire branch (all commits vs `origin/jazzy`) needs to be backported. List the
commits and affected packages:

```bash
cd /ros2_ws/src
# Commits to backport
git log --oneline origin/jazzy..HEAD

# Affected packages
git diff --name-only origin/jazzy...HEAD | cut -d'/' -f1 | sort -u
```

**Important:** Verify that the affected files/packages exist on the `humble` branch.
If they don't, the entire feature needs porting (not just this branch's delta):

```bash
# Check which changed files exist on humble
for file in $(git diff --name-only origin/jazzy...HEAD); do
  docker exec franka_ros2_humble bash -c "test -f /ros2_ws/src/$file" && echo "✓ $file" || echo "✗ $file (MISSING on humble)"
done
```

If critical files are missing, inform the user that a full feature port is needed,
not just a backport of this branch's changes.

### 2. Create the Backport Branch and Cherry-pick (in Humble container)

The backport branch is created in the **Humble worktree** based off `origin/humble`.
Then cherry-pick all commits from the Jazzy feature branch:

```bash
CURRENT_BRANCH=$(git branch --show-current)
TARGET_HUMBLE_BRANCH="${CURRENT_BRANCH}-humble"

# Get the list of commits to cherry-pick (oldest first)
COMMITS=$(git rev-list --reverse origin/jazzy..HEAD)

docker exec franka_ros2_humble bash -c "
  cd /ros2_ws/src &&
  git fetch origin &&
  git checkout -b '$TARGET_HUMBLE_BRANCH' origin/humble
"

# Cherry-pick each commit
for commit in $COMMITS; do
  docker exec franka_ros2_humble bash -c "
    cd /ros2_ws/src && git cherry-pick $commit
  "
  if [[ $? -ne 0 ]]; then
    echo "⚠ Cherry-pick of $commit failed — resolve conflicts"
    # Show conflicts
    docker exec franka_ros2_humble bash -c "cd /ros2_ws/src && git diff --name-only --diff-filter=U"
    # See § Conflict Resolution below
    break
  fi
done
```

**Best case:** Cherry-picks apply cleanly — no further work needed, skip to step 5.

**If conflicts occur:** See §3 for resolution strategies.

### 3. Conflict & Incompatibility Resolution

If cherry-picks apply cleanly, **skip to step 5**. Otherwise, resolve issues:

#### Merge conflict resolution

```bash
# View conflicting files
docker exec franka_ros2_humble bash -c "cd /ros2_ws/src && git diff --name-only --diff-filter=U"

# Read a conflicting file to understand the conflict
docker exec franka_ros2_humble cat /ros2_ws/src/path/to/conflicted_file.cpp

# After resolving, mark resolved and continue
docker exec franka_ros2_humble bash -c "
  cd /ros2_ws/src &&
  git add -A &&
  git cherry-pick --continue
"
```

#### API incompatibility resolution

When the cherry-picked code uses Jazzy-only APIs, apply the following transformations:

#### C++ / rclcpp

| Jazzy | Humble | Notes |
|-------|--------|-------|
| `#include <rclcpp/event_handler.hpp>` | Remove or replace | Not available in Humble |
| `node->get_node_clock_interface()` | Same | Available in both |
| `CallbackReturn::SUCCESS` | `CallbackReturn::SUCCESS` | Same enum, different header path |
| `hardware_interface::HW_IF_POSITION` | Same | Available in both |
| C++20 features (`std::format`, designated initializers, `<ranges>`) | Replace with C++17 equivalents | Humble uses C++17 |
| `auto` in lambda params | Use explicit types | C++17 compat |
| `std::span` | `std::vector` or pointer+size | Not in C++17 stdlib |

#### hardware_interface changes

| Jazzy | Humble |
|-------|--------|
| `on_init(const HardwareInfo&)` return `CallbackReturn` | Same signature |
| `export_state_interfaces()` returns `vector<StateInterface::SharedPtr>` | Returns `vector<StateInterface>` |
| `export_command_interfaces()` returns `vector<CommandInterface::SharedPtr>` | Returns `vector<CommandInterface>` |
| State/Command interfaces use `SharedPtr` | Interfaces returned by value |

#### ros2_control / controller_interface

| Jazzy | Humble |
|-------|--------|
| `controller_interface::return_type::OK` | `controller_interface::return_type::OK` |
| `on_configure()` signature with `rclcpp_lifecycle::State&` | Same |
| URDF passed via `hardware_info` | Same |
| `get_state()` on lifecycle node | `get_current_state()` in some contexts |

#### package.xml

| Jazzy | Humble |
|-------|--------|
| `<build_type>ament_cmake</build_type>` | Same |
| Dependency versions may differ | Pin to Humble-compatible versions |
| `ros-jazzy-*` runtime deps | Change to `ros-humble-*` |

#### CMakeLists.txt

| Jazzy | Humble |
|-------|--------|
| `CMAKE_CXX_STANDARD 20` | `CMAKE_CXX_STANDARD 17` |
| `find_package(generate_parameter_library)` | Same (if backported) or inline params |
| `pluginlib_export_plugin_description_file()` | Same |

#### Launch files (Python)

| Jazzy | Humble |
|-------|--------|
| `LaunchConfiguration` | Same |
| `PathJoinSubstitution` | Same |
| Package names unchanged | Verify with `ros2 pkg list` in Humble |

### 4. Apply Manual Fixes (if needed after conflict resolution)

For each affected file that required API adaptation, apply changes directly in the
Humble worktree using `docker exec`:

```bash
# C++ standard downgrade
docker exec franka_ros2_humble bash -c "
  cd /ros2_ws/src &&
  find . -name 'CMakeLists.txt' -path '*/franka_*' \
    -exec sed -i 's/CMAKE_CXX_STANDARD 20/CMAKE_CXX_STANDARD 17/g' {} \;
"
```

**Important:** Many transformations require semantic understanding, not just text
replacement. For complex changes, read the file from the Humble container, apply
edits, and write back:

```bash
# Read a file from Humble container
docker exec franka_ros2_humble cat /ros2_ws/src/path/to/file.cpp

# Write changes (use heredoc or pipe)
docker exec -i franka_ros2_humble tee /ros2_ws/src/path/to/file.cpp < local_modified_file.cpp
```

Review each file individually for:
- C++20 features (concepts, ranges, three-way comparison, designated initializers)
- `std::format` → `fmt::format` or `std::stringstream`
- Structured bindings in contexts that differ
- `hardware_interface` shared pointer vs value returns

### 5. Validate in Humble Container

Build the affected packages in the Humble container:

```bash
CURRENT_BRANCH=$(git branch --show-current)
# Determine packages to test (from Jazzy diff)
PACKAGES=$(git diff --name-only origin/jazzy...HEAD | cut -d'/' -f1 | sort -u | grep '^franka_' | tr '\n' ' ')

# Build in Humble container
docker exec franka_ros2_humble bash -c "
  cd /ros2_ws &&
  . /opt/ros/humble/setup.sh &&
  colcon build --base-paths src \
    --packages-select $PACKAGES \
    --cmake-args -DCMAKE_CXX_STANDARD=17 -DBUILD_TESTING=ON \
    --event-handlers console_direct+
"
```

If dependencies are missing, install them:

```bash
docker exec franka_ros2_humble bash -c "
  cd /ros2_ws &&
  . /opt/ros/humble/setup.sh &&
  sudo apt-get update &&
  rosdep install --from-paths src --ignore-src -r -y
"
```

### 6. Run Tests in Humble Container

```bash
docker exec franka_ros2_humble bash -c "
  cd /ros2_ws &&
  . /opt/ros/humble/setup.sh &&
  . install/setup.sh &&
  colcon test --base-paths src \
    --packages-select $PACKAGES \
    --event-handlers console_direct+ &&
  colcon test-result --verbose
"
```

### 7. Update Changelog

Follow the changelog rules from `.github/skills/audit-docs/SKILL.md` §Changelog
Verification. Key rules:

- New entries **must** go under the `UNRELEASED` section — never modify released
  versions (those with a date like `v2.4.0 (2026-05-04)`)
- If no `UNRELEASED` section exists, create one above the latest released version
- One entry per branch/task

The Jazzy branch likely already has changelog entries that won't apply cleanly to the
Humble `CHANGELOG.rst` (different version headings, different existing entries). Rather
than cherry-picking changelog changes, **add a fresh entry directly** in the Humble
container under `UNRELEASED`.

```bash
# Check if UNRELEASED section exists
docker exec franka_ros2_humble bash -c "head -15 /ros2_ws/src/CHANGELOG.rst"

# If UNRELEASED section exists, insert entry after its "Requires ..." line:
docker exec franka_ros2_humble bash -c "
  cd /ros2_ws/src &&
  sed -i '/^Requires.*Humble$/a * feat: <short description of the backported feature>' CHANGELOG.rst
"

# If NO UNRELEASED section exists, create one above the first version heading:
docker exec franka_ros2_humble bash -c "
  cd /ros2_ws/src &&
  sed -i '/^\^\^\^\^\^\^/a\\
\\
UNRELEASED\\
----------\\
Requires libfranka >= X.Y.Z and franka_description >= X.Y.Z requires ROS 2 Humble\\
\\
* feat: <short description of the backported feature>' CHANGELOG.rst
"
```

The entry should follow the existing format: `* <type>: <description>` where type is
one of `feat`, `fix`, `refactor`, `chore`, `breaking change`, `docu`.

**If cherry-pick already conflicted on CHANGELOG.rst:** This is expected — the Jazzy
changelog has different version headings (e.g., `v3.x`) and entries that don't belong
on Humble. Resolve by accepting the Humble side (`--ours`) and adding a fresh entry
under `UNRELEASED`:

```bash
docker exec franka_ros2_humble bash -c "
  cd /ros2_ws/src &&
  git checkout --ours CHANGELOG.rst &&
  git add CHANGELOG.rst
"
# Then add the UNRELEASED entry as shown above
```

### 8. Commit and Push

```bash
docker exec franka_ros2_humble bash -c "
  cd /ros2_ws/src &&
  git add -A &&
  git commit -s -m 'Backport ${CURRENT_BRANCH} to Humble

Adapted from Jazzy branch with the following changes:
- C++ standard downgraded from C++20 to C++17
- hardware_interface API adapted for Humble (value returns vs SharedPtr)
- [list other specific changes]
' &&
  git push origin '${CURRENT_BRANCH}-humble'
"
```

## Troubleshooting

### Humble container not found

If the `franka_ros2_humble` container doesn't exist, the user needs to:
1. Ensure the Humble worktree exists on the host (e.g., `~/git/humble/franka_ros2`)
2. Open that folder in VS Code
3. Use "Reopen in Container" to build and start the Humble dev container

### Missing dependencies in Humble container

Some packages may not exist in Humble. Check alternatives:

```bash
docker exec franka_ros2_humble bash -c \
  "apt-cache search ros-humble | grep <package_keyword>"
```

### generate_parameter_library not available

If the package uses `generate_parameter_library` and it's not available in Humble, you need to either:
1. Add it as a vendored dependency
2. Replace with manual `declare_parameter()` calls

### Hardware interface SharedPtr errors

This is the most common backport issue. In Humble, `export_state_interfaces()` and `export_command_interfaces()` return vectors of interfaces by value, not `SharedPtr`. The fix involves:

```cpp
// Jazzy:
std::vector<hardware_interface::StateInterface::SharedPtr> export_state_interfaces() override;

// Humble:
std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
```

And constructing them differently:
```cpp
// Jazzy:
state_interfaces.push_back(std::make_shared<StateInterface>(...));

// Humble:
state_interfaces.emplace_back(StateInterface(...));
```

### Compilation errors from C++20 features

Common fixes:
- `std::format(...)` → use `fmt::format(...)` (add `fmt` dependency) or `std::ostringstream`
- `auto` lambda parameters → explicit types
- `std::span<T>` → `T*` + size, or `std::vector<T>`
- Designated initializers `{.field = val}` → constructor or assignment
- `<=>` (spaceship operator) → manual comparison operators
- `requires` clauses → SFINAE or remove constraints
- `std::ranges::*` → manual loops or `<algorithm>` equivalents
