---
name: audit-docs
description: 'Audit and fix documentation drift in franka_ros2 packages. Use when: docs outdated, check documentation, audit docs, update docs, documentation drift, stale docs, verify README, doc folder, package docs not matching code.'
---

# Audit Documentation

## When to Use

- After major refactors (renamed packages, changed launch args, added/removed controllers)
- When a user suspects docs are outdated or incorrect
- Periodic hygiene check to keep docs accurate
- After adding new features that should be documented

## Documentation Locations

| Location | Content | Packages |
|----------|---------|----------|
| `<pkg>/doc/index.rst` | Package user documentation (Sphinx RST) | Most `franka_*` packages |
| `<pkg>/README.md` | Package overview, quick-start | Some packages |
| `src/README.md` | Top-level workspace guide | Root |
| `src/.github/copilot-instructions.md` | AI agent context, conventions | Workspace-wide |
| Launch file docstrings | Launch argument descriptions | `franka_bringup`, `franka_gazebo_bringup` |
| Config YAML comments | Parameter explanations | `franka_bringup/config/` |

### Packages with `doc/` folders (franka-owned)

- `franka_bringup/doc/`
- `franka_hardware/doc/`
- `franka_gripper/doc/`
- `franka_mobile/doc/`
- `franka_mobile_fr3_duo_moveit_config/doc/`
- `franka_mobile_sensors/doc/`
- `franka_vision_and_manipulation_kit/doc/`
- `franka_ros2/doc/` (compatibility matrix)

## Audit Procedure

### 1. Identify scope

Ask the user: full audit (all packages) or specific packages?

### 2. Cross-reference checks

For each package in scope, perform these checks:

#### Launch arguments vs docs

```
Source of truth: *.launch.py (DeclareLaunchArgument calls)
Docs: doc/index.rst, README.md
```

- Extract all `DeclareLaunchArgument` names and descriptions from launch files
- Compare against documented launch arguments in `doc/index.rst`
- Flag any missing, renamed, or removed arguments

#### Controller names vs docs

```
Source of truth: *_plugin.xml, controllers.yaml, CMakeLists.txt (pluginlib_export)
Docs: doc/index.rst, README.md examples
```

- Extract registered controller plugin names
- Check that documented controller names match actual registrations
- Verify example commands in docs use valid controller names

#### Parameters vs docs

```
Source of truth: generate_parameter_library YAML, or rclcpp declare_parameter() calls
Docs: doc/index.rst, config YAML comments
```

- Extract declared parameters from source
- Compare against documented parameter tables/lists

#### Package dependencies and interfaces

```
Source of truth: package.xml, CMakeLists.txt
Docs: doc/index.rst "Package Overview" sections
```

- Check that listed interfaces (state/command) match actual exports
- Verify dependency lists if documented

#### Commands and examples

```
Source of truth: actual CLI behavior
Docs: doc/index.rst, README.md code blocks
```

- Verify documented `ros2 launch`, `ros2 control` commands use correct syntax
- Check package/executable names still exist
- Verify config file paths referenced in docs exist

### 3. Report findings

Present a summary table:

```
| Package | File | Issue | Severity |
|---------|------|-------|----------|
| franka_bringup | doc/index.rst:L25 | Launch arg 'arm_id' removed but still documented | High |
| franka_hardware | doc/index.rst:L10 | Still references 'panda' naming | Medium |
```

Severity levels:
- **High**: Documented command/parameter will fail if followed
- **Medium**: Inaccurate description but won't cause errors
- **Low**: Style/formatting issues, minor wording

### 4. Propose fixes

For each finding, propose a specific edit. **Ask the user before applying changes.**

Group fixes by package for easier review.

## Quick Audit Mode

For a fast check on a single package:

1. Read the package's `doc/index.rst` or `README.md`
2. Read its launch files and source for parameter/interface declarations
3. Diff documented vs actual — report discrepancies

## Deep Audit Mode

For a full workspace audit:

1. Iterate over all `franka_*` packages with `doc/` folders
2. Run all cross-reference checks above
3. Also check `src/README.md` and `copilot-instructions.md` for accuracy
4. Produce a consolidated report

## Common Drift Patterns

- **Renamed launch arguments** — old name in docs, new name in code
- **Removed controllers** — docs still show examples with deleted controllers
- **Changed default values** — doc says default is X but code now defaults to Y
- **Deprecated interfaces** — doc references `panda/` prefix but code uses `fr3/`
- **Missing new features** — new controller/parameter added but never documented
- **Dead links** — RST cross-references to moved/renamed files
- **Missing changelog entry** — code changed but CHANGELOG.rst not updated

## Changelog Verification

The workspace changelog lives at `src/CHANGELOG.rst` and uses this format:

```rst
UNRELEASED
----------
Requires libfranka >= X.Y.Z and franka_description >= X.Y.Z requires ROS 2 Jazzy

* <type>: <description>

v3.3.0 (2026-05-04)
-------------------
Requires libfranka >= X.Y.Z and franka_description >= X.Y.Z requires ROS 2 Jazzy

* <type>: <description>
```

**RST underline rule:** The underline (`---`) must be **exactly** as long as the title
text it underlines (measured in characters). This is the project convention; RST only
requires "at least as long" but we match exactly for consistency.

New entries always go under the `UNRELEASED` section at the top. Released versions
(those with a date) are frozen and must not be modified.

**Verify the `Requires` line:** When adding entries to `UNRELEASED`, check that the
`Requires libfranka` and `franka_description` versions are still correct. Cross-reference
against `package.xml` or `dependency.repos` — if a dependency version was bumped as part
of the current task, update the `Requires` line accordingly.

Entry types: `feat`, `fix`, `refactor`, `chore`, `docu`, `BREAKING CHANGE`

### Check for current task

A "task" corresponds to the current git branch. One changelog entry per branch is expected.

1. Get the current branch: `git rev-parse --abbrev-ref HEAD`
2. Infer the task type and scope from the branch name (e.g., `feat/add-gripper-support` → `feat: ...`)
3. Read the top (latest version) section of `src/CHANGELOG.rst`
4. Check if there is already an entry that covers this branch's work in the `UNRELEASED` section
5. If missing, **ask the user** whether to add one
6. If adding, insert a new bullet under the `UNRELEASED` heading

Common branch naming patterns:
- `<JIRA-ID>-<description>` (e.g., `PRCUN-6298-collision-topic-to-best-effort`) — infer the type from the description or ask the user
- `feat/<description>` → `feat: <description>`
- `fix/<description>` → `fix: <description>`
- `refactor/<description>` → `refactor: <description>`
- `chore/<description>` → `chore: <description>`
- If the branch name doesn't follow a convention, ask the user for the entry type

### Rules

- One changelog entry per branch/task (not per commit or per file edit)
- Every user-facing change (feat, fix, breaking) **must** have a changelog entry
- Internal refactors (`chore`, `refactor`) should have entries for non-trivial changes
- Multi-line entries use 2-space indentation for continuation lines
- BREAKING CHANGE entries should include migration examples in RST code blocks
- Branches like `main`, `jazzy`, `develop` are integration branches — skip the check
