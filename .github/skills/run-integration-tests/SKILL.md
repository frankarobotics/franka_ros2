---
name: run-integration-tests
description: 'Run integration tests for franka_ros2 workspace. Use when: running tests, running integration tests, testing controllers, running gazebo tests, running hardware tests, verifying controllers work, CI testing, test failures, colcon test.'
---

# Run Integration Tests

## When to Use

- User asks to run tests or integration tests
- Verifying controller changes work end-to-end
- Debugging test failures
- Running simulation-based validation before hardware deployment

## Test Tiers

| Tier | Packages | Robot Required | Gazebo Required |
|------|----------|---------------|-----------------|
| Unit tests | `franka_*` (excl. bringup, gripper) | No | No |
| Gazebo integration | `gz_ros2_control_tests` | No | Yes |
| Franka Gazebo integration | `franka_gazebo_bringup` | No | Yes |
| Hardware tests | `franka_bringup`, `franka_gripper` | Yes | No |

## Procedure

### 1. Pre-flight Checks

Ensure the workspace is built with testing enabled:

```bash
cd /ros2_ws
colcon build --base-paths src \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
               -DBUILD_TESTING=ON
```

Source the workspace before running any tests:

```bash
. install/setup.sh
```

### 2. Unit Tests (No Hardware, No Gazebo)

Run all franka unit tests excluding hardware-dependent ones:

```bash
colcon test --base-paths src \
  --packages-select-regex '^franka_(?!bringup$|gripper$)' \
  --event-handlers console_direct+ \
  --ctest-args --exclude-regex test_hardware
colcon test-result --verbose
```

To test a single package:

```bash
colcon test --base-paths src \
  --packages-select <package_name> \
  --event-handlers console_direct+
colcon test-result --verbose
```

### 3. Gazebo Integration Tests (No Hardware)

These launch controllers in Gazebo headless and verify topics/states.

**gz_ros2_control tests** (position, velocity, effort, FT sensor):

```bash
colcon test --packages-select gz_ros2_control_tests \
  --event-handlers console_direct+
colcon test-result --verbose
```

**Franka Gazebo bringup tests** (full controller launch in simulation):

```bash
colcon test --packages-select franka_gazebo_bringup \
  --event-handlers console_direct+
colcon test-result --verbose
```

### 4. Hardware Tests (Requires Real Robot)

Hardware tests need the robot IP. They unlock joints, activate FCI, and run controllers on real hardware.

```bash
src/franka_ros2/scripts/run_hardware_tests.sh <robot_ip>
```

Or manually via colcon:

```bash
colcon test --base-paths src \
  --packages-select franka_bringup \
  --event-handlers console_direct+ \
  --ctest-args --tests-regex test_hardware
colcon test-result --verbose
```

### 5. Inspect Results

Always check results after any test run:

```bash
colcon test-result --verbose
```

XML results are in: `build/**/test_results/**/*.xml`

## Troubleshooting

### Gazebo processes not cleaned up

If Gazebo tests fail because of leftover processes:

```bash
pkill -9 -f '^gz sim'
pkill -9 -f 'ruby.*gz'
pkill -9 -f 'controller_manager'
pkill -9 -f 'robot_state_publisher'
```

### Communication constraint violations (hardware tests)

The hardware test script has built-in retry logic for transient communication constraint violations. If tests fail with this error, they will automatically retry once.

### Test timeout

Gazebo tests default to 50-600s timeout depending on the test. If running on slow hardware, consider building with optimizations or running fewer tests at once.

### Tests not found

Ensure you built with `-DBUILD_TESTING=ON`. Without this flag, test targets are not generated.
