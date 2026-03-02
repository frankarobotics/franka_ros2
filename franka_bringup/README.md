# franka_bringup 

Launch file package for booting up franka robots.

## Testing

**Note:** To activate the launch file smoke tests, you need to enable the `BUILD_TESTING` CMake flag when building this package:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

### Run the tests on your hardware

```bash
# If you want to use an IP other than 172.16.0.1
colcon build --packages-select franka_bringup --cmake-args -DROBOT_IP=<robot-ip> -DBUILD_TESTING=ON
colcon test --packages-select franka_bringup --event-handlers console_direct+
# to inspect the results
colcon test-result --all --verbose
```