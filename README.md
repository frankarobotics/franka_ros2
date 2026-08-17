# ROS 2 Integration for Franka Robotics Research Robots

[![CI](https://github.com/frankarobotics/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankarobotics/franka_ros2/actions/workflows/ci.yml)

> **Note:** _franka_ros2_ is not officially supported on Windows.

## Table of Contents
- [About](#about)
- [Caution](#caution)
- [Setup](#setup)
  - [Local Machine Installation](#local-machine-installation)
  - [Docker Container Installation](#docker-container-installation)
- [Test the Setup](#test-the-setup)
- [Troubleshooting](#troubleshooting)
  - [libfranka: UDP receive: Timeout error](#libfranka-udp-receive-timeout-error)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## About
The **franka_ros2** repository provides a **ROS 2** integration of **libfranka**, allowing efficient control of the Franka Robotics arm within the ROS 2 framework. This project is designed to facilitate robotic research and development by providing a robust interface for controlling the research versions of Franka Robotics robots.

For convenience, this repository includes a `Dockerfile` and `docker-compose.yml`.
While it is possible to build **franka_ros2** directly on your local machine, that
approach requires manual installation of some dependencies, while many others are
installed automatically by the **ROS 2** build system (for example through
**rosdep**). This can install a large number of libraries on your system and may
cause conflicts. Docker keeps these dependencies inside the container and provides
a more consistent, reproducible build environment. For most users, we recommend
using Docker.

## Caution
This package is in rapid development. Users should expect breaking changes and are encouraged to report any bugs via [GitHub Issues page](https://github.com/frankarobotics/franka_ros2/issues).

## Setup

## Franka ROS 2 Dependencies Setup

This repository contains a `.repos` file that helps you clone the required dependencies for Franka ROS 2.

## Prerequisites

## Local Machine Installation
1. **Install ROS 2 Development environment**

    _**franka_ros2**_ is built upon _**ROS 2 jazzy**_.

    To set up your ROS 2 environment, follow the official _**jazzy**_ installation instructions provided [**here**](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
    The guide discusses two main installation options: **Desktop** and **Bare Bones**.

    ### Choose **one** of the following:
    - **ROS 2 "Desktop Install"** (`ros-jazzy-desktop`)
      Includes a full ROS 2 installation with GUI tools and visualization packages (e.g., Rviz and Gazebo).
      **Recommended** for users who need simulation or visualization capabilities.

    - **"ROS-Base Install (Bare Bones)"** (`ros-jazzy-ros-base`)
      A minimal installation that includes only the core ROS 2 libraries.
      Suitable for resource-constrained environments or headless systems.

    ```bash
    sudo apt install ros-jazzy-desktop
    # or, for a minimal installation:
    sudo apt install ros-jazzy-ros-base
    ```
    ---
    Also install the **Development Tools** package:
    ```bash
    sudo apt install ros-dev-tools
    ```
    Installing the **Desktop** or **Bare Bones** should automatically source the **ROS 2** environment but, under some circumstances you may need to do this again:
    ```bash
    source /opt/ros/jazzy/setup.sh
    ```

2. **Create a ROS 2 Workspace:**
   ```bash
   mkdir -p ~/franka_ros2_ws/src
   cd ~/franka_ros2_ws  # not into src
   ```
3. **Clone the Repositories:**
   ```bash
    git clone -b jazzy https://github.com/frankarobotics/franka_ros2.git src
    ```
4. **Install the dependencies**
    ```bash
    vcs import src < src/dependency.repos --recursive --skip-existing
    ```
5. **Detect and install project dependencies**
   ```bash
   rosdep install --from-paths src --ignore-src --rosdistro jazzy -y --skip-keys=zed_wrapper
   ```
6. **Build**
   ```bash
   # use the --symlinks option to reduce disk usage, and facilitate development.
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
   ```
7. **Adjust Environment**
   ```bash
   # Adjust environment to recognize packages and dependencies in your newly built ROS 2 workspace.
   source install/setup.sh
   ```

## Docker Container Installation
This repository includes a `Dockerfile` and `docker-compose.yml` so you can use
the `franka_ros2` packages without installing **ROS 2** directly on your host
system. Support for Dev Containers in Visual Studio Code is also provided.

For detailed instructions on preparing VS Code to use `.devcontainer`, follow the
[VS Code Dev Containers tutorial](https://code.visualstudio.com/docs/devcontainers/tutorial).

1. **Clone the Repositories:**
    ```bash
    git clone -b jazzy https://github.com/frankarobotics/franka_ros2.git
    cd franka_ros2
    ```
    We provide separate instructions for using Docker from the command line or
    from Visual Studio Code. Choose one of the following options:

    Option A: Set up and use Docker from the command line (without Visual Studio Code).

    Option B: Set up and use Docker with Visual Studio Code's Docker support.

### Option A: using Docker Compose

  2. **Create a `.env` file for Docker Compose:**
      ```bash
      printf "USER_UID=%s\nUSER_GID=%s\nCONTAINER_NAME=%s_jazzy\n" "$(id -u)" "$(id -g)" "$(basename "$PWD")" > .env
      ```
      The Compose file reads `USER_UID`, `USER_GID`, and `CONTAINER_NAME` from
      this file.

  3. **Build the container:**
      ```bash
      docker compose build
      ```
  4. **Run the container:**
      ```bash
      docker compose up -d
      ```
  5. **Open a shell inside the container:**
      ```bash
      docker compose exec franka_ros2_jazzy bash
      ```
  6. **Import the workspace dependencies:**
      ```bash
      vcs import src < src/dependency.repos --recursive --skip-existing
      ```
  7. **Build the workspace:**
      ```bash
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
      ```
  8. **Source the built workspace:**
      ```bash
      source install/setup.bash
      ```
  9. **When you are done, stop and remove the container:**
      ```bash
      docker compose down -t 0
      ```

### Option B: using Dev Containers in Visual Studio Code

  2. **Open Visual Studio Code.**

        Then open the `franka_ros2` folder.

  3. **Choose `Reopen in container` when prompted.**

      The container will be built automatically, as required.

  4. **Import the workspace dependencies:**
      ```bash
      vcs import src < src/dependency.repos --recursive --skip-existing
      ```

  5. **Open a terminal and build the workspace:**
      ```bash
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
      ```
  6. **Source the built workspace environment:**
      ```bash
      source install/setup.bash
      ```


## Test the build
   ```bash
   colcon test
   ```
> Remember, franka_ros2 is under development.
> Warnings can be expected.

## Test the Setup

### Run a sample ROS 2 application

To verify that your setup works correctly without a robot, you can run the following command to use dummy hardware:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```
The same `franka_fr3_moveit_config moveit.launch.py` launch file also accepts
`load_gripper`, `ee_id`, and `namespace`. Use `load_gripper` to enable or disable
the end effector, and `ee_id` to select which end effector to use. By default,
the Franka Hand is enabled.

If you want to run this example with namespaces, pass the `namespace` launch
argument and then set the same namespace in `moveit.rviz` under **Move Group
Namespace**.

### Run a ROS 2 example controller

To run any example controller, make sure to add your desired configuration in `franka.config.yaml` and run:

```bash
ros2 launch franka_bringup example.launch.py controller_names:=your_desired_controller
```
You can select one of the controllers from `controllers.yaml`.

### Run different controllers for different robots

If you want to run a specific controller for each robot, you must specify the controllers you want to run as follows (example for three robots):

```bash
ros2 launch franka_bringup example.launch.py controller_names:="cartesian_elbow_example_controller,joint_impedance_example_controller,cartesian_velocity_example_controller"
```
If less controllers than the number of robots are specified, only the first controller would be used for all the robots. TMR controllers can also be used.

### Run FR3 Duo

For FR3 Duo setups, use the `fr3_duo.launch.py` launch file with the `fr3_duo.config.yaml` configuration:

```bash
ros2 launch franka_bringup fr3_duo.launch.py \
  controller_name:=fr3_duo_joint_impedance_example_controller
```

**Note:** The FR3 Duo setup supports only **one controller** at a time and uses the `controller_name` parameter (singular). The dual-arm setup currently only supports the torque (effort) command interface.

### Run Mobile FR3 Duo

For Mobile FR3 Duo setups (TMRv0.2 mobile base with dual FR3 arms), use the `mobile_fr3_duo.launch.py` launch file with the `mobile_fr3_duo.config.yaml` configuration:

```bash
ros2 launch franka_bringup mobile_fr3_duo.launch.py \
  controller_name:=mobile_fr3_duo_joint_impedance_example_controller
```

**Note:** Like the FR3 Duo setup, the Mobile FR3 Duo supports only **one controller** at a time. The mobile base velocity control is integrated within the controller.

### Move the TMRv0.2

Be sure to configure your robot IP in `franka_bringup/config/tmr.config.yaml` and optionally change the robot namespace.

You can move the TMRv0.2 either:

- By using a remote XBOX controller:

```bash
ros2 launch franka_bringup mobile_teleop.launch.py
```

This launch file spawns the required additional nodes for remote control.

- By using the PC keyboard:

Launch on one terminal:
```bash
ros2 launch franka_bringup example.launch.py controller_names:="swerve_drive_controller" robot_config_file:="tmr.config.yaml"
```
On another terminal launch:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true --remap /cmd_vel:=/swerve_drive_controller/cmd_vel
```

### Run Gazebo examples with ROS 2

If you want to use Gazebo to run your code, you can find examples in
[franka_gazebo_bringup](./franka_gazebo/franka_gazebo_bringup/doc/index.rst).

## Troubleshooting
### `libfranka: UDP receive: Timeout error`

If you encounter a UDP receive timeout error while communicating with the robot, avoid using Docker Desktop. It may not provide the necessary real-time capabilities required for reliable communication with the robot. Instead, using Docker Engine is sufficient for this purpose.

A real-time kernel is essential to ensure proper communication and to prevent timeout issues. For guidance on setting up a real-time kernel, please refer to the [Franka installation documentation](https://frankarobotics.github.io/docs/doc/libfranka/docs/real_time_kernel.html#setting-up-the-real-time-kernel).

## Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](https://github.com/frankarobotics/franka_ros2/blob/jazzy/CONTRIBUTING.md) for more details on how to contribute to this project.

## License

All packages of franka_ros2 are licensed under the Apache 2.0 license.

## Contact

For questions or support, please open an issue on the [GitHub Issues](https://github.com/frankarobotics/franka_ros2/issues) page.

See the [Franka Control Interface (FCI) documentation](https://frankarobotics.github.io/docs) for more information.

[def]: #docker-container-installation
