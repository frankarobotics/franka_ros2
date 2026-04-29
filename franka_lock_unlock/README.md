# Franka Lock Unlock

Locking and unlocking of Franka Emika Panda joint brakes programmatically. Open or close all joint locks either from the command-line, or from any Python program.

Also supports the activation of the Franka Control Interface (FCI) and other options.

## Problem Description

While the Franka Panda is a great robot for research and industrial use cases, it lacks the option of unlocking or locking its joints from a different source other than the Franka Desk Web UI. However, this is crucial if you want to automate the entire startup and shutdown phase of the robot. That is now possible with the introduction of this package.

## Command-Line Usage

Ensure to have python3 installed.

To start the robot, run the following:

```
usage: ./franka_lock_unlock.py [-h] [-u] [-l] [-w] [-r] [-p] [-c] [-i] hostname username password

positional arguments:
  hostname          The Franka Desk IP address or hostname, for example "1.2.3.4".
  username          The Franka Desk username, usually "admin".
  password          The Franka Desk password.

optional arguments:
  -h, --help        show this help message and exit
  -u, --unlock      Unlock the brakes. Otherwise, lock them.
  -l, --relock      Relock the brakes on exit.
  -w, --wait        Wait in case the robot web UI is currently in use.
  -r, --request     Request control by confirming physical access to the robot in case the robot web UI is currently in use.
  -p, --persistent  Keep the connection to the robot open persistently.
  -c, --fci         Activate the FCI.
  -i, --home        Home the gripper.
```

To shut the robot down, run the following:

```
usage: ./franka_shutdown.py [-h] [-w] [-r] hostname username password

Shutdown the Franka Emika Panda programmatically.

positional arguments:
  hostname       The Franka Desk IP address or hostname, for example "1.2.3.4".
  username       The Franka Desk username, usually "admin".
  password       The Franka Desk password.

options:
  -h, --help     show this help message and exit
  -w, --wait     Wait in case the robot web UI is currently in use.
  -r, --request  Request control by confirming physical access to the robot in case the robot web UI is currently in use.
```

## ROS Package

This repository exports a ROS package. This is useful for starting up the robot and FCI by means of a single launch file that also spawns the `franka_ros` interface at the same time. It will speed up your entire robot workflow dramatically.

The ROS package can be installed and invoked by the following commands.

### Installation

```sh
git clone https://github.com/jk-ethz/franka_lock_unlock.git
colcon build --packages-up-to franka_lock_unlock
```

### Simple Usage

```sh
ros2 run franka_lock_unlock franka_lock_unlock <PARAMS>
ros2 run franka_lock_unlock franka_shutdown <PARAMS>
```

### Advanced Usage

If you want to launch the controller or gripper while unlocking the joints, ensure to add the `respawn=true` parameter to the node tag, such that the control node retries to connect to the robot until all joints have been fully unlocked and the FCI has been activated.

```Python
Node(
    package='franka_gripper',
    executable='franka_gripper_node',
    name='panda_gripper',
    respawn=True,
)
```

The following launch file unlocks the joints, activates the FCI and connects to the robot via ROS. Also, it allows to re-lock the brakes automatically as soon as the launch file is exited via `SIGINT` or `SIGTERM` (`CTRL+C`).

```sh
ros2 launch franka_lock_unlock start.launch.xml hostname:=$HOSTNAME_OR_IP username:=$USERNAME password:=$PASSWORD
```

If you want to shut the robot down, you can use the related launch file with

```sh
ros2 launch franka_lock_unlock shutdown.launch.xml hostname:=<HOSTNAME> username:=<USERNAME> password:=<PASSWORD>
```
