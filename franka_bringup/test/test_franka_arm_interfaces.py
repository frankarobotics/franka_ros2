#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

"""
Behavioral tests for single-arm (franka_arm) ros2_control interfaces.

Verifies that each supported arm type registers the correct command and state
interfaces when loaded through the ros2_control resource manager. These are the
interfaces that controllers bind to at runtime — getting them wrong means
controllers fail to activate or command the wrong DOFs.
"""

import pytest

from ros2_control_test_helpers import (
    expand_wrapper_urdf,
    get_bringup_wrapper,
    get_gpio_command_interfaces,
    get_gpio_indices,
    parse_hardware_components,
)


ARM_ROBOT_TYPES = ['fer', 'fp3', 'fr3', 'fr3v2', 'fr3v2_1']


def load_franka_arm(robot_type: str, **extra_mappings) -> list:
    """Expand franka_arm wrapper and return parsed hardware components."""
    mappings = {'robot_type': robot_type}
    mappings.update(extra_mappings)
    urdf = expand_wrapper_urdf(get_bringup_wrapper('franka_arm.urdf.xacro'), mappings)
    return parse_hardware_components(urdf)


class TestFrankaArmJointInterfaces:
    """
    Each arm type must expose 7 joints with position/velocity/effort command interfaces.

    Controllers like joint_trajectory_controller bind to these interfaces by name.
    A missing interface prevents controller activation; an extra one causes unexpected
    motion commands to uncontrolled DOFs.
    """

    @pytest.mark.parametrize('robot_type', ARM_ROBOT_TYPES)
    def test_seven_joints_with_position_velocity_effort_commands(self, robot_type):
        """Real hardware exposes position, velocity, and effort commands on all 7 joints."""
        components = load_franka_arm(robot_type, robot_ip='192.168.1.1')
        assert len(components) == 1, f'Expected 1 hardware component, got {len(components)}'

        component = components[0]
        arm_joints = [
            j for j in component.joints if f'{robot_type}_joint' in j.name
        ]
        assert len(arm_joints) == 7, (
            f'Expected 7 arm joints, got {len(arm_joints)}: '
            f'{[j.name for j in arm_joints]}'
        )

        for joint in arm_joints:
            cmd_names = {ci.name for ci in joint.command_interfaces}
            assert {'position', 'velocity', 'effort'} <= cmd_names, (
                f'{joint.name} command interfaces {cmd_names} missing expected types. '
                f'All three are needed for position/velocity/effort control modes.'
            )

    @pytest.mark.parametrize('robot_type', ARM_ROBOT_TYPES)
    def test_seven_joints_with_position_velocity_effort_states(self, robot_type):
        """Each joint exposes position, velocity, and effort state interfaces for feedback."""
        components = load_franka_arm(robot_type, robot_ip='192.168.1.1')
        component = components[0]
        arm_joints = [
            j for j in component.joints if f'{robot_type}_joint' in j.name
        ]

        for joint in arm_joints:
            state_names = {si.name for si in joint.state_interfaces}
            assert {'position', 'velocity', 'effort'} <= state_names, (
                f'{joint.name} state interfaces {state_names} missing expected types'
            )


class TestFrankaArmGpioInterfaces:
    """
    GPIO-based command interfaces for cartesian velocity, elbow, and pose.

    These map to internal vectors in FrankaHardwareInterface:
    - hw_cartesian_velocities_: 6 elements (indices 0-5)
    - hw_elbow_command_: 2 elements (indices 0-1)
    - hw_cartesian_pose_: 16 elements

    Index out-of-bounds would cause undefined behavior on real hardware.
    """

    @pytest.mark.parametrize('robot_type', ARM_ROBOT_TYPES)
    def test_six_cartesian_velocity_command_interfaces(self, robot_type):
        """6 cartesian_velocity GPIO command interfaces with indices 0-5."""
        components = load_franka_arm(robot_type, robot_ip='192.168.1.1')
        component = components[0]

        cv_gpios = get_gpio_command_interfaces(component, 'cartesian_velocity')
        assert len(cv_gpios) == 6, (
            f'Expected 6 cartesian_velocity interfaces, got {len(cv_gpios)}. '
            f'FrankaHardwareInterface::hw_cartesian_velocities_ has exactly 6 elements.'
        )
        indices = get_gpio_indices(cv_gpios)
        assert indices == list(range(6)), (
            f'cartesian_velocity indices must be [0..5], got {indices}. '
            f'Index >= 6 causes out-of-bounds access on real hardware.'
        )

    @pytest.mark.parametrize('robot_type', ARM_ROBOT_TYPES)
    def test_two_elbow_command_interfaces(self, robot_type):
        """2 elbow_command GPIO command interfaces with indices 0-1."""
        components = load_franka_arm(robot_type, robot_ip='192.168.1.1')
        component = components[0]

        elbow_gpios = get_gpio_command_interfaces(component, 'elbow_command')
        assert len(elbow_gpios) == 2, (
            f'Expected 2 elbow_command interfaces, got {len(elbow_gpios)}. '
            f'FrankaHardwareInterface::hw_elbow_command_ has exactly 2 elements.'
        )
        indices = get_gpio_indices(elbow_gpios)
        assert indices == [0, 1], (
            f'elbow_command indices must be [0, 1], got {indices}. '
            f'Index >= 2 causes out-of-bounds access on real hardware.'
        )

    @pytest.mark.parametrize('robot_type', ARM_ROBOT_TYPES)
    def test_sixteen_cartesian_pose_command_interfaces(self, robot_type):
        """16 cartesian_pose_command GPIO command interfaces (4x4 homogeneous transform)."""
        components = load_franka_arm(robot_type, robot_ip='192.168.1.1')
        component = components[0]

        pose_gpios = get_gpio_command_interfaces(component, 'cartesian_pose_command')
        assert len(pose_gpios) == 16, (
            f'Expected 16 cartesian_pose interfaces (4x4 matrix), got {len(pose_gpios)}'
        )


class TestFrankaArmEffortGating:
    """
    Effort command interface gating for simulation safety.

    Protects against sending torque commands in simulation environments that
    don't support direct effort control:
    - Real hardware: effort PRESENT (needed for torque control mode)
    - Gazebo without gazebo_effort: effort ABSENT (Gazebo position/velocity only)
    - Gazebo with gazebo_effort=true: effort PRESENT (effort-capable sim)
    """

    def test_effort_present_for_real_hardware(self):
        """Real hardware includes effort command interface on all joints."""
        components = load_franka_arm('fr3', robot_ip='192.168.1.1')
        component = components[0]
        arm_joints = [j for j in component.joints if 'fr3_joint' in j.name]

        for joint in arm_joints:
            cmd_names = {ci.name for ci in joint.command_interfaces}
            assert 'effort' in cmd_names, (
                f'{joint.name} missing effort command interface for real hardware. '
                f'Torque control mode requires effort interface.'
            )

    def test_effort_absent_in_gazebo_without_gazebo_effort(self):
        """Gazebo without gazebo_effort omits effort to prevent invalid torque commands."""
        components = load_franka_arm(
            'fr3', gazebo='true', hand='true', gazebo_effort='false'
        )
        component = components[0]
        arm_joints = [j for j in component.joints if 'fr3_joint' in j.name]

        for joint in arm_joints:
            cmd_names = {ci.name for ci in joint.command_interfaces}
            assert 'effort' not in cmd_names, (
                f'{joint.name} should NOT have effort in gazebo without gazebo_effort. '
                f'Gazebo position/velocity mode cannot handle direct torque commands.'
            )

    def test_effort_present_in_gazebo_with_gazebo_effort(self):
        """Gazebo with gazebo_effort=true enables effort for torque-capable simulation."""
        components = load_franka_arm(
            'fr3', gazebo='true', hand='true', gazebo_effort='true'
        )
        component = components[0]
        arm_joints = [j for j in component.joints if 'fr3_joint' in j.name]

        for joint in arm_joints:
            cmd_names = {ci.name for ci in joint.command_interfaces}
            assert 'effort' in cmd_names, (
                f'{joint.name} should have effort with gazebo_effort=true'
            )


if __name__ == '__main__':
    pytest.main([__file__])
