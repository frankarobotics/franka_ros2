#  Copyright (c) 2026 Franka Robotics GmbH
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
Behavioral tests for fr3_duo (dual-arm) ros2_control interfaces.

The fr3_duo system registers TWO independent FrankaHardwareInterface instances —
one per physical arm. Each instance maps GPIO indices into fixed-size internal
vectors (6 elements for cartesian_velocity, 2 for elbow_command). Collapsing
both arms into a single hardware component would push indices beyond vector
bounds (e.g. 6-11), causing undefined behavior on real hardware.

These tests verify that the system exposes the correct interface topology:
two independent hardware components, each with its own complete interface set.
"""

import pytest

from ros2_control_test_helpers import (
    expand_wrapper_urdf,
    get_arm_components,
    get_bringup_wrapper,
    get_gpio_command_interfaces,
    get_gpio_indices,
    parse_hardware_components,
)


def load_fr3_duo(**extra_mappings) -> list:
    """Expand fr3_duo wrapper and return parsed hardware components."""
    mappings = {'use_fake_hardware': 'true'}
    mappings.update(extra_mappings)
    urdf = expand_wrapper_urdf(get_bringup_wrapper('fr3_duo.urdf.xacro'), mappings)
    return parse_hardware_components(urdf)


class TestFr3DuoHardwareTopology:
    """
    Two hardware components => two independent FrankaHardwareInterface instances.

    A real fr3_duo has two robot IPs and two independent control loops. Merging
    them into one component would break the 1:1 mapping between hardware instances
    and physical robots.
    """

    def test_exactly_two_hardware_components(self):
        """fr3_duo registers exactly 2 hardware components (one per arm)."""
        components = load_fr3_duo()
        assert len(components) == 2, (
            f'Expected 2 hardware components (one per arm), got {len(components)}. '
            f'Each arm needs its own FrankaHardwareInterface for independent control.'
        )

    def test_component_names_identify_left_and_right_arms(self):
        """Components are named left_/right_FrankaHardwareInterface for unambiguous binding."""
        components = load_fr3_duo()
        names = {c.name for c in components}
        assert names == {'left_FrankaHardwareInterface', 'right_FrankaHardwareInterface'}, (
            f'Expected left/right FrankaHardwareInterface, got {names}'
        )

    def test_real_hardware_has_distinct_robot_ips(self):
        """
        Each arm's hardware component connects to a different physical robot.

        Structural guard: robot_ip isn't a registered interface but is critical for
        ensuring two arms don't accidentally command the same physical robot.
        """
        components = load_fr3_duo(
            use_fake_hardware='false',
            robot_ips="['172.16.0.2','172.16.0.3']",
        )
        ips = [c.params.get('robot_ip', '') for c in components]
        assert len(ips) == 2
        assert ips[0] != ips[1], (
            f'robot_ip values must differ (two physical robots), got {ips}'
        )
        assert '172.16.0.2' in ips
        assert '172.16.0.3' in ips


class TestFr3DuoPerArmInterfaces:
    """
    Each arm independently exposes its full command interface set.

    Two hardware components with 6-element cartesian_velocity vectors means
    indices must be 0-5 per arm (not 0-5 for left and 6-11 for right).
    Collapsing onto a shared index space would index out of bounds on the
    6-element hw_cartesian_velocities_ vector in real hardware.
    """

    @pytest.fixture
    def components(self):
        return load_fr3_duo()

    def test_each_arm_has_seven_joints(self, components):
        """Each arm exposes 7 joints for the 7-DOF manipulator."""
        for component in components:
            arm_joints = [j for j in component.joints if '_joint' in j.name]
            assert len(arm_joints) == 7, (
                f'{component.name}: expected 7 joints, got {len(arm_joints)}: '
                f'{[j.name for j in arm_joints]}'
            )

    def test_each_arm_joints_have_full_command_set(self, components):
        """Each arm joint has position, velocity, and effort command interfaces."""
        for component in components:
            arm_joints = [j for j in component.joints if '_joint' in j.name]
            for joint in arm_joints:
                cmd_names = {ci.name for ci in joint.command_interfaces}
                assert {'position', 'velocity', 'effort'} <= cmd_names, (
                    f'{component.name}/{joint.name} command interfaces {cmd_names} '
                    f'missing expected types'
                )

    def test_each_arm_cartesian_velocity_indices_0_to_5(self, components):
        """
        Per-arm cartesian_velocity indices are 0-5 (NOT a shared 0-11 space).

        FrankaHardwareInterface::hw_cartesian_velocities_ is a 6-element vector.
        Index >= 6 is out-of-bounds undefined behavior on real hardware.
        """
        for component in get_arm_components(components):
            cv_gpios = get_gpio_command_interfaces(component, 'cartesian_velocity')
            assert len(cv_gpios) == 6, (
                f'{component.name}: expected 6 cartesian_velocity interfaces, '
                f'got {len(cv_gpios)}'
            )
            indices = get_gpio_indices(cv_gpios)
            assert indices == list(range(6)), (
                f'{component.name}: cartesian_velocity indices must be [0..5], '
                f'got {indices}. Index >= 6 would overflow '
                f'hw_cartesian_velocities_[6] on real hardware.'
            )

    def test_each_arm_elbow_indices_0_to_1(self, components):
        """
        Per-arm elbow_command indices are 0-1 (NOT a shared 0-3 space).

        FrankaHardwareInterface::hw_elbow_command_ is a 2-element vector.
        Index >= 2 is out-of-bounds undefined behavior on real hardware.
        """
        for component in get_arm_components(components):
            elbow_gpios = get_gpio_command_interfaces(component, 'elbow_command')
            assert len(elbow_gpios) == 2, (
                f'{component.name}: expected 2 elbow_command interfaces, '
                f'got {len(elbow_gpios)}'
            )
            indices = get_gpio_indices(elbow_gpios)
            assert indices == [0, 1], (
                f'{component.name}: elbow_command indices must be [0, 1], '
                f'got {indices}. Index >= 2 would overflow '
                f'hw_elbow_command_[2] on real hardware.'
            )


if __name__ == '__main__':
    pytest.main([__file__])
