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
Behavioral tests for mobile_fr3_duo ros2_control interfaces.

The mobile_fr3_duo system registers THREE independent hardware components:
1 TMR mobile base (TmrHardware) + 2 arms (left/right FrankaHardwareInterface).

Key invariants:
- Each arm has its own independent GPIO index space (0-5 cartesian_velocity, 0-1 elbow)
- TMR base exposes steering (position) + driving (velocity) joint interfaces
  and 6 base cartesian_velocity GPIO interfaces
- Passive base joints (rocker, casters, spine) are simulation-only: they appear
  in Gazebo for physics fidelity but are NOT registered on real hardware where
  the compliance is mechanical, not software-controlled
"""

import pytest

from ros2_control_test_helpers import (
    expand_wrapper_urdf,
    get_arm_components,
    get_bringup_wrapper,
    get_components_by_name,
    get_gazebo_bringup_wrapper,
    get_gpio_command_interfaces,
    get_gpio_indices,
    parse_hardware_components,
)


def load_mobile_fr3_duo(**extra_mappings) -> list:
    """Expand mobile_fr3_duo wrapper and return parsed hardware components."""
    mappings = {'use_fake_hardware': 'true'}
    mappings.update(extra_mappings)
    urdf = expand_wrapper_urdf(
        get_bringup_wrapper('mobile_fr3_duo.urdf.xacro'), mappings
    )
    return parse_hardware_components(urdf)


def load_mobile_fr3_duo_gazebo() -> list:
    """Expand the Gazebo variant (includes simulation-only passive joints)."""
    urdf = expand_wrapper_urdf(
        get_gazebo_bringup_wrapper('mobile_fr3_duo_v0_2.gazebo.urdf.xacro'),
        mappings={'use_fake_hardware': 'true'},
    )
    return parse_hardware_components(urdf)


class TestMobileHardwareTopology:
    """
    Three hardware components => independent control loops for base and each arm.

    The TMR base has its own hardware interface (TmrHardware) with its own
    cartesian_velocity vector. The two arms each have a FrankaHardwareInterface.
    Merging any of these would break the per-component index invariant.
    """

    @pytest.fixture
    def components(self):
        return load_mobile_fr3_duo()

    def test_exactly_three_hardware_components(self, components):
        """mobile_fr3_duo registers 3 components: TMR base + 2 arms."""
        assert len(components) == 3, (
            f'Expected 3 hardware components (TMR + 2 arms), got {len(components)}'
        )

    def test_component_names(self, components):
        """Components are TmrHardware, left_/right_FrankaHardwareInterface."""
        names = sorted(c.name for c in components)
        expected = sorted([
            'TmrHardware',
            'left_FrankaHardwareInterface',
            'right_FrankaHardwareInterface',
        ])
        assert names == expected, f'Expected {expected}, got {names}'


class TestMobileArmInterfaces:
    """Per-arm interface set on the mobile platform, identical to standalone fr3_duo."""

    @pytest.fixture
    def arm_components(self):
        return get_arm_components(load_mobile_fr3_duo())

    def test_each_arm_cartesian_velocity_indices_0_to_5(self, arm_components):
        """
        Per-arm cartesian_velocity indices are 0-5 (independent of base).

        The base ALSO has cartesian_velocity 0-5 in its own TmrHardware component.
        These are separate index spaces — the resource manager routes by component name.
        """
        for component in arm_components:
            cv_gpios = get_gpio_command_interfaces(component, 'cartesian_velocity')
            assert len(cv_gpios) == 6, (
                f'{component.name}: expected 6 cartesian_velocity interfaces, '
                f'got {len(cv_gpios)}'
            )
            indices = get_gpio_indices(cv_gpios)
            assert indices == list(range(6)), (
                f'{component.name}: cv indices must be [0..5], got {indices}'
            )

    def test_each_arm_elbow_indices_0_to_1(self, arm_components):
        """Per-arm elbow_command indices are 0-1."""
        for component in arm_components:
            elbow_gpios = get_gpio_command_interfaces(component, 'elbow_command')
            assert len(elbow_gpios) == 2, (
                f'{component.name}: expected 2 elbow_command interfaces, '
                f'got {len(elbow_gpios)}'
            )
            indices = get_gpio_indices(elbow_gpios)
            assert indices == [0, 1], (
                f'{component.name}: elbow indices must be [0, 1], got {indices}'
            )


class TestMobileBaseInterfaces:
    """
    TMR base exposes steering/driving joint interfaces and base cartesian_velocity.

    Steering joints use position command (wheel angle), driving joints use velocity
    command (wheel speed). The base cartesian_velocity GPIOs enable whole-body
    velocity control via the mobile platform controller.
    """

    @pytest.fixture
    def tmr_component(self):
        components = load_mobile_fr3_duo()
        tmr = get_components_by_name(components, 'TmrHardware')
        assert len(tmr) == 1
        return tmr[0]

    def test_base_has_six_cartesian_velocity_interfaces(self, tmr_component):
        """TMR base exposes 6 cartesian_velocity GPIO command interfaces (indices 0-5)."""
        cv_gpios = get_gpio_command_interfaces(tmr_component, 'cartesian_velocity')
        assert len(cv_gpios) == 6, (
            f'Expected 6 base cartesian_velocity interfaces, got {len(cv_gpios)}'
        )
        indices = get_gpio_indices(cv_gpios)
        assert indices == list(range(6)), (
            f'Base cartesian_velocity indices must be [0..5], got {indices}'
        )

    def test_steering_joints_have_position_command(self, tmr_component):
        """Steering joints (joint_0, joint_2) expose position command for wheel angle."""
        steering_joints = [
            j for j in tmr_component.joints
            if 'joint_0' in j.name or 'joint_2' in j.name
        ]
        assert len(steering_joints) >= 2, (
            f'Expected at least 2 steering joints, got {len(steering_joints)}'
        )
        for joint in steering_joints:
            cmd_names = {ci.name for ci in joint.command_interfaces}
            assert 'position' in cmd_names, (
                f'{joint.name} missing position command interface. '
                f'Steering joints require position control for wheel angle.'
            )

    def test_driving_joints_have_velocity_command(self, tmr_component):
        """Driving joints (joint_1, joint_3) expose velocity command for wheel speed."""
        driving_joints = [
            j for j in tmr_component.joints
            if 'joint_1' in j.name or 'joint_3' in j.name
        ]
        assert len(driving_joints) >= 2, (
            f'Expected at least 2 driving joints, got {len(driving_joints)}'
        )
        for joint in driving_joints:
            cmd_names = {ci.name for ci in joint.command_interfaces}
            assert 'velocity' in cmd_names, (
                f'{joint.name} missing velocity command interface. '
                f'Driving joints require velocity control for wheel speed.'
            )

    def test_four_drive_joints_total(self, tmr_component):
        """TMR base registers exactly 4 drive joints (2 steering + 2 driving)."""
        drive_joints = [j for j in tmr_component.joints if 'joint_' in j.name]
        assert len(drive_joints) == 4, (
            f'Expected 4 drive joints (2 steering + 2 driving), got {len(drive_joints)}: '
            f'{[j.name for j in drive_joints]}'
        )


class TestMobilePassiveBaseGating:
    """
    Passive base joints (rocker arm, casters, spine) are simulation-only.

    On real mobile hardware, only the drive joints are present as ros2_control
    interfaces. The passive joints represent mechanical compliance modeled in
    Gazebo but not actuated or sensed on the physical platform. Exposing them
    on real hardware would create phantom state interfaces that report zero
    forever — misleading for controllers and state estimators.
    """

    def test_rocker_absent_without_gazebo(self):
        """Rocker arm joint is NOT registered on real/fake hardware."""
        components = load_mobile_fr3_duo()
        all_joint_names = {j.name for c in components for j in c.joints}
        assert 'rocker_arm_joint' not in all_joint_names, (
            'rocker_arm_joint must not be registered without gazebo '
            '(passive joints are simulation-only)'
        )

    def test_casters_absent_without_gazebo(self):
        """Caster joints are NOT registered on real/fake hardware."""
        components = load_mobile_fr3_duo()
        all_joint_names = {j.name for c in components for j in c.joints}
        caster_joints = {n for n in all_joint_names if 'caster' in n}
        assert len(caster_joints) == 0, (
            f'Caster joints must not be registered without gazebo, found: {caster_joints}'
        )

    def test_spine_absent_without_gazebo(self):
        """Spine vertical joint is NOT registered on real/fake hardware."""
        components = load_mobile_fr3_duo()
        all_joint_names = {j.name for c in components for j in c.joints}
        assert 'franka_spine_vertical_joint' not in all_joint_names, (
            'franka_spine_vertical_joint must not be registered without gazebo '
            '(simulation-only joint)'
        )

    def test_passive_joints_present_with_gazebo_as_state_only(self):
        """
        In Gazebo, rocker/caster joints are state-only but the spine is actively held.

        Rocker and casters provide feedback for physics simulation (joint angle
        readback) but have no command interfaces — their compliance is passive,
        not actuated. The prismatic spine is the exception: it carries the full
        upper-body weight and would collapse under gravity, so it gets a single
        position command interface and is held by spine_joint_trajectory_controller
        (see configure_passive_mobile_base_joints in franka_ros2_control_macros.xacro).
        """
        components = load_mobile_fr3_duo_gazebo()
        all_joints = {j.name: j for c in components for j in c.joints}

        # Rocker arm
        assert 'rocker_arm_joint' in all_joints, (
            'rocker_arm_joint must be registered in gazebo'
        )
        rocker = all_joints['rocker_arm_joint']
        assert len(rocker.command_interfaces) == 0, (
            'rocker_arm_joint must be state-only (no command interfaces)'
        )
        assert len(rocker.state_interfaces) > 0, (
            'rocker_arm_joint must have state interfaces in gazebo'
        )

        # Casters
        caster_joints = {
            name: joint for name, joint in all_joints.items() if 'caster' in name
        }
        assert len(caster_joints) == 4, (
            f'Expected 4 caster joints in gazebo, got {len(caster_joints)}'
        )
        for name, joint in caster_joints.items():
            assert len(joint.command_interfaces) == 0, (
                f'{name} must be state-only (no command interfaces)'
            )

        # Spine — actively held (carries upper-body weight), not passive
        assert 'franka_spine_vertical_joint' in all_joints, (
            'franka_spine_vertical_joint must be registered in gazebo'
        )
        spine = all_joints['franka_spine_vertical_joint']
        assert len(spine.command_interfaces) == 1, (
            'franka_spine_vertical_joint must have exactly one command interface '
            'in gazebo (actively held against gravity, not passive)'
        )
        assert spine.command_interfaces[0].name == 'position', (
            'franka_spine_vertical_joint must have a position command interface in gazebo'
        )
        assert len(spine.state_interfaces) > 0, (
            'franka_spine_vertical_joint must have state interfaces in gazebo'
        )


if __name__ == '__main__':
    pytest.main([__file__])
