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
Path-resolution contract tests for composition xacros.

Proves that every in-scope xacro entry point resolves all its $(find …) includes
and processes without xacro error. This catches stale package paths after migrations
(e.g. ros2_control xacro moving from franka_description to franka_hardware).

These tests run against the INSTALLED tree (same as launch files use at runtime).
"""

import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro


def _share(package: str, *path_parts: str) -> str:
    """Resolve a file path under a package's share directory."""
    import os
    return os.path.join(get_package_share_directory(package), *path_parts)


def _expand(xacro_file: str, mappings: dict = None) -> str:
    """Expand a xacro file and return the XML string. Raises on any xacro error."""
    return xacro.process_file(xacro_file, mappings=mappings or {}).toxml()


# ---------------------------------------------------------------------------
# Parametrized test cases: (description, package, path_parts, mappings)
# ---------------------------------------------------------------------------

XACRO_ENTRIES = [
    (
        'franka_arm_bringup',
        'franka_bringup',
        ('urdf', 'franka_arm.urdf.xacro'),
        {'robot_type': 'fr3', 'robot_ip': '192.168.1.1'},
    ),
    (
        'franka_arm_gazebo',
        'franka_gazebo_bringup',
        ('urdf', 'franka_arm.gazebo.xacro'),
        {'robot_type': 'fr3', 'gazebo': 'true', 'hand': 'true'},
    ),
    (
        'tmrv0_2_gazebo',
        'franka_gazebo_bringup',
        ('urdf', 'tmrv0_2.gazebo.urdf.xacro'),
        {'gazebo': 'true'},
    ),
    (
        'tmrv0_2_with_sensors_gazebo',
        'franka_gazebo_bringup',
        ('urdf', 'tmrv0_2_with_sensors.gazebo.urdf.xacro'),
        {'gazebo': 'true'},
    ),
    (
        'tmrv0_2_with_sensors',
        'franka_mobile_sensors',
        ('robots', 'tmrv0_2_with_sensors.urdf.xacro'),
        {},
    ),
]


@pytest.mark.parametrize(
    'description,package,path_parts,mappings',
    XACRO_ENTRIES,
    ids=[e[0] for e in XACRO_ENTRIES],
)
class TestXacroPathResolution:
    """Verify that composition xacros expand without errors."""

    def test_expands_without_error(self, description, package, path_parts, mappings):
        """xacro.process_file succeeds — no unresolved $(find) or missing includes."""
        xacro_file = _share(package, *path_parts)
        # This will raise xacro.XacroException on any resolution failure
        _expand(xacro_file, mappings)

    def test_produces_valid_xml(self, description, package, path_parts, mappings):
        """Expanded output is well-formed XML with a <robot> root element."""
        xacro_file = _share(package, *path_parts)
        xml_str = _expand(xacro_file, mappings)
        root = ET.fromstring(xml_str)
        assert root.tag == 'robot', f'Expected <robot> root, got <{root.tag}>'


# ---------------------------------------------------------------------------
# Gazebo-specific contract: TMR variants must emit ros2_control + gz plugin
# ---------------------------------------------------------------------------

GAZEBO_TMR_ENTRIES = [
    (
        'tmrv0_2_gazebo',
        'franka_gazebo_bringup',
        ('urdf', 'tmrv0_2.gazebo.urdf.xacro'),
        {'gazebo': 'true'},
    ),
    (
        'tmrv0_2_with_sensors_gazebo',
        'franka_gazebo_bringup',
        ('urdf', 'tmrv0_2_with_sensors.gazebo.urdf.xacro'),
        {'gazebo': 'true'},
    ),
]


@pytest.mark.parametrize(
    'description,package,path_parts,mappings',
    GAZEBO_TMR_ENTRIES,
    ids=[e[0] for e in GAZEBO_TMR_ENTRIES],
)
class TestGazeboTmrContract:
    """Gazebo TMR variants must contain ros2_control and gz_ros2_control plugin."""

    def test_contains_ros2_control_element(
        self, description, package, path_parts, mappings
    ):
        """At least one <ros2_control> block present for controller_manager."""
        xacro_file = _share(package, *path_parts)
        xml_str = _expand(xacro_file, mappings)
        root = ET.fromstring(xml_str)
        ros2_control_blocks = root.findall('.//ros2_control')
        assert len(ros2_control_blocks) > 0, (
            'Gazebo TMR variant must emit at least one <ros2_control> block'
        )

    def test_contains_gz_ros2_control_plugin(
        self, description, package, path_parts, mappings
    ):
        """A <gazebo> block with gz_ros2_control/GazeboSimSystem or similar plugin."""
        xacro_file = _share(package, *path_parts)
        xml_str = _expand(xacro_file, mappings)
        root = ET.fromstring(xml_str)
        # Look for gz_ros2_control plugin in any <gazebo> block
        gazebo_blocks = root.findall('.//gazebo')
        plugin_found = False
        for gz in gazebo_blocks:
            for plugin in gz.findall('.//plugin'):
                plugin_text = ET.tostring(plugin, encoding='unicode')
                if 'gz_ros2_control' in plugin_text or 'ign_ros2_control' in plugin_text:
                    plugin_found = True
                    break
        assert plugin_found, (
            'Gazebo TMR variant must include a gz_ros2_control or ign_ros2_control plugin'
        )


if __name__ == '__main__':
    pytest.main([__file__])
