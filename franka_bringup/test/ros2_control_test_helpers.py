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
Shared test utilities for ros2_control interface validation.

Provides a Python-side equivalent of hardware_interface::parse_control_resources_from_urdf:
expands wrapper xacros and extracts the hardware components, joints, and GPIOs that the
ros2_control resource manager would register. Tests assert on the resulting interface sets
rather than raw XML structure.
"""

from dataclasses import dataclass, field
from os import path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

import xacro


# ---------------------------------------------------------------------------
# Data structures mirroring hardware_interface::HardwareInfo
# ---------------------------------------------------------------------------


@dataclass
class InterfaceInfo:
    """A single command or state interface on a joint or GPIO."""

    name: str


@dataclass
class JointInfo:
    """A joint registered by a hardware component, with its interfaces."""

    name: str
    command_interfaces: list = field(default_factory=list)
    state_interfaces: list = field(default_factory=list)


@dataclass
class GpioInfo:
    """A GPIO registered by a hardware component, with index and interfaces."""

    name: str
    index: int
    command_interfaces: list = field(default_factory=list)
    state_interfaces: list = field(default_factory=list)


@dataclass
class HardwareComponentInfo:
    """A hardware component parsed from a <ros2_control> block."""

    name: str
    plugin: str
    params: dict = field(default_factory=dict)
    joints: list = field(default_factory=list)
    gpios: list = field(default_factory=list)


# ---------------------------------------------------------------------------
# URDF expansion
# ---------------------------------------------------------------------------


def expand_wrapper_urdf(wrapper_file: str, mappings: dict) -> str:
    """Expand a wrapper xacro and return the URDF string."""
    return xacro.process_file(wrapper_file, mappings=mappings).toxml()


def get_bringup_wrapper(name: str) -> str:
    """Resolve a wrapper xacro path from franka_bringup/urdf/."""
    return path.join(
        get_package_share_directory('franka_bringup'),
        'urdf',
        name,
    )


def get_gazebo_bringup_wrapper(name: str) -> str:
    """Resolve a wrapper xacro path from franka_gazebo_bringup/urdf/."""
    return path.join(
        get_package_share_directory('franka_gazebo_bringup'),
        'urdf',
        name,
    )


# ---------------------------------------------------------------------------
# Parsing: URDF XML → HardwareComponentInfo list
# ---------------------------------------------------------------------------


def parse_hardware_components(urdf_string: str) -> list:
    """
    Parse <ros2_control> blocks from a URDF string into HardwareComponentInfo structs.

    This mirrors the behavior of hardware_interface::parse_control_resources_from_urdf —
    it extracts the hardware components, their joints (with command/state interfaces),
    and their GPIOs (with command interfaces and index parameters) as the resource manager
    would register them.
    """
    root = ET.fromstring(urdf_string)
    components = []

    for block in root.findall('.//ros2_control'):
        component = _parse_ros2_control_block(block)
        components.append(component)

    return components


def _parse_ros2_control_block(block: ET.Element) -> HardwareComponentInfo:
    """Parse a single <ros2_control> element into a HardwareComponentInfo."""
    name = block.get('name', '')

    # Extract hardware plugin and params
    hardware_el = block.find('.//hardware')
    plugin = ''
    params = {}
    if hardware_el is not None:
        plugin_el = hardware_el.find('plugin')
        if plugin_el is not None:
            plugin = plugin_el.text or ''
        for param_el in hardware_el.findall('param'):
            param_name = param_el.get('name', '')
            params[param_name] = (param_el.text or '').strip()

    # Extract joints
    joints = []
    for joint_el in block.findall('.//joint'):
        joint = JointInfo(name=joint_el.get('name', ''))
        for ci in joint_el.findall('command_interface'):
            joint.command_interfaces.append(InterfaceInfo(name=ci.get('name', '')))
        for si in joint_el.findall('state_interface'):
            joint.state_interfaces.append(InterfaceInfo(name=si.get('name', '')))
        joints.append(joint)

    # Extract GPIOs
    gpios = []
    for gpio_el in block.findall('.//gpio'):
        index = _extract_gpio_index(gpio_el)
        gpio = GpioInfo(name=gpio_el.get('name', ''), index=index)
        for ci in gpio_el.findall('command_interface'):
            gpio.command_interfaces.append(InterfaceInfo(name=ci.get('name', '')))
        for si in gpio_el.findall('state_interface'):
            gpio.state_interfaces.append(InterfaceInfo(name=si.get('name', '')))
        gpios.append(gpio)

    return HardwareComponentInfo(
        name=name,
        plugin=plugin,
        params=params,
        joints=joints,
        gpios=gpios,
    )


def _extract_gpio_index(gpio_el: ET.Element) -> int:
    """Extract the numeric index parameter from a GPIO element."""
    param = gpio_el.find("param[@name='index']")
    if param is None:
        param = gpio_el.find('param')
    if param is not None and param.text is not None:
        return int(param.text)
    return -1


# ---------------------------------------------------------------------------
# Assertion helpers
# ---------------------------------------------------------------------------


def get_components_by_name(
    components: list, name: str
) -> list:
    """Filter components by exact name."""
    return [c for c in components if c.name == name]


def get_arm_components(
    components: list,
) -> list:
    """Return components whose name contains 'FrankaHardwareInterface'."""
    return [c for c in components if 'FrankaHardwareInterface' in c.name]


def get_joint_command_interface_names(component: HardwareComponentInfo) -> set:
    """Return the set of (joint_name, interface_name) tuples for command interfaces."""
    return {
        (j.name, ci.name) for j in component.joints for ci in j.command_interfaces
    }


def get_gpio_command_interfaces(
    component: HardwareComponentInfo, interface_name: str
) -> list:
    """Return GPIOs that have a command interface with the given name."""
    return [
        g
        for g in component.gpios
        if any(ci.name == interface_name for ci in g.command_interfaces)
    ]


def get_gpio_indices(gpios: list) -> list:
    """Return sorted list of indices from a GPIO list."""
    return sorted(g.index for g in gpios)


def get_all_registered_joint_names(components: list) -> set:
    """Return all joint names across all components."""
    return {j.name for c in components for j in c.joints}
