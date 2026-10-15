# Copyright (c) 2026 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Regression tests for standalone mobile sensor URDF xacro entry points."""

import os
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro


MOBILE_SENSOR_CASES = (
    ('tmr_default', 'tmrv0_2_with_sensors.urdf.xacro', {}),
    (
        'tmr_reduced_with_namespace',
        'tmrv0_2_with_sensors.urdf.xacro',
        {'reduced_version': 'true', 'robot_namespace': 'tmr'},
    ),
    (
        'mobile_duo_default',
        'mobile_fr3_duo_v0_2_with_sensors.urdf.xacro',
        {},
    ),
    (
        'mobile_duo_without_arms',
        'mobile_fr3_duo_v0_2_with_sensors.urdf.xacro',
        {'hand': 'false', 'use_arms': 'false'},
    ),
)

EXPECTED_MOUNTING_POINT_PARENTS = {
    'imu_mounting_point_joint': ('base_link', 'imu_mounting_point'),
    'front_mounting_point_joint': ('base_link', 'front_mounting_point'),
    'rear_mounting_point_joint': ('base_link', 'rear_mounting_point'),
    'left_mounting_point_joint': ('base_link', 'left_mounting_point'),
    'right_mounting_point_joint': ('base_link', 'right_mounting_point'),
    'lidar_front_mounting_point_joint': ('base_link', 'lidar_front_mounting_point'),
    'lidar_rear_mounting_point_joint': ('base_link', 'lidar_rear_mounting_point'),
}

EXPECTED_SENSOR_PARENTS = {
    'imu_body_joint': 'imu_mounting_point',
    'camera_front_joint': 'front_mounting_point',
    'camera_rear_joint': 'rear_mounting_point',
    'camera_left_joint': 'left_mounting_point',
    'camera_right_joint': 'right_mounting_point',
    'lidar_front_base_joint': 'lidar_front_mounting_point',
    'lidar_rear_base_joint': 'lidar_rear_mounting_point',
}

INVALID_PLACEHOLDER_ELEMENTS = {
    'included_robot_macro',
    'includeded_robot_macro',
}


def mobile_sensor_xacro_path(filename: str) -> str:
    """Resolve a mobile sensor xacro through the installed package index."""
    return os.path.join(
        get_package_share_directory('franka_mobile_sensors'),
        'robots',
        filename,
    )


@pytest.fixture(
    scope='module',
    params=MOBILE_SENSOR_CASES,
    ids=[case[0] for case in MOBILE_SENSOR_CASES],
)
def expanded_mobile_sensor(request):
    """Expand one installed standalone mobile sensor entry point."""
    _, filename, mappings = request.param
    # xacro adds the defaults to the dict it is given.
    urdf = xacro.process_file(
        mobile_sensor_xacro_path(filename),
        mappings=dict(mappings),
    ).toxml()
    return ElementTree.fromstring(urdf), mappings


def joint_links(root: ElementTree.Element, joint_name: str) -> tuple[str, str]:
    """Return the parent and child links for one uniquely named joint."""
    joints = [
        joint
        for joint in root.findall('joint')
        if joint.get('name') == joint_name
    ]
    assert len(joints) == 1, f'Expected one joint named {joint_name!r}'
    parent = joints[0].find('parent')
    child = joints[0].find('child')
    assert parent is not None, f'Joint {joint_name!r} has no parent element'
    assert child is not None, f'Joint {joint_name!r} has no child element'
    parent_link = parent.get('link')
    child_link = child.get('link')
    assert parent_link is not None, f'Joint {joint_name!r} has no parent link'
    assert child_link is not None, f'Joint {joint_name!r} has no child link'
    return parent_link, child_link


def named_elements(root: ElementTree.Element, element_name: str) -> list[str]:
    """Return the names of all links or joints in an expanded description."""
    names = [
        element.get('name')
        for element in root.findall(element_name)
    ]
    assert all(name for name in names), f'Unnamed {element_name} element found'
    return names


def assert_joint_links_are_emitted(root: ElementTree.Element) -> None:
    """Assert every joint references links emitted by the description."""
    emitted_link_names = set(named_elements(root, 'link'))
    for joint in root.findall('joint'):
        joint_name = joint.get('name')
        for relation in ('parent', 'child'):
            link_element = joint.find(relation)
            assert link_element is not None, (
                f'Joint {joint_name!r} has no {relation} element'
            )
            link_name = link_element.get('link')
            assert link_name is not None, (
                f'Joint {joint_name!r} has no {relation} link'
            )
            assert link_name in emitted_link_names, (
                f'Joint {joint_name!r} {relation} references missing link '
                f'{link_name!r}'
            )


def minimal_joint_graph() -> ElementTree.Element:
    """Build a valid one-joint graph for mutation tests."""
    return ElementTree.fromstring(
        '<robot><link name="base_link"/><link name="sensor_link"/>'
        '<joint name="sensor_joint"><parent link="base_link"/>'
        '<child link="sensor_link"/></joint></robot>'
    )


def test_rejects_dangling_joint_parent():
    """A joint parent must reference an emitted link."""
    root = minimal_joint_graph()
    root.find('joint/parent').set('link', 'missing_link')

    with pytest.raises(AssertionError, match='parent references missing link'):
        assert_joint_links_are_emitted(root)


def test_rejects_dangling_joint_child():
    """A joint child must reference an emitted link."""
    root = minimal_joint_graph()
    root.find('joint/child').set('link', 'missing_link')

    with pytest.raises(AssertionError, match='child references missing link'):
        assert_joint_links_are_emitted(root)


def test_expands_to_well_formed_xml(expanded_mobile_sensor):
    """Each public mobile sensor entry point expands to a robot description."""
    root, _ = expanded_mobile_sensor
    assert root.tag == 'robot'
    assert root.findall('.//link')
    assert root.findall('.//joint')


def test_emits_unique_link_and_joint_names(expanded_mobile_sensor):
    """Expanded descriptions do not contain duplicate link or joint names."""
    root, _ = expanded_mobile_sensor
    link_names = named_elements(root, 'link')
    joint_names = named_elements(root, 'joint')
    assert len(link_names) == len(set(link_names)), 'Duplicate link name found'
    assert len(joint_names) == len(set(joint_names)), 'Duplicate joint name found'


def test_joint_links_reference_emitted_links(expanded_mobile_sensor):
    """Every expanded joint references an emitted link."""
    root, _ = expanded_mobile_sensor
    assert_joint_links_are_emitted(root)


def test_attaches_sensors_to_tmr_mounting_points(expanded_mobile_sensor):
    """Every sensor remains attached to its TMR mounting point."""
    root, _ = expanded_mobile_sensor
    for joint_name, expected_links in EXPECTED_MOUNTING_POINT_PARENTS.items():
        assert joint_links(root, joint_name) == expected_links
    for joint_name, expected_parent in EXPECTED_SENSOR_PARENTS.items():
        parent_link, _ = joint_links(root, joint_name)
        assert parent_link == expected_parent


def test_is_rooted_at_base_link(expanded_mobile_sensor):
    """The tree starts at base_link, so base -> base_link stays with the odometry."""
    root, _ = expanded_mobile_sensor
    link_names = set(named_elements(root, 'link'))
    child_link_names = {joint.find('child').get('link') for joint in root.findall('joint')}
    assert link_names - child_link_names == {'base_link'}
    assert 'base' not in link_names
    assert 'base_joint' not in named_elements(root, 'joint')


def test_has_no_gazebo_or_placeholder_elements(expanded_mobile_sensor):
    """Standalone descriptions contain no Gazebo-only or placeholder output."""
    root, _ = expanded_mobile_sensor
    element_tags = {element.tag for element in root.iter()}
    assert not element_tags & INVALID_PLACEHOLDER_ELEMENTS
    assert not root.findall('.//gazebo')
    assert not any(
        'gazebo' in attribute_name.lower()
        for element in root.iter()
        for attribute_name in element.attrib
    )
