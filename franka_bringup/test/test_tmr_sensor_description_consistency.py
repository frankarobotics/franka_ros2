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

"""
Consistency guard for the two TMR descriptions that publish TF side by side.

tmrv0_2.launch.py publishes franka_bringup/urdf/tmrv0_2.urdf.xacro, and
franka_mobile_sensors.launch.py (start_rviz:=false) publishes
tmrv0_2_with_sensors.urdf.xacro for the sensor frames. Both instantiate the
same base, so every base joint is published twice and the two models must not
drift apart.
"""

import os
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)
import pytest
from ros2_control_test_helpers import expand_wrapper_urdf, get_bringup_wrapper


def sensor_description_path() -> str:
    """Resolve the sensor xacro, or skip when franka_mobile_sensors is absent."""
    try:
        package_directory = get_package_share_directory('franka_mobile_sensors')
    except PackageNotFoundError:
        pytest.skip('franka_mobile_sensors not available in this build')
    return os.path.join(package_directory, 'robots', 'tmrv0_2_with_sensors.urdf.xacro')


def joints_by_name(urdf: str) -> dict[str, ElementTree.Element]:
    """Return the joints of an expanded description keyed by name."""
    return {joint.get('name'): joint for joint in ElementTree.fromstring(urdf).findall('joint')}


def joint_signature(joint: ElementTree.Element) -> tuple:
    """Return type, parent, child and origin of a joint for comparison."""
    origin = joint.find('origin')
    xyz = origin.get('xyz', '0 0 0') if origin is not None else '0 0 0'
    rpy = origin.get('rpy', '0 0 0') if origin is not None else '0 0 0'
    return (
        joint.get('type'),
        joint.find('parent').get('link'),
        joint.find('child').get('link'),
        tuple(float(value) for value in f'{xyz} {rpy}'.split()),
    )


@pytest.fixture(scope='module')
def bringup_joints() -> dict[str, ElementTree.Element]:
    """Expand the bringup TMR description."""
    return joints_by_name(expand_wrapper_urdf(
        get_bringup_wrapper('tmrv0_2.urdf.xacro'),
        mappings={'use_fake_hardware': 'true'},
    ))


@pytest.fixture(scope='module')
def sensor_joints() -> dict[str, ElementTree.Element]:
    """Expand the sensor TMR description as the sensor TF publisher does."""
    return joints_by_name(expand_wrapper_urdf(sensor_description_path(), mappings={}))


def test_sensor_model_contains_every_bringup_joint(bringup_joints, sensor_joints):
    """A joint added to or renamed in only one model shows up here."""
    missing = sorted(bringup_joints.keys() - sensor_joints.keys())
    assert not missing, f'Joints only in the bringup model: {missing}'


def test_shared_joints_match(bringup_joints, sensor_joints):
    """Shared joints agree on type, parent, child and origin."""
    shared = sorted(bringup_joints.keys() & sensor_joints.keys())
    assert shared
    for name in shared:
        bringup = joint_signature(bringup_joints[name])
        sensor = joint_signature(sensor_joints[name])
        assert bringup[:3] == sensor[:3], f'{name}: {bringup[:3]} != {sensor[:3]}'
        assert bringup[3] == pytest.approx(sensor[3], abs=1e-9), (
            f'{name}: origin {bringup[3]} != {sensor[3]}'
        )
