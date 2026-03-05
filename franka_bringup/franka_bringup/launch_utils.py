# Copyright 2026 Franka Robotics GmbH
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

import ast
import os
import sys

from typing import List, Sized

import yaml


def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f'File not found: {file_path}')
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


def parse_string_list(string_list_repr):
    """
    Parse a string representation of a list into an actual Python list.

    Handles formats like "['item1','item2']" or "['item1', 'item2']".
    """
    try:
        return ast.literal_eval(string_list_repr)
    except (ValueError, SyntaxError):
        # Fallback: try to parse manually if ast fails
        cleaned = string_list_repr.strip(
            '[]').replace("'", '').replace('"', '')
        return [item.strip() for item in cleaned.split(',')]


def is_duo_config(config):
    """
    Detect duo setup by checking for plural keys unique to multi-robot configs.

    @param config: Configuration dictionary from YAML file.
    @return: True if configuration is for a duo setup, False otherwise.
    """
    duo_keys = {'robot_types', 'robot_ips', 'arm_prefixes'}
    return duo_keys.issubset(config.keys())


def is_mobile_duo_config(config):
    """
    Detect mobile duo setup by checking for plural keys unique to multi-robot configs.

    @param config: Configuration dictionary from YAML file.
    @return: True if configuration is for a mobile duo setup, False otherwise.
    """
    duo_keys = {'robot_types', 'robot_ips', 'robot_prefixes'}
    return duo_keys.issubset(config.keys())


def _assert_same_length(*items: List[Sized]):
    """
    Assert that all provided lists have the same length.

    @param items: Variable number of list arguments to compare.
    @raise ValueError: If lists have different lengths.
    """
    if not items:
        return  # No lists to compare, consider them as having the same length
    length = len(items[0])
    for lst in items[1:]:
        if len(lst) != length:
            raise ValueError('All lists must have the same length.')


def validate_duo_arrays_length(
        robot_types_list, robot_ips_list, arm_prefixes_list):
    """
    Validate that all duo configuration arrays have the same length.

    @param robot_types_list: List of robot types.
    @param robot_ips_list: List of robot IP addresses.
    @param arm_prefixes_list: List of arm prefixes.
    @raise SystemExit: If arrays have different lengths.
    """
    try:
        _assert_same_length(robot_types_list, robot_ips_list, arm_prefixes_list)
    except ValueError:
        print(
            f'Error: Duo configuration arrays must have the same length.\n'
            f'  robot_types:  {len(robot_types_list)} items: {
                robot_types_list}\n'
            f'  robot_ips:    {len(robot_ips_list)} items: {robot_ips_list}\n'
            f'  arm_prefixes: {len(arm_prefixes_list)} items: {
                arm_prefixes_list}\n'
            f'Please check your configuration file and ensure all arrays '
            f'have the same number of elements.'
        )
        sys.exit(1)


def validate_arm_prefixes_unique(arm_prefixes_list):
    """
    Validate that arm_prefixes are unique within the list.

    @param arm_prefixes_list: List of arm prefixes.
    @raise SystemExit: If duplicate arm_prefixes are found.
    """
    if len(arm_prefixes_list) != len(set(arm_prefixes_list)):
        duplicates = [
            p for p in arm_prefixes_list if arm_prefixes_list.count(p) > 1]
        print(
            f'Error: arm_prefixes must be unique.\n'
            f'  arm_prefixes: {arm_prefixes_list}\n'
            f'  Duplicate values: {list(set(duplicates))}\n'
            f'Each robot arm requires a unique prefix to avoid naming conflicts.'
        )
        sys.exit(1)


def get_parameter_for_config(parameter_str, num_configs=1, config_index=0):
    """
    Determine which parameter to use for a given configuration.

    @param parameter_names_str: Comma-separated string of parameters.
    @param num_configs: Total number of robot configurations (default: 1).
    @param config_index: Index of the current configuration (default: 0).
    @return: Controller name to use for this configuration, or empty string if none.
    """
    if not parameter_str or not parameter_str.strip():
        return ''

    parameter_vector = [name.strip() for name in parameter_str.split(',')]

    if not parameter_vector or not any(parameter_vector):
        return ''

    # If number of parameters matches number of configs, use corresponding
    # parameter
    if len(parameter_vector) == num_configs:
        return parameter_vector[config_index]
    else:
        # Otherwise, use the first parameter for all configs
        if num_configs > 1 and len(parameter_vector) != num_configs:
            print(
                'Warning: Number of parameter does not match number of robot configs. '
                'Using the first parameter for all robots.'
            )
        return parameter_vector[0]
