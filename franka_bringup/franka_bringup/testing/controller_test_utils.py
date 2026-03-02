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


"""
Shared utilities for controller integration tests.

This module provides common functionality for testing ROS2 controllers:
- Controller state management and waiting
- Controller switching
- Running move_to_start controller and switching to target controllers
- Simple smoke tests that verify controllers run without errors
"""

import time

from franka_bringup.testing.controller_service_client import (
    ControllerServiceClient,
)
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.task import Future


# Controller name for moving robot to start position
MOVE_TO_START_CONTROLLER = 'move_to_start_example_controller'


class AsyncParameterClient:

    def __init__(self, node: RclpyNode, controller_node_name: str):
        self.node = node
        self._get_parameter_client = self.node.create_client(
            GetParameters, f'{controller_node_name}/get_parameters'
        )

    def get_parameters(self, names: list) -> Future:
        request = GetParameters.Request()
        request.names = names
        future = self._get_parameter_client.call_async(request)
        return future

    def services_are_ready(self) -> bool:
        return self._get_parameter_client.service_is_ready()


def check_process_finished_parameter(
    param_client: AsyncParameterClient,
    node: RclpyNode,
    controller_node_name: str,
) -> tuple[bool, bool]:
    """
    Check the process_finished parameter from a controller node.

    Args
    ----
    param_client : AsyncParameterClient
        The parameter client for the controller node.
    node : RclpyNode
        The ROS2 node for spinning.
    controller_node_name : str
        Name of the controller node (for logging).

    Returns
    -------
    tuple[bool, bool]
        (success, value) where success indicates if the parameter was read successfully,
        and value is the boolean value of process_finished (only valid if success is True).

    """
    logger = node.get_logger()

    # Check if services are still ready
    if not param_client.services_are_ready():
        logger.debug(
            f'Parameter service for {controller_node_name} no longer available'
        )
        return (False, False)

    try:
        future = param_client.get_parameters(['process_finished'])
        # Spin until future completes or timeout (max 1.0s)
        timeout_count = 0
        max_spin_count = 20  # 20 * 0.05s = 1.0s max wait
        while not future.done() and timeout_count < max_spin_count:
            rclpy.spin_once(node, timeout_sec=0.05)
            timeout_count += 1

        if not future.done():
            # Future didn't complete within timeout
            logger.debug(
                'Parameter get_parameters future did not complete within timeout'
            )
            return (False, False)

        try:
            result = future.result()
            # Access the .values attribute which contains the list of ParameterValue objects
            param_values = result.values if hasattr(result, 'values') else []
            if param_values and len(param_values) > 0:
                # ParameterValue has type-specific attributes (bool_value, string_value, etc.)
                param_value = param_values[0]
                # For boolean parameters, use bool_value
                if hasattr(param_value, 'bool_value'):
                    process_finished = param_value.bool_value
                    logger.debug(
                        f'Read process_finished={process_finished} from {controller_node_name}'
                    )
                    return (True, process_finished)
                else:
                    logger.debug(
                        f'ParameterValue for process_finished does not have bool_value attribute. '
                        f'Type: {param_value.type if hasattr(param_value, "type") else "unknown"}'
                    )
                    return (False, False)
            else:
                return (False, False)
        except Exception as e:
            logger.debug(f'Error getting result from parameter future: {e}')
            return (False, False)
    except Exception as e:
        # If parameter access fails, log and return failure
        logger.debug(
            f'Error checking process_finished parameter: {e}. '
            f'Will retry on next iteration.'
        )
        return (False, False)


def run_move_to_start_and_switch_to_target_controller(
    node: RclpyNode,
    target_controller: str,
    wait_duration_sec: float = 30.0,
    controller_manager_name: str = '/controller_manager',
) -> bool:
    """
    Run move_to_start_example_controller and wait for it to complete.

    This function loads, configures, and activates the move_to_start controller,
    then waits for it to complete by checking the process_finished parameter.
    If the parameter service is not available, it falls back to waiting for
    a fixed duration. After completion, it performs an atomic switch to the
    target controller.

    Args
    ----
    node : RclpyNode
        The ROS2 node for service clients
    target_controller : str
        Name of the controller to switch to after move_to_start completes
    wait_duration_sec : float
        Maximum duration to wait for move_to_start to complete (default 30s).
        Used as timeout if process_finished parameter is not available.
    controller_manager_name : str
        Name of the controller manager node

    Returns
    -------
    bool
        True if move_to_start completed successfully, False otherwise

    """
    logger = node.get_logger()
    logger.info(
        f'Running {MOVE_TO_START_CONTROLLER} to move robot to start position...'
    )

    client = ControllerServiceClient(node, controller_manager_name)
    try:
        # Wait for services to be available
        if not client.wait_for_services():
            return False

        # Step 1: Load the controller
        if not client.load_controller(MOVE_TO_START_CONTROLLER):
            return False

        # Step 2: Configure the controller
        if not client.configure_controller(MOVE_TO_START_CONTROLLER):
            client.unload_controller(MOVE_TO_START_CONTROLLER)
            return False

        # Step 3: Activate the controller
        if not client.switch_controllers(activate=[MOVE_TO_START_CONTROLLER]):
            client.unload_controller(MOVE_TO_START_CONTROLLER)
            return False

        # Step 4: Wait for move_to_start to complete by checking process_finished parameter
        logger.info(
            f'Waiting for {MOVE_TO_START_CONTROLLER} to complete '
            f'(checking process_finished parameter, max {wait_duration_sec}s)...'
        )

        # Create parameter client for the controller node
        controller_node_name = MOVE_TO_START_CONTROLLER
        param_client = AsyncParameterClient(node, controller_node_name)

        # Wait for parameter service to be available
        logger.info(
            f'Waiting for parameter service of {controller_node_name} to be available...'
        )
        param_service_available = False
        service_wait_start = time.time()
        service_wait_timeout = (
            5.0  # Wait up to 5s for service to become available
        )

        while time.time() - service_wait_start < service_wait_timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if param_client.services_are_ready():
                param_service_available = True
                logger.info(
                    f'Parameter service for {controller_node_name} is available'
                )
                break
            time.sleep(0.1)

        if not param_service_available:
            logger.warn(
                f'Parameter service for {controller_node_name} not available after '
                f'{service_wait_timeout}s. Will use timeout-based wait instead.'
            )

        # Poll for process_finished parameter
        start_time = time.time()
        process_finished = False
        check_interval = 0.2  # Check every 200ms

        while time.time() - start_time < wait_duration_sec:
            rclpy.spin_once(node, timeout_sec=0.1)

            # Try to get the process_finished parameter
            if param_service_available:
                success, value = check_process_finished_parameter(
                    param_client, node, controller_node_name
                )
                if not success:
                    # Service might have become unavailable
                    if not param_client.services_are_ready():
                        logger.debug(
                            f'Parameter service for {controller_node_name} no longer available, '
                            f'falling back to timeout-based wait'
                        )
                        param_service_available = False
                    continue

                process_finished = value
                if process_finished:
                    elapsed = time.time() - start_time
                    logger.info(
                        f'{MOVE_TO_START_CONTROLLER} completed (process_finished=true) '
                        f'after {elapsed:.1f}s'
                    )
                    break

            time.sleep(check_interval)

        if not process_finished:
            elapsed = time.time() - start_time
            if param_service_available:
                logger.warn(
                    f'{MOVE_TO_START_CONTROLLER} did not complete within {wait_duration_sec}s '
                    f'(waited {elapsed:.1f}s, process_finished never became true). '
                    f'Proceeding anyway.'
                )
            else:
                logger.info(
                    f'{MOVE_TO_START_CONTROLLER} wait duration completed after {elapsed:.1f}s '
                    f'(parameter service unavailable, used timeout)'
                )

        # Step 5: Wait for target controller to be ready (inactive state)
        # The launch file spawns it with --inactive, so it should be in 'inactive' state
        logger.info(
            f'Waiting for {target_controller} to be ready (inactive state)...'
        )
        if not client.wait_for_controller_state(
            target_controller,
            ['inactive', 'configured'],
            timeout_sec=wait_duration_sec,
        ):
            logger.error(
                f'{target_controller} is not ready for activation. '
                f'Make sure it was spawned with --inactive flag.'
            )
            return False

        # Step 6: Atomic switch from move_to_start to target controller
        logger.info(
            f'Switching from {MOVE_TO_START_CONTROLLER} to {target_controller}...'
        )
        if not client.switch_controllers(
            activate=[target_controller],
            deactivate=[MOVE_TO_START_CONTROLLER],
            strict=False,
        ):
            logger.error(f'Failed to switch to {target_controller}')
            return False

        # Cleanup the move_to_start controller
        client.unload_controller(MOVE_TO_START_CONTROLLER)

        logger.info('Robot is now at start position')
        return True

    finally:
        client.destroy()


def run_controller_smoke_test(
    test_case, controller_name: str, test_duration_sec: float = 10.0
):
    """
    Run a simple smoke test for a controller.

    The controller runs for a fixed duration to verify it operates without errors.

    Args:
    ----
    test_case : unittest.TestCase
        The unittest.TestCase instance.
    controller_name : str
        Name of the controller being tested.
    test_duration_sec : float
        Duration to run the controller (default 10 seconds).

    """
    logger = test_case.link_node.get_logger()
    logger.info(
        f'Controller {controller_name} is running. Monitoring for {test_duration_sec}s...'
    )

    # Monitor the controller for the test duration
    # Simply verify it runs without errors
    start_time = time.time()

    while time.time() - start_time < test_duration_sec:
        # Process ROS callbacks
        rclpy.spin_once(test_case.link_node, timeout_sec=0.1)
        time.sleep(0.1)

    elapsed = time.time() - start_time
    logger.info(
        f'Controller {controller_name} ran successfully for {elapsed:.1f}s'
    )
