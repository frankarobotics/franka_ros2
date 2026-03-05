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
Client for ROS2 controller_manager service operations.

This module provides a ControllerServiceClient class that encapsulates
all service calls to the controller_manager, including loading, configuring,
switching, and unloading controllers.
"""

import time

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
    UnloadController,
)
import rclpy
from rclpy.node import Node as RclpyNode


def _create_duration(seconds: int = 5) -> Duration:
    """
    Create a Duration message with specified seconds.

    Args
    ----
    seconds : int
        Number of seconds for the duration (default 5)

    Returns
    -------
    Duration
        A Duration message with the specified seconds

    """
    duration = Duration()
    duration.sec = seconds
    duration.nanosec = 0
    return duration


class ControllerServiceClient:
    """
    Client for ROS2 controller_manager service operations.

    This class encapsulates all service calls to the controller_manager,
    providing a clean interface for loading, configuring, switching,
    and unloading controllers.

    Example:
    -------
    >>> client = ControllerServiceClient(node)
    >>> try:
    ...     if client.wait_for_services():
    ...         client.load_controller('my_controller')
    ...         client.configure_controller('my_controller')
    ...         client.switch_controllers(activate=['my_controller'])
    ... finally:
    ...     client.destroy()

    """

    def __init__(
        self,
        node: RclpyNode,
        controller_manager_name: str = '/controller_manager',
    ):
        """
        Initialize the controller service client.

        Args:
        ----
        node : RclpyNode
            The ROS2 node for creating service clients.
        controller_manager_name : str
            Name of the controller manager node (default '/controller_manager').

        """
        self._node = node
        self._controller_manager_name = controller_manager_name
        self._logger = node.get_logger()

        # Create all service clients
        self._load_client = node.create_client(
            LoadController, f'{controller_manager_name}/load_controller'
        )
        self._configure_client = node.create_client(
            ConfigureController,
            f'{controller_manager_name}/configure_controller',
        )
        self._switch_client = node.create_client(
            SwitchController, f'{controller_manager_name}/switch_controller'
        )
        self._unload_client = node.create_client(
            UnloadController, f'{controller_manager_name}/unload_controller'
        )
        self._list_client = node.create_client(
            ListControllers, f'{controller_manager_name}/list_controllers'
        )

    def wait_for_services(self, timeout_sec: float = 10.0) -> bool:
        """
        Wait for all controller manager services to be available.

        Args
        ----
        timeout_sec : float
            Maximum time to wait for each service (default 10.0)

        Returns
        -------
        bool
            True if all services are available, False otherwise

        """
        services = [
            (self._load_client, 'load_controller'),
            (self._configure_client, 'configure_controller'),
            (self._switch_client, 'switch_controller'),
            (self._unload_client, 'unload_controller'),
            (self._list_client, 'list_controllers'),
        ]
        for client, name in services:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self._logger.error(f'{name} service not available')
                return False
        return True

    def load_controller(self, name: str, timeout_sec: float = 10.0) -> bool:
        """
        Load a controller.

        Args
        ----
        name : str
            Name of the controller to load
        timeout_sec : float
            Timeout for the service call (default 10.0)

        Returns
        -------
        bool
            True if controller was loaded successfully

        """
        self._logger.info(f'Loading controller {name}...')
        request = LoadController.Request()
        request.name = name

        future = self._load_client.call_async(request)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=timeout_sec
        )

        if not future.done():
            self._logger.error(f'Load request for {name} did not complete')
            return False

        if future.result() is None or not future.result().ok:
            self._logger.error(f'Failed to load controller {name}')
            return False

        self._logger.info(f'Controller {name} loaded successfully')
        return True

    def configure_controller(
        self, name: str, timeout_sec: float = 10.0
    ) -> bool:
        """
        Configure a controller.

        Args
        ----
        name : str
            Name of the controller to configure
        timeout_sec : float
            Timeout for the service call (default 10.0)

        Returns
        -------
        bool
            True if controller was configured successfully

        """
        self._logger.info(f'Configuring controller {name}...')
        request = ConfigureController.Request()
        request.name = name

        future = self._configure_client.call_async(request)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=timeout_sec
        )

        if not future.done():
            self._logger.error(
                f'Configure request for {name} did not complete'
            )
            return False

        if future.result() is None or not future.result().ok:
            self._logger.error(f'Failed to configure controller {name}')
            return False

        self._logger.info(f'Controller {name} configured successfully')
        return True

    def switch_controllers(
        self,
        activate: list = None,
        deactivate: list = None,
        strict: bool = True,
        timeout_sec: float = 10.0,
    ) -> bool:
        """
        Switch controllers (activate and/or deactivate).

        When both activate and deactivate lists are provided, the switch
        is performed atomically.

        Args
        ----
        activate : list
            List of controller names to activate (default None)
        deactivate : list
            List of controller names to deactivate (default None)
        strict : bool
            If True, use STRICT mode; if False, use BEST_EFFORT (default True)
        timeout_sec : float
            Timeout for the service call (default 10.0)

        Returns
        -------
        bool
            True if switch was successful

        """
        activate = activate or []
        deactivate = deactivate or []

        if not activate and not deactivate:
            self._logger.warning(
                'switch_controllers called with no controllers to switch'
            )
            return True

        action_desc = []
        if activate:
            action_desc.append(f'activate {activate}')
        if deactivate:
            action_desc.append(f'deactivate {deactivate}')
        self._logger.info(
            f'Switching controllers: {", ".join(action_desc)}...'
        )

        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = (
            SwitchController.Request.STRICT
            if strict
            else SwitchController.Request.BEST_EFFORT
        )
        request.activate_asap = True
        request.timeout = _create_duration(5)

        future = self._switch_client.call_async(request)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=timeout_sec
        )

        if not future.done():
            self._logger.error('Switch controller request did not complete')
            return False

        if future.result() is None or not future.result().ok:
            self._logger.error(
                f'Failed to switch controllers. '
                f'Activate: {activate}, Deactivate: {deactivate}'
            )
            return False

        self._logger.info('Controller switch successful')
        return True

    def unload_controller(self, name: str, timeout_sec: float = 5.0) -> bool:
        """
        Unload a controller.

        This is a best-effort operation - failures are logged as warnings
        but do not indicate a critical error.

        Args
        ----
        name : str
            Name of the controller to unload
        timeout_sec : float
            Timeout for the service call (default 5.0)

        Returns
        -------
        bool
            True if controller was unloaded successfully

        """
        self._logger.info(f'Unloading controller {name}...')
        request = UnloadController.Request()
        request.name = name

        future = self._unload_client.call_async(request)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=timeout_sec
        )

        if not future.done():
            self._logger.warning(f'Unload request for {name} did not complete')
            return False

        if future.result() is None or not future.result().ok:
            self._logger.warning(f'Failed to unload controller {name}')
            return False

        self._logger.info(f'Controller {name} unloaded successfully')
        return True

    def list_controllers(self, timeout_sec: float = 5.0) -> list:
        """
        List all controllers.

        Returns
        -------
        list
            List of controller info objects, or empty list on failure

        """
        request = ListControllers.Request()
        future = self._list_client.call_async(request)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=timeout_sec
        )

        if future.done() and future.result() is not None:
            return list(future.result().controller)
        return []

    def wait_for_controller_state(
        self,
        controller_name: str,
        target_states: list,
        timeout_sec: float = 30.0,
    ) -> bool:
        """
        Wait for a controller to reach one of the target states.

        Args
        ----
        controller_name : str
            Name of the controller to check
        target_states : list
            List of acceptable states (e.g., ['inactive', 'active'])
        timeout_sec : float
            Maximum time to wait (default 30.0)

        Returns
        -------
        bool
            True if controller reached one of the target states

        """
        self._logger.info(
            f'Waiting for controller {controller_name} to reach state {target_states}...'
        )

        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            controllers = self.list_controllers()

            controller_found = False
            for controller in controllers:
                if controller.name == controller_name:
                    controller_found = True
                    if controller.state in target_states:
                        self._logger.info(
                            f'Controller {controller_name} is in state: {controller.state}'
                        )
                        return True
                    else:
                        self._logger.debug(
                            f'Controller {controller_name} is in state: {controller.state}, '
                            f'waiting for {target_states}'
                        )
                    break

            if not controller_found:
                self._logger.debug(
                    f'Controller {controller_name} not found in controller list yet'
                )

            time.sleep(0.5)
            rclpy.spin_once(self._node, timeout_sec=0.1)

        self._logger.error(
            f'Timeout waiting for {controller_name} to reach state {target_states}'
        )
        return False

    def destroy(self):
        """Destroy all service clients."""
        self._node.destroy_client(self._load_client)
        self._node.destroy_client(self._configure_client)
        self._node.destroy_client(self._switch_client)
        self._node.destroy_client(self._unload_client)
        self._node.destroy_client(self._list_client)
