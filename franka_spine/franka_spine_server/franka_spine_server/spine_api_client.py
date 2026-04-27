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

"""HTTP client for the Franka Spine REST API."""

from typing import Any, Dict, Optional, Tuple

import requests


class SpineApiClient:
    """Client for the Franka Spine REST API."""

    def __init__(self, spine_ip: str, timeout: float = 5.0):
        """
        Initialize the Spine API client.

        :param spine_ip: IP address or hostname of the spine device.
        :param timeout: HTTP request timeout in seconds.
        """
        self.base_url = f'https://{spine_ip}/spine/api'
        self.timeout = timeout
        self.session = requests.Session()
        self.session.headers.update({'Content-Type': 'application/json'})
        self.session.verify = False

    def _get(self, endpoint: str) -> Tuple[bool, Any]:
        """
        Perform a GET request.

        :return: Tuple of (success, response_data_or_error_message).
        """
        try:
            response = self.session.get(f'{self.base_url}/{endpoint}', timeout=self.timeout)
            response.raise_for_status()
            return True, response.json()
        except requests.exceptions.RequestException as e:
            return False, str(e)

    def _post(self, endpoint: str, data: Optional[Dict] = None) -> Tuple[bool, Any]:
        """
        Perform a POST request.

        :return: Tuple of (success, response_data_or_error_message).
        """
        try:
            response = self.session.post(
                f'{self.base_url}/{endpoint}',
                json=data,
                timeout=self.timeout,
            )
            response.raise_for_status()
            return True, response.json()
        except requests.exceptions.RequestException as e:
            return False, str(e)

    def get_position(self) -> Tuple[bool, Any]:
        """
        GET /api/spine/position-mm.

        Returns position converted from mm to metres.
        """
        success, data = self._get('position-mm')
        if success and isinstance(data, dict) and 'position' in data:
            data['position'] = data['position'] / 1000.0
        return success, data

    def get_state(self) -> Tuple[bool, Any]:
        """GET /api/spine/state."""
        return self._get('state')

    def fault_reset(self) -> Tuple[bool, Any]:
        """POST /api/spine/spine:fault-reset."""
        return self._post('spine:fault-reset')

    def switch_off(self) -> Tuple[bool, Any]:
        """POST /api/spine/spine:switch-off."""
        return self._post('spine:switch-off')

    def switch_on(self) -> Tuple[bool, Any]:
        """POST /api/spine/spine:switch-on."""
        return self._post('spine:switch-on')

    def start_motion(
        self,
        position: float,
        velocity: float,
        acceleration: float,
        deceleration: float,
    ) -> Tuple[bool, Any]:
        """
        POST /api/spine/motion-mm:start.

        Accepts values in metres and converts from (m/s) / (m/s²) to (mm/s) / (mm/s²).

        :param position: Target position in metres.
        :param velocity: Motion velocity in m/s.
        :param acceleration: Motion acceleration in m/s².
        :param deceleration: Motion deceleration in m/s².
        """
        data = {
            'position': int(round(position * 1000)),
            'velocity': int(round(velocity * 1000)),
            'acceleration': int(round(acceleration * 1000)),
            'deceleration': int(round(deceleration * 1000)),
        }
        return self._post('motion-mm:start', data)

    def halt_motion(self) -> Tuple[bool, Any]:
        """POST /api/spine/motion:halt."""
        return self._post('motion:halt')

    def get_parameters(self) -> Tuple[bool, Any]:
        """
        GET /api/spine/parameters.

        Converts user-limit values from mm to metres.
        """
        success, data = self._get('parameters')
        if success and isinstance(data, dict):
            limits = data.get('user_limits', {})
            if limits:
                try:
                    limits['lower_limit'] = float(limits.pop('lower_limit_in_mm', 0)) / 1000.0
                    limits['upper_limit'] = float(limits.pop('upper_limit_in_mm', 0)) / 1000.0
                except (ValueError, TypeError):
                    limits.setdefault('lower_limit', 0.0)
                    limits.setdefault('upper_limit', 0.0)
        return success, data
