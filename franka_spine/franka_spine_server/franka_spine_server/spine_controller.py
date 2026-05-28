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

"""Spine controller that encapsulates all spine business logic."""

from dataclasses import dataclass
import logging
import threading
import time
from typing import Callable, Optional

from franka_spine_server.spine_api_client import SpineApiClient

FAULT_STATES = ('Fault', 'FaultReactionActive')


@dataclass
class MotionResult:
    """Result of a move operation."""

    success: bool
    stop_by: str = ''
    error: str = ''
    cancelled: bool = False


@dataclass
class StateResult:
    """Result of a state query."""

    success: bool
    state: str = ''
    error_code: str = ''
    error_description: str = ''


@dataclass
class PositionResult:
    """Result of a position query."""

    success: bool
    position: float = 0.0


@dataclass
class CommandResult:
    """Result of a simple command (switch on/off, fault reset, halt)."""

    success: bool
    state: str = ''
    message: str = ''


@dataclass
class UserLimitsData:
    """User-defined motion limits."""

    lower_limit: float = 0.0
    upper_limit: float = 0.0


@dataclass
class ParametersResult:
    """Result of a parameters query."""

    success: bool
    user_limits: Optional[UserLimitsData] = None
    message: str = ''


class SpineController:
    """Handles all spine operations, independent of the ROS 2 layer."""

    def __init__(
        self,
        spine_ip: str,
        timeout: float = 5.0,
        feedback_rate: float = 10.0,
        logger: Optional[logging.Logger] = None,
    ):
        self.api = SpineApiClient(spine_ip, timeout=timeout)
        self.feedback_rate = feedback_rate
        self.logger = logger or logging.getLogger(__name__)
        self._motion_lock = threading.Lock()

    @property
    def motion_in_progress(self) -> bool:
        """Return True if a motion is currently running."""
        return self._motion_lock.locked()

    def get_state(self) -> StateResult:
        """Query the current spine state."""
        success, data = self.api.get_state()
        if not success:
            self.logger.error(f'get_state failed: {data}')
            return StateResult(success=False)

        result = StateResult(success=True)
        if isinstance(data, str):
            result.state = data
        elif isinstance(data, dict):
            result.state = data.get('state', '')
            error = data.get('error', {})
            if error:
                result.error_code = str(error.get('code', ''))
                result.error_description = error.get('description', '')
        return result

    def get_position(self) -> PositionResult:
        """Query the current position in metres."""
        success, data = self.api.get_position()
        if not success:
            self.logger.error(f'get_position failed: {data}')
            return PositionResult(success=False)
        return PositionResult(success=True, position=data.get('position', 0.0))

    def fault_reset(self) -> CommandResult:
        """Reset the spine from a fault state."""
        success, data = self.api.fault_reset()
        if not success:
            msg = f'Fault reset failed: {data}'
            self.logger.error(msg)
            return CommandResult(success=False, message=msg)
        state = data if isinstance(data, str) else str(data)
        return CommandResult(success=True, state=state, message='Fault reset successful')

    def switch_on(self) -> CommandResult:
        """Switch the spine on."""
        success, data = self.api.switch_on()
        if not success:
            msg = f'Switch on failed: {data}'
            self.logger.error(msg)
            return CommandResult(success=False, message=msg)
        state = data if isinstance(data, str) else str(data)
        return CommandResult(success=True, state=state, message='Switch on successful')

    def switch_off(self) -> CommandResult:
        """Switch the spine off."""
        success, data = self.api.switch_off()
        if not success:
            msg = f'Switch off failed: {data}'
            self.logger.error(msg)
            return CommandResult(success=False, message=msg)
        state = data if isinstance(data, str) else str(data)
        return CommandResult(success=True, state=state, message='Switch off successful')

    def halt(self) -> CommandResult:
        """Halt any ongoing motion."""
        success, data = self.api.halt_motion()
        if not success:
            msg = f'Halt failed: {data}'
            self.logger.error(msg)
            return CommandResult(success=False, message=msg)
        state = data if isinstance(data, str) else str(data)
        return CommandResult(success=True, state=state, message='Halt successful')

    def get_parameters(self) -> ParametersResult:
        """Retrieve spine parameters (user limits)."""
        success, data = self.api.get_parameters()
        if not success:
            msg = f'Failed to get parameters: {data}'
            self.logger.error(msg)
            return ParametersResult(success=False, message=msg)

        user_limits_data = data.get('user_limits', {})
        return ParametersResult(
            success=True,
            user_limits=UserLimitsData(
                lower_limit=user_limits_data.get('lower_limit', 0.0),
                upper_limit=user_limits_data.get('upper_limit', 0.0),
            ),
            message='Parameters retrieved successfully',
        )

    def move_absolute(
        self,
        position: float,
        velocity: float,
        acceleration: float,
        deceleration: float,
        is_cancelled: Callable[[], bool] = lambda: False,
        on_feedback: Optional[Callable[[float], None]] = None,
        is_active: Callable[[], bool] = lambda: True,
    ) -> MotionResult:
        """
        Execute an absolute move and block until completion.

        :param position: Target position in metres.
        :param velocity: Motion velocity in m/s.
        :param acceleration: Motion acceleration in m/s².
        :param deceleration: Motion deceleration in m/s².
        :param is_cancelled: Callable returning True when cancelled.
        :param on_feedback: Called with current position (m) each cycle.
        :param is_active: Callable returning True while system is running.
        """
        with self._motion_lock:
            success, data = self.api.start_motion(position, velocity, acceleration, deceleration)
            if not success:
                self.logger.error(f'Failed to start motion: {data}')
                return MotionResult(success=False, error=str(data))

            stop_by = data.get('StopBy', '') if isinstance(data, dict) else ''

            cancelled = self._poll_position_until_done(
                is_cancelled=is_cancelled,
                on_feedback=on_feedback,
                is_active=is_active,
            )

        if cancelled:
            return MotionResult(
                success=False,
                stop_by=stop_by,
                error='Motion cancelled',
                cancelled=True,
            )
        return MotionResult(success=True, stop_by=stop_by)

    def _poll_position_until_done(
        self,
        is_cancelled: Callable[[], bool],
        on_feedback: Optional[Callable[[float], None]],
        is_active: Callable[[], bool],
    ) -> bool:
        """
        Poll position until stable or cancelled.

        Returns True if cancelled.
        """
        period = 1.0 / self.feedback_rate
        position_stable_count = 0
        last_position = None

        while is_active():
            if is_cancelled():
                self.logger.info('Motion cancelled, sending halt')
                self.api.halt_motion()
                return True

            success, data = self.api.get_position()
            if success:
                current_position = data.get('position', 0.0)
                if on_feedback:
                    on_feedback(current_position)

                if last_position is not None and current_position == last_position:
                    position_stable_count += 1
                else:
                    position_stable_count = 0
                last_position = current_position

                if position_stable_count >= 3:
                    self.logger.info(f'Motion complete at position {current_position:.4f} m')
                    return False
            else:
                self.logger.warning(f'Failed to read position: {data}')

            state_success, state_data = self.api.get_state()
            if state_success and isinstance(state_data, str):
                if state_data in FAULT_STATES:
                    self.logger.error(f'Spine entered fault state: {state_data}')
                    return False

            time.sleep(period)

        return False
