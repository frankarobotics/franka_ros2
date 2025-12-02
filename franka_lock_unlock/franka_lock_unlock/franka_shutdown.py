#!/usr/bin/env python3
# Copyright jk-ethz
# Released under GNU AGPL-3.0
# Contact us for other licensing options.

# Developed and tested on system version
# 4.2.1

# Inspired by
# https://github.com/frankaemika/libfranka/issues/63
# https://github.com/ib101/DVK/blob/master/Code/DVK.py

from itertools import count
from time import sleep
import argparse
from requests.exceptions import ConnectionError
from urllib.parse import urljoin
from franka_client import FrankaClient


class FrankaShutdown(FrankaClient):
    def __init__(self, hostname: str, username: str, password: str, protocol: str = 'https'):
        super().__init__(hostname, username, password, protocol=protocol)

    def run(self, wait: bool = False, request: bool = False) -> None:
        assert not request or wait, "Requesting control without waiting for obtaining control is not supported."
        self._login()
        try:
            assert self._token is not None or self._get_active_token_id() is None or wait, "Error requesting control, the robot is currently in use."
            while True:
                self._request_token(physically=False)
                try:
                    # Consider the timeout of 20 s for requesting physical access to the robot
                    for _ in range(20) if request else count():
                        if (not wait and not request) or self._is_active_token():
                            print('Successfully acquired control over the robot.')
                            return
                        if request:
                            print('Please press the button with the (blue) circle on the robot to confirm physical access.')
                        elif wait:
                            print('Please confirm the request message in the web interface on the logged in user.')
                        sleep(1)
                    # In case physical access was not confirmed, try again
                    self._release_token()
                finally:
                    self._shutdown()
        finally:
            pass


def main():
    parser = argparse.ArgumentParser(
                                     prog = 'franka_shutdown.py',
                                     description = 'Shutdown the Franka Emika Panda programmatically.',
                                     epilog = '(c) jk-ethz, https://github.com/jk-ethz'
                                    )
    parser.add_argument('hostname', help='The Franka Desk IP address or hostname, for example "1.2.3.4".')
    parser.add_argument('username', help='The Franka Desk username, usually "admin".')
    parser.add_argument('password', help='The Franka Desk password.')
    parser.add_argument('-w', '--wait', action='store_true', help='Wait in case the robot web UI is currently in use.')
    parser.add_argument('-r', '--request', action='store_true', help='Request control by confirming physical access to the robot in case the robot web UI is currently in use.')
    args, _ = parser.parse_known_args()

    franka_lock_unlock = FrankaShutdown(hostname=args.hostname, username=args.username, password=args.password)
    franka_lock_unlock.run(wait=args.wait, request=args.request)

if __name__ == '__main__':
    main()
