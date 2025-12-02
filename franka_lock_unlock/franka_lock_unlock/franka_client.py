# Copyright jk-ethz
# Released under GNU AGPL-3.0
# Contact us for other licensing options.

# Developed and tested on system version
# 4.2.1

# Inspired by
# https://github.com/frankaemika/libfranka/issues/63
# https://github.com/ib101/DVK/blob/master/Code/DVK.py

from abc import ABC, abstractmethod
import hashlib
import base64
import requests
from urllib.parse import urljoin
from http import HTTPStatus


class FrankaClient(ABC):
    def __init__(self, hostname: str, username: str, password: str, protocol: str = 'https'):
        requests.packages.urllib3.disable_warnings()
        self._session = requests.Session()
        self._session.verify = False
        self._hostname = f'{protocol}://{hostname}'
        self._username = username
        self._password = password
        self._logged_in = False
        self._token = None
        self._token_id = None

    @staticmethod
    def _encode_password(username, password):
        bs = ','.join([str(b) for b in hashlib.sha256((f'{password}#{username}@franka').encode('utf-8')).digest()])
        return base64.encodebytes(bs.encode('utf-8')).decode('utf-8')

    def _login(self):
        print("Logging in...")
        if self._logged_in:
            print("Already logged in.")
            return
        login = self._session.post(urljoin(self._hostname, '/admin/api/login'), \
                                           json={'login': self._username, \
                                                 'password': self._encode_password(self._username, self._password)})
        assert login.status_code == HTTPStatus.OK, "Error logging in."
        self._session.cookies.set('authorization', login.text)
        self._logged_in = True
        print("Successfully logged in.")

    def _logout(self):
        print("Logging out...")
        assert self._logged_in
        logout = self._session.post(urljoin(self._hostname, '/admin/api/logout'))
        assert logout.status_code == HTTPStatus.OK, "Error logging out"
        self._session.cookies.clear()
        self._logged_in = False
        print("Successfully logged out.")

    def _shutdown(self):
        print("Shutting down...")
        assert self._is_active_token(), "Cannot shutdown without an active control token."
        try:
            self._session.post(urljoin(self._hostname, '/admin/api/shutdown'), json={'token': self._token})
        except requests.exceptions.RequestException as _:
            # Sometimes, the server can shut down before sending a complete response, possibly raising an exception.
            # Anyways, the server has still received the request, thus the robot shutdown procedure will start.
            # So, we can ignore the cases when these exceptions are raised.
            pass
        finally:
            print("The robot is shutting down. Please wait for the yellow lights to turn off, then switch the control box off.")

    def _get_active_token_id(self):
        token_query = self._session.get(urljoin(self._hostname, '/admin/api/control-token'))
        assert token_query.status_code == HTTPStatus.OK, "Error getting control token status."
        json = token_query.json()
        return None if json['activeToken'] is None else json['activeToken']['id']

    def _is_active_token(self):
        active_token_id = self._get_active_token_id()
        return active_token_id is None or active_token_id == self._token_id

    def _request_token(self, physically=False):
        print("Requesting a control token...")
        if self._token is not None:
            assert self._token_id is not None
            print("Already having a control token.")
            return
        token_request = self._session.post(urljoin(self._hostname, f'/admin/api/control-token/request{"?force" if physically else ""}'), \
                                           json={'requestedBy': self._username})
        assert token_request.status_code == HTTPStatus.OK, "Error requesting control token."
        json = token_request.json()
        self._token = json['token']
        self._token_id = json['id']
        print(f'Received control token is {self._token} with id {self._token_id}.')

    def _release_token(self):
        print("Releasing control token...")
        token_delete = self._session.delete(urljoin(self._hostname, '/admin/api/control-token'), \
                                                    json={'token': self._token})
        assert token_delete.status_code == 200, "Error releasing control token."
        self._token = None
        self._token_id = None
        print("Successfully released control token.")

    @abstractmethod
    def run(self) -> None:
        pass
