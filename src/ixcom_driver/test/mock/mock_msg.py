#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH


class MockMsg:

    def __init__(self):
        self.active = False
        self._node = None
        self._maintiming = 0
        self._leap_seconds = 0
        self._timestamp_mode = ''

    def activate(self, node=None, maintiming=500, leap_seconds=18, timestamp_mode='GPS_SYNC'):
        self._node = node
        self._maintiming = maintiming
        self._leap_seconds = leap_seconds
        self._timestamp_mode = timestamp_mode
        self.active = True

    def is_active(self):
        return True

    def set_msg_data(self, msg):
        pass

    def is_complete(self):
        return True

    def set_par_data(self, par):
        pass

    def set_header(self, node, msg):
        pass
