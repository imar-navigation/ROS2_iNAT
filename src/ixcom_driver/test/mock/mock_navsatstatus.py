#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

from .mock_msg import MockMsg
from sensor_msgs.msg import NavSatStatus

class MockNavSatStatus_(MockMsg):
    def __init__(self):
        self.navSatStatus = NavSatStatus()

    def get_navSatStatus(self):
        return self.navSatStatus
