#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

from .mock_msg import MockMsg
from sensor_msgs.msg import NavSatFix

class MockNavSatFix_INS_(MockMsg):
    def __init__(self):
        self.navSatFix = NavSatFix()

    def get_navSatFix(self):
        return self.navSatFix