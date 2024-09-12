#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

from .mock_msg import MockMsg
from nav_msgs.msg import Odometry

class MockOdometry_(MockMsg):
    def __init__(self):
        self.odometry = Odometry()

    def get_odometry(self):
        return self.odometry