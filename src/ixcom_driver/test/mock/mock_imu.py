#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

from sensor_msgs.msg import Imu
from .mock_msg import MockMsg

class MockImu_(MockMsg):
    def __init__(self):
        self.imu = Imu()

    def get_imu(self):
        return self.imu