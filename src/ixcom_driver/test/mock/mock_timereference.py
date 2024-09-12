#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

from .mock_msg import MockMsg
from sensor_msgs.msg import TimeReference

class MockTimeReference_(MockMsg):
    def __init__(self):
        self.timeReference = TimeReference()

    def get_timeReference(self):
        return self.timeReference