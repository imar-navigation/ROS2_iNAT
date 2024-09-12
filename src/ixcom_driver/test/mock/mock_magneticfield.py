#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

from .mock_msg import MockMsg
from sensor_msgs.msg import MagneticField

class MockMagneticField_(MockMsg):
    def __init__(self):
        self.magneticField = MagneticField()

    def get_magneticField(self):
        return self.magneticField