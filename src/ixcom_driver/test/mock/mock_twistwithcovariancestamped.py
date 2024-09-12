#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

from .mock_msg import MockMsg
from geometry_msgs.msg import TwistWithCovarianceStamped

class MockTwistWithCovarianceStamped_(MockMsg):
    def __init__(self):
        self.twistWithCovarianceStamped = TwistWithCovarianceStamped()

    def get_twistwithcovariancestamped(self):
        self.twistWithCovarianceStamped