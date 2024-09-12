#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

from .mock_msg import MockMsg
from geometry_msgs.msg import PoseWithCovarianceStamped

class MockPoseWithCovarianceStamped_(MockMsg):
    def __init__(self):
        self.poseWithCovarianceStamped = PoseWithCovarianceStamped()

    def get_posewithcovariancestamped(self):
        return self.poseWithCovarianceStamped