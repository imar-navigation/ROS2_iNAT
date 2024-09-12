#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

from .mock_msg import MockMsg
from geometry_msgs.msg import PoseStamped

class MockPoseStamped_(MockMsg):
    def __init__(self):
        self.poseStamped = PoseStamped()

    def get_posestamped(self):
        return self.poseStamped