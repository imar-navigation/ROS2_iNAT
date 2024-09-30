#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

from rclpy.node import Node
import os
from ..mock.mock_imu import MockImu_
from ..mock.mock_navsatstatus import MockNavSatStatus_
from ..mock.mock_navsatfix import MockNavSatFix_
from ..mock.mock_navsatfixins import MockNavSatFix_INS_
from ..mock.mock_timereference import MockTimeReference_
from ..mock.mock_magneticfield import MockMagneticField_
from ..mock.mock_odometry import MockOdometry_
from ..mock.mock_posewithcovariancestamped import MockPoseWithCovarianceStamped_
from ..mock.mock_twistwithcovariancestamped import MockTwistWithCovarianceStamped_
from ..mock.mock_posestamped import MockPoseStamped_
from ..mock.mock_publisher import MockPublisher
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

class DummyNode(Node):
    def __init__(self):
        self.qos = qos_profile_system_default
        self.dev_ip = '0.0.0.0.'
        self.dev_port = '0'
        self.running = True
        self.nanoseconds = 123456789
        self.t = 88888888
        self.log_dir = f'test{os.sep}log{os.sep}run'
        self.imu = MockImu_()
        self.navSatStatus = MockNavSatStatus_()
        self.navSatFix = MockNavSatFix_()
        self.navSatFixIns = MockNavSatFix_INS_()
        self.timeReference = MockTimeReference_()
        self.magneticField = MockMagneticField_()
        self.odometry = MockOdometry_()
        self.poseWithCovarianceStamped = MockPoseWithCovarianceStamped_()
        self.twistWithCovarianceStamped = MockTwistWithCovarianceStamped_()
        self.poseStamped = MockPoseStamped_()
        self.publisher_imu = MockPublisher()
        self.publisher_magneticfield = MockPublisher()
        self.publisher_odometry = MockPublisher()
        self.publisher_navsatstatus = MockPublisher()
        self.publisher_navsatfix = MockPublisher()
        self.publisher_navsatfix_ins = MockPublisher()
        self.publisher_timereference = MockPublisher()
        self.publisher_posewithcovariancestamped = MockPublisher()

    def get_log_dir(self):
        return self.log_dir

    def get_clock(self):
        return self

    def now(self):
        return self

    def get_logger(self):
        return self

    def info(self, msg):
        print(msg)

    def warning(self, msg):
        print(msg)

    def error(self, msg):
        print(msg)

    def create_publisher(self, Msg, topic_name, l):
        return 'id', 'ret_msg'