#!/usr/bin/python3

from nav_msgs.msg import Odometry
import ixcom.protocol
# import numpy as np
from .fun import rpyToQuat, TimestampMode, gps_timestamp


class Odometry_(Odometry):

    def __init__(self):
        self.node = None
        self.active = False
        self._leap_seconds = 0
        self._timestamp_mode = TimestampMode.ros
        self.INS_data_is_set = False
        self.EKF_data_is_set = False
        self.IMUCORR_data_is_set = False
        self.odometry = Odometry()
        # self.frame_cnt = 0

    def activate(self, node, leap_seconds, timestamp_mode):
        self.node = node
        self.active = True
        self._leap_seconds = leap_seconds
        self._timestamp_mode = timestamp_mode
        self.odometry.header.frame_id = 'enu'

        # self.f = open("ts_odo_pub.csv", "a")
        # self.old_s = 0
        # self.old_us = 0
        # self.old_utc_s = 0
        # self.old_utc_ns = 0

    def is_active(self):
        return self.active

    def __set_timestamp(self, msg):

        # self.odometry.header.frame_id = str(self.frame_cnt)
        # self.frame_cnt += 1
        # if self.frame_cnt == 255:
        #     self.frame_cnt = 0

        # self.odometry.header.frame_id = 'enu'
        # print(f'timestamp mode: {self._timestamp_mode}')
        if self._timestamp_mode == TimestampMode.gps:
            self.odometry.header.stamp = gps_timestamp(msg.header, self._leap_seconds)
        else:
            self.odometry.header.stamp = self.node.get_clock().now().to_msg()
        # print(f'stamp: {self.odometry.header.stamp}')

        # try:
        #     s = msg.header.timeOfWeek_sec
        #     us = msg.header.timeOfWeek_usec
        #     add = 0
        #     add_utc = 0
        #     if s > self.old_s:
        #         add = 1000000
        #     if self.odometry.header.stamp.sec > self.old_utc_s:
        #         add_utc = 1000000000
        #     self.f.write(f'{self.odometry.header.frame_id},{s},{us},{us + add - self.old_us},'
        #                  f'{self.odometry.header.stamp.sec},{self.odometry.header.stamp.nanosec},'
        #                  f'{self.odometry.header.stamp.nanosec + add_utc - self.old_utc_ns}\n')
        #     self.f.flush()
        #     self.old_s = s
        #     self.old_us = us
        #     self.old_utc_s = self.odometry.header.stamp.sec
        #     self.old_utc_ns = self.odometry.header.stamp.nanosec
        # except Exception as e:
        #     print(f'EXCEPTION: {e}')

    def set_msg_data(self, msg):
        self.odometry.child_frame_id = 'inat_enclosure'
        if isinstance(msg.payload, ixcom.messages.INSSOL_Payload):
            self.__set_INS_data(msg)
        if isinstance(msg.payload, ixcom.messages.EKFSTDDEV_Payload):
            self.__set_EKF_data(msg)
        if isinstance(msg.payload, ixcom.messages.IMUCORR_Payload):
            self.__set_IMUCORR_data(msg)

    def __set_INS_data(self, msg):
        self.odometry.pose.pose.position.x = msg.data['lon']
        self.odometry.pose.pose.position.y = msg.data['lat']
        self.odometry.pose.pose.position.z = msg.data['alt']
        q = rpyToQuat(msg.data['rpy'])
        self.odometry.pose.pose.orientation.x = q[0]
        self.odometry.pose.pose.orientation.y = q[1]
        self.odometry.pose.pose.orientation.z = q[2]
        self.odometry.pose.pose.orientation.w = q[3]
        self.odometry.twist.twist.linear.x = msg.data['vel'][0]
        self.odometry.twist.twist.linear.y = msg.data['vel'][1]
        self.odometry.twist.twist.linear.z = msg.data['vel'][2]

        self.INS_data_is_set = True

    def __set_EKF_data(self, msg):
        # self.odometry.pose.covariance[0] = pow(msg.data['pos'][0], 2)
        # self.odometry.pose.covariance[7] = pow(msg.data['pos'][1], 2)
        # self.odometry.pose.covariance[14] = pow(msg.data['pos'][2], 2)
        # self.odometry.pose.covariance[21] = pow(msg.data['tilt'][0], 2)
        # self.odometry.pose.covariance[28] = pow(msg.data['tilt'][1], 2)
        # self.odometry.pose.covariance[35] = pow(msg.data['tilt'][2], 2)
        self.odometry.pose.covariance[0] = msg.data['pos'][0] ** 2
        self.odometry.pose.covariance[7] = msg.data['pos'][1] ** 2
        self.odometry.pose.covariance[14] = msg.data['pos'][2] ** 2
        self.odometry.pose.covariance[21] = msg.data['tilt'][0] ** 2
        self.odometry.pose.covariance[28] = msg.data['tilt'][1] ** 2
        self.odometry.pose.covariance[35] = msg.data['tilt'][2] ** 2

        self.EKF_data_is_set = True

    def __set_IMUCORR_data(self, msg):
        self.odometry.twist.twist.angular.x = msg.data['omg'][0]
        self.odometry.twist.twist.angular.y = msg.data['omg'][1]
        self.odometry.twist.twist.angular.z = msg.data['omg'][2]

        self.__set_timestamp(msg)

        self.IMUCORR_data_is_set = True

    def is_complete(self):
        return self.INS_data_is_set \
            and self.EKF_data_is_set \
            and self.IMUCORR_data_is_set

    def get_odometry(self):
        self.IMUCORR_data_is_set = False  # data will only be sent if this data is set
        return self.odometry
