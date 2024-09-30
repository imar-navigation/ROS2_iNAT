#!/usr/bin/python3

from geometry_msgs.msg import PoseWithCovarianceStamped
import ixcom.protocol
from .fun import rpyToQuat
from .fun import valid_topics, TimestampMode, gps_timestamp

class PoseWithCovarianceStamped_(PoseWithCovarianceStamped):

    def __init__(self):
        self.node = None
        self.active = False
        self._leap_seconds = 0
        self._timestamp_mode = TimestampMode.ros
        self.INS_data_is_set = False
        self.EKF_data_is_set = False
        # self.IMUCORR_data_is_set = False
        self.posewithcovariancestamped = PoseWithCovarianceStamped()

    def activate(self, node, leap_seconds, timestamp_mode):
        self.node = node
        self.active = True
        self._leap_seconds = leap_seconds
        self._timestamp_mode = timestamp_mode
        self.posewithcovariancestamped.header.frame_id = 'ecef'

    def is_active(self):
        return self.active

    def __set_timestamp(self, msg):
        # self.posewithcovariancestamped.header.frame_id = 'ecef'
        if self._timestamp_mode == TimestampMode.gps:
            self.posewithcovariancestamped.header.stamp = gps_timestamp(msg.header, self._leap_seconds)
        else:
            self.posewithcovariancestamped.header.stamp = self.node.get_clock().now().to_msg()

    def set_msg_data(self, msg):
        if isinstance(msg.payload, ixcom.messages.INSSOL_Payload):
            self.__set_INS_data(msg)
        if isinstance(msg.payload, ixcom.messages.EKFSTDDEV_Payload):
            self.__set_EKF_data(msg)
        # if isinstance(msg.payload, ixcom.messages.IMUCORR_Payload):
        #     self.__set_IMUCORR_data(msg)

    def __set_INS_data(self, msg):
        self.posewithcovariancestamped.pose.pose.position.x = msg.data['lon']
        self.posewithcovariancestamped.pose.pose.position.y = msg.data['lat']
        self.posewithcovariancestamped.pose.pose.position.z = msg.data['alt']
        q = rpyToQuat(msg.data['rpy'])
        self.posewithcovariancestamped.pose.pose.orientation.x = q[0]
        self.posewithcovariancestamped.pose.pose.orientation.y = q[1]
        self.posewithcovariancestamped.pose.pose.orientation.z = q[2]
        self.posewithcovariancestamped.pose.pose.orientation.w = q[3]

        self.__set_timestamp(msg)

        self.INS_data_is_set = True

    def __set_EKF_data(self, msg):
        # self.posewithcovariancestamped.pose.covariance[0] = pow(msg.data['pos'][0], 2)
        # self.posewithcovariancestamped.pose.covariance[7] = pow(msg.data['pos'][1], 2)
        # self.posewithcovariancestamped.pose.covariance[14] = pow(msg.data['pos'][2], 2)
        # self.posewithcovariancestamped.pose.covariance[21] = pow(msg.data['tilt'][0], 2)
        # self.posewithcovariancestamped.pose.covariance[28] = pow(msg.data['tilt'][1], 2)
        # self.posewithcovariancestamped.pose.covariance[35] = pow(msg.data['tilt'][2], 2)
        self.posewithcovariancestamped.pose.covariance[0] = msg.data['pos'][0] ** 2
        self.posewithcovariancestamped.pose.covariance[7] = msg.data['pos'][1] ** 2
        self.posewithcovariancestamped.pose.covariance[14] = msg.data['pos'][2] ** 2
        self.posewithcovariancestamped.pose.covariance[21] = msg.data['tilt'][0] ** 2
        self.posewithcovariancestamped.pose.covariance[28] = msg.data['tilt'][1] ** 2
        self.posewithcovariancestamped.pose.covariance[35] = msg.data['tilt'][2] ** 2

        self.EKF_data_is_set = True

    def is_complete(self):
        return self.INS_data_is_set \
            and self.EKF_data_is_set

    def get_posewithcovariancestamped(self):
        self.INS_data_is_set = False  # data will only be sent if INS data is set
        return self.posewithcovariancestamped
