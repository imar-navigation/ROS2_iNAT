#!/usr/bin/python3

from sensor_msgs.msg import NavSatFix
import ixcom.protocol
from .navsatstatus import NavSatStatus_
from .fun import CovarianceType
from .fun import valid_topics, TimestampMode, gps_timestamp
from math import pi


class NavSatFix_(NavSatFix):

    def __init__(self):
        self.node = None
        self.active = False
        self._leap_seconds = 0
        self._timestamp_mode = TimestampMode.ros
        self.navSatFix = NavSatFix()
        self.navSatStatus = NavSatStatus_()

    def activate(self, node, leap_seconds, timestamp_mode):
        self.node = node
        self.active = True
        self._leap_seconds = leap_seconds
        self._timestamp_mode = timestamp_mode
        self.navSatFix.header.frame_id = 'primary_antenna'

    def is_active(self):
        return self.active

    def __set_timestamp(self, msg):
        # self.navSatFix.header.frame_id = 'primary_antenna'
        if self._timestamp_mode == TimestampMode.gps:
            self.navSatFix.header.stamp = gps_timestamp(msg.header, self._leap_seconds)
        else:
            self.navSatFix.header.stamp = self.node.get_clock().now().to_msg()

    def set_msg_data(self, msg):
        if isinstance(msg.payload, ixcom.messages.GNSSSOL_Payload):
            self.__set_GNSS_data(msg)

    def set_par_data(self, par):
        self.navSatStatus.set_par_data(par)

    def __set_GNSS_data(self, msg):
        self.navSatStatus.set_msg_data(msg)
        self.__set_lla(msg)
        self.__set_pos_covariance(msg)
        self.__set_pos_covariance_type(msg)

        self.__set_timestamp(msg)

    def __set_lla(self, msg):
        self.navSatFix.latitude = float(msg.payload.data['lat']) / pi * 180.0
        self.navSatFix.longitude = float(msg.payload.data['lon']) / pi * 180.0
        self.navSatFix.altitude = float(msg.payload.data['alt']) + float(msg.payload.data['undulation'])
        # print(f"GNSS alt above geoid: {float(msg.payload.data['alt'])}")
        # print(f"GNSS alt above ellipsoid: {self.navSatFix.altitude}")

    def __set_pos_covariance(self, msg):
        # self.navSatFix.position_covariance[0] = pow(msg.data['stdDevPos'][0], 2)
        self.navSatFix.position_covariance[0] = msg.data['stdDevPos'][0] ** 2
        self.navSatFix.position_covariance[1] = 0
        self.navSatFix.position_covariance[2] = 0
        self.navSatFix.position_covariance[3] = 0
        # self.navSatFix.position_covariance[4] = pow(msg.data['stdDevPos'][1], 2)
        self.navSatFix.position_covariance[4] = msg.data['stdDevPos'][1] ** 2
        self.navSatFix.position_covariance[5] = 0
        self.navSatFix.position_covariance[6] = 0
        self.navSatFix.position_covariance[7] = 0
        # self.navSatFix.position_covariance[8] = pow(msg.data['stdDevPos'][2], 2)
        self.navSatFix.position_covariance[8] = msg.data['stdDevPos'][2] ** 2

    def __set_pos_covariance_type(self, msg):
        res = CovarianceType.UNKNOWN

        pt = int(msg.payload.data['posType'])
        # STATUS_FIX       if  XCOM-GNSSSOL.PosVelType.FIXEDPOS                                                val:  1
        if pt > 0:
            res = CovarianceType.DIAGONAL_KNOWN

        self.navSatFix.position_covariance_type = res

    def get_navSatFix(self):
        self.navSatFix.status = self.navSatStatus.get_navSatStatus()
        # print(f'GNSS: lat({self.navSatFix.latitude}), lon({self.navSatFix.longitude}), alt({self.navSatFix.altitude})')
        return self.navSatFix
