#!/usr/bin/python3
import time

from sensor_msgs.msg import NavSatFix
import ixcom.protocol
# import numpy as np
from .navsatstatus import NavSatStatus_
from .fun import CovarianceType
from .fun import valid_topics, TimestampMode, gps_timestamp, get_param
from math import pi
import time


class NavSatFix_INS_(NavSatFix):

    def __init__(self):
        self.node = None
        self.active = False
        self._alt_mode = 0
        self._leap_seconds = 0
        self._timestamp_mode = TimestampMode.ros
        self.INS_data_is_set = False
        self.EKF_data_is_set = False
        self.GNSS_data_is_set = False
        self.navSatFix = NavSatFix()
        self.navSatStatus = NavSatStatus_()

    def activate(self, node, leap_seconds, timestamp_mode):
        self.node = node
        self.active = True
        self._leap_seconds = leap_seconds
        self._timestamp_mode = timestamp_mode
        self.navSatFix.header.frame_id = 'wgs84'

        par = get_param(self.node, ixcom.parameters.PARDAT_POS_Payload.parameter_id)
        # posMode = int(par.payload.data['posMode'])
        self._alt_mode = int(par.payload.data['altMode'])  # 0: WGS84, 1: MSL, 2: BARO
        return self._alt_mode

    def is_active(self):
        return self.active

    def __set_timestamp(self, msg):
        # self.navSatFix.header.frame_id = 'wgs84'
        if self._timestamp_mode == TimestampMode.gps:
            self.navSatFix.header.stamp = gps_timestamp(msg.header, self._leap_seconds)
        else:
            self.navSatFix.header.stamp = self.node.get_clock().now().to_msg()

    def set_msg_data(self, msg):
        # print('navsatfix_ins set_msg_data')
        if isinstance(msg.payload, ixcom.messages.INSSOL_Payload):
            # print('ins')
            self.__set_INS_data(msg)
        if isinstance(msg.payload, ixcom.messages.EKFSTDDEV_Payload):
            # print('ekf')
            self.__set_EKF_data(msg)
        if isinstance(msg.payload, ixcom.messages.GNSSSOL_Payload):
            # print('gnss')
            self.__set_GNSS_data(msg)

    def set_par_data(self, par):
        self.navSatStatus.set_par_data(par)

    def __set_INS_data(self, msg):
        self.navSatStatus.set_msg_data(msg)
        self.__set_lla(msg)
        # self.__set_pos_covariance(msg)
        self.__set_pos_covariance_type(msg)
        # print('******** SET INS')

        self.__set_timestamp(msg)

        self.INS_data_is_set = True

    def __set_EKF_data(self, msg):
        self.__set_pos_covariance(msg)
        # print('******** SET EKF')
        self.EKF_data_is_set = True

    def __set_GNSS_data(self, msg):
        # print('setting GNSS data')
        self.navSatStatus.set_msg_data(msg)
        self.GNSS_data_is_set = True

    def __set_lla(self, msg):
        self.navSatFix.latitude = float(msg.payload.data['lat']) / pi * 180.0
        self.navSatFix.longitude = float(msg.payload.data['lon']) / pi * 180.0

        undulation = 0.0
        ds = msg.payload.data['DatSel']
        # if ds & (1 << 7):
        #     print('WGS84')
        if self._alt_mode == 1:
        # if ds & (1 << 8):
            # print('MSL')
            undulation = int(msg.payload.data['undulation']) / 100.0
        # if ds & (1 << 9):
        #     print('BARO')
        self.navSatFix.altitude = float(msg.payload.data['alt']) + undulation
        # print(f"INS alt above geoid: {float(msg.payload.data['alt'])}")
        # print(f"INS alt above ellipsoid: {self.navSatFix.altitude}")

    def __set_pos_covariance(self, msg):
        # self.navSatFix.position_covariance[0] = pow(msg.data['pos'][0], 2)
        self.navSatFix.position_covariance[0] = msg.data['pos'][0] ** 2
        self.navSatFix.position_covariance[1] = 0
        self.navSatFix.position_covariance[2] = 0
        self.navSatFix.position_covariance[3] = 0
        # self.navSatFix.position_covariance[4] = pow(msg.data['pos'][1], 2)
        self.navSatFix.position_covariance[4] = msg.data['pos'][1] ** 2
        self.navSatFix.position_covariance[5] = 0
        self.navSatFix.position_covariance[6] = 0
        self.navSatFix.position_covariance[7] = 0
        # self.navSatFix.position_covariance[8] = pow(msg.data['pos'][2], 2)
        self.navSatFix.position_covariance[8] = msg.data['pos'][2] ** 2

    def __set_pos_covariance_type(self, msg):
        res = CovarianceType.UNKNOWN
        st = msg.bottom.gStatus
        if (st & (1 << 14)) or (st & (1 << 15)):
            res = CovarianceType.DIAGONAL_KNOWN
        self.navSatFix.position_covariance_type = res

    def is_complete(self):
        return self.INS_data_is_set \
            and self.EKF_data_is_set \
            and self.GNSS_data_is_set

    def get_navSatFix(self):
        self.INS_data_is_set = False  # data will only be sent if INS data is set
        self.navSatFix.status = self.navSatStatus.get_navSatStatus()
        # print(f'_INS: lat({self.navSatFix.latitude}), lon({self.navSatFix.longitude}), alt({self.navSatFix.altitude})')
        return self.navSatFix
