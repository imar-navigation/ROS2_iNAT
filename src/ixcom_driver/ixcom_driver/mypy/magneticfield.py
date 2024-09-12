#!/usr/bin/python3

from sensor_msgs.msg import MagneticField
import ixcom.protocol
from .fun import valid_topics, TimestampMode, gps_timestamp
# import numpy as np


class MagneticField_(MagneticField):

    def __init__(self):
        self.node = None
        self.active = False
        self._leap_seconds = 0
        self._timestamp_mode = TimestampMode.ros
        self.magneticField = MagneticField()

    def activate(self, node, leap_seconds, timestamp_mode):
        self.node = node
        self.active = True
        self._leap_seconds = leap_seconds
        self._timestamp_mode = timestamp_mode

    def is_active(self):
        return self.active

    def __set_timestamp(self, msg):
        self.magneticField.header.frame_id = str(valid_topics.items['magneticfield']['id'])
        if self._timestamp_mode == TimestampMode.gps:
            self.magneticField.header.stamp = gps_timestamp(msg.header, self._leap_seconds)
        else:
            self.magneticField.header.stamp = self.node.get_clock().now().to_msg()

    def set_msg_data(self, msg):
        if isinstance(msg.payload, ixcom.messages.MAGDATA_Payload):
            self.__set_MAG_data(msg)

    def set_par_data(self, par):
        if (par is not None) and isinstance(par.payload, ixcom.parameters.PAREKF_MAGATTAID_Payload):
            self.__set_mag_covariance(par)

    def __set_MAG_data(self, msg):
        self.magneticField.magnetic_field.x = msg.data['field'][0] * 1e-7    # mG to Tesla (TBD: coming data is not mG)
        self.magneticField.magnetic_field.y = msg.data['field'][1] * 1e-7    # mG to Tesla (TBD: coming data is not mG)
        self.magneticField.magnetic_field.z = msg.data['field'][2] * 1e-7    # mG to Tesla (TBD: coming data is not mG)

        self.__set_timestamp(msg)

    def __set_mag_covariance(self, par):
        self.magneticField.magnetic_field_covariance[0] = par.data['magFieldStdDev'][0] * 1e-7       # mG to Tesla
        # self.magneticField.magnetic_field_covariance[1] =
        # self.magneticField.magnetic_field_covariance[2] =
        # self.magneticField.magnetic_field_covariance[3] =
        self.magneticField.magnetic_field_covariance[4] = par.data['magFieldStdDev'][1] * 1e-7       # mG to Tesla
        # self.magneticField.magnetic_field_covariance[5] =
        # self.magneticField.magnetic_field_covariance[6] =
        # self.magneticField.magnetic_field_covariance[7] =
        self.magneticField.magnetic_field_covariance[8] = par.data['magFieldStdDev'][2] * 1e-7       # mG to Tesla

    def get_magneticField(self):
        return self.magneticField
