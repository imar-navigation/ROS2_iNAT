#!/usr/bin/python3

from sensor_msgs.msg import TimeReference
import ixcom.protocol
from .fun import valid_topics, TimestampMode, gps_timestamp
# import numpy as np


class TimeReference_(TimeReference):

    def __init__(self):
        self.node = None
        self.active = False
        self._leap_seconds = 0
        self._timestamp_mode = TimestampMode.ros
        self.timeReference = TimeReference()

    def activate(self, node, leap_seconds, timestamp_mode):
        self.node = node
        self.active = True
        self._leap_seconds = leap_seconds
        self._timestamp_mode = timestamp_mode
        self.timeReference.header.frame_id = str(valid_topics.items['timereference']['id'])

    def is_active(self):
        return self.active

    def __set_timestamp(self, msg):
        # self.timeReference.header.frame_id = str(valid_topics.items['timereference']['id'])
        if self._timestamp_mode == TimestampMode.gps:
            self.timeReference.header.stamp = gps_timestamp(msg.header, self._leap_seconds)
            self.timeReference.source = 'GPS'
        else:
            self.timeReference.header.stamp = self.node.get_clock().now().to_msg()
            self.timeReference.source = 'ROS'

    def set_msg_data(self, msg):
        if isinstance(msg.payload, ixcom.messages.SYSSTAT_Payload):
            self.timeReference.time_ref.sec = msg.header.timeOfWeek_sec
            self.timeReference.time_ref.nanosec = msg.header.timeOfWeek_usec * 1000
            self.__set_timestamp(msg)

    def get_timeReference(self):
        return self.timeReference
