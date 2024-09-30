#!/usr/bin/python3

from geometry_msgs.msg import TwistStamped
import ixcom.protocol
from .fun import valid_topics, TimestampMode, gps_timestamp

class TwistStamped_(TwistStamped):

    def __init__(self):
        self.node = None
        self.active = False
        self._leap_seconds = 0
        self._timestamp_mode = TimestampMode.ros
        self.twiststamped = TwistStamped()
        self.timestamp_mode = TimestampMode.ros

    def activate(self, node, leap_seconds, timestamp_mode):
        self.node = node
        self.active = True
        self._leap_seconds = leap_seconds
        self._timestamp_mode = timestamp_mode
        self.twiststamped.header.frame_id = 'inat_output_frame'

    def is_active(self):
        return self.active

    def __set_timestamp(self, msg):
        # self.twiststamped.header.frame_id = 'inat_output_frame'
        if self._timestamp_mode == TimestampMode.gps:
            self.twiststamped.header.stamp = gps_timestamp(msg.header, self._leap_seconds)
        else:
            self.twiststamped.header.stamp = self.node.get_clock().now().to_msg()

    def set_msg_data(self, msg):
        if isinstance(msg.payload, ixcom.messages.INSSOL_Payload):
            self.__set_INS_data(msg)

    def __set_INS_data(self, msg):
        self.twiststamped.twist.linear.x = msg.data['vel'][0]
        self.twiststamped.twist.linear.y = msg.data['vel'][1]
        self.twiststamped.twist.linear.z = msg.data['vel'][2]
        self.twiststamped.twist.angular.x = msg.data['omg'][0]
        self.twiststamped.twist.angular.y = msg.data['omg'][1]
        self.twiststamped.twist.angular.z = msg.data['omg'][2]

        self.__set_timestamp(msg)


    def get_twiststamped(self):
        return self.twiststamped
