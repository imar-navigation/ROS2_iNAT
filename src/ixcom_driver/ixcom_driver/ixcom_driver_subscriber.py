#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

import sys
# sys.path.append('/home/zp/files/git/mdt/ext/ixcomcl/dependencies/ixcom-public')
# sys.path.append('/home/zp/files/git/ixcom_client_to_bytes_fix/dependencies/ixcom-public')
import os
import time
from timeit import default_timer as timer
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import json
from .mypy.fun import *
import argparse

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from ixcom_interfaces.msg import Num
from ixcom_interfaces.msg import Sphere
from ixcom_interfaces.msg import Array

from sensor_msgs.msg import Imu, NavSatStatus, NavSatFix, TimeReference, MagneticField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from tf2_msgs.msg import TFMessage

from ixcom_interfaces.action import GetConfigFile
from rclpy.qos import *


class DataSubscriber(Node):

    def __init__(self, measure_frequency=False, namespace=''):
        # f = open('config.json', 'r')
        # device = json.load(f)
        # topic = device['topic']
        # topic = self.get_parameter('topic').get_parameter_value().string_value
        # self.namespace = '' if namespace == '' else f'{namespace}/'
        self.namespace = namespace
        super().__init__('ixcom_driver_sub', namespace=self.namespace)

        self.qos = None
        # self.qos = qos_profile_sensor_data
        # self.qos = qos_profile_system_default

        self._action_client_config = ActionClient(self, GetConfigFile, 'get_config_file')
        self.config_file = None
        self.requesting_config_file = True
        self.measure_frequency = measure_frequency
        self.nmb_of_frames_for_frq = 10
        self.frq_imu = []
        self.frq_navsatstatus = []
        self.frq_navsatfixgnss = []
        self.frq_navsatfixins = []
        self.frq_timereference = []
        self.frq_magneticfield = []
        self.frq_odometry = []
        self.frq_posewithcovariancestamped = []
        self.frq_twiststamped = []
        self.frq_transformstamped = []
        self.ms = 0

        # self.imu_f = open("ts_imu_sub.csv", "a")
        # self.imu_old_utc_s = 0
        # self.imu_old_utc_ns = 0
        #
        # self.odo_f = open("ts_odo_sub.csv", "a")
        # self.odo_old_utc_s = 0
        # self.odo_old_utc_ns = 0

    def rx_data(self):
        config_ok = False
        if self.config_file is not None:
            wrkdir = os.getcwd()
            self.get_logger().info(f'using config file: {os.path.join(wrkdir, self.config_file)}')
            try:
                config = load_config(self.config_file)
                config_ok = True
            except Exception as e:
                self.get_logger().info(f'error while loading config: {str(e)}')

        topic_Imu = 'Imu'
        topic_NavSatStatus = 'NavSatStatus'
        topic_NavSatFixGnss = 'NavSatFix_GNSS'
        topic_NavSatFixIns = 'NavSatFix_INS'
        topic_TimeReference = 'TimeReference'
        topic_MagneticField = 'MagneticField'
        topic_Odometry = 'Odometry'
        topic_PoseWithCovarianceStamped = 'PoseWithCovarianceStamped'
        # topic_PoseStamped = 'PoseStamped'
        # topic_TwistWithCovarianceStamped = 'TwistWithCovarianceStamped'
        topic_TwistStamped = 'TwistStamped'
        topic_TransformStamped = 'tf_static'
        if config_ok:
            self.qos = get_qos(config[keys.qos])
            remap_Imu = config[keys.topics]['Imu'][keys.remap_to]
            if len(remap_Imu) > 0:
                topic_Imu = remap_Imu

            remap_NavSatStatus = config[keys.topics]['NavSatStatus'][keys.remap_to]
            if len(remap_NavSatStatus) > 0:
                topic_NavSatStatus = remap_NavSatStatus

            remap_NavSatFixGnss = config[keys.topics]['NavSatFix_GNSS'][keys.remap_to]
            if len(remap_NavSatFixGnss) > 0:
                topic_NavSatFixGnss = remap_NavSatFixGnss

            remap_NavSatFixIns = config[keys.topics]['NavSatFix_INS'][keys.remap_to]
            if len(remap_NavSatFixIns) > 0:
                topic_NavSatFixIns = remap_NavSatFixIns

            remap_TimeReference = config[keys.topics]['TimeReference'][keys.remap_to]
            if len(remap_TimeReference) > 0:
                topic_TimeReference = remap_TimeReference

            remap_MagneticField = config[keys.topics]['MagneticField'][keys.remap_to]
            if len(remap_MagneticField) > 0:
                topic_MagneticField = remap_MagneticField

            remap_Odometry = config[keys.topics]['Odometry'][keys.remap_to]
            if len(remap_Odometry) > 0:
                topic_Odometry = remap_Odometry

            remap_PoseWithCovarianceStamped = config[keys.topics]['PoseWithCovarianceStamped'][keys.remap_to]
            if len(remap_PoseWithCovarianceStamped) > 0:
                topic_PoseWithCovarianceStamped = remap_PoseWithCovarianceStamped

            # remap_PoseStamped = config[keys.topics]['PoseStamped'][keys.remap_to]
            # if len(remap_PoseStamped) > 0:
            #     topic_PoseStamped = remap_PoseStamped
            #
            # remap_TwistWithCovarianceStamped = config[keys.topics]['TwistWithCovarianceStamped'][keys.remap_to]
            # if len(remap_TwistWithCovarianceStamped) > 0:
            #     topic_TwistWithCovarianceStamped = remap_TwistWithCovarianceStamped

            remap_TwistStamped = config[keys.topics]['TwistStamped'][keys.remap_to]
            if len(remap_TwistStamped) > 0:
                topic_TwistStamped = remap_TwistStamped
        else:
            self.get_logger().info('using implemented standard topics without config file')
            self.get_logger().info(f'using default QoS settings: {valid_qos.names.SYSTEMDEFAULTS.value}')
            self.qos = qos_profile_system_default



        self.subscription = self.create_subscription(
            Imu,
            topic_Imu,
            self.listener_callback_Imu,
            self.qos)
        self.get_logger().info(f'Subscribed to topic: {topic_Imu}')

        self.subscription = self.create_subscription(
            NavSatStatus,
            topic_NavSatStatus,
            self.listener_callback_NavSatStatus,
            self.qos)
        self.get_logger().info(f'Subscribed to topic: {topic_NavSatStatus}')

        self.subscription = self.create_subscription(
            NavSatFix,
            topic_NavSatFixGnss,
            self.listener_callback_NavSatFixGnss,
            self.qos)
        self.get_logger().info(f'Subscribed to topic: {topic_NavSatFixGnss}')

        self.subscription = self.create_subscription(
            NavSatFix,
            topic_NavSatFixIns,
            self.listener_callback_NavSatFixIns,
            self.qos)
        self.get_logger().info(f'Subscribed to topic: {topic_NavSatFixIns}')

        self.subscription = self.create_subscription(
            TimeReference,
            topic_TimeReference,
            self.listener_callback_TimeReference,
            self.qos)
        self.get_logger().info(f'Subscribed to topic: {topic_TimeReference}')

        self.subscription = self.create_subscription(
            MagneticField,
            topic_MagneticField,
            self.listener_callback_MagneticField,
            self.qos)
        self.get_logger().info(f'Subscribed to topic: {topic_MagneticField}')
        
        self.subscription = self.create_subscription(
            Odometry,
            topic_Odometry,
            self.listener_callback_Odometry,
            self.qos)
        self.get_logger().info(f'Subscribed to topic: {topic_Odometry}')

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            topic_PoseWithCovarianceStamped,
            self.listener_callback_PoseWithCovarianceStamped,
            self.qos)
        self.get_logger().info(f'Subscribed to topic: {topic_PoseWithCovarianceStamped}')

        # self.subscription = self.create_subscription(
        #     PoseStamped,
        #     topic_PoseStamped,
        #     self.listener_callback_PoseStamped,
        #     10)
        # self.get_logger().info(f'Subscribed to topic: {topic_PoseStamped}')

        # self.subscription = self.create_subscription(
        #     TwistWithCovarianceStamped,
        #     topic_TwistWithCovarianceStamped,
        #     self.listener_callback_TwistWithCovarianceStamped,
        #     10)
        # self.get_logger().info(f'Subscribed to topic: {topic_TwistWithCovarianceStamped}')

        self.subscription = self.create_subscription(
            TwistStamped,
            topic_TwistStamped,
            self.listener_callback_TwistStamped,
            self.qos)
        self.get_logger().info(f'Subscribed to topic: {topic_TwistStamped}')
        
        self.subscription = self.create_subscription(
            TFMessage,
            topic_TransformStamped,
            self.listener_callback_TransformStamped,
            self.qos)
        self.get_logger().info(f'Subscribed to topic: {topic_TransformStamped}')

    def get_config_file(self, opt):

        to = 10

        self.get_logger().info(f'requesting config file from publisher with {to} s timeout...')

        goal_msg = GetConfigFile.Goal()
        goal_msg.opt = int(opt)

        got_response = self._action_client_config.wait_for_server(timeout_sec=to)
        if got_response:
            self.get_logger().info('server response arrived')
            self._send_get_config_future = self._action_client_config.send_goal_async(goal_msg)
            self._send_get_config_future.add_done_callback(self.get_config_file_callback)
        else:
            self.get_logger().info('server response timeout')
            self.requesting_config_file = False

    def get_config_file_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('config file request rejected!')
            self.requesting_config_file = False
            return

        self.get_logger().info('config file request accepted!')

        self._get_config_result_future = goal_handle.get_result_async()
        self._get_config_result_future.add_done_callback(self.get_config_file_result_callback)

    def requesting_complete(self):
        return not self.requesting_config_file

    def get_config_file_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'response from publisher: {result.resp}')
        self.config_file = result.resp
        self.requesting_config_file = False
        # rclpy.shutdown()

    def listener_callback_Imu(self, msg):
        if self.measure_frequency:
            if len(self.frq_imu) == 10:
                self.get_logger().info(f'Imu topic frequency: {get_calculated_frequency(self.frq_imu)}')
                self.frq_imu.pop(0)
            self.frq_imu.append(timer())
        else:
        # ct = int(self.get_clock().now().nanoseconds / 1000 / 1000)
        # ms = ct - self.ms
        # self.ms = ct
        # self.get_logger().info(f'IMU {msg.header.frame_id} dt: {ms} / {int(1 / self.frq_imu * 1000)}')
        # self.get_logger().info(f'IMU ({hex(int(msg.header.frame_id))}) '
        #                        f'Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
            self.get_logger().info(f'IMU ({msg.header.frame_id}) '
                                   f'Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
            # try:
            #     add_utc = 0
            #     if msg.header.stamp.sec > self.imu_old_utc_s:
            #         add_utc = 1000000000
            #     self.imu_f.write(f'{msg.header.frame_id},'
            #                      f'{msg.header.stamp.sec},{msg.header.stamp.nanosec},'
            #                      f'{msg.header.stamp.nanosec + add_utc - self.imu_old_utc_ns}\n')
            #     self.imu_f.flush()
            #     self.imu_old_utc_s = msg.header.stamp.sec
            #     self.imu_old_utc_ns = msg.header.stamp.nanosec
            # except Exception as e:
            #     print(f'EXCEPTION: {e}')

    def listener_callback_NavSatStatus(self, msg):
        if self.measure_frequency:
            if len(self.frq_navsatstatus) == 10:
                self.get_logger().info(f'NavSatStatus topic frequency: {get_calculated_frequency(self.frq_navsatstatus)}')
                self.frq_navsatstatus.pop(0)
            self.frq_navsatstatus.append(timer())
        else:
            self.get_logger().info(f'NavSat Status: {msg.status} '
                                   f'Service: {msg.service}')

    def listener_callback_NavSatFixGnss(self, msg):
        if self.measure_frequency:
            if len(self.frq_navsatfixgnss) == 10:
                self.get_logger().info(f'NavSatFix_GNSS topic frequency: {get_calculated_frequency(self.frq_navsatfixgnss)}')
                self.frq_navsatfixgnss.pop(0)
            self.frq_navsatfixgnss.append(timer())
        else:
            # self.get_logger().info(f'NavSatFix ({hex(int(msg.header.frame_id))}) '
            #                        f'Status: {msg.status.status} '
            #                        f'Service: {msg.status.service}')
            self.get_logger().info(f'NavSatFix_GNSS ({msg.header.frame_id}) '
                                   f'Status: {msg.status.status} '
                                   f'Service: {msg.status.service} '
                                   f'lat/lon/alt: {msg.latitude}/{msg.longitude}/{msg.altitude}')

    def listener_callback_NavSatFixIns(self, msg):
        if self.measure_frequency:
            if len(self.frq_navsatfixins) == 10:
                self.get_logger().info(f'NavSatFix_INS topic frequency: {get_calculated_frequency(self.frq_navsatfixins)}')
                self.frq_navsatfixins.pop(0)
            self.frq_navsatfixins.append(timer())
        else:
            # self.get_logger().info(f'NavSatFix_INS ({hex(int(msg.header.frame_id))}) '
            #                        f'Status: {msg.status.status} '
            #                        f'Service: {msg.status.service}')
            self.get_logger().info(f'NavSatFix_INS ({msg.header.frame_id}) '
                                   f'Status: {msg.status.status} '
                                   f'Service: {msg.status.service} '
                                   f'lat/lon/alt: {msg.latitude}/{msg.longitude}/{msg.altitude}')

    def listener_callback_TimeReference(self, msg):
        if self.measure_frequency:
            if len(self.frq_timereference) == 10:
                self.get_logger().info(f'TimeReference topic frequency: {get_calculated_frequency(self.frq_timereference)}')
                self.frq_timereference.pop(0)
            self.frq_timereference.append(timer())
        else:
            # self.get_logger().info(f'TimeReference ({hex(int(msg.header.frame_id))}) '
            #                        f'Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec} '
            #                        f'TimeReference: {msg.time_ref.sec}.{msg.time_ref.nanosec}')
            self.get_logger().info(f'TimeReference ({msg.header.frame_id}) '
                                   f'Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec} '
                                   f'TimeReference: {msg.time_ref.sec}.{msg.time_ref.nanosec} '
                                   f'Source: {msg.source}')

    def listener_callback_MagneticField(self, msg):
        if self.measure_frequency:
            if len(self.frq_magneticfield) == 10:
                self.get_logger().info(f'MagneticField topic frequency: {get_calculated_frequency(self.frq_magneticfield)}')
                self.frq_magneticfield.pop(0)
            self.frq_magneticfield.append(timer())
        else:
            # self.get_logger().info(f'MagneticField ({hex(int(msg.header.frame_id))}) '
            #                        f'x: {msg.magnetic_field.x} '
            #                        f'y: {msg.magnetic_field.y} '
            #                        f'z: {msg.magnetic_field.z} ')
            self.get_logger().info(f'MagneticField ({msg.header.frame_id}) '
                                   f'x: {msg.magnetic_field.x} '
                                   f'y: {msg.magnetic_field.y} '
                                   f'z: {msg.magnetic_field.z} ')
        
    def listener_callback_Odometry(self, msg):
        if self.measure_frequency:
            if len(self.frq_odometry) == 10:
                self.get_logger().info(f'Odometry topic frequency: {get_calculated_frequency(self.frq_odometry)}')
                self.frq_odometry.pop(0)
            self.frq_odometry.append(timer())
        else:
            self.get_logger().info(f'ODO ({msg.header.frame_id}) '
                                   f'Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}'
                                   f'x: {msg.pose.pose.position.x} '
                                   f'y: {msg.pose.pose.position.y} '
                                   f'z: {msg.pose.pose.position.z} '
                                   f'tlx: {msg.twist.twist.linear.x} '
                                   f'tly: {msg.twist.twist.linear.y} '
                                   f'tlz: {msg.twist.twist.linear.z} '
                                   f'tax: {msg.twist.twist.angular.x} '
                                   f'tay: {msg.twist.twist.angular.y} '
                                   f'taz: {msg.twist.twist.angular.z} '
                                   )
            # try:
            #     add_utc = 0
            #     if msg.header.stamp.sec > self.odo_old_utc_s:
            #         add_utc = 1000000000
            #     self.odo_f.write(f'{msg.header.frame_id},'
            #                  f'{msg.header.stamp.sec},{msg.header.stamp.nanosec},'
            #                  f'{msg.header.stamp.nanosec + add_utc - self.odo_old_utc_ns}\n')
            #     self.odo_f.flush()
            #     self.odo_old_utc_s = msg.header.stamp.sec
            #     self.odo_old_utc_ns = msg.header.stamp.nanosec
            # except Exception as e:
            #     print(f'EXCEPTION: {e}')

    def listener_callback_PoseWithCovarianceStamped(self, msg):
        if self.measure_frequency:
            if len(self.frq_posewithcovariancestamped) == 10:
                self.get_logger().info(f'PoseStamped topic frequency: {get_calculated_frequency(self.frq_posewithcovariancestamped)}')
                self.frq_posewithcovariancestamped.pop(0)
            self.frq_posewithcovariancestamped.append(timer())
        else:
            # self.get_logger().info(f'PoseWithCovarianceStamped ({hex(int(msg.header.frame_id))}) '
            #                        f'x: {msg.pose.pose.position.x} '
            #                        f'y: {msg.pose.pose.position.y} '
            #                        f'z: {msg.pose.pose.position.z} ')
            self.get_logger().info(f'PoseWithCovarianceStamped ({msg.header.frame_id}) '
                                   f'x: {msg.pose.pose.position.x} '
                                   f'y: {msg.pose.pose.position.y} '
                                   f'z: {msg.pose.pose.position.z} ')

    # def listener_callback_PoseStamped(self, msg):
    #     # self.get_logger().info(f'PoseStamped ({hex(int(msg.header.frame_id))}) '
    #     #                        f'x: {msg.pose.position.x} '
    #     #                        f'y: {msg.pose.position.y} '
    #     #                        f'z: {msg.pose.position.z} ')
    #     self.get_logger().info(f'PoseStamped ({msg.header.frame_id}) '
    #                            f'x: {msg.pose.position.x} '
    #                            f'y: {msg.pose.position.y} '
    #                            f'z: {msg.pose.position.z} ')
    #
    # def listener_callback_TwistWithCovarianceStamped(self, msg):
    #     # self.get_logger().info(f'TwistWithCovarianceStamped ({hex(int(msg.header.frame_id))}) '
    #     #                        f'x: {msg.twist.twist.angular.x} '
    #     #                        f'y: {msg.twist.twist.angular.y} '
    #     #                        f'z: {msg.twist.twist.angular.z} ')
    #     self.get_logger().info(f'TwistWithCovarianceStamped ({msg.header.frame_id}) '
    #                            f'x: {msg.twist.twist.angular.x} '
    #                            f'y: {msg.twist.twist.angular.y} '
    #                            f'z: {msg.twist.twist.angular.z} ')

    def listener_callback_TwistStamped(self, msg):
        if self.measure_frequency:
            if len(self.frq_twiststamped) == 10:
                self.get_logger().info(f'TwistStamped topic frequency: {get_calculated_frequency(self.frq_twiststamped)}')
                self.frq_twiststamped.pop(0)
            self.frq_twiststamped.append(timer())
        else:
            self.get_logger().info(f'TwistStamped ({msg.header.frame_id}) '
                                   f'x: {msg.twist.angular.x} '
                                   f'y: {msg.twist.angular.y} '
                                   f'z: {msg.twist.angular.z} ')
                                   
    def listener_callback_TransformStamped(self, msg):
        if self.measure_frequency:
            if len(self.frq_transformstamped) == 10:
                self.get_logger().info(f'TransformStamped topic frequency: {get_calculated_frequency(self.frq_transformstamped)}')
                self.frq_transformstamped.pop(0)
            self.frq_transformstamped.append(timer())
        else:
            for transform in msg.transforms:
              self.get_logger().info(f'frame ID: {transform.header.frame_id}')
              self.get_logger().info(f'  child frame ID: {transform.child_frame_id}')
              self.get_logger().info(f'    translation:')
              self.get_logger().info(f'      x: {transform.transform.translation.x}')
              self.get_logger().info(f'      y: {transform.transform.translation.y}')
              self.get_logger().info(f'      z: {transform.transform.translation.z}')
              self.get_logger().info(f'    rotation:')
              self.get_logger().info(f'      x: {transform.transform.rotation.x}')
              self.get_logger().info(f'      y: {transform.transform.rotation.y}')
              self.get_logger().info(f'      z: {transform.transform.rotation.z}')
              self.get_logger().info(f'      w: {transform.transform.rotation.w}')

def main():
    rclpy.init()

    parser = argparse.ArgumentParser(description='ROS2 iXCOM Driver')
    parser.add_argument('-f', '--measure_frequency',  action='store_true', default=False, help='measure frequencies of incoming topics')
    parser.add_argument('-ns', '--namespace', type=str, help='node namspace', default='')
    args = parser.parse_args()

    dSub = DataSubscriber(args.measure_frequency, args.namespace)
    # dSub = DataSubscriber()
    try:
        # print('getting config file...')
        dSub.get_config_file(0)
        # print('done')
        # rclpy.spin_once(dSub)
        for i in range(10):
            if dSub.requesting_complete():
                break
            rclpy.spin_once(dSub)

        dSub.rx_data()
        rclpy.spin(dSub)
    except KeyboardInterrupt:
        dSub.destroy_node()
        exit(0)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dSub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
