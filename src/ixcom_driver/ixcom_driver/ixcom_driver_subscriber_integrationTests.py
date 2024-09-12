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
import array as arr

import ixcom.grep
import csv
import os.path

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from interfaces.msg import Num
from interfaces.msg import Sphere
from interfaces.msg import Array

from sensor_msgs.msg import Imu, NavSatStatus, NavSatFix, TimeReference, MagneticField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped

from interfaces.action import GetConfigFile
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


class DataSubscriberIntegrationTests(Node):

    def __init__(self, measure_frequency=False):
        # f = open('config.json', 'r')
        # device = json.load(f)
        # topic = device['topic']
        # topic = self.get_parameter('topic').get_parameter_value().string_value
        super().__init__('ixcom_driver_sub')

        # self.qos = qos_profile_sensor_data
        self.qos = qos_profile_system_default

        self._action_client_config = ActionClient(self, GetConfigFile, 'get_config_file')
        self.config_file = None
        self.requesting_config_file = True
        self.measure_frequency = False#measure_frequency
        self.nmb_of_frames_for_frq = 10
        self.frq_imu = []
        self.frq_imu_counter = 0
        self.frq_imu_desired = 0
        self.frq_navsatstatus = []
        self.frq_navsatstatus_counter = 0
        self.frq_navsatstatus_desired = 0
        self.frq_navsatfix = []
        self.frq_navsatfix_counter = 0
        self.frq_navsatfix_desired = 0
        self.frq_navsatfixins = []
        self.frq_navsatfixins_counter = 0
        self.frq_navsatfixins_desired = 0
        self.frq_timereference = []
        self.frq_timereference_counter = 0
        self.frq_timereference_desired = 0
        self.frq_magneticfield = []
        self.frq_magneticfield_counter = 0
        self.frq_magneticfield_desired = 0
        self.frq_odometry = []
        self.frq_odometry_counter = 0
        self.frq_odometry_desired = 0
        self.frq_posewithcovariancestamped = []
        self.frq_posewithcovariancestamped_counter = 0
        self.frq_posewithcovariancestamped_desired = 0
        self.frq_twiststamped = []
        self.frq_twiststamped_counter = 0
        self.frq_twiststamped_desired = 0
        self.ms = 0

        self.test_timediff_header = False
        self.timestampNodeDataSeconds = []
        self.timestampNodeDataNanoSeconds = []
        self.timestampSum = arr.array("d",[])
        self.imu_f = open("ts_imu_sub.csv", "a")
        self.imu_old_utc_s = 0
        self.imu_old_utc_ns = 0
        #
        # self.odo_f = open("ts_odo_sub.csv", "a")
        # self.odo_old_utc_s = 0
        # self.odo_old_utc_ns = 0
        self.logForTests = True
        #IMU MSG
        self.imuMsgHeaderTimeSec = []
        self.imuMsgHeaderTimeNanoSec = []
        self.imuMsgOrientation = []
        self.imuMsgLinearAcceleration = []
        self.imuMsgAngularVelocity = []




    def rx_data(self, configFileDirectory):
        config_ok = False
        #if self.config_file is not None:
        wrkdir = os.getcwd()
        self.get_logger().info(f'wrkdir: {wrkdir}')
        self.get_logger().info(f'using config file: {os.path.join(configFileDirectory,"config.json")}')
        try:
            config = load_config(configFileDirectory)
            config_ok = True
        except Exception as e:
            self.get_logger().info(f'error while loading config: {str(e)}')

        topic_Imu = 'Imu'
        topic_NavSatStatus = 'NavSatStatus'
        topic_NavSatFix = 'NavSatFix'
        topic_NavSatFixIns = 'NavSatFix_INS'
        topic_TimeReference = 'TimeReference'
        topic_MagneticField = 'MagneticField'
        topic_Odometry = 'Odometry'
        topic_PoseWithCovarianceStamped = 'PoseWithCovarianceStamped'
        # topic_PoseStamped = 'PoseStamped'
        # topic_TwistWithCovarianceStamped = 'TwistWithCovarianceStamped'
        topic_TwistStamped = 'TwistStamped'
        if config_ok:
            remap_Imu = config[keys.topics]['Imu'][keys.remap_to]
            if len(remap_Imu) > 0:
                topic_Imu = remap_Imu

            frequency_hz = config[keys.topics]['Imu'][keys.frequency_hz]
            frq_desired = int(frequency_hz)

            if frq_desired > 0 and frq_desired < 1000:
                self.frq_imu_desired = frq_desired
                self.get_logger().info(f'desired Imu Topic frequenz is: {self.frq_imu_desired}')

            remap_NavSatStatus = config[keys.topics]['NavSatStatus'][keys.remap_to]
            if len(remap_NavSatStatus) > 0:
                topic_NavSatStatus = remap_NavSatStatus

            frequency_hz = config[keys.topics]['NavSatStatus'][keys.frequency_hz]
            frq_desired = int(frequency_hz)
            if frq_desired > 0 and frq_desired < 1000:
                self.frq_navsatstatus_desired = frq_desired
                self.get_logger().info(f'desired NavSatStatus Topic frequenz is: {self.frq_navsatstatus_desired}')

            remap_NavSatFix = 0
            frequency_hz = 0
            try:
                remap_NavSatFix = config[keys.topics]['NavSatFix'][keys.remap_to]
                frequency_hz = config[keys.topics]['NavSatFix'][keys.frequency_hz]
            except:
                remap_NavSatFix = config[keys.topics]['NavSatFix_GNSS'][keys.remap_to]
                frequency_hz = config[keys.topics]['NavSatFix_GNSS'][keys.frequency_hz]

            if len(remap_NavSatFix) > 0:
                topic_NavSatFix = remap_NavSatFix

            #requency_hz = config[keys.topics]['NavSatFix'][keys.frequency_hz]
            frq_desired = int(frequency_hz)
            if frq_desired > 0 and frq_desired < 1000:
                self.frq_navsatfix_desired = frq_desired
                self.get_logger().info(f'desired NavSatFix Topic frequenz is: {self.frq_navsatfix_desired}')


            remap_NavSatFixIns = config[keys.topics]['NavSatFix_INS'][keys.remap_to]
            if len(remap_NavSatFixIns) > 0:
                topic_NavSatFixIns = remap_NavSatFixIns

            frequency_hz = config[keys.topics]['NavSatFix_INS'][keys.frequency_hz]
            frq_desired = int(frequency_hz)
            if frq_desired > 0 and frq_desired < 1000:
                self.frq_navsatfixins_desired = frq_desired
                self.get_logger().info(f'desired NavSatFix_INS Topic frequenz is: {self.frq_navsatfixins_desired}')


            remap_TimeReference = config[keys.topics]['TimeReference'][keys.remap_to]
            if len(remap_TimeReference) > 0:
                topic_TimeReference = remap_TimeReference

            frequency_hz = config[keys.topics]['TimeReference'][keys.frequency_hz]
            frq_desired = int(frequency_hz)
            if frq_desired > 0 and frq_desired < 1000:
                self.frq_timereference_desired = frq_desired
                self.get_logger().info(f'desired TimeReference Topic frequenz is: {self.frq_timereference_desired}')

            remap_MagneticField = config[keys.topics]['MagneticField'][keys.remap_to]
            if len(remap_MagneticField) > 0:
                topic_MagneticField = remap_MagneticField

            frequency_hz = config[keys.topics]['MagneticField'][keys.frequency_hz]
            frq_desired = int(frequency_hz)
            if frq_desired > 0 and frq_desired < 1000:
                self.frq_magneticfield_desired = frq_desired
                self.get_logger().info(f'desired MagneticField Topic frequenz is: {self.frq_magneticfield_desired}')

            remap_Odometry = config[keys.topics]['Odometry'][keys.remap_to]
            if len(remap_Odometry) > 0:
                topic_Odometry = remap_Odometry

            frequency_hz = config[keys.topics]['Odometry'][keys.frequency_hz]
            frq_desired = int(frequency_hz)
            if frq_desired > 0 and frq_desired < 1000:
                self.frq_odometry_desired = frq_desired
                self.get_logger().info(f'desired Odometry Topic frequenz is: {self.frq_odometry_desired}')

            remap_PoseWithCovarianceStamped = config[keys.topics]['PoseWithCovarianceStamped'][keys.remap_to]
            if len(remap_PoseWithCovarianceStamped) > 0:
                topic_PoseWithCovarianceStamped = remap_PoseWithCovarianceStamped

            frequency_hz = config[keys.topics]['PoseWithCovarianceStamped'][keys.frequency_hz]
            frq_desired = int(frequency_hz)
            if frq_desired > 0 and frq_desired < 1000:
                self.frq_posewithcovariancestamped_desired = frq_desired
                self.get_logger().info(f'desired PoseWithCovarianceStamped Topic frequenz is: {self.frq_posewithcovariancestamped_desired}')

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

            frequency_hz = config[keys.topics]['TwistStamped'][keys.frequency_hz]
            frq_desired = int(frequency_hz)
            if frq_desired > 0 and frq_desired < 1000:
                self.frq_twiststamped_desired = frq_desired
                self.get_logger().info(f'desired TwistStamped Topic frequenz is: {self.frq_twiststamped_desired}')
        else:
            self.get_logger().info('using implemented standard topics without config file')


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
            topic_NavSatFix,
            self.listener_callback_NavSatFix,
            self.qos)
        self.get_logger().info(f'Subscribed to topic: {topic_NavSatFix}')

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

    def get_config_file(self, opt):

        to = 5

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

    def waitForFrqCounter(self, startTime, secondsToWait, topic):
        # while ((startTime + secondsToWait*1.1) - time.time() > 0):
            time.sleep(secondsToWait)
            if topic == "Imu":
                counter = self.frq_imu_counter
                print(time.time()-startTime)
                desired_freq = self.frq_imu_desired
            elif topic == "NavSatStatus":
                counter = self.frq_navsatstatus_counter
                desired_freq = self.frq_navsatstatus_desired
                self.get_logger().info(f'frq_navsatstatus_desired: {self.frq_navsatstatus_desired} '
                                    f'Counter: {self.frq_navsatstatus_counter}')
            elif topic == "NavSatFix":
                counter = self.frq_navsatfix_counter
                desired_freq = self.frq_navsatfix_desired
                self.get_logger().info(f'frq_navsatfix_desired: {self.frq_navsatfix_desired} '
                                    f'Counter: {self.frq_navsatfix_counter}')
            elif topic == "NavSatFix_INS":
                counter = self.frq_navsatfixins_counter
                desired_freq = self.frq_navsatfixins_desired
            elif topic == "TimeReference":
                counter = self.frq_timereference_counter
                desired_freq = self.frq_timereference_desired
            elif topic == "MagneticField":
                counter = self.frq_magneticfield_counter
                desired_freq = self.frq_magneticfield_desired
            elif topic == "Odometry":
                counter = self.frq_odometry_counter
                desired_freq = self.frq_odometry_desired
            elif topic == "PoseWithCovarianceStamped":
                counter = self.frq_posewithcovariancestamped_counter
                desired_freq = self.frq_posewithcovariancestamped_desired
            elif topic == "TwistStamped":
                counter = self.frq_twiststamped_counter
                desired_freq = self.frq_twiststamped_desired


            self.get_logger().info(f'WaitForFrqCounter-> {topic} counter {counter}')
            return counter
            if counter >= secondsToWait * desired_freq:

                return counter
            else:
                time.sleep(secondsToWait/(desired_freq*2))

        #return counter

    def waitForMessage(self,topic, secondsToWait)-> int:
        startTime = time.time()
        counter = self.waitForFrqCounter(startTime,secondsToWait, topic)
        return counter


    def resetMessageCounter(self):
        self.frq_imu_counter=0
        self.frq_navsatfixins_counter=0
        self.frq_posewithcovariancestamped_counter = 0
        self.frq_magneticfield_counter = 0
        self.frq_navsatfix_counter = 0
        self.frq_posewithcovariancestamped_counter = 0
        self.frq_navsatstatus_counter = 0
        self.frq_odometry_counter = 0
        self.frq_timereference_counter = 0
        self.frq_twiststamped_counter = 0

    def setLogForTests(self):
        self.logForTests = True
    def setTimeDiffTestPara(self):
        self.test_timediff_header = True

    def getTimeStampSum (self):
        return self.timestampSum

    def listener_callback_Imu(self, msg):
        #self.get_logger().info(f'self.listener_callback_Imu {self.frq_imu_counter}')
        #print(msg)
        # print(msg.orientation)
        # print(msg.orientation.x)

        #print()
        self.frq_imu_counter = self.frq_imu_counter+1

        if self.test_timediff_header == True:
            self.timestampNodeDataSeconds.append(msg.header.stamp.sec)
            nanosecondsInSeconds = msg.header.stamp.nanosec / 1000 / 1000 / 1000
            self.timestampSum.append(nanosecondsInSeconds + msg.header.stamp.sec)

        if self.logForTests == True:
            path = 'imuTopicMsgs.csv'
            check_file = os.path.isfile(path)
            if check_file == True:
                with open(path, 'a') as new_file:
                    csv_writer = csv.writer(new_file)
                    csv_writer.writerow([msg.header.stamp.sec,msg.header.stamp.nanosec,msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w,
                                         msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                                         msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z] )
            else:
                with open(path, 'w') as new_file:
                    fieldnames = ['sec', 'nsec', 'orientationQuatX', 'orientationQuatY', 'orientationQuatZ', 'orientationQuatW', 'angularVelX','angularVelY','angularVelZ','linearAccelerationX','linearAccelerationY','linearAccelerationZ']
                    csv_writer = csv.DictWriter(new_file, fieldnames=fieldnames)
                    csv_writer.writeheader()
                    csv_writer = csv.writer(new_file)
                    csv_writer.writerow([msg.header.stamp.sec,msg.header.stamp.nanosec,msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w,
                                         msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                                         msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])



            # with open('names.csv', 'r') as csv_file:
            #     csv_reader = csv.DictReader(csv_file)

            #     with open('new_names.csv', 'w') as new_file:
            #         fieldnames = ['first_name', 'last_name']

            #         csv_writer = csv.DictWriter(new_file, fieldnames=fieldnames, delimiter='\t')

            #         csv_writer.writeheader()

            #         for line in csv_reader:
            #             del line['email']
            #             csv_writer.writerow(line)



    def listener_callback_NavSatStatus(self, msg):
        #print(msg)
        self.frq_navsatstatus_counter = self.frq_navsatstatus_counter + 1
            # self.get_logger().info(f'NavSat Status: {msg.status} '
            #                        f'Service: {msg.service} '
            #                         f'Counter: {self.frq_navsatstatus_counter}')

    def listener_callback_NavSatFix(self, msg):
        self.frq_navsatfix_counter = self.frq_navsatfix_counter + 1

        # if self.logForTests == True:
        #     path = 'NavSatFixTopicMsgs.csv'

        #     check_file = os.path.isfile(path)
        #     if check_file == True:
        #         with open(path, 'a') as new_file:
        #             csv_writer = csv.writer(new_file)
        #             csv_writer.writerow([msg.header.stamp.sec,msg.header.stamp.nanosec,msg.latitude,msg.longitude,msg.altitude,
        #                                  msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
        #                                  msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        #         # writer = csv.writer(file)
        #         # writer.writerow(['name', 'age'])
        #         # writer.writerow(['John Doe', 30])     
        #     else:
        #         with open(path, 'w') as new_file:
        #             fieldnames = ['sec', 'usec', 'latitude', 'longitude', 'altitude', 'angularVelX','angularVelY','angularVelZ','linearAccelerationX','linearAccelerationY','linearAccelerationZ']
        #             csv_writer = csv.DictWriter(new_file, fieldnames=fieldnames)
        #             csv_writer.writeheader()
        #             csv_writer = csv.writer(new_file)
        #             csv_writer.writerow([msg.header.stamp.sec,msg.header.stamp.nanosec,msg.latitude,msg.longitude,msg.altitude])

            # self.get_logger().info(f'CALLBACK rq_navsatfix_desired: {self.frq_navsatfix_desired} '
            #                         f'Counter: {self.frq_navsatfix_counter}')
            # self.get_logger().info(f'NavSatFix ({msg.header.frame_id}) '
            #                        f'Status: {msg.status.status} '
            #                        f'Service: {msg.status.service} '
            #                        f'lat/lon/alt: {msg.latitude}/{msg.longitude}/{msg.altitude}')

    def listener_callback_NavSatFixIns(self, msg):
        #print(msg)
        self.frq_navsatfixins_counter = self.frq_navsatfixins_counter + 1
        path = 'NavSatFixINSTopicMsgs.csv'

        check_file = os.path.isfile(path)
        if check_file == True:
            with open(path, 'a') as new_file:
                csv_writer = csv.writer(new_file)
                csv_writer.writerow([msg.header.stamp.sec,msg.header.stamp.nanosec,msg.latitude,msg.longitude,msg.altitude])
                                    # msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                                    # msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
   
        else:
            with open(path, 'w') as new_file:
                fieldnames = ['sec', 'nsec', 'latitude', 'longitude', 'altitude']
                csv_writer = csv.DictWriter(new_file, fieldnames=fieldnames)
                csv_writer.writeheader()
                csv_writer = csv.writer(new_file)
                csv_writer.writerow([msg.header.stamp.sec,msg.header.stamp.nanosec,msg.latitude,msg.longitude,msg.altitude])#,
                                        # msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                                        # msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def listener_callback_TimeReference(self, msg):

        self.frq_timereference_counter = self.frq_timereference_counter + 1
        #print(msg)
            # self.get_logger().info(f'TimeReference ({hex(int(msg.header.frame_id))}) '
            #                        f'Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec} '
            #                        f'TimeReference: {msg.time_ref.sec}.{msg.time_ref.nanosec}')
            # self.get_logger().info(f'TimeReference ({msg.header.frame_id}) '
            #                        f'Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec} '
            #                        f'TimeReference: {msg.time_ref.sec}.{msg.time_ref.nanosec}')

    def listener_callback_MagneticField(self, msg):

        self.frq_magneticfield_counter = self.frq_magneticfield_counter + 1
        #print(msg)
            # self.get_logger().info(f'MagneticField ({hex(int(msg.header.frame_id))}) '
            #                        f'x: {msg.magnetic_field.x} '
            #                        f'y: {msg.magnetic_field.y} '
            #                        f'z: {msg.magnetic_field.z} ')
            # self.get_logger().info(f'MagneticField ({msg.header.frame_id}) '
            #                        f'x: {msg.magnetic_field.x} '
            #                        f'y: {msg.magnetic_field.y} '
            #                        f'z: {msg.magnetic_field.z} ')

    def listener_callback_Odometry(self, msg):

        self.frq_odometry_counter = self.frq_odometry_counter + 1
        #print(msg)
            # self.get_logger().info(f'ODO ({msg.header.frame_id}) '
            #                        f'Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}'
                                   # f'x: {msg.pose.pose.position.x} '
                                   # f'y: {msg.pose.pose.position.y} '
                                   # f'z: {msg.pose.pose.position.z} '
                                   # f'tlx: {msg.twist.twist.linear.x} '
                                   # f'tly: {msg.twist.twist.linear.y} '
                                   # f'tlz: {msg.twist.twist.linear.z} '
                                   # f'tax: {msg.twist.twist.angular.x} '
                                   # f'tay: {msg.twist.twist.angular.y} '
                                   # f'taz: {msg.twist.twist.angular.z} '
                                   #)
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

        self.frq_posewithcovariancestamped_counter = self.frq_posewithcovariancestamped_counter + 1
        if self.logForTests == True:
            path = 'PoseWithCovarianceStampedMsg.csv'
            check_file = os.path.isfile(path)
            if check_file == True:
                with open(path, 'a') as new_file:
                    csv_writer = csv.writer(new_file)
                    csv_writer.writerow([msg.header.stamp.sec,msg.header.stamp.nanosec,msg.pose.pose.position.x, msg.pose.pose.position.y,msg.pose.pose.position.z,
                                         msg.pose.covariance[0],msg.pose.covariance[7],msg.pose.covariance[14],msg.pose.covariance[21],msg.pose.covariance[28],msg.pose.covariance[35]] )
            else:
                with open(path, 'w') as new_file:
                    fieldnames = ['sec', 'nsec', 'poseX', 'poseY', 'poseZ','covarianceX','covarianceY','covarianceZ','covarianceRoll','covariancePitch','covarianceYaw']
                    csv_writer = csv.DictWriter(new_file, fieldnames=fieldnames)
                    csv_writer.writeheader()
                    csv_writer = csv.writer(new_file)
                    csv_writer.writerow([msg.header.stamp.sec,msg.header.stamp.nanosec,msg.pose.pose.position.x, msg.pose.pose.position.y,msg.pose.pose.position.z,
                                         msg.pose.covariance[0],msg.pose.covariance[7],msg.pose.covariance[14],msg.pose.covariance[21],msg.pose.covariance[28],msg.pose.covariance[35]])
            # self.get_logger().info(f'PoseWithCovarianceStamped ({hex(int(msg.header.frame_id))}) '
            #                        f'x: {msg.pose.pose.position.x} '
            #                        f'y: {msg.pose.pose.position.y} '
            #                        f'z: {msg.pose.pose.position.z} ')
            # self.get_logger().info(f'PoseWithCovarianceStamped ({msg.header.frame_id}) '
            #                        f'x: {msg.pose.pose.position.x} '
            #                        f'y: {msg.pose.pose.position.y} '
            #                        f'z: {msg.pose.pose.position.z} ')

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
        self.frq_twiststamped_counter = self.frq_twiststamped_counter + 1
        #print(msg)
            # self.get_logger().info(f'TwistStamped ({msg.header.frame_id}) '
            #                        f'x: {msg.twist.angular.x} '
            #                        f'y: {msg.twist.angular.y} '
            #                        f'z: {msg.twist.angular.z} ')

def main():
    rclpy.init()

    parser = argparse.ArgumentParser(description='ROS2 iXCOM Driver')
    parser.add_argument('-f', '--measure_frequency',  action='store_true', default=False, help='measure frequencies of incoming topics')
    args = parser.parse_args()

    dSub = DataSubscriberIntegrationTests(args.measure_frequency)
    try:
        # print('getting config file...')
        dSub.get_config_file(0)
        # print('done')
        # rclpy.spin_once(dSub)
        for i in range(10):
            if dSub.requesting_complete():
                break
            rclpy.spin_once(dSub)

        dSub.rx_data("/home/dev/ros2TestWs/ixcom-ros2-driver/ixcom_driver/src/ixcom_driver/integrationTests/")
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
