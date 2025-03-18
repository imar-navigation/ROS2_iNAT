#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

import argparse
import sys
# sys.path.append('/home/zp/files/git/mdt/ext/ixcomcl/dependencies/ixcom-public')
# sys.path.append('/home/zp/files/git/ixcom_client_to_bytes_fix/dependencies/ixcom-public')
import threading
import rclpy
from rclpy.node import Node
import ixcom.protocol
import os

from .mypy.fun import *
from .mypy.imu import Imu_
from .mypy.navsatstatus import NavSatStatus_
from .mypy.navsatfix import NavSatFix_
from .mypy.navsatfix_ins import NavSatFix_INS_
from .mypy.timereference import TimeReference_
from .mypy.magneticfield import MagneticField_
from .mypy.odometry import Odometry_
from .mypy.posewithcovariancestamped import PoseWithCovarianceStamped_
from .mypy.twiststamped import TwistStamped_
from ixcom_interfaces.action import Example, Stop, Realign, Reboot, GetConfigFile
# from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.qos import *

# from datetime import datetime
# from std_msgs.msg import String
# from std_msgs.msg import Float64MultiArray
#
# from sensor_msgs.msg import Imu, NavSatStatus, NavSatFix, TimeReference
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TwistWithCovarianceStamped
#
# from interfaces.msg import Num
# from interfaces.msg import Sphere
# from interfaces.msg import Array


class DataPublisher(Node):

    def __init__(self, config_file_name, debug_mode):

        super().__init__('ixcom_driver_pub')     # node name

        # self.qos = qos_profile_sensor_data
        # self.qos = qos_profile_system_default

        self._config_file_name = config_file_name
        self.running = True
        self.configuring = True
        self.topic_clients = {}
        self.imu = Imu_()
        self.navSatStatus = NavSatStatus_()
        self.navSatFix = NavSatFix_()
        self.navSatFixIns = NavSatFix_INS_()
        self.timeReference = TimeReference_()
        self.magneticField = MagneticField_()
        self.odometry = Odometry_()
        self.poseWithCovarianceStamped = PoseWithCovarianceStamped_()
        self.twistStamped = TwistStamped_()
        self.t = int(self.get_clock().now().nanoseconds / 1e9)
        self.dbg = debug_mode
        self.log_dir = f'log{os.sep}run'
        wrkdir = os.getcwd()
        log_init(self, f'{wrkdir}{os.sep}{self.log_dir}')

        # self.ms = 0
        # self.old_s_imu = 0
        # self.old_us_imu = 0
        # self.old_utc_s_imu = 0
        # self.old_utc_ns_imu = 0
        # self.old_s_odo = 0
        # self.old_us_odo = 0
        # self.old_utc_s_odo = 0
        # self.old_utc_ns_odo = 0

        leap_seconds = 0
        log_info(self, f'using config file: {os.path.join(wrkdir, config_file_name)}')
        try:
            config = load_config(config_file_name)
            self.qos = get_qos(config[keys.qos])

            log_info(self, f'configured QoS: {config[keys.qos]}')

            self.dev_ip = config[keys.ip_config][keys.ip_address]
            self.dev_port = int(config[keys.ip_config][keys.ip_port])

            log_info(self, f'connecting to iNAT @: {self.dev_ip}:{self.dev_port}...')
            self.cl, self.ch = connect_inat(self.dev_ip, self.dev_port)
            log_info(self, f'connected to iNAT on channel: {self.ch}')

            log_info(self, 'getting INS velocity frame type')
            frame_type = self.cl.get_ins_vel_output_frame().payload.data['velMode']
            log_info(self, f'current setting is: {ixcom.protocol.ParDatVelMode(frame_type).name}')
            if frame_type != ixcom.protocol.ParDatVelMode.BODY:
                log_info(self, 'changing INS velocity to body frame...')
                self.cl.set_ins_vel_output_frame(ixcom.protocol.ParDatVelMode.BODY)
                log_info(self, 'done')
            log_info(self, 'no changes required')

            log_info(self, 'serial configuration...')
            res = set_serial_config(self.cl, config[keys.serial_config])
            if res == res.ok:
                log_info(self, 'done')
            elif res == res.ignored:
                log_info(self, 'ignored')

            log_info(self, 'getting leap seconds from config file')
            leap_seconds = abs(int(config[keys.leap_seconds]))
            log_info(self, f'done: {leap_seconds} s')

        except (LoadConfigFileErr,
                ParseJsonErr,
                MissingKeysErr,
                TooMuchTopicsErr,
                QoSConfErr,
                ClientConnectErr,
                ClientTimeoutError,
                ResponseError,
                SerialConfErr,
                Exception) as e:
            log_error(self, str(e))
            exit(1)

        maintiming, prescaler, assumed = get_maintiming(self.cl)
        if assumed:
            log_warning(self, f'system maintiming assumed: {maintiming} Hz (failed getting system info)')
            log_warning(self, f'system prescaler assumed: {prescaler} (failed getting system info)')
        else:
            log_info(self, f'system maintiming obtained: {maintiming} Hz')
            log_info(self, f'system prescaler obtained: {prescaler}')

        if leap_seconds == 0:
            c = 0
            while not gnss_sol_valid(self):
                if c == 0:
                    print('waiting for GNSS solution...', end='', flush=True)
                else:
                    print('.', end='', flush=True)
                c += 1
                time.sleep(1)
            if c > 0:
                print('')  # end line after the waiting sequence
            leap_seconds = get_leap_seconds(self)

        self.topics = config[keys.topics]
        unsupported_topics = []
        for topic_name in self.topics.keys():
            frq = get_frequency(self.topics[topic_name][keys.frequency_hz])
            if frq > 0:
                try:
                    log_info(self, f'connecting to iNAT @: {self.dev_ip}:{self.dev_port} for the topic {topic_name}')
                    tcl, tch = connect_inat(self.dev_ip, self.dev_port)
                    log_info(self, f'connected to iNAT on channel {tch}')
                except ClientConnectErr as e:
                    log_error(self, str(e))
                    exit(1)

                tid, retmsg = prepare_publisher(self, tcl, maintiming, prescaler, topic_name,
                                                self.topics[topic_name][keys.frequency_hz],
                                                self.topics[topic_name][keys.remap_to],
                                                config[keys.timestamp_mode], leap_seconds)
                if tid == -1:
                    tcl.close_channel()
                    log_info(self, f'- Topic {topic_name} not supported')
                    unsupported_topics.append(topic_name)
                    continue
                tcl.add_callback(self.inat_logger)
                self.topic_clients[tch] = {}
                self.topic_clients[tch]['topic_name'] = topic_name
                self.topic_clients[tch]['cl'] = tcl
                self.topic_clients[tch]['id'] = tid
                log_info(self, retmsg)
            else:
                log_info(self, f'- Topic {topic_name} skipped')

        pls = '' if (len(self.topic_clients) == 1) else 's'
        log_info(self, f'= {len(self.topic_clients)} topic{pls} configured')
        if len(unsupported_topics) > 0:
            log_info(self, 'config contains unsupported topics:')
            for t in unsupported_topics:
                log_info(self, f'- {t}')
            log_info(self, 'currently supported topics:')
            for item in valid_topics.items.items():
                n = item[1]['name']
                log_info(self, f'- {n}')

        log_info(self, 'installing action servers...')
        log_info(self, prepare_action_server_example(self))
        log_info(self, prepare_action_server_stop(self))
        log_info(self, prepare_action_server_realign(self))
        log_info(self, prepare_action_server_reboot(self))
        log_info(self, prepare_action_server_config(self))

        # add_callbacks(self)

        if len(self.topic_clients) > 0:
            log_info(self, 'starting inat log receiver...')
            log_info(self, 'active subscribers should now be receiving data \n')
        else:
            log_info(self, f'used configuration file {config_file_name} does not contain valid topics')
        self.configuring = False

    def get_log_dir(self):
        return self.log_dir

    def inat_logger(self, msg, dev):
        if self.configuring:
            return
        # print('************* msg came')
        # print(f'*** dev open channel: {dev.get_open_ch()}')
        tch = dev.get_open_channel()
        # id = self.topic_clients[tch]['id']
        # print(f'tid: {hex(id)}')

        # f_odo = open("ts_odo.csv", "a")
        # f_imu = open("ts_imu.csv", "a")
        try:
            if isinstance(msg.payload, ixcom.messages.INSSOL_Payload)\
                    or isinstance(msg.payload, ixcom.messages.EKFSTDDEV_Payload)\
                    or isinstance(msg.payload, ixcom.messages.IMUCORR_Payload):

                # if isinstance(msg.payload, ixcom.messages.INSSOL_Payload):
                #     t = gps_timestamp(msg.header, 18)
                #     s = msg.header.timeOfWeek_sec
                #     us = msg.header.timeOfWeek_usec
                #     add = 0
                #     add_utc = 0
                #     if s > self.old_s:
                #         add = 1000000
                #     if t.sec > self.old_utc_s:
                #         add_utc = 1000000000
                #     f.write(f'{s},{us},{us + add - self.old_us},{t.sec},{t.nanosec},{t.nanosec + add_utc - self.old_utc_ns}\n')
                #     f.flush()
                #     self.old_s = s
                #     self.old_us = us
                #     self.old_utc_s = t.sec
                #     self.old_utc_ns = t.nanosec
                # #     log_info(self, f'{msg.header.week}.{msg.header.timeOfWeek_sec}.{msg.header.timeOfWeek_usec}')

                # if isinstance(msg.payload, ixcom.messages.INSSOL_Payload):
                #     f.write(f'INSSOL : {msg.header.timeOfWeek_sec}.{msg.header.timeOfWeek_usec}\n')
                #     f.flush()
                # if isinstance(msg.payload, ixcom.messages.IMUCORR_Payload):
                #     f.write(f'IMUCORR: {msg.header.timeOfWeek_sec}.{msg.header.timeOfWeek_usec}\n')
                #     f.flush()


                # ct = int(self.get_clock().now().nanoseconds / 1000 / 1000)
                # ms = ct - self.ms
                # self.ms = ct
                # self.get_logger().info(f'IMU msg dt: {ms} / {int(1 / self.frq_imu * 1000)}')

                # if self.topic_clients[tch]['id'] == topic_ids.tid_imu:
                if self.topic_clients[tch]['id'] == valid_topics.items['imu']['id']:
                    # if isinstance(msg.payload, ixcom.messages.IMUCORR_Payload):
                    #     t = gps_timestamp(msg.header, 18)
                    #     s = msg.header.timeOfWeek_sec
                    #     us = msg.header.timeOfWeek_usec
                    #     add = 0
                    #     add_utc = 0
                    #     if s > self.old_s_imu:
                    #         add = 1000000
                    #     if t.sec > self.old_utc_s_imu:
                    #         add_utc = 1000000000
                    #     f_imu.write(f'{s},{us},{us + add - self.old_us_imu},{t.sec},{t.nanosec},{t.nanosec + add_utc - self.old_utc_ns_imu}\n')
                    #     f_imu.flush()
                    #     self.old_s_imu = s
                    #     self.old_us_imu = us
                    #     self.old_utc_s_imu = t.sec
                    #     self.old_utc_ns_imu = t.nanosec
                    pub_imu(self, msg)
                # elif self.topic_clients[tch]['id'] == topic_ids.tid_odometry:
                elif self.topic_clients[tch]['id'] == valid_topics.items['odometry']['id']:
                    # if isinstance(msg.payload, ixcom.messages.IMUCORR_Payload):
                    #     t = gps_timestamp(msg.header, 18)
                    #     s = msg.header.timeOfWeek_sec
                    #     us = msg.header.timeOfWeek_usec
                    #     add = 0
                    #     add_utc = 0
                    #     if s > self.old_s_odo:
                    #         add = 1000000
                    #     if t.sec > self.old_utc_s_odo:
                    #         add_utc = 1000000000
                    #     f_odo.write(f'{s},{us},{us + add - self.old_us_odo},{t.sec},{t.nanosec},{t.nanosec + add_utc - self.old_utc_ns_odo}\n')
                    #     f_odo.flush()
                    #     self.old_s_odo = s
                    #     self.old_us_odo = us
                    #     self.old_utc_s_odo = t.sec
                    #     self.old_utc_ns_odo = t.nanosec

                    # if isinstance(msg.payload, ixcom.messages.IMUCORR_Payload):
                    #     t = gps_timestamp(msg.header, 18)
                    #     ros_t = self.get_clock().now()
                    #
                    #     msg_ts = t.sec * 1000000000 + t.nanosec
                    #     ros_ts = ros_t.nanoseconds
                    #     # print(f'{msg_ts} / {ros_ts} / {ros_ts - msg_ts}')
                    #     f_odo.write(f'{msg_ts},{ros_ts},{ros_ts - msg_ts}\n')
                    #     f_odo.flush()

                    pub_odometry(self, msg)
                # elif self.topic_clients[tch]['id'] == topic_ids.tid_posewithcovariancestamped:
                elif self.topic_clients[tch]['id'] == valid_topics.items[valid_topics.names.posewithcovariancestamped_]['id']:
                    pub_posewithcovariancestamped(self, msg)
                # elif self.topic_clients[tch]['id'] == topic_ids.tid_navsatfix_ins:
                elif self.topic_clients[tch]['id'] == valid_topics.items[valid_topics.names.navsatfix_ins_]['id']:
                    pub_navsatfix_ins(self, msg)
                elif self.topic_clients[tch]['id'] == valid_topics.items[valid_topics.names.twiststamped_]['id']:
                    pub_twiststamped(self, msg)
            elif isinstance(msg.payload, ixcom.messages.GNSSSOL_Payload):
                # if self.topic_clients[tch]['id'] == topic_ids.tid_navsatstatus:
                if self.topic_clients[tch]['id'] == valid_topics.items[valid_topics.names.navsatstatus_]['id']:
                    pub_navsatstatus(self, msg)
                # elif self.topic_clients[tch]['id'] == topic_ids.tid_navsatfix:
                elif self.topic_clients[tch]['id'] == valid_topics.items[valid_topics.names.navsatfix_gnss_]['id']:
                    pub_navsatfix(self, msg)
                elif self.topic_clients[tch]['id'] == valid_topics.items[valid_topics.names.navsatfix_ins_]['id']:
                    pub_navsatfix_ins(self, msg)
            elif isinstance(msg.payload, ixcom.messages.SYSSTAT_Payload):
                # if self.topic_clients[tch]['id'] == topic_ids.tid_timereference:
                if self.topic_clients[tch]['id'] == valid_topics.items[valid_topics.names.timereference_]['id']:
                    pub_timereference(self, msg)
            elif isinstance(msg.payload, ixcom.messages.MAGDATA_Payload):
                # if self.topic_clients[tch]['id'] == topic_ids.tid_magneticfield:
                if self.topic_clients[tch]['id'] == valid_topics.items[valid_topics.names.magneticfield_]['id']:
                    pub_magneticfield(self, msg)
        except:
        # except Exception as e:
            # print(f'************* err: {str(e)}')
            pass

    def example_callback(self, goal_handle):
        log_info(self, 'executing example...')
        feedback_msg = Example.Feedback()
        feedback_msg.partial_sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1])
            log_info(self, f'feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        goal_handle.succeed()
        result = Example.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

    def stop_callback(self, goal_handle):
        log_info(self, 'stop request received...')
        # self.destroy_node()
        goal_handle.succeed()
        result = Stop.Result()
        result.resp = 'ok'
        threading.Thread(target=self.end).start()
        return result
        # rclpy.shutdown()

    def reboot_callback(self, goal_handle):
        log_info(self, 'reboot request received...')
        threading.Thread(target=self.end).start()
        result = Reboot.Result()
        try:
            log_info(self, 'starting device reboot...')
            self.cl.reboot()
        except Exception as e:
            log_error(self, f'failed ({str(e)})')
            goal_handle.abort()
            result.resp = f'failed ({str(e)})'
            return result
        log_info(self, 'done')
        log_info(self, 'restart publisher when reboot finished')
        goal_handle.succeed()
        result.resp = 'ok'
        return result
        # log_info(self, 'reboot request received...')
        # log_info(self, 'restart publisher when reboot finished')
        # # self.destroy_node()
        # goal_handle.succeed()
        # result = Reboot.Result()
        # result.resp = 'ok'
        # threading.Thread(target=self.end).start()
        # return result

    def end(self):
        remove_callbacks(self)
        delete_clients(self)
        time.sleep(1)
        self.destroy_node()
        log_info(self, 'stopped.')
        time.sleep(0.2)
        self.running = False
        # rclpy.shutdown()
        return

    def realign_callback(self, goal_handle):
        log_info(self, 'realignment request received...')
        result = Realign.Result()
        try:
            log_info(self, 'starting realignment...')
            # cl, ch = connect_inat(self.dev_ip, self.dev_port)
            self.cl.set_alignment(ixcom.protocol.AlignmentMode.STATIONARY)
            self.cl.realign()
        except Exception as e:
            log_error(self, f'failed ({str(e)})')
            goal_handle.abort()
            result.resp = f'failed ({str(e)})'
            return result
        log_info(self, 'done')
        goal_handle.succeed()
        result.resp = 'ok'
        return result

    def config_callback(self, goal_handle):
        log_info(self, 'config file request received...')
        log_info(self, f'answering: {self._config_file_name}')
        result = GetConfigFile.Result()
        goal_handle.succeed()
        result.resp = self._config_file_name
        return result


def main():

    rclpy.init()
    parser = argparse.ArgumentParser(description='ROS2 iXCOM Driver')
    parser.add_argument('-c', '--config_file', type=str, default='src/ixcom_driver/params/config.json', help='specify your config file')
    parser.add_argument('-D', '--debug_mode',  action='store_true', default=False, help='enable debug logs')
    args = parser.parse_args()

    dp = DataPublisher(config_file_name=args.config_file, debug_mode=args.debug_mode)
    try:
        rclpy.spin(dp)
    except KeyboardInterrupt:
        print('\n\nKeyboard Interrupt detected\n')
        for tch, _ in dp.topic_clients.items():
            print(f"closing client channel {tch} for topic {dp.topic_clients[tch]['topic_name']} "
                  f"({dp.topic_clients[tch]['id']}) ... ", end='')
            try:
                dp.topic_clients[tch]['cl'].close_channel()
                print('done')
            except:
                print('failed')
        dp.destroy_node()
        exit(0)

    dp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

