#!/usr/bin/python3

from sensor_msgs.msg import Imu
# import numpy as np
import ixcom.protocol
from .fun import rpyToQuat
from .fun import valid_topics, TimestampMode, gps_timestamp, get_param


class Imu_(Imu):

    def __init__(self):
        self.node = None
        self.active = False
        self._maintiming = 0
        self._leap_seconds = 0
        self._timestamp_mode = TimestampMode.ros
        self.INS_data_is_set = False
        self.EKF_data_is_set = False
        self.IMUCORR_data_is_set = False
        self.imu = Imu()
        self.imu.angular_velocity_covariance[0] = 0
        self.imu.angular_velocity_covariance[1] = 0
        self.imu.angular_velocity_covariance[2] = 0
        self.imu.angular_velocity_covariance[3] = 0
        self.imu.angular_velocity_covariance[4] = 0
        self.imu.angular_velocity_covariance[5] = 0
        self.imu.angular_velocity_covariance[6] = 0
        self.imu.angular_velocity_covariance[7] = 0
        self.imu.angular_velocity_covariance[8] = 0
        self.imu.linear_acceleration_covariance[0] = 0
        self.imu.linear_acceleration_covariance[1] = 0
        self.imu.linear_acceleration_covariance[2] = 0
        self.imu.linear_acceleration_covariance[3] = 0
        self.imu.linear_acceleration_covariance[4] = 0
        self.imu.linear_acceleration_covariance[5] = 0
        self.imu.linear_acceleration_covariance[6] = 0
        self.imu.linear_acceleration_covariance[7] = 0
        self.imu.linear_acceleration_covariance[8] = 0
        # self.frame_cnt = 0

    def activate(self, node, maintiming, leap_seconds, timestamp_mode):
        self.node = node
        self._maintiming = maintiming
        self._leap_seconds = leap_seconds
        self.active = True
        self._timestamp_mode = timestamp_mode
        self.imu.header.frame_id = 'inat_output_frame'
        # workaround: not yet working like always but so or with id as number
        # par = get_param(node, ixcom.parameters.PAREKF_IMUCONFIG2_Payload.parameter_id)
        # par = get_param(node, 760)
        par = get_param(node, ixcom.data.getMessageByName("PAREKF_IMUCONFIG2").payload.parameter_id)
        self.__set_par_data(par)

        # self.f = open("ts_imu_pub.csv", "a")
        # self.old_s = 0
        # self.old_us = 0
        # self.old_utc_s = 0
        # self.old_utc_ns = 0

    def is_active(self):
        return self.active

    def set_msg_data(self, msg):
        if isinstance(msg.payload, ixcom.messages.INSSOL_Payload):
            self.__set_INS_data(msg)
        if isinstance(msg.payload, ixcom.messages.EKFSTDDEV_Payload):
            self.__set_EKF_data(msg)
        if isinstance(msg.payload, ixcom.messages.IMUCORR_Payload):
            self.__set_IMUCORR_data(msg)

    def __set_timestamp(self, msg):

        # self.imu.header.frame_id = str(self.frame_cnt)
        # self.frame_cnt += 1
        # if self.frame_cnt == 255:
        #     self.frame_cnt = 0

        if self._timestamp_mode == TimestampMode.gps:
            self.imu.header.stamp = gps_timestamp(msg.header, self._leap_seconds)
        else:
            self.imu.header.stamp = self.node.get_clock().now().to_msg()

        # try:
        #     s = msg.header.timeOfWeek_sec
        #     us = msg.header.timeOfWeek_usec
        #     add = 0
        #     add_utc = 0
        #     if s > self.old_s:
        #         add = 1000000
        #     if self.imu.header.stamp.sec > self.old_utc_s:
        #         add_utc = 1000000000
        #     self.f.write(f'{s}.{us},{us + add - self.old_us},'
        #                  f'{self.imu.header.stamp.sec}.{self.imu.header.stamp.nanosec},'
        #                  f'{self.imu.header.stamp.nanosec + add_utc - self.old_utc_ns}\n')
        #     self.f.flush()
        #     self.old_s = s
        #     self.old_us = us
        #     self.old_utc_s = self.imu.header.stamp.sec
        #     self.old_utc_ns = self.imu.header.stamp.nanosec
        # except Exception as e:
        #     print(f'EXCEPTION: {e}')

    def __set_INS_data(self, msg):
        # print('set INS data')
        q = [1.0, 0.0, 0.0, 0.0]
        st = msg.bottom.gStatus
        if st & (1 << 13):                              # bit 13 set: alignment complete
            q = rpyToQuat(msg.data['rpy'])
        self.imu.orientation.x = q[0]
        self.imu.orientation.y = q[1]
        self.imu.orientation.z = q[2]
        self.imu.orientation.w = q[3]

        # self.imu.angular_velocity.x = msg.data['omg'][0]
        # self.imu.angular_velocity.y = msg.data['omg'][1]
        # self.imu.angular_velocity.z = msg.data['omg'][2]
        # self.imu.angular_velocity_covariance[0] =
        # self.imu.angular_velocity_covariance[0] =
        # self.imu.angular_velocity_covariance[0] =
        # self.imu.angular_velocity_covariance[0] =
        # self.imu.angular_velocity_covariance[0] =
        # self.imu.angular_velocity_covariance[0] =
        # self.imu.angular_velocity_covariance[0] =
        # self.imu.angular_velocity_covariance[0] =
        # self.imu.angular_velocity_covariance[0] =
        self.imu.linear_acceleration.x = msg.data['acc'][0]
        self.imu.linear_acceleration.y = msg.data['acc'][1]
        self.imu.linear_acceleration.z = msg.data['acc'][2]
        # self.imu.linear_acceleration_covariance[0] =
        # self.imu.linear_acceleration_covariance[0] =
        # self.imu.linear_acceleration_covariance[0] =
        # self.imu.linear_acceleration_covariance[0] =
        # self.imu.linear_acceleration_covariance[0] =
        # self.imu.linear_acceleration_covariance[0] =
        # self.imu.linear_acceleration_covariance[0] =
        # self.imu.linear_acceleration_covariance[0] =
        # self.imu.linear_acceleration_covariance[0] =

        # print('INS data is set')
        self.INS_data_is_set = True

    def __set_EKF_data(self, msg):
        # print('set EKF data')
        st = msg.bottom.gStatus
        if st & (1 << 13):  # bit 13 set: alignment complete
            self.imu.orientation_covariance[0] = pow(msg.data['tilt'][0], 2)
        else:
            self.imu.orientation_covariance[0] = -1
        # self.imu.orientation_covariance[1] =
        # self.imu.orientation_covariance[2] =
        # self.imu.orientation_covariance[3] =
        self.imu.orientation_covariance[4] = pow(msg.data['tilt'][1], 2)
        # self.imu.orientation_covariance[5] =
        # self.imu.orientation_covariance[6] =
        # self.imu.orientation_covariance[7] =
        self.imu.orientation_covariance[8] = pow(msg.data['tilt'][2], 2)

        # print('EKF data is set')
        self.EKF_data_is_set = True

    def __set_IMUCORR_data(self, msg):
        # print('set IMUCORR data')
        # acc = msg.data['acc'][0]
        # omg = msg.data['omg'][0]
        # print(f'************ IMUCORR ACC: {acc}')
        # print(f'************ IMUCORR OMG: {omg}')
        self.imu.angular_velocity.x = msg.data['omg'][0]
        self.imu.angular_velocity.y = msg.data['omg'][1]
        self.imu.angular_velocity.z = msg.data['omg'][2]

        self.__set_timestamp(msg)

        # print('IMUCORR data is set')

        self.IMUCORR_data_is_set = True

    def __set_par_data(self, par):
        # print('setting imu par data')
        # print(f'par: {par}')
        # not yet working like this
        # if (not par is None) and isinstance(par.payload, ixcom.parameters.PAREKF_IMUCONFIG2_Payload):
        if par is not None:                 # check isinstance(...) not working
            self.__set_IMUCONFIG2_data(par)

    def __set_IMUCONFIG2_data(self, par):
        # print(f'par: {par.payload.data}\n')
        noise_acc = par.payload.data['acc']['root_noise_psd']
        noise_gyro = par.payload.data['gyro']['root_noise_psd']
        self.imu.angular_velocity_covariance[0] = (noise_acc[0] ** 2) * self._maintiming
        self.imu.angular_velocity_covariance[1] = 0
        self.imu.angular_velocity_covariance[2] = 0
        self.imu.angular_velocity_covariance[3] = 0
        self.imu.angular_velocity_covariance[4] = (noise_acc[1] ** 2) * self._maintiming
        self.imu.angular_velocity_covariance[5] = 0
        self.imu.angular_velocity_covariance[6] = 0
        self.imu.angular_velocity_covariance[7] = 0
        self.imu.angular_velocity_covariance[8] = (noise_acc[2] ** 2) * self._maintiming

        self.imu.linear_acceleration_covariance[0] = (noise_gyro[0] ** 2) * self._maintiming
        self.imu.linear_acceleration_covariance[1] = 0
        self.imu.linear_acceleration_covariance[2] = 0
        self.imu.linear_acceleration_covariance[3] = 0
        self.imu.linear_acceleration_covariance[4] = (noise_gyro[1] ** 2) * self._maintiming
        self.imu.linear_acceleration_covariance[5] = 0
        self.imu.linear_acceleration_covariance[6] = 0
        self.imu.linear_acceleration_covariance[7] = 0
        self.imu.linear_acceleration_covariance[8] = (noise_gyro[2] ** 2) * self._maintiming

    def is_complete(self):
        return self.INS_data_is_set \
            and self.EKF_data_is_set \
            and self.IMUCORR_data_is_set

    def get_imu(self):
        self.IMUCORR_data_is_set = False
        # self.EKF_data_is_set = False  # data will only be sent if this data is set
        return self.imu
