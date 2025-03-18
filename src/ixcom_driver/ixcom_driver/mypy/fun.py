#!/usr/bin/python3
# Copyright 2023 iMAR Navigation GmbH

from .log import Log, LogLevel
from .exceptions import *
from math import *
import os
from enum import Enum, IntEnum
import json
import time
import ixcom.protocol
from ixcom.exceptions import CommunicationError, ResponseError, ClientTimeoutError
from ixcom_interfaces.action import Example, Stop, Realign, Reboot, GetConfigFile

from sensor_msgs.msg import Imu, NavSatStatus, NavSatFix, TimeReference, MagneticField
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from rclpy.action import ActionServer
from rclpy.qos import *


class res(IntEnum):
    ok = 0
    err = 1
    ignored = 2


class valid_topics:
    class names:
        imu_ = 'imu'
        magneticfield_ = 'magneticfield'
        navsatfix_gnss_ = 'navsatfix_gnss'
        navsatfix_ins_ = 'navsatfix_ins'
        navsatstatus_ = 'navsatstatus'
        posewithcovariancestamped_ = 'posewithcovariancestamped'
        odometry_ = 'odometry'
        timereference_ = 'timereference'
        twiststamped_ = 'twiststamped'
    items = {names.imu_: {'id': 0xFD01, 'name': 'Imu'},
             names.magneticfield_: {'id': 0xFD02, 'name': 'MagneticField'},
             names.navsatfix_gnss_: {'id': 0xFD03, 'name': 'NavSatFix_GNSS'},
             names.navsatstatus_: {'id': 0xFD04, 'name': 'NavSatStatus'},
             names.posewithcovariancestamped_: {'id': 0xFD05, 'name': 'PoseWithCovarianceStamped'},
             names.odometry_: {'id': 0xFD08, 'name': 'Odometry'},
             names.timereference_: {'id': 0xFD09, 'name': 'TimeReference'},
             names.navsatfix_ins_: {'id': 0xFD10, 'name': 'NavSatFix_INS'},
             names.twiststamped_: {'id': 0xFD11, 'name': 'TwistStamped'}
             }


class keys:
    ip_config = 'ip_config'
    ip_address = 'ip_address'
    ip_port = 'ip_port'
    serial_config = 'serial_config'
    ignore = 'ignore'
    serial_port = 'serial_port'
    baud_rate = 'baud_rate'
    enable = 'enable'
    topics = 'topics'
    frequency_hz = 'frequency_hz'
    remap_to = 'remap_to'
    timestamp_mode = 'timestamp_mode'
    qos = 'qos'
    leap_seconds = 'leap_seconds'


class valid_qos:
    class names(Enum):
        SYSTEMDEFAULTS = 'SYSTEMDEFAULTS'
        SENSORDATA = 'SENSORDATA'
        # clock = 'CLOCK'
        ACTIONSTATUS = 'ACTIONSTATUS'
        PARAMETEREVENTS = 'PARAMETEREVENTS'
        PARAMETERS = 'PARAMETERS'
        SERVICES = 'SERVICES'
    items = {
        names.SYSTEMDEFAULTS.value: qos_profile_system_default,
        names.SENSORDATA.value: qos_profile_sensor_data,
        # 'clock': qos_profile_clock,
        names.ACTIONSTATUS.value: qos_profile_action_status_default,
        names.PARAMETEREVENTS.value: qos_profile_parameter_events,
        names.PARAMETERS.value: qos_profile_parameters,
        names.SERVICES.value: qos_profile_services_default
    }


def load_config(config_file_name):
    try:
        f = open(config_file_name, 'r')
    except Exception as e:
        raise LoadConfigFileErr(f'could not read file: {str(e)}')
    try:
        config = json.load(f)
    except Exception as e:
        raise ParseJsonErr(f'could not load json: {str(e)}')

    missingKeys = ''
    if keys.ip_config not in config:
        missingKeys += (keys.ip_config + ' ')
    if keys.ip_address not in config[keys.ip_config]:
        missingKeys += (keys.ip_address + ' ')
    if keys.ip_port not in config[keys.ip_config]:
        missingKeys += (keys.ip_port + ' ')
    if keys.serial_config not in config:
        missingKeys += (keys.serial_config + ' ')
    if keys.ignore not in config[keys.serial_config]:
        missingKeys += (keys.ignore + ' ')
    if keys.serial_port not in config[keys.serial_config]:
        missingKeys += (keys.serial_port + ' ')
    if keys.baud_rate not in config[keys.serial_config]:
        missingKeys += (keys.baud_rate + ' ')
    if keys.enable not in config[keys.serial_config]:
        missingKeys += (keys.enable + ' ')
    if keys.topics not in config:
        missingKeys += (keys.topics + ' ')
    if keys.timestamp_mode not in config:
        missingKeys += (keys.timestamp_mode + ' ')
    if keys.qos not in config:
        missingKeys += (keys.qos + ' ')
    if keys.leap_seconds not in config:
        missingKeys += (keys.leap_seconds + ' ')

    # print(f'topics keys: {config[keys.topics].keys()}')

    for topic_key in config[keys.topics].keys():
        topic_conf_keys = config[keys.topics][topic_key].keys()
        if keys.frequency_hz not in topic_conf_keys:
            missingKeys += (topic_key + ': ' + keys.frequency_hz + ' ')
        if keys.remap_to not in topic_conf_keys:
            missingKeys += (topic_key + ': ' + keys.remap_to + ' ')

    if len(missingKeys) > 0:
        raise MissingKeysErr(f'missing keys in json file: {missingKeys}')

    if len(config[keys.topics]) > 32:
        raise TooMuchTopics(f'number of topics ({len(config[keys.topics])}) must not exceed 32')

    return config


def get_qos(config_qos):
    # print(f'getting qos: {config_qos}')
    # print(f'systemdefaults: {valid_qos.names.SYSTEMDEFAULTS.value}')
    if config_qos.upper() == valid_qos.names.SYSTEMDEFAULTS.value:
        return valid_qos.items[valid_qos.names.SYSTEMDEFAULTS.value]
    if config_qos.upper() == valid_qos.names.PARAMETERS.value:
        return valid_qos.items[valid_qos.names.PARAMETERS.value]
    # if config_qos.upper() == valid_qos.names.clock:
    #     return valid_qos.items[valid_qos.names.clock]
    if config_qos.upper() == valid_qos.names.ACTIONSTATUS.value:
        return valid_qos.items[valid_qos.names.ACTIONSTATUS.value]
    if config_qos.upper() == valid_qos.names.PARAMETEREVENTS.value:
        return valid_qos.items[valid_qos.names.PARAMETEREVENTS.value]
    if config_qos.upper() == valid_qos.names.SENSORDATA.value:
        return valid_qos.items[valid_qos.names.SENSORDATA.value]
    if config_qos.upper() == valid_qos.names.SERVICES.value:
        return valid_qos.items[valid_qos.names.SERVICES.value]
    print('qos ok')
    raise QoSConfErr(f'Invalid QoS: {config_qos}. '
                     f'Valid values: '
                     f'{valid_qos.names.SYSTEMDEFAULTS.value}, '
                     f'{valid_qos.names.SENSORDATA.value}, '
                     f'{valid_qos.names.PARAMETERS.value}, '
                     f'{valid_qos.names.PARAMETEREVENTS.value}, '
                     f'{valid_qos.names.SERVICES.value}, '
                     f'{valid_qos.names.ACTIONSTATUS.value}')


def set_serial_config(cl, config):
    if not int(config['ignore']):
        print(f'config: {config}')
        msgToSend = ixcom.protocol.getParameterWithID(ixcom.parameters.PARXCOM_SERIALPORT_Payload.parameter_id)
        msgToSend.payload.data['action'] = ixcom.protocol.ParameterAction.CHANGING
        msgToSend.payload.data['port'] = int(config['serial_port'])
        msgToSend.payload.data['switch'] = int(config['enable'])
        msgToSend.payload.data['baudRate'] = int(config['baud_rate'])
        # cl.send_msg_and_dont_waitfor_okay(msgToSend)
        try:
            cl.send_msg_and_waitfor_okay(msgToSend)
        except(CommunicationError, ResponseError, Exception) as e:
            raise SerialConfErr(str(e))
        return res.ok
    else:
        return res.ignored


def connect_inat(ip, port):
    try:
        cl = ixcom.Client(ip, int(port))
        ch = cl.open_first_free_channel()
        return cl, ch
    except:
        raise ClientConnectErr('iNAT: could not set up client')


def get_maintiming(cl):
    try:
        maintiming = cl.get_parameter(ixcom.parameters.PARSYS_MAINTIMING_Payload.parameter_id).payload.data['maintiming']
        prescaler = cl.get_parameter(ixcom.parameters.PARSYS_PRESCALER_Payload.parameter_id).payload.data['prescaler']
        # print(f'maintiming: {maintiming}, prescaler: {prescaler}')
        return maintiming, prescaler, False
        # dev_info = cl.get_device_info()
        # print(dev_info)
        # return dev_info['maintiming'], dev_info['prescaler'], False
    except:
        return 500, 1, True


def gnss_sol_valid(node):
    gnss_sol = 0
    try:
        # msg = node.cl.poll_log(ixcom.messages.GNSSSOL_Payload.message_id)
        gnss_sol = node.cl.poll_log(ixcom.messages.GNSSSOL_Payload.message_id).data['posType']
    except:
        log_warning(node, 'failed while getting GNSS Solution')
    return gnss_sol


def get_leap_seconds(node):
    utc_offset = 0
    log_info(node, 'getting leap seconds from iNAT GNSSTIME')
    try:
        msg = node.cl.poll_log(ixcom.messages.GNSSTIME_Payload.message_id)
        # print(f'*** *** *** msg: {msg.data}')
        utc_offset = abs(round(msg.data['utcOffset']))
        # offset = msg.data['offset']
        # print(f'*** *** *** utc_offset: {utc_offset}, offset: {offset}')
        log_info(node, f'done: {utc_offset} s')
    except:
        log_error(node, 'failed')
        pass
    return utc_offset


# def add_callbacks(node):
#     for topic_name in node.topics.keys():
#         try:
#             node.topic_clients[topic_name]['cl'].add_callback(node.inat_logger)
#             # print('******** maintiming {}'.format(self.topic_clients[topic_name].get_device_info()['maintiming']))
#         except:
#             pass


def remove_callbacks(node):
    for tch in node.topic_clients.keys():
        try:
            node.topic_clients[tch]['cl'].remove_callback(node.inat_logger)
        except:
            pass
    # for topic_name in node.topics.keys():
    #     try:
    #         node.topic_clients[topic_name]['cl'].remove_callback(node.inat_logger)
    #     except:
    #         pass


def delete_clients(node):
    for tch in node.topic_clients.keys():
        try:
            del node.topic_clients[tch]['cl']
        except:
            pass
    # for topic_name in node.topics.keys():
    #     try:
    #         del node.topic_clients[topic_name]['cl']
    #         # del node.cl
    #         # print(f'deleted client {topic_name}')
    #     except:
    #         pass


def add_logs_imu(node, tcl, maintiming, prescaler, frq):
    # print(f'adding logs for imu on channel {tcl.get_open_ch()}')
    # print(f'trying to add INSSOL Log...')
    # i_frq = int(frq)
    # frequencies_set = []
    # log_info(node, 'clearing INSSOL log...')
    # __clear_log(node, ixcom.messages.INSSOL_Payload.message_id)
    # log_info(node, 'clearing EKFSTDDEV log...')
    # __clear_log(node, ixcom.messages.EKFSTDDEV_Payload.message_id)
    # log_info(node, 'clearing IMUCORR log...')
    # __clear_log(node, ixcom.messages.IMUCORR_Payload.message_id)
    #
    time.sleep(1)

    # print(f'log list imu: {tcl.get_loglist(tcl.get_open_ch())}')

    ok, i_frq, f_max = __get_freq(frq, maintiming, prescaler)
    if not ok:
        log_error(node, 'could not determine a possible log frequency')
    if i_frq < int(frq):
        log_warning(node, f'frequency for the Imu Topic will be reduced to the next possible')

    try:
        # print(f'***************requestet frequency {i_frq}')
        # print(f'***************system frequency {maintiming}')
        # i_frq = int(frq)
        # if i_frq > maintiming: i_frq = maintiming
        # frequencies_set.append(i_frq)
        # print(f'***************frequency set to {i_frq}')
        log_info(node, f'adding INSSOL log @ {i_frq} Hz (max frequency of this log is {f_max} Hz)...')
        tcl.add_log_with_rate(ixcom.messages.INSSOL_Payload.message_id, i_frq)
        log_info(node, 'done')
        # cl.add_log_sync(0x03, 50)
        # print(f'done')
    except Exception as e:
        # print(f'INSSOL log failure: {str(e)}')
        log_error(node, f'INSSOL log failure: {str(e)}')
        pass
    try:
        # i_frq = int(frq)
        f = Frq.ekf
        if int(frq) < Frq.ekf:
            f = int(frq)
        # frequencies_set.append(i_frq)
        log_info(node, f'adding EKFSTDDEV log @ {f} Hz (max frequency of this log is {Frq.ekf} Hz)...')
        tcl.add_log_with_rate(ixcom.messages.EKFSTDDEV_Payload.message_id, f)
        log_info(node, 'done')
        # cl.add_log_sync(0x03, 50)
        # print(f'done')
    except Exception as e:
        # print(f'EKFSTDDEV log failure: {str(e)}')
        log_error(node, f'EKFSTDDEV log failure: {str(e)}')
        pass
    try:
        # i_frq = int(frq)
        # if i_frq > maintiming: i_frq = maintiming
        # frequencies_set.append(i_frq)
        log_info(node, f'adding IMUCORR log @ {i_frq} Hz (max frequency of this log is {f_max} Hz)...')
        tcl.add_log_with_rate(ixcom.messages.IMUCORR_Payload.message_id, i_frq)
        log_info(node, 'done')
    except Exception as e:
        log_error(node, f'IMUCORR log failure: {str(e)}')
        pass

    # return frequencies_set
    return i_frq


def add_logs_odometry(node, tcl, maintiming, prescaler, frq):
    # print(f'trying to add INSSOL Log...')
    # print(f'adding logs for odometry on channel {tcl.get_open_ch()}')
    # i_frq = int(frq)
    # frequencies_set = []
    # log_info(node, 'clearing INSSOL log...')
    # __clear_log(node, tcl, ixcom.messages.INSSOL_Payload.message_id)
    # log_info(node, 'clearing EKFSTDDEV log...')
    # __clear_log(node, tcl, ixcom.messages.EKFSTDDEV_Payload.message_id)
    # log_info(node, 'clearing IMUCORR log...')
    # __clear_log(node, tcl, ixcom.messages.IMUCORR_Payload.message_id)

    time.sleep(1)

    ok, i_frq, f_max = __get_freq(frq, maintiming, prescaler)
    if not ok:
        log_error(node, 'could not determine a possible log frequency')
    if i_frq < int(frq):
        log_warning(node, 'frequency for the Odometry Topic will be reduced to the next possible')

    try:
        # i_frq = int(frq)
        # if i_frq > maintiming: i_frq = maintiming
        # frequencies_set.append(i_frq)
        log_info(node, f'adding INSSOL log @ {i_frq} Hz (max frequency of this log is {f_max} Hz)...')
        tcl.add_log_with_rate(ixcom.messages.INSSOL_Payload.message_id, i_frq)
        log_info(node, 'done')
        # cl.add_log_sync(0x03, 50)
        # print(f'done')
    except Exception as e:
        # print(f'INSSOL log failure: {str(e)}')
        log_error(node, f'INSSOL log failure: {str(e)}')
        pass
    try:
        # i_frq = int(frq)
        # if i_frq > 20: i_frq = 20
        # frequencies_set.append(i_frq)
        f = Frq.ekf
        if int(frq) < Frq.ekf:
            f = int(frq)
        log_info(node, f'adding EKFSTDDEV log @ {f} Hz (max frequency of this log is {Frq.ekf} Hz)...')
        tcl.add_log_with_rate(ixcom.messages.EKFSTDDEV_Payload.message_id, f)
        log_info(node, 'done')
        # cl.add_log_sync(0x03, 50)
        # print(f'done')
    except Exception as e:
        # print(f'EKFSTDDEV log failure: {str(e)}')
        log_error(node, f'EKFSTDDEV log failure: {str(e)}')
        pass
    try:
        # i_frq = int(frq)
        # if i_frq > maintiming: i_frq = maintiming
        # frequencies_set.append(i_frq)
        log_info(node, f'adding IMUCORR log @ {i_frq} Hz (max frequency of this log is {f_max} Hz)')
        tcl.add_log_with_rate(ixcom.messages.IMUCORR_Payload.message_id, i_frq)
        log_info(node, 'done')
    except Exception as e:
        log_error(node, f'IMUCORR log failure: {str(e)}')
        pass

    # return frequencies_set
    return i_frq


def add_logs_posewithcovariancestamped(node, tcl, maintiming, prescaler, frq):
    # print(f'trying to add INSSOL Log...')
    # i_frq = int(frq)
    # frequencies_set = []
    # log_info(node, 'clearing INSSOL log...')
    # __clear_log(node, ixcom.messages.INSSOL_Payload.message_id)
    # log_info(node, 'clearing EKFSTDDEV log...')
    # __clear_log(node, ixcom.messages.EKFSTDDEV_Payload.message_id)
    #
    time.sleep(1)

    ok, i_frq, f_max = __get_freq(frq, maintiming, prescaler)
    if not ok:
        log_error(node, 'could not determine a possible log frequency')
    if i_frq < int(frq):
        log_warning(node, 'frequency for the PoseWithCovarianceStamped Topic will be reduced to the next possible')

    try:
        # i_frq = int(frq)
        # if i_frq > maintiming: i_frq = maintiming
        # frequencies_set.append(i_frq)
        log_info(node, f'adding INSSOL log @ {i_frq} Hz (max frequency of this log is {f_max} Hz)...')
        tcl.add_log_with_rate(ixcom.messages.INSSOL_Payload.message_id, i_frq)
        log_info(node, 'done')
    except Exception as e:
        log_error(node, f'INSSOL log failure: {str(e)}')
        pass
    try:
        # i_frq = int(frq)
        # if i_frq > 20: i_frq = 20
        # frequencies_set.append(i_frq)
        f = Frq.ekf
        if int(frq) < Frq.ekf:
            f = int(frq)
        log_info(node, f'adding EKFSTDDEV log @ {f} Hz (max frequency of this log is {Frq.ekf} Hz)...')
        tcl.add_log_with_rate(ixcom.messages.EKFSTDDEV_Payload.message_id, f)
        log_info(node, 'done')
        # cl.add_log_sync(0x03, 50)
        # print(f'done')
    except Exception as e:
        # print(f'EKFSTDDEV log failure: {str(e)}')
        log_error(node, f'EKFSTDDEV log failure: {str(e)}')
        pass

    # return frequencies_set
    return i_frq


def add_logs_twiststamped(node, tcl, maintiming, prescaler, frq):
    # i_frq = int(frq)
    # frequencies_set = []
    time.sleep(1)

    ok, i_frq, f_max = __get_freq(frq, maintiming, prescaler)
    if not ok:
        log_error(node, 'could not determine a possible log frequency')
    if i_frq < int(frq):
        log_warning(node, 'frequency for the TwistStamped Topic will be reduced to the next possible')

    try:
        # i_frq = int(frq)
        # if i_frq > maintiming: i_frq = maintiming
        # frequencies_set.append(i_frq)
        log_info(node, f'adding INSSOL log @ {i_frq} Hz (max frequency of this log is {f_max} Hz)...')
        tcl.add_log_with_rate(ixcom.messages.INSSOL_Payload.message_id, i_frq)
        log_info(node, 'done')
    except Exception as e:
        log_error(node, f'INSSOL log failure: {str(e)}')
        pass
    try:
        # i_frq = int(frq)
        # if i_frq > maintiming: i_frq = maintiming
        # frequencies_set.append(i_frq)
        log_info(node, f'adding IMUCORR log @ {i_frq} Hz (max frequency of this log is {f_max} Hz)...')
        tcl.add_log_with_rate(ixcom.messages.IMUCORR_Payload.message_id, i_frq)
        log_info(node, 'done')
    except Exception as e:
        log_error(node, f'IMUCORR log failure: {str(e)}')
        pass

    # return frequencies_set
    return i_frq


def add_logs_navsatstatus(node, tcl, frq):
    # i_frq = int(frq)
    # frequencies_set = []
    # log_info(node, 'clearing GNSSSOL log...')
    # __clear_log(node, ixcom.messages.GNSSSOL_Payload.message_id)
    #
    time.sleep(1)

    # i_frq = int(frq)
    # if i_frq > 2: i_frq = 2
    # frequencies_set.append(i_frq)
    f = Frq.gnss
    # if int(frq) < Frq.gnss:
    #     f = int(frq)
    try:
        log_info(node, f'adding GNSSSOL log @ {f} Hz (max frequency of this log is {Frq.gnss} Hz)...')
        tcl.add_log_with_rate(ixcom.messages.GNSSSOL_Payload.message_id, f)
        log_info(node, 'done')
    except Exception as e:
        log_error(node, f'GNSSSOL log failure: {str(e)}')
        pass

    # return frequencies_set
    return f


def add_logs_navsatfix(node, tcl, frq):
    # i_frq = int(frq)
    # frequencies_set = []
    # log_info(node, 'clearing GNSSSOL log...')
    # __clear_log(node, ixcom.messages.GNSSSOL_Payload.message_id)
    #
    time.sleep(1)

    # i_frq = int(frq)
    # if i_frq > 2: i_frq = 2
    # frequencies_set.append(i_frq)
    f = Frq.gnss
    # if int(frq) < Frq.gnss:
    #     f = int(frq)
    try:
        log_info(node, f'adding GNSSSOL log @ {f} Hz (max frequency of this log is {Frq.gnss} Hz)...')
        tcl.add_log_with_rate(ixcom.messages.GNSSSOL_Payload.message_id, f)
        log_info(node, 'done')
    except Exception as e:
        log_error(node, f'GNSSSOL log failure: {str(e)}')
        pass

    # return frequencies_set
    return f


def add_logs_navsatfix_ins(node, tcl, maintiming, prescaler, frq):
    # i_frq = int(frq)
    # frequencies_set = []

    time.sleep(1)

    ok, i_frq, f_max = __get_freq(frq, maintiming, prescaler)
    if not ok:
        log_error(node, 'could not determine a possible log frequency')
    if i_frq < int(frq):
        log_warning(node, 'frequency for the NavSatFix_INS Topic will be reduced to the next possible')

    try:
        # i_frq = int(frq)
        # if i_frq > maintiming: i_frq = maintiming
        # frequencies_set.append(i_frq)
        log_info(node, f'adding INSSOL log @ {i_frq} Hz (max frequency of this log is {f_max} Hz)...')
        tcl.add_log_with_rate(ixcom.messages.INSSOL_Payload.message_id, i_frq)
        log_info(node, 'done')
    except Exception as e:
        log_error(node, f'INSSOL log failure: {str(e)}')
        pass
    try:
        # i_frq = int(frq)
        # if i_frq > 20: i_frq = 20
        # frequencies_set.append(i_frq)
        f = Frq.ekf
        if int(frq) < Frq.ekf:
            f = int(frq)
        log_info(node, f'adding EKFSTDDEV log @ {f} Hz (max frequency of this log is {Frq.ekf} Hz)...')
        tcl.add_log_with_rate(ixcom.messages.EKFSTDDEV_Payload.message_id, f)
        log_info(node, 'done')
    except Exception as e:
        log_error(node, f'EKFSTDDEV log failure: {str(e)}')
        pass
    try:
        # i_frq = int(frq)
        # if i_frq > 2: i_frq = 2
        # frequencies_set.append(i_frq)
        f = Frq.gnss
        # if int(frq) < Frq.gnss:
        #     f = int(frq)
        log_info(node, f'adding GNSSSOL log @ {f} Hz (max frequency of this log is {Frq.gnss} Hz)...')
        tcl.add_log_with_rate(ixcom.messages.GNSSSOL_Payload.message_id, f)
        log_info(node, 'done')
    except Exception as e:
        log_error(node, f'GNSSSOL log failure: {str(e)}')
        pass

    # return frequencies_set
    return i_frq


def add_logs_timereference(node, tcl, maintiming, prescaler, frq):
    # i_frq = int(frq)
    # frequencies_set = []
    # log_info(node, 'clearing SYSSTAT log...')
    # __clear_log(node, ixcom.messages.SYSSTAT_Payload.message_id)
    #
    time.sleep(1)

    ok, i_frq, f_max = __get_freq(frq, maintiming, prescaler)
    if not ok:
        log_error(node, 'could not determine a possible log frequency')
    if i_frq < int(frq):
        log_warning(node, 'frequency for the TimeReference Topic will be reduced to the next possible')

    try:
        # i_frq = int(frq)
        # if i_frq > maintiming: i_frq = maintiming
        # frequencies_set.append(i_frq)
        log_info(node, f'adding SYSSTAT log @ {i_frq} Hz (max frequency of this log is {f_max} Hz)...')
        tcl.add_log_with_rate(ixcom.messages.SYSSTAT_Payload.message_id, i_frq)
        log_info(node, 'done')
    except Exception as e:
        log_error(node, f'SYSSTAT log failure: {str(e)}')
        pass

    # return frequencies_set
    return i_frq


def add_logs_magneticfield(node, tcl, maintiming, prescaler, frq):
    # i_frq = int(frq)
    # frequencies_set = []
    # log_info(node, 'clearing MAGDATA log...')
    # __clear_log(node, ixcom.messages.MAGDATA_Payload.message_id)
    #
    time.sleep(1)

    ok, i_frq, f_max = __get_freq(frq, maintiming, prescaler)
    if not ok:
        log_error(node, 'could not determine a possible log frequency')
    if i_frq < int(frq):
        log_warning(node, 'frequency for the MagneticField Topic will be reduced to the next possible')

    try:
        # i_frq = int(frq)
        # if i_frq > maintiming: i_frq = maintiming
        # frequencies_set.append(i_frq)
        log_info(node, f'adding MAGDATA log @ {i_frq} Hz (max frequency of this log is {f_max} Hz)...')
        tcl.add_log_with_rate(ixcom.messages.MAGDATA_Payload.message_id, i_frq)
        log_info(node, 'done')
    except Exception as e:
        log_error(node, f'MAGDATA log failure: {str(e)}')
        pass

    # return frequencies_set
    return i_frq


def __clear_log(node, cl, log):
    try:
        cl.clear_log(log)
        log_info(node, 'done')
    except Exception as e:
        log_info(node, f'failure: {str(e)}')
        pass


def __get_freq(requested_f, maintiming, prescaler):
    requested_f = int(requested_f)
    maintiming = int(maintiming)
    prescaler = int(prescaler)
    OK = 1
    ERR = 0
    f_max = int(maintiming / prescaler)
    divider = int(ceil(maintiming / prescaler / requested_f))
    if divider == 0:
        return ERR, 1, f_max
    f = int(maintiming / divider / prescaler)
    if int(f) > maintiming:
        return ERR, 1, f_max
    return OK, f, f_max


def log_init(node, path):
    # d = os.path.join('log', 'run')
    d = os.path.join(node.get_log_dir())
    if not os.path.exists(d):
        os.makedirs(d)
    # print(f'dir: {d}')
    fn = os.path.join(d, f'log_{node.t}.log')
    f = open(fn, 'w')
    # f = open(f'log_{t}.log', 'w')
    f.write('')
    f.close()
    log_info(node, f'log file: {path}{os.sep}log_{node.t}.log')


def log_debug(node, msg):
    Log.log(node, msg, LogLevel.debug)


def log_info(node, msg):
    Log.log(node, msg, LogLevel.info)


def log_warning(node, msg):
    Log.log(node, msg, LogLevel.warning)


def log_error(node, msg):
    Log.log(node, msg, LogLevel.error)


def pub_imu(node, msg):
    # print('publishing imu...')
    if node.imu.is_active():
        # print('publishing imu active...')
        node.imu.set_msg_data(msg)
        if node.imu.is_complete():
            # ct = int(node.get_clock().now().nanoseconds / 1000 / 1000)
            # dt = ct - node.ms
            # node.ms = ct
            # node.get_logger().info(f'IMU msg dt: {dt} / {int(1 / node.frq_imu * 1000)}')
            # print('publishing imu complete...')

            # workaround: not yet working like always but so or with id as number
            # par = get_param(node, ixcom.parameters.PAREKF_IMUCONFIG2_Payload.parameter_id)
            # par = get_param(node, 760)
            # par = get_param(node, ixcom.data.getMessageByName("PAREKF_IMUCONFIG2").payload.parameter_id)
            # print(f'par: {par}')
            # node.imu.set_par_data(par)
            # node.imu.set_header(node, msg)
            node.publisher_imu.publish(node.imu.get_imu())
            # print('Imu published')


def pub_navsatstatus(node, msg):
    if node.navSatStatus.is_active():
        node.navSatStatus.set_msg_data(msg)
        # par = get_par(ixcom.parameters.PARGNSS_LOCKOUTSYSTEM_Payload.parameter_id, 'lockoutMask')
        par = get_param(node, ixcom.parameters.PARGNSS_LOCKOUTSYSTEM_Payload.parameter_id)
        node.navSatStatus.set_par_data(par)
        # if node.navSatStatus.is_complete():
        # node.navSatStatus.set_header(node, msg)
        node.publisher_navsatstatus.publish(node.navSatStatus.get_navSatStatus())
        # print('NavSatStatus published')


def get_param(node, par_id):
    try:
        # par = node.cl.get_parameter(par_id).payload.data[d_name]
        par = node.cl.get_parameter(par_id)
        # log_info(node, f'got data of the parameter {par_id}')
        return par
    except Exception as e:
        log_warning(node, f'failed while getting data of the parameter {par_id}: {str(e)}')
        pass
    return None


def pub_navsatfix(node, msg):
    if node.navSatFix.is_active():
        # node.navSatFix.set_header(node, msg)
        node.navSatFix.set_msg_data(msg)
        par = get_param(node, ixcom.parameters.PARGNSS_LOCKOUTSYSTEM_Payload.parameter_id)
        node.navSatFix.set_par_data(par)
        node.publisher_navsatfix.publish(node.navSatFix.get_navSatFix())
        # print('NavSatFix published')


def pub_navsatfix_ins(node, msg):
    # print('******** NAVSATFIX_INS')
    if node.navSatFixIns.is_active():
        # print('******** is active')
        node.navSatFixIns.set_msg_data(msg)
        if node.navSatFixIns.is_complete():
            # print('******** is complete')
            par = get_param(node, ixcom.parameters.PARGNSS_LOCKOUTSYSTEM_Payload.parameter_id)
            node.navSatFixIns.set_par_data(par)
            # node.navSatFixIns.set_header(node, msg)
            node.publisher_navsatfix_ins.publish(node.navSatFixIns.get_navSatFix())
            # print('NavSatFix_INS published')


def pub_timereference(node, msg):
    if node.timeReference.is_active():
        # node.timeReference.set_header(node, msg)
        node.timeReference.set_msg_data(msg)
        node.publisher_timereference.publish(node.timeReference.get_timeReference())
        # print('TimeReference published')


def pub_magneticfield(node, msg):
    if node.magneticField.is_active():
        # node.magneticField.set_header(node, msg)
        node.magneticField.set_msg_data(msg)
        par = get_param(node, ixcom.parameters.PAREKF_MAGATTAID_Payload.parameter_id)
        node.magneticField.set_par_data(par)
        node.publisher_magneticfield.publish(node.magneticField.get_magneticField())
        # print('MagneticField published')


def pub_odometry(node, msg):
    if node.odometry.is_active():
        node.odometry.set_msg_data(msg)
        if node.odometry.is_complete():
            # node.odometry.set_header(node, msg)
            node.publisher_odometry.publish(node.odometry.get_odometry())
            # print('Odometry published')


def pub_posewithcovariancestamped(node, msg):
    if node.poseWithCovarianceStamped.is_active():
        node.poseWithCovarianceStamped.set_msg_data(msg)
        if node.poseWithCovarianceStamped.is_complete():
            # node.poseWithCovarianceStamped.set_header(node, msg)
            node.publisher_posewithcovariancestamped.publish(node.poseWithCovarianceStamped.get_posewithcovariancestamped())
            # print('PoseWithCovarianceStamped published')


# def pub_posestamped(node, msg):
#     if node.poseStamped.is_active():
#         node.poseStamped.set_msg_data(msg)
#         if node.poseStamped.is_complete():
#             # node.poseStamped.set_header(node, msg)
#             node.publisher_posestamped.publish(node.poseStamped.get_posestamped())
#             # print('PoseStamped published')


# def pub_twistwithcovariancestamped(node, msg):
#     if node.twistWithCovarianceStamped.is_active():
#         node.twistWithCovarianceStamped.set_msg_data(msg)
#         if node.twistWithCovarianceStamped.is_complete():
#             node.twistWithCovarianceStamped.set_header(node, msg)
#             node.publisher_twistwithcovariancestamped.publish(node.twistWithCovarianceStamped.get_twistwithcovariancestamped())
#             # print('TwistWithCovarianceStamped published')


def pub_twiststamped(node, msg):
    if node.twistStamped.is_active():
        node.twistStamped.set_msg_data(msg)
        # node.twistStamped.set_header(node, msg)
        node.publisher_twiststamped.publish(node.twistStamped.get_twiststamped())
        # print('TwistStamped published')


def get_frequency(frequency):
    _frequency = 0
    try:
        _frequency = int(frequency)
    except:
        pass
    return _frequency


def get_calculated_frequency(times):
    if len(times) < 10:
        return 0
    durations = []
    for i in range(1, len(times)):
        durations.append(times[i] - times[i - 1])
    # print(f'durations: {durations}')
    av = sum(durations) / len(durations)
    # print(f'average: {av}')
    return round((1 / av), 2)


def prepare_publisher(node, cl, maintiming, prescaler, topic_name, _frequency, remap_to, timestamp_mode, leap_seconds):
    remaps = '' if (len(remap_to) == 0) else f' (remapping to {remap_to})'
    log_info(node, '====================================================================')
    log_info(node, f'configuring topic {topic_name}{remaps}')
    frq_set = 0
    topic_id = -1

    if type(topic_name) != str:
        return topic_id, f'+ Topic {topic_name} has wrong type {type(topic_name)} \n'

    if topic_name.lower() == valid_topics.names.imu_:
        # topic_id = topic_ids.tid_imu
        topic_id = valid_topics.items[valid_topics.names.imu_]['id']
        if not remap_to == '':
            topic_name = remap_to
        # frq_set = max(add_logs_imu(node, cl, maintiming, prescaler, _frequency))
        frq_set = add_logs_imu(node, cl, maintiming, prescaler, _frequency)
        node.publisher_imu = node.create_publisher(Imu, topic_name, node.qos)
        node.imu.activate(node, maintiming, leap_seconds, timestamp_mode)
    if topic_name.lower() == valid_topics.names.navsatstatus_:
        # topic_id = topic_ids.tid_navsatstatus
        topic_id = valid_topics.items[valid_topics.names.navsatstatus_]['id']
        if not remap_to == '':
            topic_name = remap_to
        # frq_set = max(add_logs_navsatstatus(node, cl, maintiming, _frequency))
        frq_set = add_logs_navsatstatus(node, cl, _frequency)
        node.publisher_navsatstatus = node.create_publisher(NavSatStatus, topic_name, node.qos)
        node.navSatStatus.activate()
    if topic_name.lower() == valid_topics.names.navsatfix_gnss_:
        # topic_id = topic_ids.tid_navsatfix
        topic_id = valid_topics.items[valid_topics.names.navsatfix_gnss_]['id']
        if not remap_to == '':
            topic_name = remap_to
        # frq_set = max(add_logs_navsatfix(node, cl, maintiming, _frequency))
        frq_set = add_logs_navsatfix(node, cl, _frequency)
        node.publisher_navsatfix = node.create_publisher(NavSatFix, topic_name, node.qos)
        node.navSatFix.activate(node, leap_seconds, timestamp_mode)
    if topic_name.lower() == valid_topics.names.navsatfix_ins_:
        # topic_id = topic_ids.tid_navsatfix_ins
        topic_id = valid_topics.items[valid_topics.names.navsatfix_ins_]['id']
        if not remap_to == '':
            topic_name = remap_to
        # frq_set = max(add_logs_navsatfix_ins(node, cl, maintiming, _frequency))
        frq_set = add_logs_navsatfix_ins(node, cl, maintiming, prescaler, _frequency)
        node.publisher_navsatfix_ins = node.create_publisher(NavSatFix, topic_name, node.qos)
        res_activate_nsfi = node.navSatFixIns.activate(node, leap_seconds, timestamp_mode)
        if res_activate_nsfi == 1:
            log_warning(node, 'Alt. Mode is set to MSL (above Geoid). '
                              'Altitude will be converted to WGS84 (above Ellipsoid).')
        if res_activate_nsfi == 2:
            log_error(node, 'Altitude above WGS84 Ellipsoid not available (Alt. Mode: BARO). '
                            'Change Alt. Mode to WGS84 or MSL and restart topic.')
    if topic_name.lower() == valid_topics.names.timereference_:
        # topic_id = topic_ids.tid_timereference
        topic_id = valid_topics.items[valid_topics.names.timereference_]['id']
        if not remap_to == '':
            topic_name = remap_to
        # frq_set = max(add_logs_timereference(node, cl, maintiming, _frequency))
        frq_set = add_logs_timereference(node, cl, maintiming, prescaler, _frequency)
        node.publisher_timereference = node.create_publisher(TimeReference, topic_name, node.qos)
        node.timeReference.activate(node, leap_seconds, timestamp_mode)
    if topic_name.lower() == valid_topics.names.magneticfield_:
        # topic_id = topic_ids.tid_magneticfield
        topic_id = valid_topics.items[valid_topics.names.magneticfield_]['id']
        if not remap_to == '':
            topic_name = remap_to
        # frq_set = max(add_logs_magneticfield(node, cl, maintiming, _frequency))
        frq_set = add_logs_magneticfield(node, cl, maintiming, prescaler, _frequency)
        node.publisher_magneticfield = node.create_publisher(MagneticField, topic_name, node.qos)
        node.magneticField.activate(node, leap_seconds, timestamp_mode)
    if topic_name.lower() == valid_topics.names.odometry_:
        # topic_id = topic_ids.tid_odometry
        topic_id = valid_topics.items[valid_topics.names.odometry_]['id']
        if not remap_to == '':
            topic_name = remap_to
        # frq_set = max(add_logs_odometry(node, cl, maintiming, _frequency))
        frq_set = add_logs_odometry(node, cl, maintiming, prescaler, _frequency)
        node.publisher_odometry = node.create_publisher(Odometry, topic_name, node.qos)
        node.odometry.activate(node, leap_seconds, timestamp_mode)
    if topic_name.lower() == valid_topics.names.posewithcovariancestamped_:
        # topic_id = topic_ids.tid_posewithcovariancestamped
        topic_id = valid_topics.items[valid_topics.names.posewithcovariancestamped_]['id']
        if not remap_to == '':
            topic_name = remap_to
        # frq_set = max(add_logs_posewithcovariancestamped(node, cl, maintiming, _frequency))
        frq_set = add_logs_posewithcovariancestamped(node, cl, maintiming, prescaler, _frequency)
        node.publisher_posewithcovariancestamped = node.create_publisher(PoseWithCovarianceStamped, topic_name, node.qos)
        node.poseWithCovarianceStamped.activate(node, leap_seconds, timestamp_mode)
    # if topic_name.lower() == 'posestamped':                             # deprecated
    #     # topic_id = topic_ids.tid_posestamped
    #     topic_id = valid_topics.items['posestamped']['id']
    #     if not remap_to == '':
    #         topic_name = remap_to
    #     frq_set = max(add_logs_posestamped(node, cl, maintiming, _frequency))
    #     node.publisher_posestamped = node.create_publisher(PoseStamped, topic_name, node.qos)
    #     node.poseStamped.activate(node)
    # if topic_name.lower() == 'twistwithcovariancestamped':              # deprecated
    #     # topic_id = topic_ids.tid_twistwithcovariancestamped
    #     topic_id = valid_topics.items['twistwithcovariancestamped']['id']
    #     if not remap_to == '':
    #         topic_name = remap_to
    #     frq_set = max(add_logs_twistwithcovariancestamped(node, cl, maintiming, _frequency))
    #     node.publisher_twistwithcovariancestamped = node.create_publisher(TwistWithCovarianceStamped, topic_name, node.qos)
    #     node.twistWithCovarianceStamped.activate()
    if topic_name.lower() == valid_topics.names.twiststamped_:
        topic_id = valid_topics.items[valid_topics.names.twiststamped_]['id']
        if not remap_to == '':
            topic_name = remap_to
        # frq_set = max(add_logs_twiststamped(node, cl, maintiming, _frequency))
        frq_set = add_logs_twiststamped(node, cl, maintiming, prescaler, _frequency)
        node.publisher_twiststamped = node.create_publisher(TwistStamped, topic_name, node.qos)
        node.twistStamped.activate(node, leap_seconds, timestamp_mode)

    return topic_id, f'+ Topic {topic_name} is set up @ {frq_set} Hz \n'


def prepare_action_server_example(node):
    node._action_server = ActionServer(
        node,
        Example,
        'example',
        node.example_callback)
    return f'Example Action Server installed'


def prepare_action_server_stop(node):
    node._action_server = ActionServer(
        node,
        Stop,
        'stop',
        node.stop_callback)
    return f'Stop Action Server installed'


def prepare_action_server_realign(node):
    node._action_server = ActionServer(
        node,
        Realign,
        'realign',
        node.realign_callback)
    return f'Realign Action Server installed'


def prepare_action_server_reboot(node):
    node._action_server = ActionServer(
        node,
        Reboot,
        'reboot',
        node.reboot_callback)
    return f'Reboot Action Server installed'


def prepare_action_server_config(node):
    node._action_server = ActionServer(
        node,
        GetConfigFile,
        'get_config_file',
        node.config_callback)
    return f'Config Action Server installed'


def rpyToQuat(rpy):
    q = [1, 0, 0, 0]
    q[3] = cos(rpy[0] / 2.0) * cos(rpy[1] / 2.0) * cos(rpy[2] / 2.0) + sin(rpy[0] / 2.0) * sin(rpy[1] / 2.0) * sin(rpy[2] / 2.0)
    q[0] = sin(rpy[0] / 2.0) * cos(rpy[1] / 2.0) * cos(rpy[2] / 2.0) - cos(rpy[0] / 2.0) * sin(rpy[1] / 2.0) * sin(rpy[2] / 2.0)
    q[1] = cos(rpy[0] / 2.0) * sin(rpy[1] / 2.0) * cos(rpy[2] / 2.0) + sin(rpy[0] / 2.0) * cos(rpy[1] / 2.0) * sin(rpy[2] / 2.0)
    q[2] = cos(rpy[0] / 2.0) * cos(rpy[1] / 2.0) * sin(rpy[2] / 2.0) - sin(rpy[0] / 2.0) * sin(rpy[1] / 2.0) * cos(rpy[2] / 2.0)
    return q


def gps_timestamp(msg_header, leap_seconds):
    t = Time()
    # w = msg_header.week
    t.sec = msg_header.timeOfWeek_sec + msg_header.week * 604800 + 315964800 - leap_seconds
    t.nanosec = msg_header.timeOfWeek_usec * 1000

    # print(f'imu timestamp: {msg_header.week}:{msg_header.timeOfWeek_sec}.{msg_header.timeOfWeek_usec}')
    # print(f'imu utc: {msg_header.timeOfWeek_sec + msg_header.week * 604800 + 315964800 - leap_seconds}.{msg_header.timeOfWeek_usec}')
    # print(f'out time: {t.sec}.{t.nanosec}')
    # towus = t.nanosec / 1000
    # tows = t.sec - w * 604800 - 315964800 + leap_seconds
    # print(f'imu  calcback: {w}:{tows}.{towus}')

    # print(f'{msg_header.week}:{msg_header.timeOfWeek_sec}.{msg_header.timeOfWeek_usec}/'
    #       f'{msg_header.timeOfWeek_sec + msg_header.week * 604800 + 315964800 - leap_seconds}.{msg_header.timeOfWeek_usec}/'
    #       f'{t.sec}.{t.nanosec}/'
    #       f'{w}:{tows}.{towus}')

    return t


class Service:
    GPS = 1
    GLONASS = 2
    # COMPASS = 4
    GALILEO = 8


class Status:
    NO_FIX = -1
    FIX = 0
    SBAS_FIX = 1
    GBAS_FIX = 2


class CovarianceType:
    UNKNOWN = 0
    APPROXIMATED = 1
    DIAGONAL_KNOWN = 2
    KNOWN = 3


class TimestampMode:
    ros = 'ROS_EXECUTION'
    gps = 'GPS_SYNC'

class Frq:
    gnss = 1
    ekf = 20
