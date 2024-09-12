#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

import pytest
from ixcom_driver.ixcom_driver.mypy.fun import *
import ixcom.protocol
from ixcom.exceptions import CommunicationError, ResponseError, ClientTimeoutError
from .mock.mock_client import MockClient
from .dummy.dummy_node import DummyNode
import os

log_dir = f'log{os.sep}run'
wrkdir = os.getcwd()
log_path = f'{wrkdir}{os.sep}{log_dir}'

@pytest.mark.test_config_inat
class TestConfig_iNAT:

    def test_set_serial_config(self):
        dummy_config = {'ignore': 0,
                        'serial_port': 1,
                        'enable': True,
                        'baud_rate': 19200}
        mcl = MockClient()
        assert set_serial_config(mcl, dummy_config) == res.ok

    def test_set_serial_config_ignored(self):
        dummy_config = {'ignore': 1,
                        'serial_port': 1,
                        'enable': True,
                        'baud_rate': 19200}
        mcl = MockClient()
        assert set_serial_config(mcl, dummy_config) == res.ignored

    def test_set_serial_config_comm_err(self):
        dummy_config = {'ignore': 0,
                        'serial_port': 1,
                        'enable': True,
                        'baud_rate': 19200}
        mcl = MockClient()
        mcl.setCommErr()
        with pytest.raises(SerialConfErr):
            set_serial_config(mcl, dummy_config)

    def test_set_serial_config_resp_err(self):
        dummy_config = {'ignore': 0,
                        'serial_port': 1,
                        'enable': True,
                        'baud_rate': 19200}
        mcl = MockClient()
        mcl.setRespErr()
        with pytest.raises(SerialConfErr):
            set_serial_config(mcl, dummy_config)

    def test_connect_inat(self):
        with pytest.raises(Exception):
            connect_inat('0.0.0.0', 12)

    def test_connect_inat_none(self):
        with pytest.raises(Exception):
            connect_inat(None, None)

    def test_connect_inat_empty(self):
        with pytest.raises(Exception):
            connect_inat('', '')

    # def test_get_maintiming_good(self):
    #     mcl = MockClient()
    #     mcl.set_good_maintiming()
    #     maintiming, prescaler, assumed = get_maintiming(mcl)
    #     assert (maintiming == 1000) and (assumed is False)
    #
    # def test_get_maintiming_bad(self):
    #     mcl = MockClient()
    #     mcl.set_bad_maintiming()
    #     maintiming, prescaler, assumed = get_maintiming(mcl)
    #     assert (maintiming == 500) and (assumed is True)

    def test_add_logs_imu(self):
        mcl = MockClient()
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_imu(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_imu_resp_err(self):
        mcl = MockClient()
        mcl.set_bad_log_response(ResponseError)
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        # with pytest.raises(ResponseError):
        #     add_logs_imu(dummy_node, mcl, maintiming, prescaler, frq)
        assert add_logs_imu(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_imu_to_err(self):
        mcl = MockClient()
        mcl.set_bad_log_response(ClientTimeoutError)
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        # with pytest.raises(ClientTimeoutError):
        #     add_logs_imu(dummy_node, mcl, maintiming, prescaler, frq)
        assert add_logs_imu(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_odometry(self):
        mcl = MockClient()
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_odometry(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_odometry_resp_err(self):
        mcl = MockClient()
        mcl.set_bad_log_response(ResponseError)
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        # with pytest.raises(ResponseError):
        #     add_logs_odometry(dummy_node, mcl, maintiming, prescaler, frq)
        assert add_logs_odometry(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_odometry_to_err(self):
        mcl = MockClient()
        mcl.set_bad_log_response(ClientTimeoutError)
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        # with pytest.raies(ClientTimeoutError):
        #     add_logs_odometry(dummy_node, mcl, maintiming, frq)
        assert add_logs_odometry(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_posewithcovariancestamped(self):
        mcl = MockClient()
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_posewithcovariancestamped(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_posewithcovariancestamped_resp_err(self):
        mcl = MockClient()
        mcl.set_bad_log_response(ResponseError)
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_posewithcovariancestamped(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_posewithcovariancestamped_to_err(self):
        mcl = MockClient()
        mcl.set_bad_log_response(ClientTimeoutError)
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_posewithcovariancestamped(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_navsatstatus(self):
        mcl = MockClient()
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        frq = 500
        assert add_logs_navsatstatus(dummy_node, mcl, frq) > 0

    def test_add_logs_navsatstatus_resp_err(self):
        mcl = MockClient()
        mcl.set_bad_log_response(ResponseError)
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        frq = 500
        assert add_logs_navsatstatus(dummy_node, mcl, frq) > 0

    def test_add_logs_navsatstatus_to_err(self):
        mcl = MockClient()
        mcl.set_bad_log_response(ClientTimeoutError)
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        frq = 500
        assert add_logs_navsatstatus(dummy_node, mcl, frq) > 0

    def test_add_logs_navsatfix(self):
        mcl = MockClient()
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_navsatfix(dummy_node, mcl, frq) > 0

    def test_add_logs_navsatfix_resp_err(self):
        mcl = MockClient()
        mcl.set_bad_log_response(ResponseError)
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_navsatfix(dummy_node, mcl, frq) > 0

    def test_add_logs_navsatfix_to_err(self):
        mcl = MockClient()
        mcl.set_bad_log_response(ClientTimeoutError)
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_navsatfix(dummy_node, mcl, frq) > 0

    def test_add_logs_navsatfix_ins(self):
        mcl = MockClient()
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_navsatfix_ins(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_navsatfix_ins_resp_err(self):
        mcl = MockClient()
        mcl.set_bad_log_response(ResponseError)
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_navsatfix_ins(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_navsatfix_ins_to_err(self):
        mcl = MockClient()
        mcl.set_bad_log_response(ClientTimeoutError)
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_navsatfix_ins(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_timereference(self):
        mcl = MockClient()
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_timereference(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_timereference_resp_err(self):
        mcl = MockClient()
        mcl.set_bad_log_response(ResponseError)
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_timereference(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_timereference_to_err(self):
        mcl = MockClient()
        mcl.set_bad_log_response(ClientTimeoutError)
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_timereference(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_magneticfield(self):
        mcl = MockClient()
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_magneticfield(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_magneticfield_resp_err(self):
        mcl = MockClient()
        mcl.set_bad_log_response(ResponseError)
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_magneticfield(dummy_node, mcl, maintiming, prescaler, frq) > 0

    def test_add_logs_magneticfield_to_err(self):
        mcl = MockClient()
        mcl.set_bad_log_response(ClientTimeoutError)
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        maintiming = 500
        prescaler = 1
        frq = 500
        assert add_logs_magneticfield(dummy_node, mcl, maintiming, prescaler, frq) > 0

    @pytest.mark.parametrize(["topic_name", "topic_id"], [
                                ("invalid", -1),
                                (4711, -1),
                                ("Imu", valid_topics.items['imu']['id']),
                                ("imu", valid_topics.items['imu']['id']),
                                ("imU", valid_topics.items['imu']['id']),
                                ("imU", valid_topics.items['imu']['id']),
                                ("NavSatStatus", valid_topics.items['navsatstatus']['id']),
                                ("navsatStatus", valid_topics.items['navsatstatus']['id']),
                                ("NavSatFix_GNSS", valid_topics.items['navsatfix_gnss']['id']),
                                ("navSatFix_GNSS", valid_topics.items['navsatfix_gnss']['id']),
                                ("NavSatFix_INS", valid_topics.items['navsatfix_ins']['id']),
                                ("NavSatFix_ins", valid_topics.items['navsatfix_ins']['id']),
                                ("TimeReference", valid_topics.items['timereference']['id']),
                                ("TimeReferENce", valid_topics.items['timereference']['id']),
                                ("MagneticField", valid_topics.items['magneticfield']['id']),
                                ("MagnEticfield", valid_topics.items['magneticfield']['id']),
                                ("Odometry", valid_topics.items['odometry']['id']),
                                ("odomeTRY", valid_topics.items['odometry']['id']),
                                ("PoseWithCovarianceStamped", valid_topics.items['posewithcovariancestamped']['id']),
                                ("PoseWithCOVARianceStamped", valid_topics.items['posewithcovariancestamped']['id'])])
    def test_prepare_publisher(self, topic_name, topic_id):
        dummy_node = DummyNode()
        log_init(dummy_node, log_path)
        mcl = MockClient()
        assert prepare_publisher(dummy_node, mcl, 500, 1, topic_name, 500, '', 'GPS_SYNC', 18)[0] == topic_id

    @pytest.mark.parametrize(["frequency", "expected"], [('10', 10),
                                                         (10, 10),
                                                         ('', 0),
                                                         ('invalid', 0),
                                                         ('0', 0),
                                                         (0, 0),
                                                         ('-10', -10),
                                                         ('10', 10)])
    def test_get_frequency(self, frequency, expected):
        assert get_frequency(frequency) == expected
