#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

import pytest
from .dummy.dummy_node import DummyNode
from ixcom_driver.ixcom_driver.mypy.fun import *
import ixcom.protocol

@pytest.mark.test_pub
class TestPub:

    def test_pub_imu(self):
        self.dummy_node = DummyNode()
        self.dummy_msg = ixcom.protocol.Message
        pub_imu(self.dummy_node, self.dummy_msg)

    def test_pub_magneticfield(self):
        self.dummy_node = DummyNode()
        self.dummy_msg = ixcom.protocol.Message
        pub_magneticfield(self.dummy_node, self.dummy_msg)

    def test_pub_odometry(self):
        self.dummy_node = DummyNode()
        self.dummy_msg = ixcom.protocol.Message
        pub_odometry(self.dummy_node, self.dummy_msg)

    def test_pub_navsatstatus(self):
        self.dummy_node = DummyNode()
        self.dummy_msg = ixcom.protocol.Message
        pub_navsatstatus(self.dummy_node, self.dummy_msg)

    def test_pub_navsatfix(self):
        self.dummy_node = DummyNode()
        self.dummy_msg = ixcom.protocol.Message
        pub_navsatfix(self.dummy_node, self.dummy_msg)

    def test_pub_navsatfix_ins(self):
        self.dummy_node = DummyNode()
        self.dummy_msg = ixcom.protocol.Message
        pub_navsatfix_ins(self.dummy_node, self.dummy_msg)

    def test_pub_timereference(self):
        self.dummy_node = DummyNode()
        self.dummy_msg = ixcom.protocol.Message
        pub_timereference(self.dummy_node, self.dummy_msg)

    def test_pub_posewithcovariancestamped(self):
        self.dummy_node = DummyNode()
        self.dummy_msg = ixcom.protocol.Message
        pub_posewithcovariancestamped(self.dummy_node, self.dummy_msg)
