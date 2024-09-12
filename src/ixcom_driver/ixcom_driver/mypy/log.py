#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

from enum import IntEnum
import os


class LogLevel(IntEnum):
    info = 0,
    warning = 1,
    error = 2,
    debug = 3


class Log:
    @staticmethod
    def log(node, msg, log_level):
        if node.running:
            if log_level == LogLevel.debug and not node.dbg:
                return
            if '\00' in msg:
                msg = msg.replace('\00', '')
            tc = int(node.get_clock().now().nanoseconds / 1e9)
            lls = '___'
            if log_level == LogLevel.info:
                lls = 'INFO'
                node.get_logger().info(msg)
            elif log_level == LogLevel.warning:
                lls = 'WARNING'
                node.get_logger().warning(msg)
            elif log_level == LogLevel.error:
                lls = 'ERROR'
                node.get_logger().error(msg)
            elif log_level == LogLevel.debug:
                lls = 'DEBUG'
                node.get_logger().debug(msg)
            fn = os.path.join(node.get_log_dir(), f'log_{node.t}.log')
            f = open(fn, 'a')
            f.write(f'[{tc}] [{lls}] ' + msg + '\n')
            f.close()
