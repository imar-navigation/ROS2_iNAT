#!/usr/bin/python3

# Copyright 2024 iMAR Navigation GmbH

class SerialConfErr(Exception):
    def __init__(self, msg=None):
        self.msg = f'SerialConfErr: {msg}'
        super().__init__(self.msg)


class QoSConfErr(Exception):
    def __init__(self, msg=None):
        self.msg = f'QoSConfErr: {msg}'
        super().__init__(self.msg)


class LoadConfigFileErr(Exception):
    def __init__(self, msg=None):
        self.msg = f'LoadConfigFileErr: {msg}'
        super().__init__(self.msg)


class ParseJsonErr(Exception):
    def __init__(self, msg=None):
        self.msg = f'ParseJsonErr: {msg}'
        super().__init__(self.msg)


class MissingKeysErr(Exception):
    def __init__(self, msg=None):
        self.msg = f'MissingKeysErr: {msg}'
        super().__init__(self.msg)


class TooMuchTopicsErr(Exception):
    def __init__(self, msg=None):
        self.msg = f'TooMuchTopicsErr: {msg}'
        super().__init__(self.msg)


class ClientConnectErr(Exception):
    def __init__(self, msg=None):
        self.msg = f'ClientConnectErr: {msg}'
        super().__init__(self.msg)
