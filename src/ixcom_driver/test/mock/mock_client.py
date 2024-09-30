
#!/usr/bin/python3

# Copyright 2023 iMAR Navigation GmbH

import ixcom.protocol
from ixcom.exceptions import CommunicationError, ResponseError, ClientTimeoutError

class MockClient:
    def __init__(self):
        self.CommErr = False
        self.RespErr = False
        self.ToErr = False
        # self.info = {}
        self.log_response = 'ok'
        class Payload:
            def __init__(self):
                self.data = {}
            def set_mt(self, mt):
                self.data['maintiming'] = mt
            def set_ps(self, ps):
                self.data['prescaler'] = ps
        class Info:
            def __init__(self):
                self._payload = Payload()
            def payload(self):
                return self._payload
        self.info = Info()

    def setCommErr(self):
        self.CommErr = True

    def setRespErr(self):
        self.RespErr = True

    def send_msg_and_waitfor_okay(self, msgToSend):
        if self.CommErr:
            raise CommunicationError
        elif self.RespErr:
            raise ResponseError

    def set_good_maintiming(self):
        self.info.payload().data['maintiming'] = 1000
        self.info.payload().data['prescaler'] = 1

    def set_bad_maintiming(self):
        self.info = {}
        self.info = {'fault': 0}

    def set_bad_log_response(self, err_type):
        if err_type == ResponseError:
            self.RespErr = True
        elif err_type == ClientTimeoutError:
            self.ToErr = True

    # def get_device_info(self):
    #     return self.info
    def get_parameter(self, par):
        return self.info
        # if par == ixcom.parameters.PARSYS_MAINTIMING_Payload.parameter_id:
        #     return self.info
        # if par == ixcom.parameters.PARSYS_PRESCALER_Payload.parameter_id:
        #     return self.info['prescaler']

    def add_log_with_rate(self, id, frq):
        if self.RespErr:
            raise ResponseError
        elif self.ToErr:
            raise ClientTimeoutError