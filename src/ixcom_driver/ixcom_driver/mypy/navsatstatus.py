#!/usr/bin/python3

from sensor_msgs.msg import NavSatStatus
import ixcom.protocol
from .fun import Status, Service


class NavSatStatus_(NavSatStatus):

    def __init__(self):
        self.active = False
        self.navSatStatus = NavSatStatus()
        self.navSatStatus.service = 0

    def activate(self):
        self.active = True

    def is_active(self):
        return self.active

    def set_msg_data(self, msg):
        if isinstance(msg.payload, ixcom.messages.GNSSSOL_Payload):
            self.__set_GNSS_data(msg)

    def set_par_data(self, par):
        if par is not None and isinstance(par.payload, ixcom.parameters.PARGNSS_LOCKOUTSYSTEM_Payload):
            self.__set_LOCKOUT_data(par)

    def __set_GNSS_data(self, msg):

        pt = int(msg.payload.data['posType'])

        # print(f'position type: {pt}')

        if pt >= 50:
            res = Status.GBAS_FIX  # NARROW_INT (50)
        else:
            if pt >= 18:
                res = Status.SBAS_FIX  # SBAS (18)
            else:
                if pt >= 1:
                    res = Status.FIX  # FIXEDPOS (1)
                else:
                    res = Status.NO_FIX  # NONE (0)

        # this fails if res is a int8
        self.navSatStatus.status = res


    def __set_LOCKOUT_data(self, par):
        # print('set LOCkOUT data')
        lm = int(par.payload.data['lockoutMask'])
        res = Service.GPS + Service.GLONASS + Service.GALILEO
        if lm & (1 << 0):
            # GPS is locked out
            res -= Service.GPS
        if lm & (1 << 1):
            # GLONAS is locked out
            res -= Service.GLONASS
        if lm & (1 << 3):
            # GALILEO is locked out
            res -= Service.GALILEO
        self.navSatStatus.service = res

    def get_navSatStatus(self):
        return self.navSatStatus
