import pytest
import os
# print(os.getcwd())
#from config import settings
# print(settings)
# print(settings.test)
# print(settings.PathToiXCOMDriver)


from ixcom_driver.ixcom_driver.mypy.fun import *
import ixcom.protocol
from ixcom.exceptions import CommunicationError, ResponseError, ClientTimeoutError
from ..test.mock.mock_client import MockClient
from ..test.dummy.dummy_node import DummyNode
from ..ixcom_driver.ixcom_driver_subscriber_integrationTests import DataSubscriberIntegrationTests
import rclpy
from threading import Thread
from ..integrationTests.config import settings
print(settings)
print(settings.test + "x")
print(settings.PathToiXCOMDriver)
from ..integrationTests.initTestNode import dSub
import warnings

#settings.load_file(path="/home/dev/ros2BranchKeHo/ixcom-ros2-driver/ixcom_driver/src/ixcom_driver/integrationTests/settings.json")
WAITTIME = 3

def setup_function():
    #settings.load_file(path="/home/dev/ros2BranchKeHo/ixcom-ros2-driver/ixcom_driver/src/ixcom_driver/integrationTests/settings.json")
    #print(settings.PublisherNodeConfig.topics.Imu.frequency_hz)
    print(settings.test)
    pass

    #assert 1 == 2
def teardown_function():
    pass

#@pytest.mark.parametrize("nmbOfParticipants", [1])
def test_Topic_Imu_freq():
    desiredFrequency = int(settings.publishernodeconfig["topics"]["Imu"]["frequency_hz"])
    dSub.resetMessageCounter()
    Counter = dSub.waitForMessage("Imu",WAITTIME)
    assert Counter >= (desiredFrequency*WAITTIME)*0.95


def test_Topic_NavSatStatus_freq():

    desiredFrequency = int(settings.publishernodeconfig["topics"]["NavSatStatus"]["frequency_hz"])
    dSub.resetMessageCounter()
    Counter = dSub.waitForMessage("NavSatStatus",WAITTIME + 0.5)
    assert Counter >= (desiredFrequency*WAITTIME)*0.95


def test_Topic_NavSatFix_freq():
    try:
        desiredFrequency = int(settings.publishernodeconfig["topics"]["NavSatFix"]["frequency_hz"] )
    except:
        desiredFrequency = int(settings.publishernodeconfig["topics"]["NavSatFix_GNSS"]["frequency_hz"] )
    dSub.resetMessageCounter()
    Counter = dSub.waitForMessage("NavSatFix",WAITTIME + 0.5)
    assert Counter >= (desiredFrequency*WAITTIME)*0.95


def test_Topic_NavSatFix_INS_freq():
    desiredFrequency = int(settings.publishernodeconfig["topics"]["NavSatFix_INS"]["frequency_hz"] )
    dSub.resetMessageCounter()
    Counter = dSub.waitForMessage("NavSatFix_INS",WAITTIME)
    assert Counter >= (desiredFrequency*WAITTIME)*0.95

def test_Topic_TimeReference_INS_freq():
    desiredFrequency = int(settings.publishernodeconfig["topics"]["TimeReference"]["frequency_hz"]) 
    dSub.resetMessageCounter()
    Counter = dSub.waitForMessage("TimeReference",WAITTIME)
    assert Counter >= (desiredFrequency*WAITTIME)*0.95

def test_Topic_MagneticField_INS_freq():
    desiredFrequency = int(settings.publishernodeconfig["topics"]["MagneticField"]["frequency_hz"]) 
    dSub.resetMessageCounter()
    Counter = dSub.waitForMessage("MagneticField",WAITTIME)
    if Counter == 0:
        warnings.warn(UserWarning("API - MangneticField (XCOM-MAGDATA) is not included in the QNX-Firmware and therefore not in the iNAT-Sim")) 
    else:
        assert Counter >= (desiredFrequency*WAITTIME)*0.95

def test_Topic_Odometry_INS_freq():
    desiredFrequency = int(settings.publishernodeconfig["topics"]["Odometry"]["frequency_hz"])
    dSub.resetMessageCounter()
    Counter = dSub.waitForMessage("Odometry",WAITTIME)
    assert Counter >= (desiredFrequency*WAITTIME)*0.95    

def test_Topic_PoseWithCovarianceStamped_freq():
    desiredFrequency = int(settings.publishernodeconfig["topics"]["PoseWithCovarianceStamped"]["frequency_hz"])
    dSub.resetMessageCounter()
    Counter = dSub.waitForMessage("PoseWithCovarianceStamped",WAITTIME)
    assert Counter >= (desiredFrequency*WAITTIME)*0.95  

def test_Topic_TwistStamped_freq():
    desiredFrequency = int(settings.publishernodeconfig["topics"]["TwistStamped"]["frequency_hz"]) 
    dSub.resetMessageCounter()
    Counter = dSub.waitForMessage("TwistStamped",WAITTIME)
    assert Counter >= (desiredFrequency*WAITTIME)*0.95      

# def main():
#     print(123)

#     pass

# if __name__ == "__main__":
#     main()
