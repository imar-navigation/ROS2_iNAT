import pytest
import os
print(os.getcwd())
from ixcom_driver.ixcom_driver.mypy.fun import *
import ixcom.protocol
from ixcom.exceptions import CommunicationError, ResponseError, ClientTimeoutError
from ..test.mock.mock_client import MockClient
from ..test.dummy.dummy_node import DummyNode
from ..ixcom_driver.ixcom_driver_subscriber_integrationTests import DataSubscriberIntegrationTests
import rclpy
from threading import Thread
from .initTestNode import dSub
from .config import settings
import warnings

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
def test_Topic_Imu_time_diff():
    desiredFrequency = int(settings.publishernodeconfig["topics"]["Imu"]["frequency_hz"])
    timeDiffPerMessage = 1/desiredFrequency
    dSub.setTimeDiffTestPara()
    time.sleep(3)
    #Counter = dSub.waitForMessage("Imu",WAITTIME)
    Sum = dSub.getTimeStampSum()
    print(len(Sum))
    MessageCounts = len(Sum)
    OnePercentOfAllMessages = MessageCounts*0.01
    SumDiff = []
    assert 0 != MessageCounts
    for x in range(MessageCounts):
        if x == 0:
            pass
        else:
            SumDiff.append(Sum[x] - Sum[x-1])
 
    lostMessages= 0
    for x in SumDiff:
        if x > timeDiffPerMessage*1.5:
            lostMessages = lostMessages + 1
            
        else:
            assert timeDiffPerMessage  == pytest.approx(x, rel=0.02)
    if lostMessages > 0:
        warnings.warn(UserWarning("Timedifferenz in header of IMU Topic shows that", lostMessages, "messages from a total number of", MessageCounts,"messages are lost"))
    assert lostMessages <= OnePercentOfAllMessages, "The time difference of more then 1'%' of the received IMU messages is to high. Too much lost messages"


def main():
    print(123)

    pass

if __name__ == "__main__":
    main()
