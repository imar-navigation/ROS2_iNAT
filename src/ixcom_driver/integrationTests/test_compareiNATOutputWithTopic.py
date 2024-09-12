import pytest
import os
print(os.getcwd())
from ixcom_driver.ixcom_driver.mypy.fun import *
import ixcom.protocol
#from ixcom.exceptions import CommunicationError, ResponseError, ClientTimeoutError
#from ..test.mock.mock_client import MockClient
#from ..test.dummy.dummy_node import DummyNode
#from ..ixcom_driver.ixcom_driver_subscriber_integrationTests import DataSubscriberIntegrationTests
#from ..ixcom_driver.mypy.fun import gps_timestamp
#import rclpy
#from threading import Thread
#from .initTestNode import dSub
from .config import settings
#import warnings

import ixcom.grep
from iMARPythonLib import iQuat
import csv
import math
from types import SimpleNamespace

def gps_timestamp(msg_header, leap_seconds):
    tsec = msg_header["timeOfWeek_sec"] + msg_header["week"] * 604800 + 315964800 - leap_seconds
    tnanosec = msg_header["timeOfWeek_usec"] * 1000
    return [tsec,tnanosec]

WAITTIME = 3
CWD = os.getcwd()
def setup_function():
    #settings.load_file(path="/home/dev/ros2BranchKeHo/ixcom-ros2-driver/ixcom_driver/src/ixcom_driver/integrationTests/settings.json")
    #print(settings.PublisherNodeConfig.topics.Imu.frequency_hz)
    print(settings.test)
    pass

    #assert 1 == 2
def teardown_function():
    pass

#@pytest.mark.parametrize("nmbOfParticipants", [1])
def test_compare_Imu_data():

    path = "/home/dev/Testdatensatz/StaticData/M300/SIL/recorder/incoming500.bin"
    path = settings.pathToCSVFileTopicData
    path = path + "incoming500.bin" 
    try:
        d = ixcom.grep.read_file(path)
        #accCorr = d["IMUCORR"]["acc"]
        omgCorr = d["IMUCORR"]["omg"]
        acc = d["INSSOL"]["acc"]
        #omg = d["INSSOL"]["omg"]
        rpy = d["INSSOL"]["rpy"]
        # vel = d["INSSOL"]["vel"]
        lon = d["INSSOL"]["lon"]
        lat = d["INSSOL"]["lat"] 
        alt = d["INSSOL"]["alt"]
        # undulation = d["INSSOL"]["undulation"]
        # gpstime = d["INSSOL"]["gpstime"] 
        timeWeek = d["INSSOL"]["week"]
        timeOfWeekSec = d["INSSOL"]["time_of_week_sec"]
        timeOfWeekUSec = d["INSSOL"]["time_of_week_usec"]

        # ecef = d["INSSOLECEF"]["pos_ecef"]
        # quatNED = d["INSSOLECEF"]["q_nb"]
        # covarianceRPY = d["EKFSTDDEVECEF"]["Rpy"]
        # covariancePos = d["EKFSTDDEVECEF"]["Pos"]
        # timeOfWeekSecSTDDDEV = d["EKFSTDDEVECEF"]["time_of_week_sec"]
        # timeWeekSTDDDEV = d["EKFSTDDEVECEF"]["week"]
        # timeOfWeekUSecSTDDDEV = d["EKFSTDDEVECEF"]["time_of_week_usec"]
        # timeOfWeekSec2 = d["INSSOLECEF"]["time_of_week_sec"]
        # timeWeek2 = d["INSSOLECEF"]["week"]
        # timeOfWeekUSec2 = d["INSSOLECEF"]["time_of_week_usec"]
        
        # timeWeekRAW = d["IMURAW"]["week"]
        # timeOfWeekSecRAW  = d["IMURAW"]["time_of_week_sec"]
        # timeOfWeekUSecRAW  = d["IMURAW"]["time_of_week_usec"]
        # accRAW = d["IMURAW"]["acc"]

        # timeWeekCORR = d["IMUCORR"]["week"]
        # timeOfWeekSecCORR  = d["IMUCORR"]["time_of_week_sec"]
        # timeOfWeekUSecCORR  = d["IMUCORR"]["time_of_week_usec"]    
    except:
        assert True == False, 'Could not open the .bin File (log from iNAT) at path {path}'

    iNATTimeInPOSIXTime = []
    for x in range(len(timeOfWeekSec)):
        time = gps_timestamp({'timeOfWeek_sec': timeOfWeekSec[x],'timeOfWeek_usec': timeOfWeekUSec[x],'week': timeWeek[x]},18)
        iNATTimeInPOSIXTime.append(time[0] + (time[1]/1000/1000/1000))

    # iNATTimeInPOSIXTimeCORR = []
    # for x in range(len(timeOfWeekSecCORR)):
    #     time = gps_timestamp({'timeOfWeek_sec': timeOfWeekSecCORR[x], 'timeOfWeek_usec': timeOfWeekUSecCORR[x], 'week': timeWeekCORR[x]},18)
    #     iNATTimeInPOSIXTimeCORR.append(time[0] + (time[1]/1000/1000/1000))    




    ################## START COMPARE NavSatFixINS DATA ########################################
    ## search for the first timestamp in the iNAT Logs that matches.
    # VERGLEICHE d mit TOPICDATA
    NavSatFixINSDataTopic = []
    path = CWD + "NavSatFixINSTopicMsgs.csv"
    try:
        reader = csv.DictReader(open(path))    
        for row in reader:
            #print(row)
            NavSatFixINSDataTopic.append(row)
    except FileNotFoundError:
        assert True == False, 'Could not open the NavSatFixINSTopicMsgs.csv File (log from topic)'
    ### Transform the data of the 
    timeStamp = []
    for x in range(len(NavSatFixINSDataTopic)):
        
        timestampSec  = int(NavSatFixINSDataTopic[x]["sec"])
        timestampNSecinSec = int(NavSatFixINSDataTopic[x]["nsec"]) / 1000 / 1000 / 1000
        timeStamp.append(timestampSec + timestampNSecinSec)


    indexOfFirstMatch = 0
    for x in range(len(iNATTimeInPOSIXTime)):
        if iNATTimeInPOSIXTime[x] ==  timeStamp[0]:
            indexOfFirstMatch = x
            break


    frequenzOfTopicMsgs = 1 / (timeStamp[1]-timeStamp[0])
    frequenzOfiNATMsgs = 1 /  (iNATTimeInPOSIXTime[1]-iNATTimeInPOSIXTime[0])
    ratioOfFrequencies = frequenzOfiNATMsgs / frequenzOfTopicMsgs
    ratioOfFrequencies = round(ratioOfFrequencies)


    timediffOfOneMessage = timeStamp[1]-timeStamp[0]
    count= 0
    for i in range(len(NavSatFixINSDataTopic)-1):
        if abs(timeStamp[i] - timeStamp[i+1]) > timediffOfOneMessage*1.1:
            count = count + 1
            print(count, "timediff high DIFF:",timeStamp[i] - timeStamp[i+1])

    if count > len(NavSatFixINSDataTopic)*0.05:
        assert False , ("more than 5% of the messages has been lost")
    ### check the NAVSatFix Data:
    index = indexOfFirstMatch
    for x in range(len(NavSatFixINSDataTopic)):
        indexFailures = 0
        while not (abs(iNATTimeInPOSIXTime[index] - timeStamp[x]) < 1e-5):
            index = index + 1
            indexFailures = indexFailures + 1
            if indexFailures > 5*ratioOfFrequencies:
                assert False ,"at least 5 consecutive messages lost"
            
        tempLatINSTopic = float(NavSatFixINSDataTopic[x]["latitude"])*math.pi/180
        if abs(lat[index] - tempLatINSTopic) < 1e-14:
            print(x,"lat=lat_topic",lat[index]," == ", tempLatINSTopic)
            #print("latitude OK")
            pass
        else:
            assert False, "inconsistent Data --- time is equal but latitude is not as expected"
        #LONGITUDE
        tempLonINSTopic = float(NavSatFixINSDataTopic[x]["longitude"])*math.pi/180
        if abs(lon[index] - tempLonINSTopic) < 1e-14:
            pass
            #print("longitude OK")
        else:

            assert False,"inconsistent Data --- time is equal but longitude is not as expected"
        #alt
        tempAltTopic = float(NavSatFixINSDataTopic[x]["altitude"])
        if abs(alt[index] - tempAltTopic) < 1e-4:
            pass
            #print("alt OK")
        else:
            
            assert False,"inconsistent Data --- time is equal but alt is not as expected"           



    ############### CHECK IMU TOPIC DATA #####################################
    ImuDataTopic = []
    path = CWD + "imuTopicMsgs.csv"
    try:
        reader = csv.DictReader(open(path))    
        for row in reader:
            #print(row)
            ImuDataTopic.append(row)
    except FileNotFoundError:
        assert True == False, 'Could not open the imuTopicMsgs.csv File (log from topic)'
    ### Transform the data of the 
    AttitudeIMU = []
    AttitudeIMUTransposed = []
    for x in range(len(ImuDataTopic)):
        QuatObj = iQuat([float(ImuDataTopic[x]["orientationQuatW"]),float(ImuDataTopic[x]["orientationQuatX"]),float(ImuDataTopic[x]["orientationQuatY"]),float(ImuDataTopic[x]["orientationQuatZ"])])
        AttitudeIMU.append(QuatObj.toRpy()) 
        Qattransposed  = QuatObj.transpose()
        QuatTrans = iQuat(Qattransposed)
        AttitudeIMUTransposed.append(QuatTrans.toRpy())  

    timeStampImu =[]
    for x in range(len(ImuDataTopic)):
        timestampSecImu  = int(ImuDataTopic[x]["sec"])
        timestampNSecinSecImu = int(ImuDataTopic[x]["nsec"]) / 1000 / 1000 / 1000
        timeStampImu.append( timestampSecImu + timestampNSecinSecImu )


    indexOfFirstMatchImu = 0
    for x in range(len(iNATTimeInPOSIXTime)):
        if iNATTimeInPOSIXTime[x] ==  timeStampImu[0]:
            indexOfFirstMatchImu = x
            break


    frequenzOfTopicMsgs = 1 / (timeStampImu[1]-timeStampImu[0])
    frequenzOfiNATMsgs = 1 /  (iNATTimeInPOSIXTime[1]-iNATTimeInPOSIXTime[0])
    ratioOfFrequencies = frequenzOfiNATMsgs / frequenzOfTopicMsgs
    ratioOfFrequenciesRounded = round(ratioOfFrequencies)


    timediffOfOneMessage = timeStampImu[1]-timeStampImu[0]
    count= 0
    for i in range(len(ImuDataTopic)-1):
        if abs(timeStampImu[i] - timeStampImu[i+1]) > timediffOfOneMessage*1.1:
            count = count + 1
            print(count, "timediff high DIFF IMU TOPIC:",timeStampImu[i] - timeStampImu[i+1])

    if count > len(ImuDataTopic)*0.02:
        assert False , ("dude, this is not right")

    # for i in range(len(rpy)):
    #        # rpy[i][0] 
    #     rpy[i][1] = -rpy[i][1]
    #     rpy[i][2] = normalizeToRange(math.pi/2 - rpy[i][2], -math.pi, math.pi)



    ### check the NAVSatFix Data:
    index = indexOfFirstMatchImu
    for x in range(len(ImuDataTopic)):
        indexFailures = 0
        while not (abs(iNATTimeInPOSIXTime[index] - timeStampImu[x]) < 1e-5):
            index = index + 1
            indexFailures = indexFailures + 1
            if indexFailures > 5*ratioOfFrequenciesRounded:
                assert False,"at least five consecutive messages lost"
            
        ############## CHECK OMG #############################    
        tempOmgXImuTopic = float(ImuDataTopic[x]["angularVelX"])
        if abs(omgCorr[index][0] - tempOmgXImuTopic) < 1e-10:
            print(x,"omgCorr_X/angularVelX",omgCorr[index][0]," == ", tempOmgXImuTopic)
            pass
        else:
            assert False,"inconsistent Data --- time is equal but OMGX is not equal/ as expected"
        
        tempOmgYImuTopic = float(ImuDataTopic[x]["angularVelY"])
        if abs(omgCorr[index][1] - tempOmgYImuTopic) < 1e-10:
            pass
        else:
            assert False,"inconsistent Data --- time is equal but OMGY is not equal/ as expected"
        tempOmgZImuTopic = float(ImuDataTopic[x]["angularVelZ"])
        if abs(omgCorr[index][2] - tempOmgZImuTopic) < 1e-10:
            pass
        else:
            assert False,"inconsistent Data --- time is equal but OMGZ is not equal/ as expected"    


        ##################### CHECK ACC ############################################
        tempACCXImuTopic = float(ImuDataTopic[x]["linearAccelerationX"])
        if abs(acc[index][0] - tempACCXImuTopic) < 1e-8:
            print(x,"acc_X/linearAccelerationX",acc[index][0]," == ", tempACCXImuTopic)
            pass
        else:
            assert False,"inconsistent Data --- time is equal but ACCX is not equal/ as expected"
        
        tempACCYImuTopic = float(ImuDataTopic[x]["linearAccelerationY"])
        if abs(acc[index][1] - tempACCYImuTopic) < 1e-8:

            pass
        else:
            assert False,"inconsistent Data --- time is equal but ACCY is not equal/ as expected"
        tempACCZImuTopic = float(ImuDataTopic[x]["linearAccelerationZ"])
        if abs(acc[index][2] - tempACCZImuTopic) < 1e-8:
            pass
        else:
            assert False,"inconsistent Data --- time is equal but ACCZ is not equal/ as expected"          

        ###################### CHECK Altitude ###################
        tempRPYROLLImuTopic = AttitudeIMUTransposed[x][0]
        if abs(rpy[index][0] - tempRPYROLLImuTopic) < 1e-10:
            print(x,"rpy_roll/quad_transposed_roll",rpy[index][0]," == ", tempRPYROLLImuTopic)
            pass
        else:
            assert False,"inconsistent Data --- time is equal but ACCX is not equal/ as expected"
        
        tempRPYPitchImuTopic = AttitudeIMUTransposed[x][1]
        if abs(rpy[index][1] - tempRPYPitchImuTopic) < 1e-10:
            pass
        else:
            assert False,"inconsistent Data --- time is equal but ACCY is not equal/ as expected"

        tempRPYPitchImuTopic = AttitudeIMUTransposed[x][2]
        if abs(rpy[index][2] - tempRPYPitchImuTopic) < 1e-10:
            pass
        else:
            assert False,"inconsistent Data --- time is equal but ACCZ is not equal/ as expected" 
    # PoseDataTopic = []
    # path = CWD + "PoseWithCovarianceStampedMsg.csv"
    # try:
    #     reader = csv.DictReader(open(path))    
    #     for row in reader:
    #         #print(row)
    #         PoseDataTopic.append(row)
    # except FileNotFoundError:
    #     assert True == False, 'Could not open the PoseWithCovarianceStampedMsg.csv File (log from topic)'
    # ### Transform the data of the 
    # timeStampPose = []
    # for x in range(len(PoseDataTopic)):
        
    #     timeStampPoseSec  = int(PoseDataTopic[x]["sec"])
    #     timeStampPoseNSecinSec = int(PoseDataTopic[x]["nsec"]) / 1000 / 1000 / 1000
    #     timeStampPose.append(timeStampPoseSec + timeStampPoseNSecinSec)




    # indexOfFirstMatch = 0
    # for x in range(len(iNATTimeInPOSIXTimePose)):
    #     if iNATTimeInPOSIXTimePose[x] ==  timeStampPose[0]:
    #         indexOfFirstMatch = x
    #         break

    # indexOfFirstCovarianceMatch = 0

    # for x in range(len(iNATTimeInPOSIXTimePoseCovariance)):
    #     if indexOfFirstCovarianceMatch != 0:
    #         break
    #     for y in range(len(timeStampPose)):
    #         if iNATTimeInPOSIXTimePoseCovariance[x] ==  timeStampPose[y]:
    #             indexOfFirstCovarianceMatch = x
    #             break    
    # frequenzOfiNATCovMsgs = 1 /  (iNATTimeInPOSIXTimePoseCovariance[1]-iNATTimeInPOSIXTimePoseCovariance[0])
    # frequenzOfTopicPoseCovMsgs = 1 / (timeStampPose[1]-timeStampPose[0])
    # ratioOfFrequencies = frequenzOfiNATCovMsgs / frequenzOfTopicPoseCovMsgs
    # ratioOfFrequenciesCov = round(ratioOfFrequencies)

    # frequenzOfiNATMsgs = 1 /  (iNATTimeInPOSIXTimePose[1]-iNATTimeInPOSIXTimePose[0])
    # ratioOfFrequencies = frequenzOfiNATMsgs / frequenzOfTopicPoseCovMsgs
    # ratioOfFrequencies = round(ratioOfFrequencies)


    # timediffOfOneMessage = timeStampPose[1]-timeStampPose[0]
    # count= 0
    # for i in range(len(PoseDataTopic)-1):
    #     if abs(timeStampPose[i] - timeStampPose[i+1]) > timediffOfOneMessage*1.1:
    #         count = count + 1
    #         print(count, "timediff high DIFF:",timeStampPose[i] - timeStampPose[i+1])

    # if count > len(PoseDataTopic)*0.05:
    #     assert False , ("more than 5% of the messages has been lost")
    # ### check the NAVSatFix Data:
    # count = 0
    # index = indexOfFirstMatch
    # for x in range(len(PoseDataTopic)):
    #     indexFailures = 0
    #     while not (abs(iNATTimeInPOSIXTimePose[index] - timeStampPose[x]) < 1e-5):
    #         index = index + 1
    #         indexFailures = indexFailures + 1
    #         if indexFailures > 20*ratioOfFrequencies:
    #             print("assert")
    #             assert False"at least two consecutive messages lost")
            
    #     tempPoseECEFx = float(PoseDataTopic[x]["poseX"])
    #     if abs(ecef[index][0] - tempPoseECEFx) < 1e-10:
    #         count = count + 1 
    #         print(count,ecef[index][0]," == ", tempPoseECEFx)
    #         pass
    #     else:
    #         assert False"inconsistent Data --- time is equal but ECEF X is not as expected")
    #     tempPoseECEFY = float(PoseDataTopic[x]["poseY"])
    #     if abs(ecef[index][1] - tempPoseECEFY) < 1e-10:
    #         #print(ecef[index][1]," == ", tempPoseECEFY)       
    #         pass
    #     else:
    #         assert False"inconsistent Data --- time is equal but ECEF Y is not as expected")
    #     tempPoseECEFZ = float(PoseDataTopic[x]["poseZ"])
    #     if abs(ecef[index][2] - tempPoseECEFZ) < 1e-10:
    #         #print(ecef[index][2]," == ", tempPoseECEFZ)
    #         pass
    #     else:
    #         assert False"inconsistent Data --- time is equal but ECEF Z is not as expected")
        








    #### POSE WITH COVARIANCE ##################
    # iNATTimeInPOSIXTimePose = []
    # for x in range(len(timeOfWeekSec2)):
    #     time = gps_timestamp({'timeOfWeek_sec': timeOfWeekSec2[x], 'timeOfWeek_usec': timeOfWeekUSec2[x], 'week': timeWeek2[x]},18)
    #     iNATTimeInPOSIXTimePose.append(time[0] + (time[1]/1000/1000/1000))

    # iNATTimeInPOSIXTimePoseCovariance = []
    # for x in range(len(timeOfWeekSecSTDDDEV)):
    #     time = gps_timestamp({'timeOfWeek_sec': timeOfWeekSecSTDDDEV[x], 'timeOfWeek_usec': timeOfWeekUSecSTDDDEV[x], 'week': timeWeekSTDDDEV[x]},18)
    #     iNATTimeInPOSIXTimePoseCovariance.append(time[0] + (time[1]/1000/1000/1000))


    # count = 0
    # index = indexOfFirstCovarianceMatch
    # if frequenzOfTopicPoseCovMsgs > 20:
    #     index UP is the ratio value which indicates how much messages should have the same values because the covariance message can just have max 20Hz
    #     indexUp = round(frequenzOfTopicPoseCovMsgs/20)
    #     topicFrequencieHigherThanCovariance = True
    #     if indexUp < 1:
    #         indexUp = 1
    # else:
    #     indexUp = 1 
    #     topicFrequencieHigherThanCovariance = False

    # firstMatchAndCovarianceMsg = False
    # skipCounter = 0
    # for x in range(len(PoseDataTopic)):
    #     indexFailures = 0
    #     while not (abs(iNATTimeInPOSIXTimePoseCovariance[index] - timeStampPose[x]) < 1e-5):
    #         firstMatchAndCovarianceMsg = True
    #         if (timeStampPose[x]- iNATTimeInPOSIXTimePoseCovariance[index]) < 0:
    #             print("timeDiff", timeStampPose[x], x, iNATTimeInPOSIXTimePoseCovariance[index], index, timeStampPose[x] -iNATTimeInPOSIXTimePoseCovariance[index])
    #             index = indexOfFirstCovarianceMatch
    #             FrequenzOfTopicHigherAndNoMatch = True
    #             break
    #         index = index + 1
    #         indexFailures = indexFailures + 1
    #         if indexFailures > 5*ratioOfFrequencies:
    #             print("assert")
    #             assert False"at least two consecutive messages lost")

    #     if FrequenzOfTopicHigherAndNoMatch == True:
    #         FrequenzOfTopicHigherAndNoMatch = False
    #         continue
    #     check for higher  
            
    #     if topicFrequencieHigherThanCovariance == True:
    #         if skipCounter < indexUp:
    #             tempPosecovarianceRPY = float(PoseDataTopic[x]["covarianceRoll"])
    #             if abs(covarianceRPY[index][0] - tempPosecovarianceRPY) > 1e-8:
    #                 print("covariance topic frequencie is higher than logmessage and the comparisson of the current topic data did not match ")
    #                 skipCounter = skipCounter + 1
    #                 continue
    #             else:
    #                 skipCounter = 0
    #         else:
    #             skipCounter = skipCounter + 1
                
    #     tempPosecovarianceRPY = float(PoseDataTopic[x]["covarianceRoll"])
    #     if abs(covarianceRPY[index][0] - tempPosecovarianceRPY) < 1e-8:
    #         print("covarianceRoll",count,covarianceRPY[index][0]," == ", tempPosecovarianceRPY)
    #         pass
    #     else:
    #         assert False"inconsistent Data --- time is equal but covarianceRPY Roll is not as expected")
    #     tempPosecovarianceRPY = float(PoseDataTopic[x]["covariancePitch"])
    #     if abs(covarianceRPY[index][1] - tempPosecovarianceRPY) < 1e-8:
    #         print(ecef[index][2]," == ", tempPoseECEFZ)
    #         pass
    #     else:
    #         assert False"inconsistent Data --- time is equal but covarianceRPY Pitch is not as expected")
    #     tempPosecovarianceRPY = float(PoseDataTopic[x]["covarianceYaw"])
    #     if abs(covarianceRPY[index][2] - tempPosecovarianceRPY) < 1e-8:
    #         print(ecef[index][2]," == ", tempPoseECEFZ)
    #         pass
    #     else:
    #         assert False"inconsistent Data --- time is equal but covarianceRPY Yaw is not as expected")  


    #     tempPosecovariance = float(PoseDataTopic[x]["covarianceX"])
    #     if abs(covariancePos[index][0] - tempPosecovariance) < 1e-8:
    #         print("covarianceX",count,covariancePos[index][0]," == ", tempPosecovariance)
    #         pass
    #     else:
    #         assert False"inconsistent Data --- time is equal but covarianceX is not as expected")
    #     tempPosecovariance = float(PoseDataTopic[x]["covarianceY"])
    #     if abs(covariancePos[index][1] - tempPosecovariance) < 1e-8:
    #         print(ecef[index][2]," == ", tempPoseECEFZ)
    #         pass
    #     else:
    #         assert False"inconsistent Data --- time is equal but covarianceY is not as expected")
    #     tempPosecovariance = float(PoseDataTopic[x]["covarianceZ"])
    #     if abs(covariancePos[index][2] - tempPosecovariance) < 1e-8:
    #         print(ecef[index][2]," == ", tempPoseECEFZ)
    #         pass
    #     else:
    #         assert False"inconsistent Data --- time is equal but covarianceZ is not as expected")       

    # covarianceRPY


def main():
    print(123)

    pass

if __name__ == "__main__":
    main()
