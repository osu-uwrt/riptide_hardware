#!/usr/bin/env python

from lib import ok
import rospy
import actionlib
from riptide_msgs.msg import AcousticsCommand, Imu, ResetControls
import riptide_controllers.msg
from std_msgs.msg import Float32
import math
import numpy as np
import cv2
import os

fpga = None

def unsignedToSigned(val):
    if (val & (1 << 23)) != 0:
        val = val | ~((1 << 24) - 1)

    return val

def commandCB(command):
    if not fpga.IsOpen():
        rospy.logerr("Housing not connected")
        return

    # Reset FIFOs
    fpga.SetWireInValue(0x00, 0x0004)
    fpga.UpdateWireIns()
    fpga.SetWireInValue(0x00, 0x0000)
    fpga.UpdateWireIns()

    rospy.loginfo("Collecting")
    # Collect Data
    fpga.SetWireInValue(0x00, 0x0002)
    fpga.UpdateWireIns()

    length = 2000

    rospy.sleep(rospy.Duration(length / 1000.0))

    rospy.loginfo("Done")
    NumOfCollections = length * 512
    data = bytearray(NumOfCollections * 16)

    PFdata = [0] * NumOfCollections
    PAdata = [0] * NumOfCollections
    SFdata = [0] * NumOfCollections
    SAdata = [0] * NumOfCollections

    # Stop recording and begin reading
    fpga.SetWireInValue(0x00, 0x0001)
    fpga.UpdateWireIns()
    err = fpga.ReadFromBlockPipeOut(0xa0, 512, data)
    fpga.SetWireInValue(0x00, 0x0000)
    fpga.UpdateWireIns()

    for i in range(NumOfCollections):
        SAdata[i] = (data[i*16 + 2] << 16) + (data[i*16 + 1] << 8) + data[i*16]
        SFdata[i] = (data[i*16 + 5] << 16) + (data[i*16 + 4] << 8) + data[i*16 + 3]
        PAdata[i] = (data[i*16 + 8] << 16) + (data[i*16 + 7] << 8) + data[i*16 + 6]
        PFdata[i] = (data[i*16 + 11] << 16) + (data[i*16 + 10] << 8) + data[i*16 + 9]

    (altitude, azimuth) = calculate(PFdata, PAdata, SFdata, SAdata, 4000)
    rospy.loginfo((altitude, azimuth))
    turn(azimuth - 90)


    if err < 0:
        rospy.logerr("Reading Failed")
        return

    file = open(command.fileName, "wb")
    file.write(data)

def restrictAngle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle
 
# def filter(data, frequency, ntaps=51):
#     data = [data[0]] * 100 + data
#     nyq = 0.5 * SAMPLE_FREQ
#     low = (frequency-100) / nyq
#     high = (frequency+100) / nyq
#     taps = firwin(ntaps, [low, high], pass_zero=False)
#     y = lfilter(taps, 1, data)
#     return np.float32(y[100:])

def getTime(data, frequency):
    amps = [0] * (len(data) // SAMPLE_LENGTH)
    index = int(SAMPLE_LENGTH * frequency / SAMPLE_FREQ)
    for i in range(1, len(data) // SAMPLE_LENGTH - 1):
        amps[i] = np.abs(np.fft.fft(data[SAMPLE_LENGTH*i:SAMPLE_LENGTH*(i+1)])[index])

    scores = [0] * (len(data) // SAMPLE_LENGTH)
    for i in range(7, len(data) // SAMPLE_LENGTH - 1):
        scores[i] = amps[i] * amps[i] / ((amps[i-1]+ amps[i - 2]+amps[i - 3]+amps[i - 4] + amps[i - 5] + amps[i - 6]) /6)

    pingApprox = scores.index(max(scores)) * SAMPLE_LENGTH

    fineAmps = [0] * (PING_DURATION * 7 // 4)
    for i in range(PING_DURATION * 7 // 4):
        sample = data[pingApprox - PING_DURATION*5//4 + i : pingApprox - PING_DURATION*5//4 + i + SAMPLE_LENGTH]
        fineAmps[i] = np.abs(np.fft.fft(sample)[index])

    maxVal = 0
    for i in range(SAMPLE_LENGTH // 2, PING_DURATION*7//4 - SAMPLE_LENGTH // 2):
        val = fineAmps[i + SAMPLE_LENGTH / 2] - 2 * fineAmps[i] + fineAmps[i - SAMPLE_LENGTH / 2]
        if val > maxVal:
            maxVal = val
            time = pingApprox + i - PING_DURATION*5//4 + SAMPLE_LENGTH
    return time
	
PING_DURATION = 2048
SAMPLE_LENGTH = 512
SAMPLE_FREQ = 512000
HYDROPHONE_SPACING = 0.09144 #meter
# 343 in air, 1482 in water
SPEED_OF_SOUND = 343 #m/s 
MAX_OFFSET = (int)(2 * HYDROPHONE_SPACING / SPEED_OF_SOUND * SAMPLE_FREQ)
rToD = 180 / math.pi

def calculate(PFdata, PAdata, SFdata, SAdata, frequency):
    period = SAMPLE_FREQ / frequency

    index = int(SAMPLE_LENGTH * frequency / SAMPLE_FREQ)
    
    PFTime = getTime(PFdata, frequency)
    PATime = getTime(PAdata, frequency)
    SFTime = getTime(SFdata, frequency)
    SATime = getTime(SAdata, frequency)

    PFphase = np.angle(np.fft.fft(PFdata[PFTime:PFTime + SAMPLE_LENGTH])[index]) / math.pi * 180
    PAphase = np.angle(np.fft.fft(PAdata[PATime:PATime + SAMPLE_LENGTH])[index]) / math.pi * 180
    SFphase = np.angle(np.fft.fft(SFdata[SFTime:SFTime + SAMPLE_LENGTH])[index]) / math.pi * 180
    SAphase = np.angle(np.fft.fft(SAdata[SATime:SATime + SAMPLE_LENGTH])[index]) / math.pi * 180

    PATime += restrictAngle(PFphase - PAphase) / 360 * period
    SFTime += restrictAngle(PFphase - SFphase) / 360 * period
    SATime += restrictAngle(PFphase - SAphase) / 360 * period

    a = PATime / SAMPLE_FREQ
    b = 1.0 * PFTime / SAMPLE_FREQ
    c = SATime / SAMPLE_FREQ
    d = SFTime / SAMPLE_FREQ
    minVal = min(min(a, b), min(c, d))
    a = a - minVal
    b = b - minVal
    c = c - minVal
    d = d - minVal
    s = HYDROPHONE_SPACING / SPEED_OF_SOUND

    cross1 = a - d
    cross2 = b - c
    azimuth = math.atan2(cross2, cross1) * 180 / math.pi + 45
    azimuth = (azimuth + 180 + 360) % 360 - 180
    sqrt = math.sqrt((cross1 * cross1 + cross2 * cross2) / 2)
    if sqrt > s:
        sqrt = s
    altitude = math.acos(sqrt / s) * 180 / math.pi

    return altitude, azimuth

def initFPGA():
	global fpga

	rospy.loginfo("Attempting to connect to housing...")
	fpga = ok.okCFrontPanel()
	err = fpga.OpenBySerial("")
	if ok.okCFrontPanel.NoError != err:
		rospy.logerr("Could not connect to housing")
		return False

	fpga.LoadDefaultPLLConfiguration()
	fpga.SetTimeout(100)

	rospy.loginfo("Configuring")
	error = fpga.ConfigureFPGA(os.path.expanduser("~/osu-uwrt/riptide_software/src/puddles_hardware/assets/acoustics/output_file.rbf"))
	if error != ok.okCFrontPanel.NoError:
		rospy.logerr("Failed to configure housing: " + ok.okCFrontPanel.GetErrorString(error))
		return False
	rospy.loginfo("Configured successfully!")
	return True


def turn(angle):
    imu = rospy.wait_for_message("/state/imu", Imu)
    resetPub.publish(False)
    client = actionlib.SimpleActionClient(
        "go_to_depth", riptide_controllers.msg.GoToDepthAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.GoToDepthGoal(1))
    client.wait_for_result()


    client = actionlib.SimpleActionClient(
        "go_to_yaw", riptide_controllers.msg.GoToYawAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.GoToYawGoal(restrictAngle(imu.rpy_deg.z + angle)))
    client.wait_for_result()

    rospy.sleep(1)
    resetPub.publish(True)


if __name__ == '__main__':
    global resetPub
    rospy.init_node("acoustics")
    #if initFPGA():
    resetPub = rospy.Publisher("/controls/reset", ResetControls, queue_size=1)
    rospy.Subscriber("command/acoustics", AcousticsCommand, commandCB)
    rospy.Subscriber("command/acousticsAngle", Float32, turnCB)
    rospy.spin()
