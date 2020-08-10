#! /usr/bin/env python3

import time
import rospy
import sys
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from mcu_control.msg import ThermistorTemps, Voltage
from decimal import *


class VoltageMonitor(object):

    def __init__(self, pub, voltages):
        self._pub = pub
        self._voltages = voltages

    # def callback(self, msg):


class TemperatureMonitor(object):

    def __init__(self, pub, temps):
        self._pub = pub
        self._temps = temps


#TODO: Use only ONE get parameter method to get ALL values (scalable)
#TODO: Take parameterState and values[] instead of voltages[] and temps[]
def get_voltage_parameter(parameterState, voltages):
    try:
        param = rospy.get_param(parameterState)
    except:
        rospy.loginfo("Failed: Voltage parameter not acquired.")
        sys.exit()

    currentGetVoltage = 0
    if (param == "high"):
        target = voltages[0][1]
        currentGetVoltage = voltages[0][1]
    elif (param == "normal"):
        target = voltages[1][1]
        currentGetVoltage = voltages[1][1]
    elif (param == "low"):
        target = voltages[2][1]
        currentGetVoltage = voltages[2][1]
    else:
        rospy.loginfo("Failed: Invalid parameter entered")

    return currentGetVoltage


def get_temp_parameter(parameterState, temps):
    try:
        param = rospy.get_param(parameterState)
    except:
        rospy.loginfo("Failed: Temperature parameter not acquired.")
        sys.exit()

    currentGetTemp = 0
    if (param == "high"):
        currentGetTemp = temps[0][1]
    elif (param == "normal"):
        currentGetTemp = temps[1][1]
    elif (param == "low"):
        currentGetTemp = temps[2][1]
    else:
        rospy.loginfo("Failed: Invalid parameter entered")

    return currentGetTemp


#def publish_mock_data(voltages, temps):
def publish_mock_data(voltages, temps):

    #Initialize node
    node_name = "mock_pds_node"
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo("Initialized: {} node for mock PDS values".format(node_name))
    rate = rospy.Rate(10)
    voltage = Voltage()
    thermistorTemps = ThermistorTemps()
    pubVoltage = rospy.Publisher('battery_voltage', Voltage, queue_size=10)
    pubTemp = rospy.Publisher('battery_temps', ThermistorTemps, queue_size=10)

    # pubVoltageTest = rospy.Publisher('battery_voltage', Float32, queue_size=10)
    # pubTempTest = rospy.Publisher('battery_temps', Point, queue_size=10)

    #Get previous state before changing parameter
    prevVoltageState = rospy.get_param("PDS_mock_voltage")
    prevTemp1State = rospy.get_param("PDS_mock_temp1")
    prevTemp2State = rospy.get_param("PDS_mock_temp2")
    prevTemp3State = rospy.get_param("PDS_mock_temp3")

    #Initialize variables for previous voltage and temp
    prevVoltageValue = None
    prevTempStates = []
    prevTempValues = []
    prevTempStates.append(("PDS_mock_temp1", prevTemp1State))
    prevTempStates.append(("PDS_mock_temp2", prevTemp2State))
    prevTempStates.append(("PDS_mock_temp3", prevTemp3State))

    #Set voltage value for particular parameter
    if (prevVoltageState == "high"):
        prevVoltageValue = ("PDS_mock_voltage", voltages[0][1])
    elif (prevVoltageState == "normal"):
        prevVoltageValue = ("PDS_mock_voltage", voltages[1][1])
    elif (prevVoltageState == "low"):
        prevVoltageValue = ("PDS_mock_voltage", voltages[2][1])
    else:
        rospy.loginfo("Failed: Invalid parameter entered")

    #Set temperature values for particular parameters
    for x in range(len(prevTempStates)):
        if (prevTempStates[x][1] == "high"):
            prevTempValues.append((prevTempStates[x][0], temps[0][1]))
        elif (prevTempStates[x][1] == "normal"):
            prevTempValues.append((prevTempStates[x][0], temps[1][1]))
        elif (prevTempStates[x][1] == "low"):
            prevTempValues.append((prevTempStates[x][0], temps[2][1]))
        else:
            rospy.loginfo("Failed: Invalid parameter entered")

    rospy.loginfo("Thermistor temp values size: " + str(len(prevTempStates)))

    rospy.loginfo("PDS_mock_voltage : " + str(prevVoltageValue))
    rospy.loginfo("PDS_mock_temp1 : " + str(prevTempValues[0]))
    rospy.loginfo("PDS_mock_temp2 : " + str(prevTempValues[1]))
    rospy.loginfo("PDS_mock_temp3 : " + str(prevTempValues[2]))

    #Acquire parameter settings
    while not rospy.is_shutdown():
        currentVoltage = get_voltage_parameter("PDS_mock_voltage", voltages)
        currentTemp1 = get_temp_parameter("PDS_mock_temp1", temps)
        currentTemp2 = get_temp_parameter("PDS_mock_temp2", temps)
        currentTemp3 = get_temp_parameter("PDS_mock_temp3", temps)

        #Set data to voltage and thermistortemp types
        voltage.data = float(currentVoltage)
        thermistorTemps.therm1 = float(currentTemp1)
        thermistorTemps.therm2 = float(currentTemp2)
        thermistorTemps.therm3 = float(currentTemp3)

        #Publish voltage and thermistortemps
        pubVoltage.publish(voltage)
        pubTemp.publish(thermistorTemps)

        rate.sleep()


if __name__ == '__main__':

    #Set voltages high, normal and low to be parametrized
    voltages = []
    voltages.append(("high", 20))
    voltages.append(("normal", 15))
    voltages.append(("low", 10))

    #Set temperatures high, normal and low to be parametrized
    temps = []
    temps.append(("high", 100))
    temps.append(("normal", 50))
    temps.append(("low", -20))

    try:
        #Initialize parameters
        rospy.set_param("PDS_mock_voltage", voltages[1][0])
        rospy.set_param("PDS_mock_temp1", temps[1][0])
        rospy.set_param("PDS_mock_temp2", temps[1][0])
        rospy.set_param("PDS_mock_temp3", temps[1][0])
        rospy.loginfo("Success: Values published")
    except:
        rospy.loginfo("Fail: Values not published")

    rospy.loginfo(rospy.get_param("PDS_mock_voltage"))
    rospy.loginfo(rospy.get_param("PDS_mock_temp1"))
    rospy.loginfo(rospy.get_param("PDS_mock_temp2"))
    rospy.loginfo(rospy.get_param("PDS_mock_temp3"))

    #Publish all new parameter settings set by user
    try:
        publish_mock_data(voltages, temps)
    except rospy.ROSInterruptException:
        pass