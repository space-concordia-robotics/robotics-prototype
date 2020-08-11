#! /usr/bin/env python3

import time
import rospy
import sys
import random
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from mcu_control.msg import ThermistorTemps, Voltage
from decimal import *


class VoltageMonitor(object):

    def __init__(self, pub, voltages):
        self._pub = pub
        self._voltages = voltages


class TemperatureMonitor(object):

    def __init__(self, pub, temps):
        self._pub = pub
        self._temps = temps


#Get parameter state and assign value
def get_parameter_state(parameterState, values, prevValue, noise, increment):
    try:
        param = rospy.get_param(parameterState)
    except:
        rospy.loginfo("Failed: Parameter state not acquired.")
        sys.exit()

    currentGetValue = 0
    if (param == "high"):
        currentGetValue = values[0][1]
    elif (param == "normal"):
        currentGetValue = values[1][1]
    elif (param == "low"):
        currentGetValue = values[2][1]
    else:
        rospy.loginfo("Failed: Invalid parameter entered")

    #Increment to the desired value with noise included
    remainder = prevValue - currentGetValue
    if ((abs(remainder) >= (noise + increment))):
        #If negative: increment, if positive: decrement
        if (remainder > 0):
            currentGetValue -= (increment + (noise * (random.randint(-1, 1))))
        else:
            currentGetValue += (increment + (noise * (random.randint(-1, 1))))

    return currentGetValue


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

    #Get previous state before changing parameter
    prevVoltageState = rospy.get_param("PDS_mock_voltage")
    prevTemp1State = rospy.get_param("PDS_mock_temp1")
    prevTemp2State = rospy.get_param("PDS_mock_temp2")
    prevTemp3State = rospy.get_param("PDS_mock_temp3")

    #Initialize variables for previous voltage and temp
    prevVoltageValue = None
    prevTempStates = []

    #Create list of temps to loop through when assigning new temps
    prevTempStates.append(("PDS_mock_temp1", prevTemp1State))
    prevTempStates.append(("PDS_mock_temp2", prevTemp2State))
    prevTempStates.append(("PDS_mock_temp3", prevTemp3State))

    #Create dict with all parameter states (easily append to for scalability)
    prevParameterStates = {
        "PDS_mock_voltage": prevVoltageState,
        "PDS_mock_temp1": prevTemp1State,
        "PDS_mock_temp2": prevTemp2State,
        "PDS_mock_temp3": prevTemp3State
    }

    #Create dict with all parameter values (easily append to for scalability)
    prevParameterValues = {
        "PDS_mock_voltage": None,
        "PDS_mock_temp1": None,
        "PDS_mock_temp2": None,
        "PDS_mock_temp3": None
    }

    #Acquire parameter settings WHILE ROS IS RUNNING
    while not rospy.is_shutdown():
        #-------------------------------------------------------------------
        #Set voltage value for particular parameter
        if (prevVoltageState == "high"):
            prevParameterValues["PDS_mock_voltage"] = voltages[0][1]
        elif (prevVoltageState == "normal"):
            prevParameterValues["PDS_mock_voltage"] = voltages[1][1]
        elif (prevVoltageState == "low"):
            prevParameterValues["PDS_mock_voltage"] = voltages[2][1]
        else:
            rospy.loginfo("Failed: Invalid parameter entered")

        #Set temperature values for particular parameters
        for x in range(len(prevTempStates)):
            if (prevTempStates[x][1] == "high"):
                prevParameterValues[prevTempStates[x][0]] = temps[0][1]
            elif (prevTempStates[x][1] == "normal"):
                prevParameterValues[prevTempStates[x][0]] = temps[1][1]
            elif (prevTempStates[x][1] == "low"):
                prevParameterValues[prevTempStates[x][0]] = temps[2][1]
            else:
                rospy.loginfo("Failed: Invalid parameter entered")
        #-------------------------------------------------------------------

        #Get new current voltage and thermistor temps
        #1) Parameter name
        #2) Parameter's previous value
        #3) Noise
        #4) Increment value
        currentVoltage = get_parameter_state(
            "PDS_mock_voltage",
            voltages,
            prevParameterValues.get("PDS_mock_voltage"),
            0.05,
            0.1,
        )
        currentTemp1 = get_parameter_state(
            "PDS_mock_temp1",
            temps,
            prevParameterValues.get("PDS_mock_temp1"),
            0.05,
            0.1,
        )
        currentTemp2 = get_parameter_state(
            "PDS_mock_temp2",
            temps,
            prevParameterValues.get("PDS_mock_temp1"),
            0.05,
            0.1,
        )
        currentTemp3 = get_parameter_state(
            "PDS_mock_temp3",
            temps,
            prevParameterValues.get("PDS_mock_temp1"),
            0.05,
            0.1,
        )

        #Change previous states for voltage and thermistor temps
        prevParameterStates["PDS_mock_voltage"] = rospy.get_param("PDS_mock_voltage")
        prevParameterStates["PDS_mock_temp1"] = rospy.get_param("PDS_mock_temp1")
        prevParameterStates["PDS_mock_temp2"] = rospy.get_param("PDS_mock_temp2")
        prevParameterStates["PDS_mock_temp3"] = rospy.get_param("PDS_mock_temp3")

        #Set data to voltage and thermistortemp types
        voltage.data = float(currentVoltage)
        thermistorTemps.therm1 = float(currentTemp1)
        thermistorTemps.therm2 = float(currentTemp2)
        thermistorTemps.therm3 = float(currentTemp3)

        #Change previous values for voltage and thermistor temps
        prevParameterValues["PDS_mock_voltage"] = currentVoltage
        prevParameterValues["PDS_mock_temp1"] = currentTemp1
        prevParameterValues["PDS_mock_temp2"] = currentTemp2
        prevParameterValues["PDS_mock_temp3"] = currentTemp3

        #Log new values when posted
        rospy.loginfo("PDS_mock_voltage : " + str(prevParameterValues.get("PDS_mock_voltage")))
        rospy.loginfo("PDS_mock_temp1 : " + str(prevParameterValues.get("PDS_mock_temp1")))
        rospy.loginfo("PDS_mock_temp2 : " + str(prevParameterValues.get("PDS_mock_temp2")))
        rospy.loginfo("PDS_mock_temp3 : " + str(prevParameterValues.get("PDS_mock_temp3")))

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
        #Initialize parameters for voltage and temperatures
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