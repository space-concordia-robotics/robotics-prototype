#! /usr/bin/env python3

import time
import rospy
import sys
import random
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from mcu_control.msg import ThermistorTemps, Voltage
from decimal import *


#Get parameter state and assign value
def get_parameter_state(parameterState, values, prevValue, noise, increment):
    try:
        param = rospy.get_param(parameterState)
    except:
        rospy.loginfo("Failed: Parameter state not acquired.")
        sys.exit()

    currentGetValue = 0
    if (param == "high"):
        currentGetValue = values.get("high")
    elif (param == "normal"):
        currentGetValue = values.get("normal")
    elif (param == "low"):
        currentGetValue = values.get("low")
    else:
        rospy.loginfo("Failed: Invalid parameter entered")

    #Increment to the desired value with noise included
    remainder = currentGetValue - prevValue
    if ((abs(remainder) >= (noise + increment))):
        #If negative: increment, if positive: decrement
        if (remainder > 0):
            prevValue = round(prevValue + (noise * (random.uniform(-1, 1))) + increment, 2)
        else:
            prevValue = round(prevValue + (noise * (random.uniform(-1, 1))) - increment, 2)
    else:
        prevValue = round(currentGetValue + (noise * (random.uniform(-1, 1))), 2)
    return prevValue


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

    # #Initialize starting state on program startup
    # prevVoltageState = "normal"
    # prevTemp1State = "normal"
    # prevTemp2State = "normal"
    # prevTemp3State = "normal"

    # #Set data to voltage and thermistortemp types
    # voltage.data = float(currentVoltage)
    # thermistorTemps.therm1 = float(currentTemp1)
    # thermistorTemps.therm2 = float(currentTemp2)
    # thermistorTemps.therm3 = float(currentTemp3)

    # #Publish voltage and thermistortemps
    # pubVoltage.publish(voltage)
    # pubTemp.publish(thermistorTemps)

    #Initialize variables for previous voltage and temp
    prevTempStates = []

    #Create list of temperatures to loop through when assigning new temperatures
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

    #Set initial current voltage
    if (prevVoltageState == "high"):
        prevParameterValues["PDS_mock_voltage"] = voltages.get("high")
    elif (prevVoltageState == "normal"):
        prevParameterValues["PDS_mock_voltage"] = voltages.get("normal")
    elif (prevVoltageState == "low"):
        prevParameterValues["PDS_mock_voltage"] = voltages.get("low")
    else:
        rospy.loginfo("Failed: Invalid parameter entered")
    currentVoltage = prevParameterValues.get("PDS_mock_voltage")

    #Set initial 3 current temperatures
    for x in range(len(prevTempStates)):
        if (prevTempStates[x][1] == "high"):
            prevParameterValues[prevTempStates[x][0]] = temps.get("high")
        elif (prevTempStates[x][1] == "normal"):
            prevParameterValues[prevTempStates[x][0]] = temps.get("normal")
        elif (prevTempStates[x][1] == "low"):
            prevParameterValues[prevTempStates[x][0]] = temps.get("low")
        else:
            rospy.loginfo("Failed: Invalid parameter entered")
    currentTemp1 = prevParameterValues.get("PDS_mock_temp1")
    currentTemp2 = prevParameterValues.get("PDS_mock_temp2")
    currentTemp3 = prevParameterValues.get("PDS_mock_temp3")

    #Acquire parameter settings WHILE ROS IS RUNNING
    while not rospy.is_shutdown():
        #Get new current voltage and thermistor temps recursively
        #1) Parameter name
        #2) Parameter's previous value
        #3) Noise
        #4) Increment value

        currentVoltage = get_parameter_state(
            "PDS_mock_voltage",
            voltages,
            currentVoltage,
            0.05,
            0.2,
        )
        currentTemp1 = get_parameter_state(
            "PDS_mock_temp1",
            temps,
            currentTemp1,
            0.05,
            0.2,
        )
        currentTemp2 = get_parameter_state(
            "PDS_mock_temp2",
            temps,
            currentTemp2,
            0.05,
            0.2,
        )
        currentTemp3 = get_parameter_state(
            "PDS_mock_temp3",
            temps,
            currentTemp3,
            0.05,
            0.2,
        )

        #Change previous states for voltage and thermistor temps
        prevParameterStates["PDS_mock_voltage"] = rospy.get_param("PDS_mock_voltage")
        prevParameterStates["PDS_mock_temp1"] = rospy.get_param("PDS_mock_temp1")
        prevParameterStates["PDS_mock_temp2"] = rospy.get_param("PDS_mock_temp2")
        prevParameterStates["PDS_mock_temp3"] = rospy.get_param("PDS_mock_temp3")

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
    voltages = {"high": 20, "normal": 15, "low": 10}

    #Set temperatures high, normal and low to be parametrized
    temps = {"high": 100, "normal": 50, "low": -20}

    rospy.set_param('PDS_mock_voltage', 'normal')
    rospy.set_param('PDS_mock_temp1', 'normal')
    rospy.set_param('PDS_mock_temp2', 'normal')
    rospy.set_param('PDS_mock_temp3', 'normal')

    #Publish all new parameter settings set by user
    try:
        publish_mock_data(voltages, temps)
    except rospy.ROSInterruptException:
        pass