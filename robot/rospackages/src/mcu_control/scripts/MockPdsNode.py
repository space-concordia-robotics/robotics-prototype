#! /usr/bin/env python3

import time
import rospy
import sys
import random
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from mcu_control.msg import ThermistorTemps, Voltage, Currents
from decimal import *


#Log all data during publishing (Added in publish_mock_data to see values in CLI)
def rospyPublishedValueLogger(prevParameterStates, prevParameterValues):
    #Log new values when posted
    rospy.loginfo("PDS_mock_voltage : " + str(prevParameterValues.get("PDS_mock_voltage")))
    rospy.loginfo("PDS_mock_temp1 : " + str(prevParameterValues.get("PDS_mock_temp1")))
    rospy.loginfo("PDS_mock_temp2 : " + str(prevParameterValues.get("PDS_mock_temp2")))
    rospy.loginfo("PDS_mock_temp3 : " + str(prevParameterValues.get("PDS_mock_temp3")))
    rospy.loginfo(
        "PDS_mock_wheel_current1 : " + str(prevParameterValues.get("PDS_mock_wheel_current1"))
    )
    rospy.loginfo(
        "PDS_mock_wheel_current2 : " + str(prevParameterValues.get("PDS_mock_wheel_current2"))
    )
    rospy.loginfo(
        "PDS_mock_wheel_current3 : " + str(prevParameterValues.get("PDS_mock_wheel_current3"))
    )
    rospy.loginfo(
        "PDS_mock_wheel_current4 : " + str(prevParameterValues.get("PDS_mock_wheel_current4"))
    )
    rospy.loginfo(
        "PDS_mock_wheel_current5 : " + str(prevParameterValues.get("PDS_mock_wheel_current5"))
    )
    rospy.loginfo(
        "PDS_mock_wheel_current6 : " + str(prevParameterValues.get("PDS_mock_wheel_current6"))
    )


#Get parameter state and assign value
#General function, needs not to be changed
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

    #Increment or decrement to the desired value with noise included
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


#Get and set parameters to be published
def publish_mock_data(voltages, temps, currents):
    #Initialize node
    node_name = "mock_pds_node"
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo("Initialized: {} node for mock PDS values".format(node_name))
    rate = rospy.Rate(10)

    #Initialize publishers (Voltage, ThermistorTemps, Currents)
    voltage = Voltage()
    thermistorTemps = ThermistorTemps()
    wheelCurrents = Currents()
    pubVoltage = rospy.Publisher('battery_voltage', Voltage, queue_size=10)
    pubTemp = rospy.Publisher('battery_temps', ThermistorTemps, queue_size=10)
    pubWheelCurrent = rospy.Publisher('wheel_motor_currents', Currents, queue_size=10)

    #Get previous states before changing parameters for each individual
    prevVoltageState = rospy.get_param("PDS_mock_voltage")
    prevTemp1State = rospy.get_param("PDS_mock_temp1")
    prevTemp2State = rospy.get_param("PDS_mock_temp2")
    prevTemp3State = rospy.get_param("PDS_mock_temp3")
    prevWheelCurrent1State = rospy.get_param("PDS_mock_wheel_current1")
    prevWheelCurrent2State = rospy.get_param("PDS_mock_wheel_current2")
    prevWheelCurrent3State = rospy.get_param("PDS_mock_wheel_current3")
    prevWheelCurrent4State = rospy.get_param("PDS_mock_wheel_current4")
    prevWheelCurrent5State = rospy.get_param("PDS_mock_wheel_current5")
    prevWheelCurrent6State = rospy.get_param("PDS_mock_wheel_current6")

    #Create list of temperatures to loop through when assigning new temperatures
    prevTempStates = []
    prevTempStates.append(("PDS_mock_temp1", prevTemp1State))
    prevTempStates.append(("PDS_mock_temp2", prevTemp2State))
    prevTempStates.append(("PDS_mock_temp3", prevTemp3State))

    #Create list of wheel currents to loop through when assigning new currents
    prevWheelCurrentStates = []
    prevWheelCurrentStates.append(("PDS_mock_wheel_current1", prevWheelCurrent1State))
    prevWheelCurrentStates.append(("PDS_mock_wheel_current2", prevWheelCurrent2State))
    prevWheelCurrentStates.append(("PDS_mock_wheel_current3", prevWheelCurrent3State))
    prevWheelCurrentStates.append(("PDS_mock_wheel_current4", prevWheelCurrent4State))
    prevWheelCurrentStates.append(("PDS_mock_wheel_current5", prevWheelCurrent5State))
    prevWheelCurrentStates.append(("PDS_mock_wheel_current6", prevWheelCurrent6State))

    #Create dict with all parameter states (easily append to for scalability)
    prevParameterStates = {
        "PDS_mock_voltage": prevVoltageState,
        "PDS_mock_temp1": prevTemp1State,
        "PDS_mock_temp2": prevTemp2State,
        "PDS_mock_temp3": prevTemp3State,
        "PDS_mock_wheel_current1": prevWheelCurrent1State,
        "PDS_mock_wheel_current2": prevWheelCurrent2State,
        "PDS_mock_wheel_current3": prevWheelCurrent3State,
        "PDS_mock_wheel_current4": prevWheelCurrent4State,
        "PDS_mock_wheel_current5": prevWheelCurrent5State,
        "PDS_mock_wheel_current6": prevWheelCurrent6State
    }

    #Create dict with all parameter values (easily append to for scalability)
    prevParameterValues = {
        "PDS_mock_voltage": None,
        "PDS_mock_temp1": None,
        "PDS_mock_temp2": None,
        "PDS_mock_temp3": None,
        "PDS_mock_wheel_current1": None,
        "PDS_mock_wheel_current2": None,
        "PDS_mock_wheel_current3": None,
        "PDS_mock_wheel_current4": None,
        "PDS_mock_wheel_current5": None,
        "PDS_mock_wheel_current6": None
    }

    #Set initial 1 current voltage
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

    #Set initial 6 current wheel currents
    for x in range(len(prevWheelCurrentStates)):
        if (prevWheelCurrentStates[x][1] == "high"):
            prevParameterValues[prevWheelCurrentStates[x][0]] = currents.get("high")
        elif (prevWheelCurrentStates[x][1] == "normal"):
            prevParameterValues[prevWheelCurrentStates[x][0]] = currents.get("normal")
        elif (prevWheelCurrentStates[x][1] == "low"):
            prevParameterValues[prevWheelCurrentStates[x][0]] = currents.get("low")
        else:
            rospy.loginfo("Failed: Invalid parameter entered")
    currentWheelCurrent1 = prevParameterValues.get("PDS_mock_wheel_current1")
    currentWheelCurrent2 = prevParameterValues.get("PDS_mock_wheel_current2")
    currentWheelCurrent3 = prevParameterValues.get("PDS_mock_wheel_current3")
    currentWheelCurrent4 = prevParameterValues.get("PDS_mock_wheel_current4")
    currentWheelCurrent5 = prevParameterValues.get("PDS_mock_wheel_current5")
    currentWheelCurrent6 = prevParameterValues.get("PDS_mock_wheel_current6")

    #Acquire and set parameters while launch file is active
    while not rospy.is_shutdown():
        #Get new current voltage and thermistor temps recursively
        #1) Parameter name
        #2) Parameter's previous value
        #3) Noise value
        #4) Increment value
        currentVoltage = get_parameter_state(
            "PDS_mock_voltage", voltages, currentVoltage, 0.05, 0.2
        )
        currentTemp1 = get_parameter_state("PDS_mock_temp1", temps, currentTemp1, 0.05, 0.2)
        currentTemp2 = get_parameter_state("PDS_mock_temp2", temps, currentTemp2, 0.05, 0.2)
        currentTemp3 = get_parameter_state("PDS_mock_temp3", temps, currentTemp3, 0.05, 0.2)
        currentWheelCurrent1 = get_parameter_state(
            "PDS_mock_wheel_current1", currents, currentWheelCurrent1, 0.05, 0.2
        )
        currentWheelCurrent2 = get_parameter_state(
            "PDS_mock_wheel_current1", currents, currentWheelCurrent2, 0.05, 0.2
        )
        currentWheelCurrent3 = get_parameter_state(
            "PDS_mock_wheel_current1", currents, currentWheelCurrent3, 0.05, 0.2
        )
        currentWheelCurrent4 = get_parameter_state(
            "PDS_mock_wheel_current1", currents, currentWheelCurrent4, 0.05, 0.2
        )
        currentWheelCurrent5 = get_parameter_state(
            "PDS_mock_wheel_current1", currents, currentWheelCurrent5, 0.05, 0.2
        )
        currentWheelCurrent6 = get_parameter_state(
            "PDS_mock_wheel_current1", currents, currentWheelCurrent6, 0.05, 0.2
        )

        #Change previous states for evaluated parameters
        prevParameterStates["PDS_mock_voltage"] = rospy.get_param("PDS_mock_voltage")
        prevParameterStates["PDS_mock_temp1"] = rospy.get_param("PDS_mock_temp1")
        prevParameterStates["PDS_mock_temp2"] = rospy.get_param("PDS_mock_temp2")
        prevParameterStates["PDS_mock_temp3"] = rospy.get_param("PDS_mock_temp3")
        prevParameterStates["PDS_mock_wheel_current1"] = rospy.get_param("PDS_mock_wheel_current1")
        prevParameterStates["PDS_mock_wheel_current2"] = rospy.get_param("PDS_mock_wheel_current2")
        prevParameterStates["PDS_mock_wheel_current3"] = rospy.get_param("PDS_mock_wheel_current3")
        prevParameterStates["PDS_mock_wheel_current4"] = rospy.get_param("PDS_mock_wheel_current4")
        prevParameterStates["PDS_mock_wheel_current5"] = rospy.get_param("PDS_mock_wheel_current5")
        prevParameterStates["PDS_mock_wheel_current6"] = rospy.get_param("PDS_mock_wheel_current6")

        #Change previous values for voltage and thermistor temps
        prevParameterValues["PDS_mock_voltage"] = currentVoltage
        prevParameterValues["PDS_mock_temp1"] = currentTemp1
        prevParameterValues["PDS_mock_temp2"] = currentTemp2
        prevParameterValues["PDS_mock_temp3"] = currentTemp3
        prevParameterValues["PDS_mock_wheel_current1"] = currentWheelCurrent1
        prevParameterValues["PDS_mock_wheel_current2"] = currentWheelCurrent2
        prevParameterValues["PDS_mock_wheel_current3"] = currentWheelCurrent3
        prevParameterValues["PDS_mock_wheel_current4"] = currentWheelCurrent4
        prevParameterValues["PDS_mock_wheel_current5"] = currentWheelCurrent5
        prevParameterValues["PDS_mock_wheel_current6"] = currentWheelCurrent6

        #Add to see published values in CLI, or comment out if not needed
        rospyPublishedValueLogger(prevParameterStates, prevParameterValues)

        #Set valus to objects which will get published
        voltage.data = float(currentVoltage)
        thermistorTemps.therm1 = float(currentTemp1)
        thermistorTemps.therm2 = float(currentTemp2)
        thermistorTemps.therm3 = float(currentTemp3)
        wheelCurrents.effort.append(float(currentWheelCurrent1))
        wheelCurrents.effort.append(float(currentWheelCurrent2))
        wheelCurrents.effort.append(float(currentWheelCurrent3))
        wheelCurrents.effort.append(float(currentWheelCurrent4))
        wheelCurrents.effort.append(float(currentWheelCurrent5))
        wheelCurrents.effort.append(float(currentWheelCurrent6))

        #Publish values to topics (Voltage, ThermistorTemps and Currents)
        pubVoltage.publish(voltage)
        pubTemp.publish(thermistorTemps)
        pubWheelCurrent.publish(wheelCurrents)

        #Add delay before each iteration
        rate.sleep()


if __name__ == '__main__':

    #Set voltages high, normal and low to be parametrized
    voltages = {"high": 20, "normal": 15, "low": 10}

    #Set temperatures high, normal and low to be parametrized
    temps = {"high": 100, "normal": 50, "low": -20}

    #Set temperatures high, normal and low to be parametrized
    currents = {"high": 10, "normal": 5, "low": 0}

    #Publish all new parameter settings set by user
    try:
        publish_mock_data(voltages, temps, currents)
    except rospy.ROSInterruptException:
        pass