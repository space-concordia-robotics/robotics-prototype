#! /usr/bin/env python3

import time
import rospy
import sys
import random
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from mcu_control.msg import ThermistorTemps, Voltage, Currents
from decimal import *

def get_parameter_value(parameter, values, valueBeforeIncrement, noise, increment):
    """Get parameter state and assign value
    General function, needs not to be changed"""

    # Acquire the value associated to the state if it exists
    currentGetValue = get_states_values(parameter, values)
    #Increment or decrement to the desired value with noise included
    remainder = currentGetValue - valueBeforeIncrement

    #If negative: increment, if positive: decrement
    if ((abs(remainder) >= (noise + increment))):
        if (remainder > 0):
            valueBeforeNoise = valueBeforeIncrement + increment #Increment the value presented on the gui
            valueAfterNoise = round(valueBeforeNoise + (noise * (random.uniform(-1, 1))), 2) #Add noise to the incremented value
        else:
            valueBeforeNoise = valueBeforeIncrement - increment #Decrement the value presented on the gui
            valueAfterNoise = round(valueBeforeNoise + (noise * (random.uniform(-1, 1))), 2) #Add noise to the incremented value
    else:
        valueAfterNoise = round(currentGetValue + (noise * (random.uniform(-1, 1))), 2) #Add noise to the value if it is not incrementing or decrementing

    return valueAfterNoise

def get_states_values(parameters, values):
    """Initialize parameters with the values according to the states (rise, stable and fall)"""

    #Check if parameters is a string (single parameter) or an array (multiple parameters)
    if isinstance(parameters, str):
        paramState = get_param_exist(parameters)
        newValue = None
        if (paramState == "rise"):
            newValue = values.get("rise")
        elif (paramState == "stable"):
            newValue = values.get("stable")
        elif (paramState == "fall"):
            newValue = values.get("fall")
        else:
            rospy.logwarn("Failed: Invalid parameter entered")
            #If noise or riseFallRate were never initialized in the launch file, set them to stable
            if ((parameters == "PDS_mock_rise_fall_rate" or parameters == "PDS_mock_global_noise") and paramState == None):
                    newValue = values.get("stable")
                    rospy.logwarn("Noise or rate set to STABLE")
        return newValue

    #If parameters is an array, return the value associated for each parameter state
    else:
        outputArray = []
        for x in range(len(parameters)):
            paramState = get_param_exist(parameters[x])
            if (paramState == "rise"):
                outputArray.append(values.get("rise"))
            elif (paramState == "stable"):
                outputArray.append(values.get("stable"))
            elif (paramState == "fall"):
                outputArray.append(values.get("fall"))
            else:
                rospy.logwarn("Failed: Invalid parameter entered for {}".format(parameters[x]))
        return outputArray

def get_param_exist(parameter):
    """Get parameter state if parameter exists and sets to None if it does not exist"""

    try:
        parameterState = rospy.get_param(parameter)
    except:
        rospy.logwarn("Parameter state None assigned to {}.".format(parameter))
        parameterState = None
    return parameterState

def publish_mock_data(voltages, temps, currents, noiseValues, riseFallRates):
    """Get and set parameters to be published"""

    #Initialize node
    node_name = "mock_pds_node"
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo("Initialized: {} node for mock PDS values".format(node_name))
    rospyRate = 10
    rate = rospy.Rate(rospyRate)

    #Initialize publishers (Voltage, ThermistorTemps, Currents)
    voltage = Voltage()
    thermistorTemps = ThermistorTemps()
    wheelCurrents = Currents()
    pubVoltage = rospy.Publisher('battery_voltage', Voltage, queue_size=10)
    pubTemp = rospy.Publisher('battery_temps', ThermistorTemps, queue_size=10)
    pubWheelCurrent = rospy.Publisher('wheel_motor_currents', Currents, queue_size=10)

    #Initialize 1 starting rise fall rate
    currentRate = get_states_values("PDS_mock_rise_fall_rate", riseFallRates)
    rospy.logwarn("currentRate: {}".format(currentRate))

    #Initialize 1 starting noise
    currentNoise = get_states_values("PDS_mock_global_noise", noiseValues)
    rospy.logwarn("currentNoise: {}".format(currentNoise))

    #Initialize 1 starting voltage
    currentVoltage = get_states_values("PDS_mock_global_noise", voltages)
    rospy.logwarn("currentVoltage: {}".format(currentVoltage))

    #Initialize 3 starting temperatures
    currentTemps = []
    parameterTemps = ["PDS_mock_temp1", "PDS_mock_temp2", "PDS_mock_temp3"]
    currentTemps = get_states_values(parameterTemps, temps)
    rospy.logwarn("currentTemps: {}".format(currentTemps))

    #Initialize 6 starting wheel currents
    currentWheelCurrents = []
    parameterCurrents = ["PDS_mock_wheel1_current", "PDS_mock_wheel2_current", "PDS_mock_wheel3_current", "PDS_mock_wheel4_current", "PDS_mock_wheel5_current", "PDS_mock_wheel6_current"]
    currentWheelCurrents = get_states_values(parameterCurrents, currents)
    rospy.logwarn("currentWheelCurrents: {}".format(currentWheelCurrents))

    # Acquire parameter and set parameter states while launch file is active
    while not rospy.is_shutdown():
        """Get new current, voltage, and thermistor temps recursively
        1) Parameter name
        2) Parameter's previous value
        3) Noise value
        4) Increment value"""

        #Get 1 rate immediately while ros is running
        currentRate = get_states_values("PDS_mock_rise_fall_rate", riseFallRates)

        #Get 1 noise immediately while ros is running
        currentNoise = get_states_values("PDS_mock_global_noise", noiseValues)

        #Get 1 voltage incrementally while ros is running
        if (currentVoltage is not None):
            currentVoltage = get_parameter_value("PDS_mock_voltage", voltages, currentVoltage, currentNoise, currentRate/rospyRate)

        #Get 3 temperatures incrementally while ros is running
        for x in range(len(currentTemps)):
            if (currentTemps[x] is not None):
                currentTemps[x] = get_parameter_value(parameterTemps[x], temps, currentTemps[x], currentNoise, currentRate/rospyRate)

        #Get 6 wheel currents incrementally while ros is running
        for x in range(len(currentWheelCurrents)):
            if (currentWheelCurrents[x] is not None):
                currentWheelCurrents[x] = get_parameter_value(parameterCurrents[x], currents, currentWheelCurrents[x], currentNoise, currentRate/rospyRate)

        #Set values to objects which will get published
        #Set voltage 1
        if (currentVoltage is not None):
            voltage.data = float(currentVoltage)
        #Set temperatures 1-3
        if (currentTemps[0] is not None):
            thermistorTemps.therm1 = float(currentTemps[0])
        if (currentTemps[1] is not None):
            thermistorTemps.therm2 = float(currentTemps[1])
        if (currentTemps[2] is not None):
            thermistorTemps.therm3 = float(currentTemps[2])
        #Set wheel currents 1-6
        for x in range(len(currentWheelCurrents)):
            if(currentWheelCurrents[x] is not None):
                wheelCurrents.effort.append(float(currentWheelCurrents[x]))

        #Publish values to topics (Voltage, ThermistorTemps and Currents)
        pubVoltage.publish(voltage)
        pubTemp.publish(thermistorTemps)
        pubWheelCurrent.publish(wheelCurrents)

        #Add delay before each iteration of loop which runs rospy
        rate.sleep()


if __name__ == '__main__':
    """Calls publisher function and passes parameter settings
    Change the values for the parameter states (rise, stable and fall) here (i.e. voltages, temps and currents)
    No other places require to be modified to change the state values"""

    #Set voltages rise, stable and fall to be parametrized
    voltages = {"rise": 20, "stable": 15, "fall": 10}

    #Set temperatures rise, stable and fall to be parametrized
    temps = {"rise": 100, "stable": 50, "fall": -20}

    #Set temperatures rise, stable and fall to be parametrized
    currents = {"rise": 1, "stable": 0.3, "fall": 0}

    #Set temperatures rise, stable and fall to be parametrized
    noiseValues = {"rise": 0.1, "stable": 0.05, "fall": 0.01}

    #Set rise fall rates rise, stable and fall to be parametrized
    riseFallRates = {"rise": 0.1, "stable": 0.04, "fall": 0.01}

    #Publish all new parameter settings set by user
    try:
        publish_mock_data(voltages, temps, currents, noiseValues, riseFallRates)
    except rospy.ROSInterruptException:
        pass
