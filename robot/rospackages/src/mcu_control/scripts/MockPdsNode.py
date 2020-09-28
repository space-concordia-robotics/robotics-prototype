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

#TODO: Include an if condition IF a user enters a NON float value
#TODO: Test launch file using direct numbers instead of strings for the rates
#TODO: Test if writing a value in CLI gives a string or a float
#TODO: Look into setting a value in the launch file as a float type
def get_param_float(parameter, safetyValue):
    """Get float value for a parameter, and if the value does not exist, 
        or an invalid parameter is entered, return the previous value it once was"""

    paramValue = get_param_exist(parameter)
    if(paramValue is None):
        paramValue = safetyValue
    try:
        floatValue = float(paramValue)
    except:
        rospy.logwarn("Invalid parameter set, value unchanged: {}".format(safetyValue))
        floatValue = float(safetyValue)
    return floatValue
    

def get_param_exist(parameter):
    """Get parameter state if parameter exists and sets to None if it does not exist"""

    try:
        parameterState = rospy.get_param(parameter)
    except:
        rospy.logwarn("Parameter state None assigned to {}.".format(parameter))
        parameterState = None
    return parameterState

def init_default_mandatory_param(parameter, defaultValue):
    """Initialize parameters which are mandatory in the proper functionality of the simulator"""
    
    paramState = get_param_exist(parameter)
    if (paramState is None):
        rospy.logwarn("Parameter state None assigned to {}".format(parameter))
    rospy.loginfo("{}: {}".format(parameter, defaultValue))
    return defaultValue

def init_default_voluntary_param(parameters, defaultValue):
    """Initialize parameters which can be missing, but not cause problems in the functionality of the simulator"""

    #Check if parameters is a string (single parameter) or an array (multiple parameters) and initialize appropriately
    if isinstance(parameters, str):
        paramValue = get_param_exist("PDS_mock_voltage")
        if (paramValue is None):
            rospy.logwarn("Parameter state None assigned to {}".format(parameters))
        else:
            paramValue = defaultValue
        rospy.loginfo("{}: {}".format(parameters, paramValue))
        return paramValue

    #If parameters is an array, check if the parameters exist, and initialize them appropriately
    else:
        outputArray = []
        for x in range(len(parameters)):
            existFlag = get_param_exist(parameters[x])
            if(existFlag is not None):
                outputArray.append(defaultValue)
            else:
                outputArray.append(None)
                rospy.logwarn("Parameter state None assigned to {}".format(parameters[x]))
        return outputArray


def publish_mock_data(voltages, temps, currents, noiseValues):
    """Get and set parameters to be published"""

    #Initialize node
    node_name = "mock_pds_node"
    rospy.init_node(node_name)
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

    #Initialize 1 starting voltage increment rate
    #Using default value if not initialized in launch file
    voltageRate = init_default_mandatory_param("PDS_mock_voltage_rate", 0.04)

    #Initialize 1 starting temp increment rate
    #Using default value if not initialized in launch file
    tempRate = init_default_mandatory_param("PDS_mock_temp_rate", 1)

    #Initialize 1 starting current increment rate
    #Using default value if not initialized in launch file
    currentRate = init_default_mandatory_param("PDS_mock_current_rate", 0.025)

    #Initialize 1 starting noise
    #Using default value if not initialized in launch file
    currentNoise = init_default_mandatory_param("PDS_mock_global_noise", noiseValues.get("stable"))

    #Initialize 1 starting voltage
    #Initialize variable to default value if it exists, or keep as None if it does not exist
    currentVoltage = init_default_voluntary_param("PDS_mock_voltage", voltages.get("stable"))

    #Initialize 3 starting temperatures
    #Initialize variables to default values if they exist, or keep as None if they do not exist
    parameterTemps = ["PDS_mock_temp1", "PDS_mock_temp2", "PDS_mock_temp3"]
    currentTemps = init_default_voluntary_param(parameterTemps, temps.get("stable"))
    rospy.loginfo("currentTemps: {}".format(currentTemps))

    #Initialize 6 starting wheel currents
    #Initialize variables to default values if they exist, or keep as None if they do not exist
    parameterCurrents = ["PDS_mock_wheel1_current", "PDS_mock_wheel2_current", "PDS_mock_wheel3_current", "PDS_mock_wheel4_current", "PDS_mock_wheel5_current", "PDS_mock_wheel6_current"]
    currentWheelCurrents = init_default_voluntary_param(parameterCurrents, currents.get("stable"))
    rospy.loginfo("currentWheelCurrents: {}".format(currentWheelCurrents))

    # Acquire parameter and set parameter states while launch file is active
    while not rospy.is_shutdown():
        """Get new current, voltage, and thermistor temps recursively
        1) Parameter name
        2) Parameter's previous value
        3) Noise value
        4) Increment value"""

        #Get float voltageRate immediately while ros is running
        voltageRate = get_param_float("PDS_mock_voltage_rate", voltageRate)

        #Get float tempRate immediately while ros is running
        tempRate = get_param_float("PDS_mock_temp_rate", tempRate)

        #Get float currentRate immediately while ros is running
        currentRate = get_param_float("PDS_mock_current_rate", currentRate)

        #Get 1 noise immediately while ros is running
        currentNoise = get_states_values("PDS_mock_global_noise", noiseValues)

        #Get 1 voltage incrementally while ros is running
        if (currentVoltage is not None):
            currentVoltage = get_parameter_value("PDS_mock_voltage", voltages, currentVoltage, currentNoise, voltageRate/rospyRate)

        #Get 3 temperatures incrementally while ros is running
        for x in range(len(currentTemps)):
            if (currentTemps[x] is not None):
                currentTemps[x] = get_parameter_value(parameterTemps[x], temps, currentTemps[x], currentNoise, tempRate/rospyRate)

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
    temps = {"rise": 100, "stable": 25, "fall": -20}

    #Set temperatures rise, stable and fall to be parametrized
    currents = {"rise": 1, "stable": 0.3, "fall": 0}

    #Set temperatures rise, stable and fall to be parametrized
    noiseValues = {"rise": 0.1, "stable": 0.05, "fall": 0.01}

    #Publish all new parameter settings set by user
    try:
        publish_mock_data(voltages, temps, currents, noiseValues)
    except rospy.ROSInterruptException:
        pass
