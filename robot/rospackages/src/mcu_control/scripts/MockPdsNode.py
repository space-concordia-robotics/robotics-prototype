#! /usr/bin/env python3

import time
import rospy
import sys
import random
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from mcu_control.msg import ThermistorTemps, Voltage, Currents
from decimal import *

def get_parameter_state(parameter, values, valueBeforeIncrement, noise, increment):
    """Get parameter state and assign value
    General function, needs not to be changed"""

    try:
        param = rospy.get_param(parameter)
    except:
        rospy.logwarn("Failed: Parameter state not acquired.")
        return None

    currentGetValue = None
    if (param == "rise"):
        currentGetValue = values.get("rise")
    elif (param == "stable"):
        currentGetValue = values.get("stable")
    elif (param == "fall"):
        currentGetValue = values.get("fall")
    else:
        rospy.logwarn("Failed: Invalid parameter entered")
        return None

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

def init_parameter_values(parameterValuesMap, parameterState, stateValue):
    """Initialize parameters with the values according to the states (rise, stable and fall)"""

    outputArray = []
    for x in range(len(parameterState)):
        if (parameterState[x][1] == "rise"):
            parameterValuesMap[parameterState[x][0]] = stateValue.get("rise")
        elif (parameterState[x][1] == "stable"):
            parameterValuesMap[parameterState[x][0]] = stateValue.get("stable")
        elif (parameterState[x][1] == "fall"):
            parameterValuesMap[parameterState[x][0]] = stateValue.get("fall")
        else:
            rospy.logwarn("Failed: Invalid parameter entered for {}".format(parameterState[x][0]))
        outputArray.append(parameterValuesMap.get(parameterState[x][0]))
    return outputArray

def get_param_exist(parameter):
    """Get parameter state if parameter exists and sets to None if it does not"""

    try:
        parameterState = rospy.get_param(parameter)
    except:
        rospy.logwarn("Parameter state None assigned.")
        parameterState = None

    return parameterState

def publish_mock_data(voltages, temps, currents, noiseValues):
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

    #Get previous states before changing parameters for each individual
    prevNoiseState = get_param_exist("PDS_mock_global_noise")
    prevVoltageState = get_param_exist("PDS_mock_voltage")
    prevTemp1State = get_param_exist("PDS_mock_temp1")
    prevTemp2State = get_param_exist("PDS_mock_temp2")
    prevTemp3State = get_param_exist("PDS_mock_temp3")
    prevWheelCurrent1State = get_param_exist("PDS_mock_wheel1_current")
    prevWheelCurrent2State = get_param_exist("PDS_mock_wheel2_current")
    prevWheelCurrent3State = get_param_exist("PDS_mock_wheel3_current")
    prevWheelCurrent4State = get_param_exist("PDS_mock_wheel4_current")
    prevWheelCurrent5State = get_param_exist("PDS_mock_wheel5_current")
    prevWheelCurrent6State = get_param_exist("PDS_mock_wheel6_current")

    #Create list of noises to loop through when assigning new voltages
    prevNoiseStates = []
    prevNoiseStates.append(("PDS_mock_global_noise", prevNoiseState))

    #Create list of voltages to loop through when assigning new voltages
    prevVoltageStates = []
    prevVoltageStates.append(("PDS_mock_voltage", prevVoltageState))

    #Create list of temperatures to loop through when assigning new temperatures
    prevTempStates = []
    prevTempStates.append(("PDS_mock_temp1", prevTemp1State))
    prevTempStates.append(("PDS_mock_temp2", prevTemp2State))
    prevTempStates.append(("PDS_mock_temp3", prevTemp3State))

    #Create list of wheel currents to loop through when assigning new currents
    prevWheelCurrentStates = []
    prevWheelCurrentStates.append(("PDS_mock_wheel1_current", prevWheelCurrent1State))
    prevWheelCurrentStates.append(("PDS_mock_wheel2_current", prevWheelCurrent2State))
    prevWheelCurrentStates.append(("PDS_mock_wheel3_current", prevWheelCurrent3State))
    prevWheelCurrentStates.append(("PDS_mock_wheel4_current", prevWheelCurrent4State))
    prevWheelCurrentStates.append(("PDS_mock_wheel5_current", prevWheelCurrent5State))
    prevWheelCurrentStates.append(("PDS_mock_wheel6_current", prevWheelCurrent6State))

    #Create dict with all parameter values (append to for scalability)
    prevParameterValues = {
        "PDS_mock_global_noise": None,
        "PDS_mock_voltage": None,
        "PDS_mock_temp1": None,
        "PDS_mock_temp2": None,
        "PDS_mock_temp3": None,
        "PDS_mock_wheel1_current": None,
        "PDS_mock_wheel2_current": None,
        "PDS_mock_wheel3_current": None,
        "PDS_mock_wheel4_current": None,
        "PDS_mock_wheel5_current": None,
        "PDS_mock_wheel6_current": None
    }

    #Initialize 1 starting noise (can be changed to accomodate more voltages)
    currentNoises = []
    currentNoises = init_parameter_values(prevParameterValues, prevNoiseStates, noiseValues)
    currentNoise = currentNoises[0]
    rospy.logwarn("currentNoise: {}".format(currentNoise))

    #Initialize 1 starting voltage (can be changed to accomodate more voltages)
    currentVoltage = []
    currentVoltage = init_parameter_values(prevParameterValues, prevVoltageStates, voltages)
    rospy.logwarn("currentVoltage: {}".format(currentVoltage))

    #Initialize 3 starting temperatures (can be changed to accomodate more temps)
    currentTemps = []
    currentTemps = init_parameter_values(prevParameterValues, prevTempStates, temps)
    rospy.logwarn("currentTemps: {}".format(currentTemps))

    #Initialize 6 starting wheel currents (can be changed to accomodate more currents)
    currentWheelCurrents = []
    currentWheelCurrents = init_parameter_values(prevParameterValues, prevWheelCurrentStates, currents)
    rospy.logwarn("currentWheelCurrents: {}".format(currentWheelCurrents))

    # Acquire parameter and set parameter states while launch file is active
    while not rospy.is_shutdown():
        """Get new current, voltage, and thermistor temps recursively
        1) Parameter name
        2) Parameter's previous value
        3) Noise value
        4) Increment value"""

        #TODO: Create method to get NOISE values (modify get_parameter_state to work with noise)

        #Get 1 voltage while ros is running
        for x in range(len(currentVoltage)):
            if (currentVoltage is not None):
                # currentVoltage = get_parameter_state("PDS_mock_voltage", voltages, currentNoise, 0.04/rospyRate) + currentValue
                # UNCOMMENT THIS LINE FOR SINGLE VOLTAGE VALUE
                # currentVoltage = get_parameter_state("PDS_mock_voltage", voltages, currentVoltage, currentNoise, 0.04/rospyRate)
                currentVoltage[x] = get_parameter_state("PDS_mock_voltage", voltages, currentVoltage[x], currentNoise, 0.04/rospyRate)

        #Get all 3 temperatures while ros is running
        for x in range(len(prevTempStates)):
            if (currentTemps[x] is not None):
                currentTemps[x] = get_parameter_state("PDS_mock_temp{}".format(x+1), temps, currentTemps[x], currentNoise, 0.04/rospyRate)

        #Get all 6 wheel currents while ros is running
        for x in range(len(prevWheelCurrentStates)):
            if (currentWheelCurrents[x] is not None):
                currentWheelCurrents[x] = get_parameter_state("PDS_mock_wheel{}_current".format(x+1), currents, currentWheelCurrents[x], currentNoise, 0.04/rospyRate)


        #Set values to objects which will get published
        #Set voltage 1
        if (currentVoltage is not None):
            voltage.data = float(currentVoltage[0])
        #Set temperatures 1-3
        if (currentTemps[0] is not None):
            thermistorTemps.therm1 = float(currentTemps[0])
        if (currentTemps[1] is not None):
            thermistorTemps.therm2 = float(currentTemps[1])
        if (currentTemps[2] is not None):
            thermistorTemps.therm3 = float(currentTemps[2])
        #Set wheel currents 1-6
        for x in range (len(currentWheelCurrents)):
            if (currentWheelCurrents[x] is not None):
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

    #Set voltages high, normal and low to be parametrized
    voltages = {"rise": 20, "stable": 15, "fall": 10}

    #Set temperatures high, normal and low to be parametrized
    temps = {"rise": 100, "stable": 50, "fall": -20}

    #Set temperatures high, normal and low to be parametrized
    currents = {"rise": 1, "stable": 0.3, "fall": 0}

    #Set temperatures high, normal and low to be parametrized
    noiseValues = {"rise": 0.1, "stable": 0.05, "fall": 0.01}

    #Publish all new parameter settings set by user
    try:
        publish_mock_data(voltages, temps, currents, noiseValues)
    except rospy.ROSInterruptException:
        pass

#-----------------Scenarios-----------------
#Normal Battery Depletion 16.5V, and fall
#Overcharged battery, 19V, fall slowly
#Rover on fire, HOT HOT HIGH TEMP
#Broken motor, rising current on X motor