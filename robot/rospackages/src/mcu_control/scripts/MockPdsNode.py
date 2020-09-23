#! /usr/bin/env python3

import time
import rospy
import sys
import random
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from mcu_control.msg import ThermistorTemps, Voltage, Currents
from decimal import *

def get_parameter_state(parameter, values, prevValue, noise, increment):
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

#TODO: Function to increment
#TODO: Function to add noise


#TODO: Refactor the parameter names to something general
def init_parameter_values(prevParameterValues, prevVoltageStates, voltages):

    outputArray = []

    for x in range(len(prevVoltageStates)):
        if (prevVoltageStates[x][1] == "rise"):
            prevParameterValues[prevVoltageStates[x][0]] = voltages.get("rise")
        elif (prevVoltageStates[x][1] == "stable"):
            prevParameterValues[prevVoltageStates[x][0]] = voltages.get("stable")
        elif (prevVoltageStates[x][1] == "fall"):
            prevParameterValues[prevVoltageStates[x][0]] = voltages.get("fall")
        else:
            rospy.logwarn("Failed: Invalid parameter entered for PDS_mock_voltage")
        outputArray.append(prevParameterValues.get(prevVoltageStates[x][0]))
    
    # else:
    #     for x in range(len(parameterStates)):
    #         if (parameterStates == "rise"):
    #             parameterValues[parameterStates[x][0]] = values.get("rise")
    #         elif (parameterStates == "stable"):
    #             parameterValues[parameterStates[x][0]] = values.get("stable")
    #         elif (parameterStates == "fall"):
    #             parameterValues[parameterStates[x][0]] = values.get("fall")
    #         else:
    #             rospy.logwarn("Failed: Invalid parameter entered for {}".format(parameterStates[x][0]))
    #         outputArray.append(parameterValues.get(parameterStates[x][0]))
    
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

    #Set initial 1 current noise
    if (prevNoiseState == "rise"):
        prevParameterValues["PDS_mock_global_noise"] = noiseValues.get("rise")
    elif (prevNoiseState == "stable"):
        prevParameterValues["PDS_mock_global_noise"] = noiseValues.get("stable")
    elif (prevNoiseState == "fall"):
        prevParameterValues["PDS_mock_global_noise"] = noiseValues.get("fall")
    else:
        rospy.logwarn("Failed: Invalid parameter entered for PDS_mock_global_noise")
        prevParameterValues["PDS_mock_global_noise"] = 0
    currentNoise = prevParameterValues.get("PDS_mock_global_noise")

    #Set initial 1 current voltage (can be changed to accomodate more voltages)
    currentVoltage = []
    # for x in range(len(prevVoltageStates)):
    #     if (prevVoltageState == "rise"):
    #         prevParameterValues[prevVoltageStates[x][0]] = voltages.get("rise")
    #     elif (prevVoltageState == "stable"):
    #         prevParameterValues[prevVoltageStates[x][0]] = voltages.get("stable")
    #     elif (prevVoltageState == "fall"):
    #         prevParameterValues[prevVoltageStates[x][0]] = voltages.get("fall")
    #     else:
    #         rospy.logwarn("Failed: Invalid parameter entered for PDS_mock_voltage")
    #     currentVoltage.append(prevParameterValues.get("PDS_mock_voltage"))

    #TODO: FIX THIS PART WHY ISN'T IT WORKING IDK
    currentVoltage = init_parameter_values(prevParameterValues, prevVoltageStates, voltages)
    rospy.logwarn("currentVoltage: {}".format(currentVoltage)) 

    #Set initial 3 current temperatures
    currentTemps = []
    # for x in range(len(prevTempStates)):
    #     if (prevTempStates[x][1] == "rise"):
    #         prevParameterValues[prevTempStates[x][0]] = temps.get("rise")
    #     elif (prevTempStates[x][1] == "stable"):
    #         prevParameterValues[prevTempStates[x][0]] = temps.get("stable")
    #     elif (prevTempStates[x][1] == "fall"):
    #         prevParameterValues[prevTempStates[x][0]] = temps.get("fall")
    #     else:
    #         rospy.logwarn("Failed: Invalid parameter entered for prevTempStates[x][0]")
    #     currentTemps.append(prevParameterValues.get("PDS_mock_temp{}".format(x+1)))
    
    currentTemps = init_parameter_values(prevParameterValues, prevTempStates, temps)
    rospy.logwarn("currentTemps: {}".format(currentTemps)) 


    #Set initial 6 current wheel currents
    currentWheelCurrents = []
    # for x in range(len(prevWheelCurrentStates)):
    #     if (prevWheelCurrentStates[x][1] == "rise"):
    #         prevParameterValues[prevWheelCurrentStates[x][0]] = currents.get("rise")
    #     elif (prevWheelCurrentStates[x][1] == "stable"):
    #         prevParameterValues[prevWheelCurrentStates[x][0]] = currents.get("stable")
    #     elif (prevWheelCurrentStates[x][1] == "fall"):
    #         prevParameterValues[prevWheelCurrentStates[x][0]] = currents.get("fall")
    #     else:
    #         rospy.logwarn("Failed: Invalid parameter entered for prevWheelCurrentStates[x][0]")
    #     currentWheelCurrents.append(prevParameterValues.get("PDS_mock_wheel{}_current".format(x+1)))

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