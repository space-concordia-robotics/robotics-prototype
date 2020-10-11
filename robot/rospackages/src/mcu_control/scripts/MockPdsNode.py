#! /usr/bin/env python3

import time
import sys
import random
import traceback
from decimal import *

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from mcu_control.msg import ThermistorTemps, Voltage, Currents


def get_parameter_value(parameter, values, valueBeforeIncrement,
                        noise, increment):
    """Get parameter state and assign value
    General function, needs not to be changed"""

    # Acquire the value associated to the state if it exists
    currentGetValue = get_states_values(
        parameter, values, valueBeforeIncrement)

    # Increment or decrement to the desired value with noise included
    remainder = currentGetValue - valueBeforeIncrement

    # Initialize values before and after noise
    valueBeforeNoise = None
    valueAfterNoise = None

    # If negative: increment, if positive: decrement
    if abs(remainder) >= (noise + increment):
        if remainder > 0:
            # Increment the value presented on the gui
            valueBeforeNoise = valueBeforeIncrement + increment

            # Add noise to the incremented value
            valueAfterNoise = round(
                valueBeforeNoise + (noise * (random.uniform(-1, 1))), 2)
        else:
            # Decrement the value presented on the gui
            valueBeforeNoise = valueBeforeIncrement - increment

            # Add noise to the incremented value
            valueAfterNoise = round(
                valueBeforeNoise + (noise * (random.uniform(-1, 1))), 2)
    else:
        # Add noise to the value if it is not incrementing or decrementing
        valueAfterNoise = round(
            currentGetValue + (noise * (random.uniform(-1, 1))), 2)

    # If the lowest value the variable can reach is 0
    # Do not allow the variable to be negative
    if currentGetValue == 0 and valueAfterNoise < 0:
        valueBeforeNoise = valueBeforeIncrement - increment
        valueAfterNoise = round(
            valueBeforeNoise + (noise * (random.uniform(0, 1))), 2)

    return valueAfterNoise


def get_states_values(parameters, values, safetyValue):
    """Initialize parameters with the values
    according to the states (rise, stable and fall)"""

    # Check if parameters is a string (single parameter)
    # Or an array (multiple parameters)
    if isinstance(parameters, str):
        paramState = get_param_exist(parameters)
        newValue = None
        if paramState == "rise":
            newValue = values.get("rise")
        elif paramState == "stable":
            newValue = values.get("stable")
        elif paramState == "fall":
            newValue = values.get("fall")
        else:
            rospy.logwarn("Failed: Invalid parameter entered")
            # If noise or voltage rate, temp rate or current rate
            # were published a wrong input
            # (i.e. float instead of rise, fall or stable)
            # then keep the value as is was before
            if newValue is None:
                newValue = safetyValue
                rospy.logwarn(
                    "Invalid parameter value entered, value left unchanged")
        return newValue

    # If parameters is an array
    # Return the value associated for each parameter state
    else:
        outputArray = []
        for x in range(len(parameters)):
            paramState = get_param_exist(parameters[x])
            if paramState == "rise":
                outputArray.append(values.get("rise"))
            elif paramState == "stable":
                outputArray.append(values.get("stable"))
            elif paramState == "fall":
                outputArray.append(values.get("fall"))
            else:
                rospy.logwarn(
                    "Failed: Invalid parameter entered for {}"
                    .format(parameters[x]))
        return outputArray


def get_param_float(parameter, safetyValue):
    """Get float value for a parameter
    and if the value does not exist
    or an invalid parameter is entered
    return the previous value it once was"""

    # Check if a value was entered for the parameter
    paramValue = get_param_exist(parameter)

    # Check if a value was initialized in the launch file
    # If it was not, set its value to the value it was before
    if paramValue is None:
        paramValue = safetyValue

    # If a value was entered by the user, check if it is a float number
    try:
        floatValue = float(paramValue)
    # If the value entered is not a float, keep the value it was set to before
    except TypeError:
        rospy.logwarn(
            "Invalid parameter set, value unchanged: {}".format(safetyValue))
        floatValue = float(safetyValue)
    return floatValue


def get_param_exist(parameter):
    """Get parameter state if parameter exists
    and sets to None if it does not exist"""

    try:
        parameterState = rospy.get_param(parameter)
    except ImportError:
        rospy.logwarn("Parameter state None assigned to {}.".format(parameter))
        parameterState = None
    return parameterState


def init_default_mandatory_param(parameter, defaultValue):
    """Initialize parameters to a default value regardless if they
    exist or not to assure proper simulator functionality"""

    # Check if parameter exists
    paramState = get_param_exist(parameter)

    # If the parameter does not exist, give a warning,
    # But return the same default initial value if the parameter exists or not
    if paramState is None:
        rospy.logwarn("Parameter state None assigned to {}".format(parameter))
    rospy.loginfo("{}: {}".format(parameter, defaultValue))
    return defaultValue


def init_default_voluntary_param(parameters, defaultValue):
    """Initialize parameters to a default value if they exist,
    but if they do not, then set them to None"""

    # Check if parameters is a string (single parameter) or an array
    # (multiple parameters) and initialize appropriately
    if isinstance(parameters, str):
        paramValue = get_param_exist("PDS_mock_voltage")
        if paramValue is None:
            rospy.logwarn(
                "Parameter state None assigned to {}".format(parameters))
        else:
            paramValue = defaultValue
        rospy.loginfo("{}: {}".format(parameters, paramValue))
        return paramValue

    # If parameters is an array, check if the parameters exist
    # And initialize them appropriately
    else:
        outputArray = []
        for x in range(len(parameters)):
            existFlag = get_param_exist(parameters[x])
            if existFlag is not None:
                outputArray.append(defaultValue)
            else:
                outputArray.append(None)
                rospy.logwarn(
                    "Parameter state None assigned to {}"
                    .format(parameters[x]))
        return outputArray


def publish_mock_data(voltages, temps, currents):
    """Get and set all parameters to be published"""

    # Default values parameters
    DEFAULT_VOLTAGE_RATE = 0.04
    DEFAULT_TEMP_RATE = 1
    DEFAULT_CURRENT_RATE = 0.025
    DEFAULT_VOLTAGE_NOISE = 0.05
    DEFAULT_TEMP_NOISE = 0.05
    DEFAULT_CURRENT_NOISE = 0.01
    DEFAULT_CURRENT_VOLTAGE = voltages.get("stable")
    DEFAULT_CURRENT_TEMPS = temps.get("stable")
    DEFAULT_CURRENT_WHEEL_CURRENTS = currents.get("stable")
    TEMP_ARRAY_SIZE = 3
    CURRENT_ARRAY_SIZE = 6

    # Initialize node
    node_name = "mock_pds_node"
    rospy.init_node(node_name)
    rospy.loginfo("Initialized: {} node for mock PDS values".format(node_name))
    rospyRate = 10
    rate = rospy.Rate(rospyRate)

    # Initialize publishers (Voltage, ThermistorTemps, Currents)
    voltage = Voltage()
    thermistorTemps = ThermistorTemps()
    wheelCurrents = Currents()
    pubVoltage = rospy.Publisher('battery_voltage', Voltage, queue_size=10)
    pubTemp = rospy.Publisher('battery_temps', ThermistorTemps, queue_size=10)
    pubWheelCurrent = rospy.Publisher(
        'wheel_motor_currents', Currents, queue_size=10)

    # Initialize 1 starting voltage increment rate
    # Using default value if not initialized in launch file
    voltageRate = init_default_mandatory_param("PDS_mock_voltage_rate",
                                               DEFAULT_VOLTAGE_RATE)

    # Initialize 1 starting temp increment rate
    # Using default value if not initialized in launch file
    tempRate = init_default_mandatory_param("PDS_mock_temp_rate",
                                            DEFAULT_TEMP_RATE)

    # Initialize 1 starting current increment rate
    # Using default value if not initialized in launch file
    currentRate = init_default_mandatory_param("PDS_mock_current_rate",
                                               DEFAULT_CURRENT_RATE)

    # Initialize 1 starting voltage noise
    # Using default value if not initialized in launch file
    voltageNoise = init_default_mandatory_param("PDS_mock_voltage_noise",
                                                DEFAULT_VOLTAGE_NOISE)

    # Initialize 1 starting temperature noise
    # Using default value if not initialized in launch file
    tempNoise = init_default_mandatory_param("PDS_mock_temperature_noise",
                                             DEFAULT_TEMP_NOISE)

    # Initialize 1 starting current noise
    # Using default value if not initialized in launch file
    currentNoise = init_default_mandatory_param("PDS_mock_current_noise",
                                                DEFAULT_CURRENT_NOISE)

    # Initialize 1 starting voltage
    # Initialize variable to default value if it exists
    # OR keep as None if it does not exist
    currentVoltage = init_default_voluntary_param(
        "PDS_mock_voltage", DEFAULT_CURRENT_VOLTAGE)

    # Initialize 3 starting temperatures
    # Initialize variables to default values if they exist
    # OR keep as None if they do not exist
    parameterTemps = []
    for x in range(TEMP_ARRAY_SIZE):
        parameterTemps.append("PDS_mock_temp{}".format(x+1))

    currentTemps = init_default_voluntary_param(
        parameterTemps, DEFAULT_CURRENT_TEMPS)
    rospy.loginfo("currentTemps: {}".format(currentTemps))

    # Initialize 6 starting wheel currents
    # Initialize variables to default values if they exist
    # OR keep as None if they do not exist
    parameterCurrents = []
    for x in range(CURRENT_ARRAY_SIZE):
        parameterCurrents.append("PDS_mock_wheel{}_current".format(x+1))

    currentWheelCurrents = init_default_voluntary_param(
        parameterCurrents, DEFAULT_CURRENT_WHEEL_CURRENTS)
    rospy.loginfo("currentWheelCurrents: {}".format(currentWheelCurrents))

    # Acquire parameter and set parameter states while launch file is active
    while not rospy.is_shutdown():
        """Get new current, voltage, and thermistor temps recursively
        1) get_param_float takes a (float) value directly from the user
        2) get_states_values takes a (state) directly from the user
        and immediately sets that value to the variable
        3) get_parameter_value takes a (state) directly from the user
        and incrementally increases/decreases to the desired state condition"""

        voltageRate = get_param_float("PDS_mock_voltage_rate", voltageRate)
        tempRate = get_param_float("PDS_mock_temp_rate", tempRate)
        currentRate = get_param_float("PDS_mock_current_rate", currentRate)
        voltageNoise = get_param_float("PDS_mock_voltage_noise", voltageNoise)
        tempNoise = get_param_float("PDS_mock_temperature_noise", tempNoise)
        currentNoise = get_param_float("PDS_mock_current_noise", currentNoise)

        if currentVoltage is not None:
            currentVoltage = get_parameter_value(
                "PDS_mock_voltage", voltages, currentVoltage,
                voltageNoise, voltageRate/rospyRate)

        for x in range(len(currentTemps)):
            if currentTemps[x] is not None:
                currentTemps[x] = get_parameter_value(
                    parameterTemps[x], temps, currentTemps[x],
                    tempNoise, tempRate/rospyRate)

        for x in range(len(currentWheelCurrents)):
            if currentWheelCurrents[x] is not None:
                currentWheelCurrents[x] = get_parameter_value(
                    parameterCurrents[x], currents, currentWheelCurrents[x],
                    currentNoise, currentRate/rospyRate)

        # Set values to objects which will get published
        # Set voltage 1
        if currentVoltage is not None:
            voltage.data = float(currentVoltage)
        # Set temperatures 1-3
        if currentTemps[0] is not None:
            thermistorTemps.therm1 = float(currentTemps[0])
        if currentTemps[1] is not None:
            thermistorTemps.therm2 = float(currentTemps[1])
        if currentTemps[2] is not None:
            thermistorTemps.therm3 = float(currentTemps[2])

        wheelCurrents.effort.clear()
        # Set wheel currents 1-6
        for x in range(len(currentWheelCurrents)):
            if currentWheelCurrents[x] is not None:
                wheelCurrents.effort.append(float(currentWheelCurrents[x]))

        # Publish values to topics (Voltage, ThermistorTemps and Currents)
        pubVoltage.publish(voltage)
        pubTemp.publish(thermistorTemps)
        pubWheelCurrent.publish(wheelCurrents)

        # Add delay before each iteration of loop which runs rospy
        rate.sleep()


if __name__ == '__main__':
    """Calls publisher function and passes parameter settings
    1) Change the values for the parameter states (rise, stable and fall)
    2) However, change the initial starting values
    for the parameters directly in publish_mock_data
    3) No other places require to be modified
    to change the state (rise, stable, and fall) values"""

    # Set voltages rise, stable and fall to be parametrized
    voltages = {"rise": 20, "stable": 15, "fall": 10}

    # Set temperatures rise, stable and fall to be parametrized
    temps = {"rise": 100, "stable": 25, "fall": -20}

    # Set temperatures rise, stable and fall to be parametrized
    currents = {"rise": 1, "stable": 0.3, "fall": 0}

    # Publish all new parameter settings set by user
    try:
        publish_mock_data(voltages, temps, currents)
    except Exception:
        print(traceback.format_exc())
        pass
