#! /usr/bin/env python3

"""Simulator used to acquire and set parameters to
user defined states, which are initialized either
manually by the user in CLI or through .launch file"""

import time
import sys
import random
import traceback
from decimal import *

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from mcu_control.msg import ThermistorTemps, Voltages, Currents


def get_parameter_value(parameter, values, value_before_increment,
                        noise, increment):
    """Get parameter state and assign value
    General function, needs not to be changed"""

    # Acquire the value associated to the state if it exists
    current_get_value = get_states_values(
        parameter, values, value_before_increment)

    # Increment or decrement to the desired value with noise included
    remainder = current_get_value - value_before_increment

    # If negative: increment, if positive: decrement
    if abs(remainder) >= (noise + increment):
        if remainder > 0:
            # Increment the value presented on the gui
            value_before_noise = value_before_increment + increment

            # Add noise to the incremented value
            value_after_noise = round(
                value_before_noise + (noise * (random.uniform(-1, 1))), 2)
        else:
            # Decrement the value presented on the gui
            value_before_noise = value_before_increment - increment

            # Add noise to the incremented value
            value_after_noise = round(
                value_before_noise + (noise * (random.uniform(-1, 1))), 2)
    else:
        # Add noise to the value if it is not incrementing or decrementing
        value_after_noise = round(
            current_get_value + (noise * (random.uniform(-1, 1))), 2)

    # If the lowest value the variable can reach is 0
    # Do not allow the variable to be negative
    if current_get_value == 0 and value_after_noise < 0:
        value_before_noise = value_before_increment - increment
        value_after_noise = round(
            value_before_noise + (noise * (random.uniform(0, 1))), 2)

    return value_after_noise


def get_states_values(parameters, values, safety_value):
    """Initialize parameters with the values
    according to the states (rise, stable and fall)"""

    # Check if parameters is a string (single parameter)
    # Or an array (multiple parameters)
    if isinstance(parameters, str):
        param_state = get_param_exist(parameters)
        new_value = None
        if param_state == "rise":
            new_value = values.get("rise")
        elif param_state == "stable":
            new_value = values.get("stable")
        elif param_state == "fall":
            new_value = values.get("fall")
        else:
            rospy.logwarn("Failed: Invalid parameter entered")
            # If noise or voltage rate, temp rate or current rate
            # were published a wrong input
            # (i.e. float instead of rise, fall or stable)
            # then keep the value as is was before
            if new_value is None:
                new_value = safety_value
                rospy.logwarn(
                    "Invalid parameter value entered, value left unchanged")
        return new_value

    # If parameters is an array
    # Return the value associated for each parameter state
    else:
        output_array = []
        for x in range(len(parameters)):
            param_state = get_param_exist(parameters[x])
            if param_state == "rise":
                output_array.append(values.get("rise"))
            elif param_state == "stable":
                output_array.append(values.get("stable"))
            elif param_state == "fall":
                output_array.append(values.get("fall"))
            else:
                rospy.logwarn(
                    "Failed: Invalid parameter entered for {}"
                    .format(parameters[x]))
        return output_array


def get_param_float(parameter, safety_value):
    """Get float value for a parameter
    and if the value does not exist
    or an invalid parameter is entered
    return the previous value it once was"""

    # Check if a value was entered for the parameter
    param_value = get_param_exist(parameter)

    # Check if a value was initialized in the launch file
    # If it was not, set its value to the value it was before
    if param_value is None:
        param_value = safety_value

    # If a value was entered by the user, check if it is a float number
    try:
        float_value = float(param_value)
    # If the value entered is not a float, keep the value it was set to before
    except Exception:
        print(traceback.format_exc())
        rospy.logwarn(
            "Invalid parameter set, value unchanged: {}".format(safety_value))
        float_value = float(safety_value)
    return float_value


def get_param_exist(parameter):
    """Get parameter state if parameter exists
    and sets to None if it does not exist"""

    try:
        parameter_state = rospy.get_param(parameter)
    except Exception:
        print(traceback.format_exc())
        rospy.logwarn("Parameter state None assigned to {}.".format(parameter))
        parameter_state = None
    return parameter_state


def init_default_mandatory_param(parameter, default_value):
    """Initialize parameters to a default value regardless if they
    exist or not to assure proper simulator functionality"""

    # Check if parameter exists
    param_state = get_param_exist(parameter)

    # If the parameter does not exist, give a warning,
    # But return the same default initial value if the parameter exists or not
    if param_state is None:
        rospy.logwarn("Parameter state None assigned to {}".format(parameter))
    rospy.loginfo("{}: {}".format(parameter, default_value))
    return default_value


def init_default_voluntary_param(parameters, default_value):
    """Initialize parameters to a default value if they exist,
    but if they do not, then set them to None"""

    # Check if parameters is a string (single parameter) or an array
    # (multiple parameters) and initialize appropriately
    if isinstance(parameters, str):
        param_value = get_param_exist("PDS_mock_voltage") # WTF? Why is PDS_mock_voltage hardcoded here? (Marc)
        if param_value is None:
            rospy.logwarn(
                "Parameter state None assigned to {}".format(parameters))
        else:
            param_value = default_value
        rospy.loginfo("{}: {}".format(parameters, param_value))
        return param_value

    # If parameters is an array, check if the parameters exist
    # And initialize them appropriately
    else:
        output_array = []
        for x in range(len(parameters)):
            exist_flag = get_param_exist(parameters[x])
            if exist_flag is not None:
                output_array.append(default_value)
            else:
                output_array.append(None)
                rospy.logwarn(
                    "Parameter state None assigned to {}"
                    .format(parameters[x]))
        return output_array


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
    DEFAULT_CURRENT_WHEEL_VOLTAGES = voltages.get("stable")
    DEFAULT_CURRENT_ARM_CURRENTS = currents.get("stable")
    DEFAULT_CURRENT_ARM_VOLTAGES = voltages.get("stable")
    DEFAULT_CURRENT_CONTROL_CURRENT = currents.get("stable")
    DEFAULT_CURRENT_CONTROL_VOLTAGE = voltages.get("stable")
    TEMP_ARRAY_SIZE = 3
    CURRENT_ARRAY_SIZE = 6

    # Initialize node
    node_name = "mock_pds_node"
    rospy.init_node(node_name)
    rospy.loginfo("Initialized: {} node for mock PDS values".format(node_name))
    rospy_rate = 10
    rate = rospy.Rate(rospy_rate)

    # Initialize publishers (Voltage, ThermistorTemps, Currents)
    voltage = Voltages()
    thermistor_temps = ThermistorTemps()
    wheel_currents = Currents()
    wheel_voltage = Voltages()
    arm_currents = Currents()
    arm_voltages = Voltages()
    control_current = Currents()
    control_voltage = Voltages()
    pub_voltage = rospy.Publisher('battery_voltage', Voltages, queue_size=10)
    pub_temp = rospy.Publisher('battery_temps', ThermistorTemps, queue_size=10)
    pub_wheel_current = rospy.Publisher(
        'wheel_motor_currents', Currents, queue_size=10)
    pub_wheel_voltage = rospy.Publisher(
        'wheel_motor_voltages', Voltages, queue_size=10)
    pub_arm_current = rospy.Publisher(
        'arm_motor_currents', Currents, queue_size=10)
    pub_arm_voltage = rospy.Publisher(
        'arm_motor_voltages', Voltages, queue_size=10)
    pub_control_current = rospy.Publisher(
        'control_current', Currents, queue_size=10)
    pub_control_voltage = rospy.Publisher(
        'control_voltage', Voltages, queue_size=10)

    # Initialize 1 starting voltage increment rate
    # Using default value if not initialized in launch file
    voltage_rate = init_default_mandatory_param("PDS_mock_voltage_rate",
                                                DEFAULT_VOLTAGE_RATE)

    # Initialize 1 starting temp increment rate
    # Using default value if not initialized in launch file
    temp_rate = init_default_mandatory_param("PDS_mock_temp_rate",
                                             DEFAULT_TEMP_RATE)

    # Initialize 1 starting current increment rate
    # Using default value if not initialized in launch file
    current_rate = init_default_mandatory_param("PDS_mock_current_rate",
                                                DEFAULT_CURRENT_RATE)

    # Initialize 1 starting voltage noise
    # Using default value if not initialized in launch file
    voltage_noise = init_default_mandatory_param("PDS_mock_voltage_noise",
                                                 DEFAULT_VOLTAGE_NOISE)

    # Initialize 1 starting temperature noise
    # Using default value if not initialized in launch file
    temp_noise = init_default_mandatory_param("PDS_mock_temperature_noise",
                                              DEFAULT_TEMP_NOISE)

    # Initialize 1 starting current noise
    # Using default value if not initialized in launch file
    current_noise = init_default_mandatory_param("PDS_mock_current_noise",
                                                 DEFAULT_CURRENT_NOISE)

    # Initialize 1 starting voltage
    # Initialize variable to default value if it exists
    # Or keep as None if it does not exist
    current_voltage = init_default_voluntary_param(
        "PDS_mock_voltage", DEFAULT_CURRENT_VOLTAGE)

    # Initialize 3 starting temperatures
    # Initialize variables to default values if they exist
    # Or keep as None if they do not exist
    parameter_temps = []
    for x in range(TEMP_ARRAY_SIZE):
        parameter_temps.append("PDS_mock_temp{}".format(x+1))

    current_temps = init_default_voluntary_param(
        parameter_temps, DEFAULT_CURRENT_TEMPS)
    rospy.loginfo("current_temps: {}".format(current_temps))

    # Initialize 6 starting wheel currents
    # Initialize variables to default values if they exist
    # Or keep as None if they do not exist
    parameter_currents = []
    for x in range(CURRENT_ARRAY_SIZE):
        parameter_currents.append("PDS_mock_wheel{}_current".format(x+1))

    current_wheel_currents = init_default_voluntary_param(
        parameter_currents, DEFAULT_CURRENT_WHEEL_CURRENTS)
    rospy.loginfo("current_wheel_currents: {}".format(current_wheel_currents))

    # Trying (Marc) to replicate above but for the wheel motor voltage CHECK THIS
    parameter_voltages = []
    for x in range(CURRENT_ARRAY_SIZE):
        parameter_voltages.append("PDS_mock_wheel{}_voltage".format(x+1))

    current_wheel_voltages = init_default_voluntary_param(
        parameter_voltages, DEFAULT_CURRENT_WHEEL_VOLTAGES)
    rospy.loginfo("current_wheel_voltages: {}".format(current_wheel_voltages))

    # Trying (Marc) to replicate above but for the arm motor voltage CHECK THIS
    parameter_arm_voltages = []

    for x in range(CURRENT_ARRAY_SIZE):
        parameter_arm_voltages.append("PDS_mock_arm{}_voltage".format(x+1))

    current_arm_voltages = init_default_voluntary_param(
        parameter_arm_voltages, DEFAULT_CURRENT_ARM_VOLTAGES)
    rospy.loginfo("current_arm_voltages: {}".format(current_arm_voltages))

    # This is pretty hacky, just using same parameters as wheel currents essentially as this had to be done very quickly
    current_arm_currents = init_default_voluntary_param(
        parameter_currents, DEFAULT_CURRENT_ARM_CURRENTS)
    rospy.loginfo("current_arm_currents: {}".format(current_arm_currents))

    current_control_current = init_default_voluntary_param(
        parameter_currents, DEFAULT_CURRENT_CONTROL_CURRENT)
    rospy.loginfo("current_obc_currents: {}".format(current_control_current[0]))

    parameter_control_voltage = 'PDS_mock_control_voltage'
    current_control_voltage = init_default_voluntary_param(
        parameter_control_voltage, DEFAULT_CURRENT_CONTROL_VOLTAGE)

    # Acquire parameter and set parameter states while launch file is active
    while not rospy.is_shutdown():
        """Get new current, voltage, and thermistor temps recursively
        1) get_param_float takes a (float) value directly from the user
        2) get_states_values takes a (state) directly from the user
        and immediately sets that value to the variable
        3) get_parameter_value takes a (state) directly from the user
        and incrementally increases/decreases to the desired state condition"""

        # Get and set float values
        voltage_rate = get_param_float("PDS_mock_voltage_rate", voltage_rate)
        temp_rate = get_param_float("PDS_mock_temp_rate", temp_rate)
        current_rate = get_param_float("PDS_mock_current_rate", current_rate)
        voltage_noise = get_param_float("PDS_mock_voltage_noise",
                                        voltage_noise)
        temp_noise = get_param_float("PDS_mock_temperature_noise", temp_noise)
        current_noise = get_param_float("PDS_mock_current_noise",
                                        current_noise)

        # Get and set to parameter state incrementally
        if current_voltage is not None:
            current_voltage = get_parameter_value(
                "PDS_mock_voltage", voltages, current_voltage,
                voltage_noise, voltage_rate/rospy_rate)

        for x in range(len(current_temps)):
            if current_temps[x] is not None:
                current_temps[x] = get_parameter_value(
                    parameter_temps[x], temps, current_temps[x],
                    temp_noise, temp_rate/rospy_rate)

        for x in range(len(current_wheel_currents)):
            if current_wheel_currents[x] is not None:
                current_wheel_currents[x] = get_parameter_value(
                    parameter_currents[x], currents, current_wheel_currents[x],
                    current_noise, current_rate/rospy_rate)
        
        for x in range(len(current_wheel_voltages)):
            if current_wheel_voltages[x] is not None:
                current_wheel_voltages[x] = get_parameter_value(
                    parameter_voltages[x], voltages, current_wheel_voltages[x],
                    current_noise, voltage_rate/rospy_rate)

        for x in range(len(current_arm_currents)):
            if current_arm_currents[x] is not None:
                current_arm_currents[x] = get_parameter_value(
                    parameter_currents[x], currents, current_arm_currents[x],
                    current_noise, current_rate/rospy_rate)

        for x in range(len(current_arm_voltages)):
            if current_arm_voltages[x] is not None:
                current_arm_voltages[x] = get_parameter_value(
                    parameter_arm_voltages[x], voltages, current_arm_voltages[x],
                    current_noise, voltage_rate/rospy_rate)

        for x in range(len(current_control_current)):
            if current_control_current[x] is not None:
                current_control_current[x] = get_parameter_value(
                    parameter_currents[x], currents, current_control_current[x],
                    current_noise, current_rate/rospy_rate)

        if current_control_voltage is not None:
            current_control_voltage = get_parameter_value(parameter_control_voltage,
                                                      voltages, current_control_voltage,
                                                      current_noise, voltage_rate/rospy_rate)

        # Set values to objects which will get published
        # Set voltage 1
        if current_voltage is not None:
            voltage.data = [float(current_voltage)]
        # CHECK IF THIS CODE IS SALVAGEABLE
        # Set temperatures 1-3
        #if current_temps[0] is not None:
        #    thermistor_temps.therm1 = float(current_temps[0])
        #if current_temps[1] is not None:
        #    thermistor_temps.therm2 = float(current_temps[1])
        #if current_temps[2] is not None:
        #    thermistor_temps.therm3 = float(current_temps[2])

        wheel_currents.effort.clear()
        # Set wheel currents 1-6
        for x in range(len(current_wheel_currents)):
            if current_wheel_currents[x] is not None:
                wheel_currents.effort.append(float(current_wheel_currents[x]))
        # Set wheel voltages 1-6 CHECK IF THIS IS RIGHT
        wheel_voltage.data.clear()
        for x in range(len(current_wheel_voltages)):
            if current_wheel_voltages[x] is not None:
                wheel_voltage.data.append(float(current_wheel_voltages[x]))

        # Set arm voltages 1-6 CHECK IF THIS IS RIGHT
        arm_voltages.data.clear()
        for x in range(len(current_arm_voltages)):
            if current_arm_voltages[x] is not None:
                arm_voltages.data.append(float(current_arm_voltages[x]))
        

        # THIS SHOULD BE DONE PROPERLY FOR ARM, OBC AND POE CURRENTS AT SOME OTHER POINT RATHER THAN JUST USING THE
        # VALUES FROM WHEEL CURRENT IN THIS HACKY WAY
        arm_currents.effort.clear()
        # Set arm currents 1-6
        for x in range(len(current_arm_currents)):
            if current_arm_currents[x] is not None:
                arm_currents.effort.append(float(current_arm_currents[x]))

        # Set control system current and voltage
        control_current.effort.clear()
        control_current.effort.append(float(current_control_current[0]))
        control_voltage.data.clear()
        control_voltage.data.append(float(current_control_voltage))

        # Publish values to topics (Voltage, ThermistorTemps and Currents)
        pub_voltage.publish(voltage)
        pub_temp.publish(thermistor_temps)
        pub_wheel_current.publish(wheel_currents)
        pub_wheel_voltage.publish(wheel_voltage)
        pub_arm_current.publish(arm_currents)
        pub_arm_voltage.publish(arm_voltages)
        pub_control_current.publish(control_current)
        pub_control_voltage.publish(control_voltage)


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