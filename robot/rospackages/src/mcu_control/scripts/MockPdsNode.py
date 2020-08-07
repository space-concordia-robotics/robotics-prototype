#! /usr/bin/env python3

import time
import rospy
import sys
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from mcu_control.msg import ThermistorTemps as ThermistorTemps
from mcu_control.msg import Voltage as Voltage
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


def get_voltage_parameter(parameterState, voltages):
    try:
        param = rospy.get_param(parameterState)
    except:
        rospy.loginfo("Failed: Voltage parameter not acquired.")
        sys.exit()

    currentGetVoltage = 0
    if (param == "high"):
        currentGetVoltage = voltages[0][1]
    elif (param == "normal"):
        currentGetVoltage = voltages[1][1]
    elif (param == "low"):
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
def publish_mock_data():

    #Initialize node
    node_name = "mock_pds_node"
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo("Initialized: {} node for mock PDS values".format(node_name))
    rate = rospy.Rate(10)
    pubVoltageTest = rospy.Publisher('battery_voltage', Float32, queue_size=10)
    pubTempTest = rospy.Publisher('battery_temps', Point, queue_size=10)

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

    while not rospy.is_shutdown():
        currentVoltage = get_voltage_parameter("PDS_mock_voltage", voltages)
        currentTemp1 = get_temp_parameter("PDS_mock_temp1", temps)
        currentTemp2 = get_temp_parameter("PDS_mock_temp2", temps)
        currentTemp3 = get_temp_parameter("PDS_mock_temp3", temps)

        pubVoltageTest.publish(currentVoltage)
        pubTempTest.publish(Point(currentTemp1, currentTemp2, currentTemp3))

        rate.sleep()


# def main():

#     #Initialize node
#     node_name = "mock_pds_node"
#     rospy.init_node(node_name, anonymous=True)
#     rospy.loginfo("Initialized: {} node for mock PDS values".format(node_name))
#     rate = rospy.Rate(10)

#     pubVoltageTest = rospy.Publisher('battery_voltage', Float32, queue_size=10)
#     pubTempTest = rospy.Publisher('battery_temps', Float32, queue_size=10)

#     #pubVoltage = rospy.Publisher('battery_voltage', Voltage, queue_size=10)
#     #pubTemp = rospy.Publisher('battery_temps', ThermistorTemps, queue_size=10)

#     #Set voltages high, normal and low to be parametrized
#     voltages = []
#     voltages.append(("high", 20))
#     voltages.append(("normal", 15))
#     voltages.append(("low", 10))

#     #Set temperatures high, normal and low to be parametrized
#     temps = []
#     temps.append(("high", 100))
#     temps.append(("normal", 50))
#     temps.append(("low", -20))

#     #Create objects to publish voltage and temperature
#     voltageMonitor = VoltageMonitor(pubVoltage, voltages)
#     tempMonitor = TemperatureMonitor(pubTemp, temps)

#     try:
#         while not rospy.is_shutdown():
#             publish_mock_pds_data(voltages, temps)
#             rate.sleep()

#     except rospy.ROSInterruptException:
#         pass

#     #Publish values after computing
#     #pubVoltage.publish(voltages[0])
#     #pubTemp.publish(temps[0], temps[1], temps[2])

#     #rospy.spin()

if __name__ == '__main__':
    try:
        #Initialize parameters
        rospy.set_param("PDS_mock_voltage", "high")
        rospy.set_param("PDS_mock_temp1", "normal")
        rospy.set_param("PDS_mock_temp2", "normal")
        rospy.set_param("PDS_mock_temp3", "normal")
        rospy.loginfo("Success: Values published")
    except:
        rospy.loginfo("Fail: Values not published")

    rospy.loginfo(rospy.get_param("PDS_mock_voltage"))
    rospy.loginfo(rospy.get_param("PDS_mock_temp1"))
    rospy.loginfo(rospy.get_param("PDS_mock_temp2"))
    rospy.loginfo(rospy.get_param("PDS_mock_temp3"))

    # #Initialize node
    # node_name = "mock_pds_node"
    # rospy.init_node(node_name, anonymous=True)
    # rospy.loginfo("Initialized: {} node for mock PDS values".format(node_name))
    # rate = rospy.Rate(10)
    # pubVoltageTest = rospy.Publisher('battery_voltage', Float32, queue_size=10)
    # pubTempTest = rospy.Publisher('battery_temps', Point, queue_size=10)

    #pubVoltage = rospy.Publisher('battery_voltage', Voltage, queue_size=10)
    #pubTemp = rospy.Publisher('battery_temps', ThermistorTemps, queue_size=10)

    # #Set voltages high, normal and low to be parametrized
    # voltages = []
    # voltages.append(("high", 20))
    # voltages.append(("normal", 15))
    # voltages.append(("low", 10))

    ## Set temperatures high, normal and low to be parametrized
    # temps = []
    # temps.append(("high", 100))
    # temps.append(("normal", 50))
    # temps.append(("low", -20))

    #Create objects to publish voltage and temperature
    #voltageMonitor = VoltageMonitor(pubVoltage, voltages)
    #tempMonitor = TemperatureMonitor(pubTemp, temps)

    #voltageMonitor = VoltageMonitor(pubVoltageTest, voltages)
    #tempMonitor = TemperatureMonitor(pubTempTest, temps)

    try:
        publish_mock_data()

        # while not rospy.is_shutdown():
        #     pubVoltageTest.publish(voltages[2][1])
        #     pubTempTest.publish(Point(temps[2][1], temps[2][1], temps[2][1]))
        #     #publish_mock_pds_data(voltages, temps)

    except rospy.ROSInterruptException:
        pass