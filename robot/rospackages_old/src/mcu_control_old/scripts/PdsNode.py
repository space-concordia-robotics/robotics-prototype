#! /usr/bin/env python3

import sys
import traceback
import time
import re

from robot.rospackages.src.mcu_control.scripts.CommandParser import *

from robot.rospackages.src.mcu_control.scripts.PdsAdapter import *

import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
from mcu_control.msg import ThermistorTemps, FanSpeeds, Voltages, Currents
from mcu_control.srv import *

from collections import deque

command_queue = deque()

PDS1_ERROR_CORRECTION = [45, 0, 0, 0, 0, 17]
PDS2_ERROR_CORRECTION = [45, 0, 0, 0, 0, 55]

K_RAW_CONVERSION = 5 / 8704
K_RAW_CONVERSION_BATTERY = 43 / 51200

# Kill power to all motors
def handle_estop(motorsToStop):
    if bool(motorsToStop[0]):
        pds1SetOffOkay = PacketOutSetSwitchChannel().setOff([*range(0, 4)]).send(port, 1).isOkay()
        if pds1SetOffOkay is not True:
            feedbackPub.publish("[ERROR]: eStop: PDS1 board threw an exception")
        else:
            feedbackPub.publish("[INFO]: Stopped motors for arm")

    if bool(motorsToStop[1]):
        pds2SetOffOkay = PacketOutSetSwitchChannel().setOff([*range(0,6)]).send(port, 2).isOkay()
        if pds2SetOffOkay is not True:
            feedbackPub.publish("[ERROR]: eStop: PDS2 board threw an exception")
        else:
            feedbackPub.publish("[INFO]: Stopped motors for wheels")

def handle_ping(args):
    pds1PingContent = PacketOutPing().setPingContent([12, 34, 56, 78]).send(port, 1).getPingContent()
    # TODO: Update address for PDS2
    pds2PingContent = PacketOutPing().setPingContent([12, 34, 56, 78]).send(port, 1).getPingContent()

    feedbackPub.publish(f"[INFO]: ping responses: PDS1: {pds1PingContent} | PDS2: {pds2PingContent}")

def handle_enable_motors(motorsToEnable):
    if bool(motorsToEnable[0]):
        pds1SetOnOkay = PacketOutSetSwitchChannel().setOn([*range(0, 4)]).send(port, 1).isOkay()
        if pds1SetOnOkay is not True:
            feedbackPub.publish("[ERROR]: Enable arm motors: PDS1 board threw an exception")
        else:
            feedbackPub.publish("[INFO]: Enabled motors for arm")

    if bool(motorsToEnable[1]):
        pds2SetOnOkay = PacketOutSetSwitchChannel().setOn([*range(0,6)]).send(port, 2).isOkay()
        if pds2SetOnOkay is not True:
            feedbackPub.publish("[ERROR]: Enable wheel motors: PDS2 board threw an exception")
        else:
            feedbackPub.publish("[INFO]: Enabled motors for wheels")

pds_command_handlers = dict([(pds_out_commands[0][1], handle_estop), (pds_out_commands[1][1], handle_ping),
                             (pds_out_commands[2][1], handle_enable_motors)])

def send_queued_commands():
    if (len(command_queue) > 0):
        command = command_queue.popleft()
        send_command(command[0], command[1], command[2])

def command_callback(message):
    rospy.loginfo('received: ' + message.data + ' command, sending to PDS')
    command, args = parse_command(message)

    temp_struct = [command, args, PDS_SELECTED]
    command_queue.append(temp_struct)

def send_command(command_name, args, deviceToSendTo):
    command = get_command(command_name, deviceToSendTo)
    if command is not None:
        commandID = command[1]

        commandHandler = pds_command_handlers[commandID]

        rospy.loginfo(commandHandler)

        commandHandler(args)

def publish_pds_data():
    try:
        publishSwitchChannelAndBatteryData()

        publishTemps()

    except Exception as e:
        print('type error: ' + str(e))
        rospy.logwarn('trouble parsing PDS sensor data')
        return
    return

def publishSwitchChannelAndBatteryData():
    batteryVoltage = Voltages()
    batteryVoltage.data.append(0.0) # there prob is a better way to init this idk

    wheelCurrents = Currents()
    armCurrents = Currents()
    controlCurrent = Currents()

    wheelVoltages = Voltages()
    armVoltages = Voltages()
    controlVoltage = Voltages()

    packetOutChannels = PacketOutReadSwitchChannel()
    packetOutBatteryChannel = PacketOutReadBatteryChannel()

    # PDS1 - [0] to [3]: arm motors drivers | [4]: OBC | [5]: Radio POE
    voltagesRawPds1 = packetOutChannels.send(port, 1).getMeasurement()

    # Error correction for raw values
    zipVoltageRawCorrection = zip(voltagesRawPds1, PDS1_ERROR_CORRECTION)
    index = 0
    for voltageRaw, correction in zipVoltageRawCorrection:
        voltagesRawPds1[index] = voltageRaw - correction

    # PDS2 - [0] to [5]: wheel motor drivers
    voltagesRawPds2 = packetOutChannels.send(port, 2).getMeasurement()

    zipVoltageRawCorrection = zip(voltagesRawPds2, PDS2_ERROR_CORRECTION)
    index = 0
    for voltageRaw, correction in zipVoltageRawCorrection:
        voltagesRawPds2[index] = voltageRaw - correction

    batteryVoltageRaw = packetOutBatteryChannel.send(port, 1).getMeasurement()

    currentPds1, currentPds2 = rawVoltagesToCurrentConversion(voltagesRawPds1, voltagesRawPds2)

    batteryVoltage.data = rawAproxRawVoltagesConversion([batteryVoltageRaw])

    armMotorCurrents, wheelMotorCurrents, controlCurrentValue = channelsParser(currentPds1, currentPds2)

    armMotorVoltages, wheelMotorVoltages, controlVoltageValue = 6*batteryVoltage.data, 6*batteryVoltage.data, batteryVoltage.data[0]

    # Publish currents
    armCurrents.effort = armMotorCurrents
    wheelCurrents.effort = wheelMotorCurrents
    controlCurrent.effort = [controlCurrentValue]

    armCurrentPub.publish(armCurrents)
    wheelCurrentPub.publish(wheelCurrents)
    controlCurrentPub.publish(controlCurrent)

    # Publish voltages
    armVoltages.data = armMotorVoltages
    wheelVoltages.data = wheelMotorVoltages
    controlVoltage.data = [controlVoltageValue]

    armVoltagePub.publish(armVoltages)
    wheelVoltagePub.publish(wheelVoltages)
    controlVoltagePub.publish(controlVoltage)

    batteryVoltagePub.publish(batteryVoltage)

def publishTemps():
    temp = ThermistorTemps()

    temp.therms = PacketOutReadTempChannel().send(port, 1).getMeasurement()

    tempPub.publish(temp)

def rawVoltagesToCurrentConversion(voltagesRawPds1, voltagesRawPds2):
    return rawAproxRawCurrentConversion(voltagesRawPds1), rawAproxRawCurrentConversion(voltagesRawPds2)

def rawVoltagesToVoltageConversion(voltagesRawPds1, voltagesRawPds2, batteryVoltageRaw):
    return rawAproxRawVoltagesConversion(voltagesRawPds1), rawAproxRawVoltagesConversion(voltagesRawPds2), \
           rawBatteryVoltageConversion(batteryVoltageRaw)

def rawAproxRawCurrentConversion(raw):
    return [((x*0.55114602 + 103.780152) / 1000) if x > 100 else 0 for x in raw]

def rawAproxRawVoltagesConversion(raw):
    return [(x*8.6374*pow(10,-4) - 0.0785959) for x in raw]

def rawBatteryVoltageConversion(rawVoltage):
    return rawVoltage * K_RAW_CONVERSION_BATTERY

def channelsParser(pds1, pds2):
    armMotor = pds1[:4]
    armMotor.append(0.0)
    armMotor.append(0.0) # Temp placeholder for smart servos
    wheelMotor = pds2

    control = pds1[4] + pds1[5]

    return armMotor, wheelMotor, control


if __name__ == '__main__':
    node_name = 'pds_node'
    rospy.init_node(node_name, anonymous = False)  # only allow one node of this type
    rospy.loginfo('Initialized "' + node_name + '" node for pub/sub/service functionality')

    # Currents
    wheel_current_pub_topic = '/wheel_motor_currents'
    rospy.loginfo('Begining to publish to "' + wheel_current_pub_topic + '" topic')
    wheelCurrentPub = rospy.Publisher(wheel_current_pub_topic, Currents, queue_size = 10)

    arm_current_pub_topic = '/arm_motor_currents'
    rospy.loginfo('Begining to publish to "' + arm_current_pub_topic + '" topic')
    armCurrentPub = rospy.Publisher(arm_current_pub_topic, Currents, queue_size=10)

    control_current_pub_topic = '/control_current'
    rospy.loginfo('Begining to publish to "' + control_current_pub_topic + '" topic')
    controlCurrentPub = rospy.Publisher(control_current_pub_topic, Currents, queue_size=10)

    # Voltages
    # PacketOutReadBatteryChannel
    battery_voltage_pub_topic = '/battery_voltage'
    rospy.loginfo('Beginning to publish to "' + battery_voltage_pub_topic + '" topic')
    batteryVoltagePub = rospy.Publisher(battery_voltage_pub_topic, Voltages, queue_size=10)

    wheel_voltage_pub_topic = '/wheel_motor_voltages'
    rospy.loginfo('Begining to publish to "' + wheel_voltage_pub_topic + '" topic')
    wheelVoltagePub = rospy.Publisher(wheel_voltage_pub_topic, Voltages, queue_size=10)

    arm_voltage_pub_topic = '/arm_motor_voltages'
    rospy.loginfo('Begining to publish to "' + arm_voltage_pub_topic + '" topic')
    armVoltagePub = rospy.Publisher(arm_voltage_pub_topic, Voltages, queue_size=10)

    control_pub_topic = '/control_voltage'
    rospy.loginfo('Begining to publish to "' + control_pub_topic + '" topic')
    controlVoltagePub = rospy.Publisher(control_pub_topic, Voltages, queue_size=10)

    # Temp
    # Probably PacketOutReadTempChannel
    temp_pub_topic = '/battery_temps'
    rospy.loginfo('Begining to publish to "' + temp_pub_topic + '" topic')
    tempPub = rospy.Publisher(temp_pub_topic, ThermistorTemps, queue_size = 10)

    # fan_speeds_pub_topic = '/fan_speeds'
    # rospy.loginfo('Beginning to publish to "' + fan_speeds_pub_topic + '" topic')
    # fanSpeedsPub = rospy.Publisher(fan_speeds_pub_topic, FanSpeeds, queue_size = 10)

    error_flags_topic = '/pds_flags'
    rospy.loginfo('Beginning to publish to "' + error_flags_topic + '" topic')
    flagsPub = rospy.Publisher(error_flags_topic, String, queue_size = 10)

    feedback_pub_topic = '/pds_feedback'
    rospy.loginfo('Beginning to publish to "' + feedback_pub_topic + '" topic')
    feedbackPub = rospy.Publisher(feedback_pub_topic, String, queue_size = 10)

    subscribe_topic = '/pds_command'
    rospy.loginfo('Beginning to subscribe to "' + subscribe_topic + '" topic')
    sub = rospy.Subscriber(subscribe_topic, String, command_callback)

    rosRate = rospy.Rate(100)

    try:
        while not rospy.is_shutdown():
            publish_pds_data()
            send_queued_commands()

            rosRate.sleep()
    except rospy.ROSInterruptException:
        pass

    def shutdown_hook():
        rospy.logwarn('This node (' + node_name + ') is shutting down')
        port.close()
        time.sleep(1)  # give ROS time to deal with the node closing (rosbridge especially)

    rospy.on_shutdown(shutdown_hook)
