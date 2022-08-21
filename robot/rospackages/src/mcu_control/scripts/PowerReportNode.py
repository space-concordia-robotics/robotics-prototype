#! /usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from mcu_control.msg import Voltages, Currents, PowerConsumption, PowerReport
from mcu_control.srv import PowerReportProvider
import csv
import datetime
import os
import functools
from Subsystem import Subsystem


wheel_data = Subsystem(6, 'wheels', 'wheel_motor_voltages', 'wheel_motor_currents')
arm_data = Subsystem(6, 'arm motors', 'arm_motor_voltages', 'arm_motor_currents')
arm_servo_data = Subsystem(2, 'arm servos (gripper, then base)', 'arm_servo_voltages', 'arm_servo_currents')
control_data = Subsystem(1, 'mobile platform: onboard computer, communications, Lidar, mast servos', 'control_voltage', 'control_current')
subsystems = [wheel_data, arm_data, control_data, arm_servo_data]

latest_power_report = PowerReport()
pub = None

def provide_report(data):
    """This is the callback for a service that provides the latest
    cumulative power consumption figures."""
    return latest_power_report

def write_report():
    """Writes a csv power report in home/Power_Reports where
    the filename is Power-Report_timestamp."""
    global latest_power_report

    folder = os.path.expanduser("~") + '/Power_Reports'
    if not os.path.exists(folder):
        os.makedirs(folder)

    timestamp = datetime.datetime.now().strftime('%Y-%m-%dT%H:%M:%S')
    filename = folder + '/Power-Report_' + timestamp + '.csv'
    with open(filename, 'w', newline='') as report_file:
        writer = csv.writer(report_file, quoting=csv.QUOTE_MINIMAL)
        writer.writerow(['Description', 'Power consumption (Wh-H)'])
        total_consumption = 0
        for subsystem in latest_power_report.report:
            # include in calculation of total power
            total_consumption += subsystem.totalWattHours

            # break down per sensor if there are multiple sensors per subsystem
            if len(subsystem.individualWattHours) > 1:
                for i, motor_consumption in enumerate(subsystem.individualWattHours, 1):
                    writer.writerow([subsystem.description + ' ' + str(i), str(motor_consumption)])
                writer.writerow([])
                writer.writerow([subsystem.description + ' total', subsystem.totalWattHours])
                writer.writerow([])
            # just write down the subsystem's consumption if there is one sensor for the system
            else:
                writer.writerow([subsystem.description, subsystem.totalWattHours])
                writer.writerow([])
        # add total power consumption
        writer.writerow(['Total', total_consumption])


def action_callback(message):
    """Runs when a message is sent to power_report_command, and starts or
    stops power measuring appropriately. When it stops measuring, it
    produces a CSV report; when is starts, it resets everything."""
    if message.data == 'start':
        initData()
        subscribe_to_PDS()
        rospy.loginfo('Power report node starting to log power consumption.')
    elif message.data == 'stop':
        unsubscribe_from_PDS()
        write_report()
        rospy.loginfo('Power report node stopping to log power consumption.')
    else:
        rospy.logerr('invalid command')


def initData():
    """Reset all data for power consumption to 0."""
    global subsystems, latest_power_report
    # reset the current power report
    latest_power_report = PowerReport()
    report = []
    # reset the internal power data
    for subsystem in subsystems:
        subsystem.reset()
        individualWattHours = subsystem.watt_hours.copy()
        report.append(PowerConsumption(description=subsystem.description, totalWattHours=0, individualWattHours=individualWattHours))
    latest_power_report.report = report

def subscribe_to_PDS():
    """Subscribes to the voltage and current feeds of the PDS and
    stores them in pds_feeds"""
    global pds_feeds, subsystem_voltage_feeds, subsystem_current_feeds
    # Define base for the callbacks for current and voltage feeds
    def current_callback_template(data, reportIndex, current_data):
        global pub, latest_power_report
        data.update(current_data.effort)
        latest_power_report.report[reportIndex] = PowerConsumption(data.description, data.total_power, data.watt_hours)

    def voltage_callback_template(data, voltage_data):
        data.voltages = voltage_data.data
    # subscribe to the current and voltage feeds
    pds_feeds = []
    for reportIndex, subsystem in enumerate(subsystems):
        pds_feeds.append(rospy.Subscriber(subsystem.current_topic, Currents, functools.partial(current_callback_template, subsystem, reportIndex)))
        pds_feeds.append(rospy.Subscriber(subsystem.voltage_topic, Voltages, functools.partial(voltage_callback_template, subsystem)))

def unsubscribe_from_PDS():
    """Unsubscribes from all PDS current and voltage feeds."""
    global pds_feeds
    for feed in pds_feeds:
        feed.unregister()
    pds_feeds = []

def start():
    initData()
    rospy.init_node('power_report_node', anonymous = False)
    # subscribe for start/stop commands
    rospy.Subscriber('power_report_command', String, action_callback)

    # publisher for power consumption, and setup service that provides power reports
    global pub
    rospy.Service('power_report_provider', PowerReportProvider, provide_report)

    # setup so we'll get the arm servo current frequently
    rate = rospy.Rate(5) # get arm servo voltage and frequency 5 times/second
    arm_pub = rospy.Publisher('arm_command', String, queue_size=10)

    rospy.loginfo('Power report node started')

    while not rospy.is_shutdown():
        arm_pub.publish('get_power')
        rate.sleep()

if __name__ == '__main__':
    start()