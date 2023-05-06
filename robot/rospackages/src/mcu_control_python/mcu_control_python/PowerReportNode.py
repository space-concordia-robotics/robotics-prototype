#! /usr/bin/env python3

import array
import sys
import rclpy
from std_msgs.msg import String, Float32
from messages_and_services.msg import Voltages, Currents, PowerConsumption, PowerReport
from messages_and_services.srv import PowerReportProvider

import csv
import datetime
import os

class SubsystemData:
    """This class handles the energy consumption calculations and 
    stores data needed for that. Must call update() with the new
    current data when it comes in, and set self.voltages to the
    voltage data for each sensor/motor in the subsystem as it
    is updated. Note: the length of self.voltages must be the same
    as the number of motors."""
    def __init__(self, number_of_motors, description):
        self.watt_hours = []
        for i in range(number_of_motors):
            self.watt_hours.append(0)
        self.total_power = 0
        self.prev_time = None
        self.description = description
        # This is a default voltage value!
        self.voltages = [15] * number_of_motors

    def reset(self):
        """Resets all numbers to default"""
        self.__init__(len(self.watt_hours), self.description)

    def update(self, currents):
        """Every time current data comes in, this is run to update the power consumption."""
        time = node.get_clock().now().secs + (node.get_clock().now().nsecs / 1000000000)
        if self.prev_time is not None:
            # find power consumed since last datapoint
            delta_hours = (time - self.prev_time) / (60 * 60)
            if len(currents) is not len(self.voltages):
                node.get_logger().err('ERROR: voltage information is not the right length')
                return
            for i in range(len(currents)):
                self.watt_hours[i] += currents[i] * self.voltages[i] * delta_hours
            # now, sum up all motors/sensors of the system
            self.total_power = 0
            for x in self.watt_hours:
                self.total_power += x
        # if this is the first time since start that we have current, just
        # store the prev time (since don't know for how long this current
        # is valid)
        self.prev_time = time

wheel_data = SubsystemData(6, 'wheels')
arm_data = SubsystemData(6, 'arm motors')
control_data = SubsystemData(1, 'control systen: onboard computer and PoE')
subsystems = [wheel_data, arm_data, control_data]
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
            total_consumption += subsystem.total_watt_hours

            # break down per sensor if there are multiple sensors per subsystem
            if len(subsystem.individual_watt_hours) > 1:
                for i, motor_consumption in enumerate(subsystem.individual_watt_hours, 1):
                    writer.writerow([subsystem.description + ' ' + str(i), str(motor_consumption)])
                writer.writerow([])
                writer.writerow([subsystem.description + ' total', subsystem.total_watt_hours])
                writer.writerow([])
            # just write down the subsystem's consumption if there is one sensor for the system
            else:
                writer.writerow([subsystem.description, subsystem.total_watt_hours])
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
        node.get_logger.info('Power report node starting to log power consumption.')
    elif message.data == 'stop':
        unsubscribe_from_PDS()
        write_report()
        node.get_logger.info('Power report node stopping to log power consumption.')
    else:
        node.get_logger.err('invalid command')


def wheel_current_callback(data):
    global wheel_data, pub, latest_power_report
    wheel_data.update(data.effort)
    latest_power_report.report[0] = PowerConsumption(wheel_data.description, wheel_data.total_power, wheel_data.watt_hours)
    # rospy.loginfo('wheel watt hours: ' + str(wheel_data.total_power))

def wheel_voltage_callback(data):
    global wheel_data
    wheel_data.voltages = data.data

def arm_voltage_callback(data):
    global arm_data
    arm_data.voltages = data.data

def arm_current_callback(data):
    global arm_data, pub, latest_power_report
    arm_data.update(data.effort)
    latest_power_report.report[1] = PowerConsumption(arm_data.description, arm_data.total_power, arm_data.watt_hours)
    # rospy.loginfo('arm watt hours: ' + str(wheel_data.total_power))

def control_current_callback(data):
    global control_data, pub, latest_power_report
    control_data.update(data.effort)
    latest_power_report.report[2] = PowerConsumption(control_data.description, control_data.total_power, control_data.watt_hours)
    # rospy.loginfo('control system watt hours: ' + str(control_data.total_power))

def control_voltage_callback(data):
    global control_data
    control_data.voltages = data.data


def initData():
    """Reset all data for power consumption to 0."""
    global subsystems, latest_power_report
    # reset the current power report
    latest_power_report = PowerReport()
    report = []
    # reset the internal power data
    for subsystem in subsystems:
        subsystem.reset()
        # ROS2 uses python's typed arrays, not the standard list
        individual_watt_hours = array.array('d', subsystem.watt_hours)
        report.append(PowerConsumption(description=subsystem.description, total_watt_hours=0.0, individual_watt_hours=individual_watt_hours))

    latest_power_report.report = report

def subscribe_to_PDS():
    """Subscribes to the voltage and current feeds of the PDS and
    stores them in pds_feeds"""
    global pds_feeds
    # subscribe to PDS feeds and store them
    pds_feeds = [node.create_subscription(Currents, 'wheel_motor_currents', wheel_current_callback),
                 node.create_subscription(Voltages, 'wheel_motor_voltages', wheel_voltage_callback),
                 node.create_subscription(Currents, 'arm_motor_currents', arm_current_callback),
                 node.create_subscription(Voltages, 'arm_motor_voltages', arm_voltage_callback),
                 node.create_subscription(Currents, 'control_current', control_current_callback),
                 node.create_subscription(Voltages, 'control_voltage', control_voltage_callback)]

def unsubscribe_from_PDS():
    """Unsubscribes from all PDS current and voltage feeds."""
    global pds_feeds
    for feed in pds_feeds:
        feed.unregister()
    pds_feeds = []

def main():
    initData()
    rclpy.init(args=sys.argv)
    global node
    node = rclpy.create_node('power_report_node')
    # subscribe for start/stop commands
    node.create_subscription(String, 'power_report_command', action_callback, 10)

    # publisher for power consumption, and setup service that provides power reports
    global pub
    node.create_service(PowerReportProvider, 'power_report_provider', provide_report)

    node.get_logger().info('Power report node started')
    rclpy.spin(node)

