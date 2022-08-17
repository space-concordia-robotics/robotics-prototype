#! /usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from mcu_control.msg import Voltage, Currents, PowerConsumption, PowerReport
from mcu_control.srv import PowerReportProvider, PowerReportProviderRequest, PowerReportProviderResponse
import csv
import datetime
import os

class SubsystemData:
    """This class handles the energy consumption calculations and 
    stores data needed for that. Must call update() with the new
    current data when it comes in, and set self.voltages to the
    voltage data for each sensor/motor in the subsystem as it
    is updated."""
    def __init__(self, number_of_motors, description):
        self.watt_hours = []
        for i in range(number_of_motors):
            self.watt_hours.append(0)
        self.total_power = 0
        self.prev_time = None
        self.description = description
        # This is a default value!
        self.voltages = [15] * number_of_motors

    def reset(self):
        """Resets all numbers to 0"""
        self.__init__(len(self.watt_hours), self.description)

    def update(self, currents):
        """Every time current data comes in, this is run to update the power consumption."""
        time = rospy.get_rostime().secs + (rospy.get_rostime().nsecs / 1000000000)
        if self.prev_time is not None:
            # find power consumed since last datapoint
            delta_hours = (time - self.prev_time) / (60 * 60)
            for i in range(len(currents)):
                self.watt_hours[i] += currents[i] * self.voltages[i] * delta_hours
                # NOTE: voltage not accurate!!
            # now, sum up all motors
            self.total_power = 0
            for x in self.watt_hours:
                self.total_power += x
        # if this is the first time since start that we have current, just
        # store the prev time (since don't know for how long this current
        # is valid)
        self.prev_time = time

wheel_data = SubsystemData(6, 'wheels')
arm_data = SubsystemData(6, 'arm motors')
obc_data = SubsystemData(1, 'onboard computer')
poe_data = SubsystemData(1, 'power over ethernet injector')
subsystems = [wheel_data, arm_data, obc_data, poe_data]
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
    elif message.data == 'stop':
        unsubscribe_from_PDS()
        write_report()
    else:
        rospy.loginfo('invalid command')


def wheel_current_callback(data):
    global wheel_data, pub, latest_power_report
    wheel_data.update(data.effort)
    latest_power_report.report[0] = PowerConsumption(wheel_data.description, wheel_data.total_power, wheel_data.watt_hours)
    rospy.loginfo('wheel watt hours: ' + str(wheel_data.total_power))

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
    rospy.loginfo('arm watt hours: ' + str(wheel_data.total_power))

def obc_current_callback(data):
    global obc_data, pub, latest_power_report
    obc_data.update(data.effort)
    latest_power_report.report[2] = PowerConsumption(obc_data.description, obc_data.total_power, obc_data.watt_hours)
    rospy.loginfo('obc watt hours: ' + str(obc_data.total_power))

def poe_current_callback(data):
    global poe_data, pub, latest_power_report
    poe_data.update(data.effort)
    latest_power_report.report[3] = PowerConsumption(poe_data.description, poe_data.total_power, poe_data.watt_hours)
    rospy.loginfo('poe watt hours: ' + str(poe_data.total_power))

def initData():
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
    global pds_feeds
    # subscribe to PDS feeds and store them
    pds_feeds = [rospy.Subscriber('wheel_motor_currents', Currents, wheel_current_callback),
             rospy.Subscriber('wheel_motor_voltages', Voltage, wheel_voltage_callback),
             rospy.Subscriber('arm_motor_currents', Currents, arm_current_callback),
             rospy.Subscriber('arm_motor_voltages', Voltage, arm_voltage_callback),
             rospy.Subscriber('obc_current', Currents, obc_current_callback),
             rospy.Subscriber('poe_current', Currents, poe_current_callback)]

def unsubscribe_from_PDS():
    global pds_feeds
    for feed in pds_feeds:
        feed.unregister()
    pds_feeds = []

def start():
    initData()
    rospy.init_node('power_report_node', anonymous = False)
    # subscribe for start/stop commands
    rospy.Subscriber('power_report_command', String, action_callback)

    # publisher for power consumption, and setup service that provides them 
    global pub
    pub = rospy.Publisher('power_consumption', PowerReport, queue_size = 10)
    s = rospy.Service('power_report_provider', PowerReportProvider, provide_report)

    rospy.loginfo('Power report node started')
    rospy.spin()

if __name__ == '__main__':
    start()