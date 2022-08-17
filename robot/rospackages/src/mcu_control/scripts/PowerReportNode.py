#! /usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from mcu_control.msg import Voltage, Currents, PowerConsumption, PowerReport
from mcu_control.srv import PowerReportProvider, PowerReportProviderRequest, PowerReportProviderResponse
import csv
import datetime
import os

class SubsystemData:
    def __init__(self, numberOfMotors, description):
        self.watt_hours = []
        for i in range(numberOfMotors):
            self.watt_hours.append(0)
        self.total_power = 0
        self.prev_time = None
        self.description = description
    
    def reset(self):
        self.__init__(len(self.watt_hours), self.description)

    def update(self, currents):
        time = rospy.get_rostime().secs + (rospy.get_rostime().nsecs / 1000000000)
        if self.prev_time is not None:
            # find power consumed since last datapoint
            delta_hours = (time - self.prev_time) / (60 * 60)
            for i in range(len(currents)):
                self.watt_hours[i] += currents[i] * delta_hours * 12
                # NOTE: voltage not accurate!!
            # now, sum up all wheels and put in report
            self.total_power = 0
            for x in self.watt_hours:
                self.total_power += x
        self.prev_time = time

wheel_data = SubsystemData(6, 'wheels')
arm_data = SubsystemData(6, 'arm motors')
obc_data = SubsystemData(1, 'onboard computer')
poe_data = SubsystemData(1, 'power over ethernet injector')
subsystems = [wheel_data, arm_data, obc_data, poe_data]
latest_power_report = PowerReport()
pub = None
running = False

def provide_report(data):
    return latest_power_report

def write_report():
    global subsystems

    folder = os.path.expanduser("~") + '/Power_Reports'
    if not os.path.exists(folder):
        os.makedirs(folder)

    timestamp = datetime.datetime.now().strftime('%Y-%m-%dT%H:%M:%S')
    filename = folder + '/Power-Report_' + timestamp + '.csv'
    with open(filename, 'w', newline='') as report_file:
        writer = csv.writer(report_file, quoting=csv.QUOTE_MINIMAL)
        writer.writerow(['Description', 'Power consumption (Wh-H)'])
        for subsystem in subsystems:
            # break down per sensor if there are multiple sensors per subsystem
            if len(subsystem.watt_hours) > 1:
                for i, motor_consumption in enumerate(subsystem.watt_hours, 1):
                    writer.writerow([subsystem.description + ' ' + str(i), str(motor_consumption)])
                writer.writerow([])
                writer.writerow([subsystem.description + ' total', subsystem.total_power])
                writer.writerow([])
            # just write down the subsystem's consumption if there is one sensor for the system
            else:
                writer.writerow([subsystem.description, subsystem.total_power])
                writer.writerow([])

def action_callback(message):
    global running
    if message.data == 'start':
        initData()
        running = True
    elif message.data == 'stop':
        running = False
        write_report()
    else:
        rospy.loginfo('invalid command')


def wheel_current_callback(data):
    global wheel_data, running, pub, latest_power_report
    if running:
        wheel_data.update(data.effort)
        latest_power_report.report[0] = PowerConsumption(wheel_data.description, wheel_data.total_power)
        rospy.loginfo('wheel watt hours: ' + str(wheel_data.total_power))

def arm_current_callback(data):
    global arm_data, running, pub, latest_power_report
    if running:
        arm_data.update(data.effort)
        latest_power_report.report[1] = PowerConsumption(arm_data.description, arm_data.total_power)
        rospy.loginfo('arm watt hours: ' + str(wheel_data.total_power))

def obc_current_callback(data):
    global obc_data, running, pub, latest_power_report
    if running:
        obc_data.update(data.effort)
        latest_power_report.report[2] = PowerConsumption(obc_data.description, obc_data.total_power)
        rospy.loginfo('obc watt hours: ' + str(obc_data.total_power))

def poe_current_callback(data):
    global poe_data, running, pub, latest_power_report
    if running:
        poe_data.update(data.effort)
        latest_power_report.report[3] = PowerConsumption(poe_data.description, poe_data.total_power)
        rospy.loginfo('poe watt hours: ' + str(poe_data.total_power))

def initData():
    global subsystems, latest_power_report
    # reset the internal power data
    for subsystem in subsystems:
        subsystem.reset()
    # reset the current power report
    latest_power_report = PowerReport()
    latest_power_report.report = [PowerConsumption(description=system.description, wattHours=0) for system in subsystems]

def start():
    initData()
    rospy.init_node('power_report_node', anonymous = False)
    # subscribe to PDS feeds
    rospy.Subscriber('wheel_motor_currents', Currents, wheel_current_callback)
    rospy.Subscriber('arm_motor_currents', Currents, arm_current_callback)
    rospy.Subscriber('obc_current', Currents, obc_current_callback)
    rospy.Subscriber('poe_current', Currents, poe_current_callback)
    
    # subscribe for start/stop commands
    rospy.Subscriber('power_report_command', String, action_callback)

    global pub
    pub = rospy.Publisher('power_consumption', PowerReport, queue_size = 10)
    s = rospy.Service('power_report_provider', PowerReportProvider, provide_report)

    rospy.loginfo('Power report node started')
    rospy.spin()

if __name__ == '__main__':
    start()