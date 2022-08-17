#! /usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from mcu_control.msg import Voltage, Currents, PowerConsumption, PowerReport
from mcu_control.srv import PowerReportProvider, PowerReportProviderRequest, PowerReportProviderResponse
import csv
import datetime

wheel_data = None
latest_power_report = PowerReport()
pub = None
running = False

class SubsystemData:
    def __init__(self, numberOfMotors, description):
        self.watt_hours = []
        for i in range(numberOfMotors):
            self.watt_hours.append(0)
        self.total_power = 0
        self.prev_time = None
        self.description = description

    def update(self, currents):
        time = rospy.get_rostime().secs + (rospy.get_rostime().nsecs / 1000000000)
        if self.prev_time is not None:
            # find power consumed since last datapoint
            delta = time - self.prev_time
            for i in range(len(currents)):
                self.watt_hours[i] += currents[i] * delta
                # NOTE: voltage missing!!
            # now, sum up all wheels and put in report
            self.total_power = 0
            for x in self.watt_hours:
                self.total_power += x
        self.prev_time = time


def provide_report(data):
    return latest_power_report

def write_report():
    timestamp = datetime.datetime.now().strftime('%Y-%m-%dT%H:%M:%S')
    filename = '../../../../../Power-Report_' + timestamp + '.csv'
    with open(filename, 'w', newline='') as report_file:
        writer = csv.writer(report_file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(['Description', 'Power consumption (Wh-H)'])
        for subsystem in latest_power_report.report:
            writer.writerow([subsystem.description, str(subsystem.wattHours)])

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


def initData():
    global wheel_data, latest_power_report
    wheel_data = SubsystemData(6, 'wheels')
    latest_power_report = PowerReport()
    latest_power_report.report = [PowerConsumption(description='wheels', wattHours=0)]

def start():
    initData()
    rospy.init_node('power_report_node', anonymous = False)
    # subscribe to PDS feeds
    rospy.Subscriber('wheel_motor_currents', Currents, wheel_current_callback)
    # subscribe for start/stop commands
    rospy.Subscriber('power_report_command', String, action_callback)

    global pub
    pub = rospy.Publisher('power_consumption', PowerReport, queue_size = 10)
    s = rospy.Service('power_report_provider', PowerReportProvider, provide_report)

    rospy.loginfo('Power report node started')
    rospy.spin()

if __name__ == '__main__':
    start()