#! /usr/bin/env python3

import rospy

class Subsystem:
    """This class handles the energy consumption calculations and 
    stores data needed for that. Must call update() with the new
    current data when it comes in, and set self.voltages to the
    voltage data for each sensor/motor in the subsystem as it
    is updated. Note: the length of self.voltages must be the same
    as the number of motors."""
    def __init__(self, number_of_motors, description, voltage_topic, current_topic):
        self.watt_hours = []
        for i in range(number_of_motors):
            self.watt_hours.append(0)
        self.total_power = 0
        self.prev_time = None
        self.description = description
        # This is a default voltage value to prevent power measurement while waiting on first voltage
        self.voltages = [0] * number_of_motors
        self.voltage_topic = voltage_topic
        self.current_topic = current_topic

    def reset(self):
        """Resets all numbers to default"""
        self.__init__(len(self.watt_hours), self.description, self.voltage_topic, self.current_topic)

    def update(self, currents):
        """Every time current data comes in, this is run to update the power consumption."""
        time = rospy.get_rostime().secs + (rospy.get_rostime().nsecs / 1000000000)
        if self.prev_time is not None:
            # find power consumed since last datapoint
            delta_hours = (time - self.prev_time) / (60 * 60)
            if len(currents) is not len(self.voltages):
                rospy.logerr('ERROR: voltage information is not the right length')
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
