#!/usr/bin/env python3

import rclpy
import rclpy.executors
from sensor_msgs.msg import Joy
import math
from rclpy.qos import QoSProfile
from rclpy.lifecycle import State, TransitionCallbackReturn, LifecycleNode
from sensor_msgs.msg import JointState
from absenc_interface.msg import EncoderValues

devpath = "/dev/cadmouse"


def map_range(value, min, max, new_min, new_max):
    value = value - min
    value = value / (max - min)
    value = value * (new_max - new_min)
    value = value + new_min
    return value


def any_out_of_range(min, max, *values):
    for value in values:
        if value < min or value > max:
            return True
    return False


# Publishes angles in radians for the motors
class IkNode(LifecycleNode):

    def __init__(self):
        node_name = "ik_node"
        super().__init__(node_name)
        self.get_logger().info(
            'Initialized "' + node_name + '" node for pub functionality'
        )

        # Must set values when launching
        self.declare_parameter("joint_lengths", [1.0, 1.0, 1.0])
        self.declare_parameter("joint_angle_mins", [-180.0, -180.0, -180.0, -180.0])
        self.declare_parameter("joint_angle_maxes", [180.0, 180.0, 180.0, 180.0])
        self.declare_parameter("sensitivity", 1.0)
        self.declare_parameter("mode", "")
        self.declare_parameter("solution", 0)
        self.declare_parameter("local_mode", False)

        self.abs_angles = None
        self.initialized = False
        self.angles = None

        # Cartesian coordinates of desired location of end effector
        self.x = 1
        self.y = 0
        self.z = 1
        # Keeps track of spin (think of it as roll angle)
        # Not touched by ik, sent straight to MCUs
        self.th = 0
        # The angle of the last joint with respect to vertical
        self.pitch = 0

    def on_configure(self, previous_state: State) -> TransitionCallbackReturn:

        self.get_logger().info(
            f"on_configure(), previous state was: '{previous_state}'"
        )

        self.setup_initial_joint_params()

        # Sensitivity
        self.sensitivity = (
            self.get_parameter("sensitivity").get_parameter_value().double_value
        )
        # Local mode
        self.local_mode = (
            self.get_parameter("local_mode").get_parameter_value().bool_value
        )
        # Which IK solution to use
        self.solution = (
            self.get_parameter("solution").get_parameter_value().integer_value
        )
        # Mode - if 2D y value stays 0
        self.mode = self.get_parameter("mode").get_parameter_value().string_value

        self.timer = self.create_timer(1 / 30, self.publish_joint_state)

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_lifecycle_publisher(
            JointState, "joint_states", qos_profile
        )

        cad_joy_topic = "/cad_mouse_joy"
        self.cad_joy_sub = self.create_subscription(
            Joy, cad_joy_topic, self.cad_joy_callback, 10
        )
        self.get_logger().info('Created publisher for topic "' + cad_joy_topic + '"')

        joy_topic = "/joy"
        self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        self.get_logger().info('Created publisher for topic "' + joy_topic + '"')

        # Get the initial angle values
        absenc_topic = "/absenc_values"

        if self.local_mode:
            self.abs_angles = [
                0.0,
                math.radians(0.04),
                math.radians(13.65),
                math.radians(-14.42),
            ]
            self.initialize_angles_coords()

        # Will hold the previous joy message (used to toggle values)
        self.last_message = None

        self.absenc_sub = self.create_subscription(
            EncoderValues, absenc_topic, self.absenc_callback, 10
        )
        self.get_logger().info('Created subscriber for topic "' + absenc_topic)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, previous_state: State) -> TransitionCallbackReturn:
        self.get_logger().info(
            f"LifecycleNode '{self.get_name()} is in previous_state '{previous_state.label}. Transitioning to 'unconfigured'"
        )
        self.destroy_publisher(self.joint_pub)
        self.destroy_timer(self.timer)

        if self.cad_joy_sub:
            self.destroy_subscription(self.cad_joy_sub)
            self.cad_joy_sub = None

        if self.joy_sub:
            self.destroy_subscription(self.joy_sub)
            self.joy_sub = None

        if self.absenc_sub:
            self.destroy_subscription(self.absenc_sub)
            self.absenc_sub = None

        return super().on_cleanup(previous_state)

    def on_activate(self, previous_state: State) -> TransitionCallbackReturn:
        self.get_logger().info(
            f"LifecycleNode '{self.get_name()} is in previous_state '{previous_state.label}. Transitioning to 'activate'"
        )
        return super().on_activate(previous_state)

    def on_deactivate(self, previous_state: State) -> TransitionCallbackReturn:
        self.get_logger().info(
            f"LifecycleNode '{self.get_name()} is in previous_state '{previous_state.label}. Transitioning to 'deactivate'"
        )
        return super().on_deactivate(previous_state)

    def on_shutdown(self, previous_state: State) -> TransitionCallbackReturn:
        self.destroy_lifecycle_publisher(self.joint_pub)

        self.get_logger().info(
            f"LifecycleNode '{self.get_name()} is in previous_state '{previous_state.label}. Transitioning to 'shutdown'"
        )

        rclpy.shutdown()
        return TransitionCallbackReturn.SUCCESS

    def have_abs_angles(self):
        return len(self.abs_angles) > 0

    def initialize_angles_coords(self):
        absenc_angles = self.abs_angles
        self.angles = self.abs_angles
        self.get_logger().info(f"encoder angles: {absenc_angles}")

        # Start in cylindrical coords
        # The first angle swivel, it's the phi angle in the cylindrical coords system
        self.phi = absenc_angles[0]

        # This calculates u and v from the angles
        # The last three angles are flex, so add the displacement caused by these joints
        self.u, self.v = self.coords_from_flex(absenc_angles[1:])

        # The pitch is the angle of the gripper (wrt to the vertical)
        self.pitch = 0
        # accumulate all angles
        for i in absenc_angles:
            self.pitch += i
        # make pitch angle with respect to bottom
        self.pitch = math.pi - self.pitch

        # Turn to cartesian (stored in self.x, y, z)
        self.calculate_cartesian()
        self.get_logger().info(f"Initial coordinates {self.x} {self.y} {self.z}")

        # Move slightly toward the origin to prevent floating point error from causing angle 0.0 0.0 0.0 from triggering out of range
        if absenc_angles == [0.0, 0.0, 0.0, 0.0]:
            self.x -= math.copysign(1, self.x) * 0.0000000000000004
            self.y -= math.copysign(1, self.y) * 0.0000000000000004
            self.z -= math.copysign(1, self.z) * 0.0000000000000004

        self.initialized = True

    def absenc_callback(self, message):
        self.abs_angles = [
            0.0,
            math.radians(message.angle_1),
            math.radians(message.angle_2),
            math.radians(message.angle_3),
        ]

    def coords_from_flex(self, angles):
        # Finds the 2d coords from a series of flex joints (ie arms rotating)
        # The angles are only relative to the previous joint, cumulative_angle holds the cumulative
        # angle from the previous joints
        cumulative_angle = 0.0
        u, v = 0.0, 0.0
        for angle, length in zip(angles, self.lengths):
            cumulative_angle += angle
            u += length * math.sin(cumulative_angle)
            v += length * math.cos(cumulative_angle)
        return u, v

    def publish_joint_state(self):
        # If not initialized, initialize from abs enc values
        if not self.angles and self.abs_angles and not self.initialized:
            self.get_logger().warn("1 publish_joint_state")
            self.initialize_angles_coords()

        # If not initialized yet, don't publish
        if self.initialized:
            if not self.angles:
                self.get_logger().warn("Angles not initialized")
                return

            self.get_logger().warn("2 publish_joint_state")
            joint_state = JointState()

            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = [
                "Shoulder Swivel",
                "Shoulder Flex",
                "Elbow Flex",
                "Wrist Flex",
            ]
            joint_state.position = self.angles
            self.joint_pub.publish(joint_state)

    def calculate_angles(self):
        """Performs IK calculation and stores values in self.angles.
        Returns True if within range, False otherwise"""
        try:
            # cu and cv are the coordinates of the critical point
            cu = self.u - self.L3 * math.sin(self.pitch)
            cv = self.v + self.L3 * math.cos(self.pitch)

            if cu >= 0 and cv < 0:
                self.get_logger().warn(
                    f"Point (xyz) {self.x} {self.y} {self.z} out of range, cu {cu} cv {cv}"
                )
                return False

            # In this context, cu and cv are lengths, so must be positive
            a1 = math.atan(abs(cv / cu))
            a2 = math.atan(abs(cu / cv))

            L = math.sqrt((cu**2) + (cv**2))
            B1 = (self.L1**2 + L**2 - self.L2**2) / (2 * self.L1 * L)
            B2 = (self.L1**2 + self.L2**2 - L**2) / (2 * self.L1 * self.L2)
            B3 = (self.L2**2 + L**2 - self.L1**2) / (2 * self.L2 * L)
            if any_out_of_range(-1, 1, B1, B2, B3):
                self.get_logger().warn(
                    f"Point (xyz) {self.x} {self.y} {self.z} pitch {self.pitch} "
                    + f"(uvp) {self.u} {self.v} {self.phi} out of range (B1 B2 B3) {B1} {B2} {B3}"
                )
                return False

            b1 = math.acos(B1)
            b2 = math.acos(B2)
            b3 = math.acos(B3)

            # contains Shoulder Swivel, Shoulder Flex, Elbow Flex, Wrist Flex (in that order)
            solution0_angles = None
            solution1_angles = None

            if cu < 0 and cv > 0:  # second quadrant
                solution0_angles = [
                    float(self.phi),
                    -((math.pi / 2) - a1 + b1),
                    math.pi - b2,
                    math.pi - (self.pitch + b3 - a2),
                ]
                x = self.pitch - a2 - b3
                solution1_angles = [
                    float(self.phi),
                    -(math.pi / 2) + a1 + b1,
                    -(math.pi - b2),
                    math.pi - x,
                ]
            elif cv < 0 and cu <= 0:  # Third quadrant
                solution0_angles = None
                # solution0_angles = [float(self.phi), (math.pi) - (b1 + a2),
                #           math.pi - b2, math.pi / 2 - (self.pitch + b3 + a1)]
                x = b1 - a1
                y = a2 - b3
                a_3 = y - (2 * math.pi - self.pitch)

                solution1_angles = [
                    float(self.phi),
                    -(math.pi / 2 - x),
                    -(math.pi - b2),
                    -a_3,
                ]
            elif cu >= 0 and cv >= 0:  # first quadrant (fourth is not implemented)
                solution0_angles = [
                    float(self.phi),
                    (math.pi / 2) - (b1 + a1),
                    math.pi - b2,
                    math.pi - (self.pitch + a2 + b3),
                ]
                # If want to pick alternate solution
                a3 = a2 - b3
                solution1_angles = [
                    solution0_angles[0],
                    solution0_angles[1] + 2 * b1,
                    b2 - math.pi,
                    math.pi - (a3 + self.pitch),
                ]
            else:
                self.get_logger().warn(f"Fourth (ERROR)")

            # Try to use desired solution
            if self.solution == 0 and self.valid_angles(solution0_angles):
                self.angles = solution0_angles
            elif self.solution == 1 and self.valid_angles(solution1_angles):
                self.angles = solution1_angles
            # If can't use desired solution use any valid one
            elif self.valid_angles(solution0_angles):
                self.angles = solution0_angles
            elif self.valid_angles(solution1_angles):
                self.angles = solution1_angles
            else:
                self.get_logger().warn(
                    f"Outside joint limits, angles: {solution0_angles} {solution1_angles}"
                )
                return False

            # self.get_logger().info(f"angles: {self.angles}")
            return True

        except ValueError as e:
            self.get_logger().error(
                f"Caught error {e} with coordinates (x,y,z) {self.x} {self.y} {self.z} (u,v,phi,pitch) {self.u} {self.v} {self.phi} {self.pitch}"
            )

    def cad_joy_callback(self, message: Joy):
        self.get_logger().info(f"Received from cad mouse `{message}")
        old_values = (self.x, self.y, self.z, self.th, self.pitch)
        left_button, right_button = message.buttons
        x, y, z, pitch, roll, yaw = message.axes
        # If right button, moving end effector, stop these calculations
        if right_button == 1:
            return

        self.x += self.sensitivity * x / 30000  # left-right of mouse
        self.y += self.sensitivity * -y / 30000  # forward-back of mouse
        self.z += self.sensitivity * -z / 30000  # vertical axis of mouse
        self.th += (
            self.sensitivity * yaw / 150
        )  # use rotation axis that is activated by spinning the top

        # Pitch is the one angle which is directly set, so check bounds here
        new_pitch = self.pitch + (self.sensitivity * roll / 30000)
        self.pitch = new_pitch

        self.perform_calculations(old_values)

    def joy_callback(self, message: Joy):
        # For logitech joystick, moving wheels if button 3 is pressed; don't move arm
        if message.buttons[2] == 1:
            return

        # self.get_logger().info(f"Received from cad mouse")
        old_values = (self.x, self.y, self.z, self.th, self.pitch)
        x, y, spin, trim, dpad_x, dpad_y = message.axes
        trim = map_range(trim, -1.0, 1.0, 0.5, 3.0)

        # Move up and down with trigger and button 2
        up_down = message.buttons[0] - message.buttons[1]

        # Toggle between solutions with button 8 (index 7)
        if (
            self.last_message
            and self.last_message.buttons[7] != message.buttons[7]
            and message.buttons[7] == 1
        ):
            self.solution = 0 if self.solution == 1 else 1

        self.x += self.sensitivity * -y / 75  # forward-back of joystick moves along x
        self.y += (
            self.sensitivity * x / 75
        )  # left-right would move along y axis (doesn't since in 2D mode)
        self.z += (
            trim * self.sensitivity * -up_down / 75
        )  # trigger/button 2 moves up/down
        self.th += self.sensitivity * spin / 75  # spin of joystick (yaw) spins base

        # Pitch is the one angle which is directly set, so check bounds here
        new_pitch = self.pitch + (self.sensitivity * dpad_x / 50)
        self.pitch = new_pitch

        self.perform_calculations(old_values)

        self.last_message = message

    def perform_calculations(self, old_values):
        self.calculate_cylindical()

        # if 2d, force y to be 0
        if self.mode == "2D":
            self.y = 0
        # self.get_logger().info(f"Pitch: {self.pitch}")

        # Perform IK. If out of range, restore point where it was
        if not self.calculate_angles():
            self.x, self.y, self.z, self.th, self.pitch = old_values
        # self.get_logger().info(f"Current location: {self.x} {self.y} {self.z} roll {self.th} pitch {self.pitch}")
        # self.get_logger().info(f"Cylindical coordinates: u {self.u} v {self.v} phi {self.phi}")

    def calculate_cylindical(self):
        if self.mode == "2D":
            self.u = self.x
            self.v = self.z
            self.phi = 0
        else:
            self.u = math.sqrt(self.x**2 + self.y**2)
            self.v = self.z
            if self.x == 0:
                self.phi = 0
            else:
                if self.x >= 0:
                    self.phi = math.atan(self.y / self.x)
                else:
                    if self.y >= 0:
                        # In second quadrant
                        # domain issue: atan is limited to -pi/2 to pi/2 so it loops over when x < 0
                        self.phi = math.pi + math.atan(self.y / self.x)
                    else:
                        # In third quadrant
                        self.phi = -(math.pi - math.atan(self.y / self.x))

    def calculate_cartesian(self):
        # The v axis is the same as the z axis, the u is perpendicular to it
        self.z = self.v
        self.x = self.u * math.cos(self.phi)
        self.y = self.u * math.sin(self.phi)

    def valid_angles(self, angles):
        if angles == None or len(angles) != len(self.mins):
            return False

        for angle, min, max in zip(angles, self.mins, self.maxes):
            # Validate angle is between min and max
            if not (min <= angle and angle <= max):
                return False

        return True

    def setup_initial_joint_params(self):
        # Joint lengths
        lengths = (
            self.get_parameter("joint_lengths").get_parameter_value().double_array_value
        )
        self.L1 = lengths[0]
        self.L2 = lengths[1]
        self.L3 = lengths[2]
        # For convenience, store as array and as individual values
        self.lengths = lengths

        # Joint angle limits
        mins = (
            self.get_parameter("joint_angle_mins")
            .get_parameter_value()
            .double_array_value
        )
        maxes = (
            self.get_parameter("joint_angle_maxes")
            .get_parameter_value()
            .double_array_value
        )
        # Convert them to radians
        self.mins = [math.radians(x) for x in mins]
        self.maxes = [math.radians(x) for x in maxes]


def main(args=None):
    rclpy.init(args=args)

    ik_node = IkNode()

    rclpy.spin(ik_node)
    ik_node.destroy_node()

    rclpy.shutdown()
