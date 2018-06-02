# This uses Gtk 3, so make sure that is installed before proceeding.
import gi
import datetime  # time stamps for test console logs
import random  # for generating random values in motor table
from ArmPosition import ArmPosition  # arm position model rendering
import os  # to perform linux os operations
import platform  # to determine what os we're running on
import subprocess  # to perform windows/mac os operations
from Microcontroller import Microcontroller
from Motor import Motor
from Port import Port
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk


'''
This will access and run the glade file
If your glade file is not in the same directory as this python file,
then you must specify the directory in the
builder.add_from_file("suhdude"), where suhdude is the name of the glade file, including relative path to it.

The glade file is currently using textboxes as placeholders
for the windows that will be implemented in the future.
'''


class AsimovOperation(Gtk.Window):
    # static/class attributes go here

    def __init__(self):
        # intialize any object instance variables here
        Gtk.Window.__init__(self, title="Asimov Operation")
        # max_angle, min_angle, max_current, min_current, home_angle
        max_angle = 180
        min_angle = 0
        max_current = 3
        min_current = 0
        home_angle = 0

        # initialize a Port instance used to connect to arduino via serial
        port = Port(path="/dev/tty.usbmodem1411", baudrate=9600, timeout=1)
        # initialize Motor class object instance variables to map to various motor button objects
        self.smotor1 = Motor("Stepper Motor 1 Angle", max_angle,
                             min_angle, max_current, min_current, home_angle)
        self.smotor2 = Motor(
            "Stepper Motor 2", max_angle, min_angle, max_current, min_current, home_angle)
        self.smotor3 = Motor(
            "Stepper Motor 3", max_angle, min_angle, max_current, min_current, home_angle)
        self.smotor4 = Motor(
            "Stepper Motor 4", max_angle, min_angle, max_current, min_current, home_angle)
        self.asmotor1 = Motor(
            "Arm Servo Motor 1", max_angle, min_angle, max_current, min_current, home_angle)
        self.asmotor2 = Motor(
            "Arm Servo Motor 2", max_angle, min_angle, max_current, min_current, home_angle)
        # intialize Microcontroller object representing the mother Arduino object containing Motor instance array
        self.arduino = Microcontroller(
            "Arduino", port, [self.smotor1, self.smotor2, self.smotor3, self.smotor4, self.asmotor1, self.asmotor2])

    # utility function for sending appending text into console
    def send_to_console(self, label):
        label = label + "\n"  # pass by reference ftw
        now = datetime.datetime.now()
        text_buffer = textarea.get_buffer()
        end_iter = text_buffer.get_end_iter()
        text_buffer.insert(end_iter, label +
                           str(now.strftime("%Y-%m-%d %H:%M:%S")) + "\n")

    # Test button for error log. Will remove once error log can
    # read errors from other windows
    def test_btn(self, button):
        textinput = open("ErrorLogTest.txt", "r")
        self.send_to_console(textinput.read())
        textinput.close()

    def clear_btn(self, button):
        textbuffer = textarea.get_buffer()
        textbuffer.set_text("")

    def open_log_btn(self, button):
        os_name = platform.system()
        file_name = "ErrorLogTest.txt"

        # windows
        if os_name == 'Windows':
            subprocess.Popen(r'explorer /select,' + file_name)
        # linux
        elif os_name == 'Linux':
            os.system("xdg-open '%s'" % file_name)
        # macOS
        elif os_name == 'Darwin':
            subprocess.call(['open', file_name])

    # definitions of the handlers for the buttons
    def pitch_up_clicked(self, button):
        self.send_to_console("> Pitch Up")

    def pitch_down_clicked(self, button):
        self.send_to_console("> Pitch Down")

    def roll_left_clicked(self, button):
        self.send_to_console("> Roll Left")

    def roll_right_clicked(self, button):
        self.send_to_console("> Roll Right")

    def claw_open_clicked(self, button):
        self.send_to_console("> Claw Open")

    def claw_close_clicked(self, button):
        self.send_to_console("> Claw Close")

    def arm_up_clicked(self, button):
        self.send_to_console("> Arm Up")

    def arm_down_clicked(self, button):
        self.send_to_console("> Arm Down")

    def arm_left_clicked(self, button):
        self.send_to_console("> Arm Left")

    def arm_right_clicked(self, button):
        self.send_to_console("> Arm Right")

    def arm_back_clicked(self, button):
        self.send_to_console("> Arm Back")

    def arm_fwd_clicked(self, button):
        self.send_to_console("> Arm Fwd")

    # not linked to anything currently
    def toggle_switch_clicked(self, button):
        text_buffer = textarea.get_buffer()
        end_iter = text_buffer.get_end_iter()
        text_buffer.insert(end_iter, "> onn")

    def home_position_clicked(self, button):
        # For the the motor position the home value is set to 10 (is it done when in manual or auto)
        buffers = [smotor1.get_buffer(), smotor2.get_buffer(), smotor3.get_buffer(
        ), smotor4.get_buffer(), asmotor1.get_buffer(), asmotor2.get_buffer()]
        for text_buffer in buffers:
            text_buffer.set_text("10")

    # Get value in motor postion text box, change it an then re-enter it
    def alter_angle(self, button, motor, delta):
        text_buffer = motor.get_buffer()
        text = text_buffer.get_text(
            text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
        text_buffer.set_text(str(int(text) + delta))
        # upon receiving button event trigger, send command to Microcontroller using `write` method
        # using the GTKObject `motor` object `get_name` method (the names won't be exactly the same,
        # but a substring of one another)
        self.arduino.write(motor.get_name(), text)

    def st_m1_left_clicked(self, button):
        self.alter_angle(button, smotor1, -1)

    def st_m1_right_clicked(self, button):
        self.alter_angle(button, smotor1, +1)

    def dc_m2_left_clicked(self, button):
        self.alter_angle(button, smotor2, -1)

    def dc_m2_right_clicked(self, button):
        self.alter_angle(button, smotor2, +1)

    def st_m3_left_clicked(self, button):
        self.alter_angle(button, smotor3, -1)

    def st_m3_right_clicked(self, button):
        self.alter_angle(button, smotor3, +1)

    def st_m4_left_clicked(self, button):
        self.alter_angle(button, smotor4, -1)

    def st_m4_right_clicked(self, button):
        self.alter_angle(button, smotor4, +1)

    def as_m1_left_clicked(self, button):
        self.alter_angle(button, asmotor1, -1)

    def as_m1_right_clicked(self, button):
        self.alter_angle(button, asmotor1, +1)

    def as_m2_left_clicked(self, button):
        self.alter_angle(button, asmotor2, -1)

    def as_m2_right_clicked(self, button):
        self.alter_angle(button, asmotor2, +1)

    def init_rand_angles(self):
        # fill each of the displacement boxes with random numbers between and including 10 and 71
        buffers = [smotor1.get_buffer(), smotor2.get_buffer(), smotor3.get_buffer(
        ), smotor4.get_buffer(), asmotor1.get_buffer(), asmotor2.get_buffer()]

        for text_buffer in buffers:
            rand = random.randint(10, 71)
            text_buffer.set_text(str(rand))

    # switches call back functions for buttons from manual to auto
    @staticmethod
    def switch_controls(button, event, handler_name, callback_func):
        button.disconnect_by_func(handlers[handler_name])
        handlers[handler_name] = callback_func
        button.connect(event, handlers[handler_name])

    @staticmethod
    def on_switch_activated(self, sw):
        global state

        clawCtn = builder.get_object("Claw Button Layout1")

        # Get references to buttons that are then changed on switch
        roll_left_btn = builder.get_object("Roll Left1")
        roll_right_btn = builder.get_object("Roll Right1")
        claw_open_btn = builder.get_object("Claw Open1")
        claw_close_btn = builder.get_object("Claw Close1")
        pitch_up_btn = builder.get_object("Pitch Up1")
        pitch_down_btn = builder.get_object("Pitch Down1")
        arm_up_btn = builder.get_object("Arm Up1")
        arm_down_btn = builder.get_object("Arm Down1")
        arm_left_btn = builder.get_object("Arm Left1")
        arm_right_btn = builder.get_object("Arm Right1")
        arm_back_btn = builder.get_object("Arm Back1")
        arm_fwd_btn = builder.get_object("Arm Forward1")

        # it's important that the order and size of these arrays correspond for the following loops to function properly
        buttons = [roll_left_btn, roll_right_btn, claw_open_btn, claw_close_btn, pitch_up_btn,
                   pitch_down_btn, arm_up_btn, arm_down_btn, arm_left_btn, arm_right_btn, arm_back_btn, arm_fwd_btn]
        automatic_labels = ["Roll\nLeft", "Roll\nRight", "Claw\nOpen", "Claw\nClose", "Pitch\nUp",
                            "Pitch\nDown", "Arm\nUp", "Arm\nDown", "Arm\nLeft", "Arm\nRight", "Arm\nBack", "Arm\nFwd"]
        manual_labels = ["DCM2\n <<", "DCM2\n >>", "STM3\n <<", "STM3\n >>", "STM1\n <<",
                         "STM1\n >>", "ASM1\n <<", "ASM1\n >>", "ASM2\n <<", "ASM2\n >>", "STM4\n <<", "STM4\n >>"]

        automatic_event_names = ["onRollLeftClicked", "onRollRightClicked", "onClawOpenClicked", "onClawCloseClicked", "onPitchUpClicked",
                                 "onPitchDownClicked", "onArmUpClicked", "onArmDownClicked", "onArmLeftClicked", "onArmRightClicked", "onArmBackClicked", "onArmFwdClicked"]
        automatic_callbacks = [asimov_op.roll_left_clicked, asimov_op.roll_right_clicked, asimov_op.claw_open_clicked, asimov_op.claw_close_clicked, asimov_op.pitch_up_clicked,
                               asimov_op.pitch_down_clicked, asimov_op.arm_up_clicked, asimov_op.arm_down_clicked, asimov_op.arm_left_clicked, asimov_op.arm_right_clicked, asimov_op.arm_back_clicked, asimov_op.arm_fwd_clicked]
        manual_callbacks = [asimov_op.dc_m2_left_clicked, asimov_op.dc_m2_right_clicked, asimov_op.st_m3_left_clicked, asimov_op.st_m3_right_clicked, asimov_op.st_m1_left_clicked, asimov_op.st_m1_right_clicked,
                            asimov_op.st_m4_left_clicked, asimov_op.st_m4_right_clicked, asimov_op.as_m1_left_clicked, asimov_op.as_m1_right_clicked, asimov_op.as_m2_left_clicked, asimov_op.as_m2_right_clicked]

        if sw:
            state = "on"

            for button, label in zip(buttons, manual_labels):
                Gtk.Button.set_label(button, label)

            # make manual buttons switch handler code

            for button, event_name, event_callback in zip(buttons, automatic_event_names, manual_callbacks):
                asimov_op.switch_controls(
                    button, "clicked", event_name, event_callback)

        else:
            state = "off"

            for button, label in zip(buttons, automatic_labels):
                Gtk.Button.set_label(button, label)

            # Remove manual buttons switch handler code
            for button, event_name, event_callback in zip(buttons, automatic_event_names, automatic_callbacks):
                asimov_op.switch_controls(
                    button, "clicked", event_name, event_callback)


if __name__ == "__main__":
    asimov_op = AsimovOperation()

    # CSS Styling part to make GUI look better
    screen = Gdk.Screen.get_default()
    css_provider = Gtk.CssProvider()
    # css_provider.load_from_path('Styling.css')
    context = Gtk.StyleContext()
    context.add_provider_for_screen(
        screen, css_provider, Gtk.STYLE_PROVIDER_PRIORITY_USER)
    # End of CSS

    builder = Gtk.Builder()
    builder.add_from_file("Layout.glade")

    textarea = builder.get_object("Error Log ")

    # Get a reference to motor table text boxes
    smotor1 = builder.get_object("Stepper Motor 1 Angle")
    smotor2 = builder.get_object("Stepper Motor 2 Angle")
    smotor3 = builder.get_object("Stepper Motor 3 Angle")
    smotor4 = builder.get_object("Stepper Motor 4 Angle")
    asmotor1 = builder.get_object("Arm Servo Motor 1 Angle")
    asmotor2 = builder.get_object("Arm Servo Motor 2 Angle")

    # Get reference to button so we can update the label when switched to manual
    switch = builder.get_object("man-auto-switch")
    switch.connect("state-set", AsimovOperation.on_switch_activated)
    switch.set_active(False)

    # This is used to connect signals sent by the widgets in Glade
    # and assign them functionality using python code
    handlers = {
        "onTestClick": asimov_op.test_btn,
        "onClearClick": asimov_op.clear_btn,
        "onOpenLogClick": asimov_op.open_log_btn,
        "onPitchUpClicked": asimov_op.pitch_up_clicked,
        "onRollLeftClicked": asimov_op.roll_left_clicked,
        "onPitchDownClicked": asimov_op.pitch_down_clicked,
        "onRollRightClicked": asimov_op.roll_right_clicked,
        "onClawOpenClicked": asimov_op.claw_open_clicked,
        "onClawCloseClicked": asimov_op.claw_close_clicked,
        "onArmUpClicked": asimov_op.arm_up_clicked,
        "onArmLeftClicked": asimov_op.arm_left_clicked,
        "onArmDownClicked": asimov_op.arm_down_clicked,
        "onArmRightClicked": asimov_op.arm_right_clicked,
        "onArmBackClicked": asimov_op.arm_back_clicked,
        "onArmFwdClicked": asimov_op.arm_fwd_clicked,
        "onHomePostionClicked": asimov_op.home_position_clicked,
        "on": asimov_op.CREATE-A-FUNC
    }

    builder.connect_signals(handlers)

    armPos = ArmPosition(600)  # 600 scatterpoints
    armPos.setVariables()
    armPos.positionArm()

    # add arm positon panel
    placeholder = builder.get_object("Arm Position Placeholder")
    # placeholder.add_with_viewport(armPos.getCanvas())

    # boiler plate gtk stuff
    window = builder.get_object("ArmGuiLayoutWindow")
    # sw = Gtk.ScrolledWindow()
    # window.add(sw)
    # sw.add_with_viewport(canvas)
    window.set_title("Asimov Operation")
    window.connect("destroy", Gtk.main_quit)
    window.show_all()
    asimov_op.init_rand_angles()
    Gtk.main()
