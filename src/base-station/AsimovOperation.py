# This uses Gtk 3, so make sure that is installed before proceeding.
import gi
import datetime # time stamps for test console logs
import random # for generating random values in motor table
from ArmPosition import ArmPosition
import os
import platform
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

	# Test button for error log. Will remove once error log can
	# read errors from other windows
	def testButton(self, button):
		textinput = open("ErrorLogTest.txt", "r")
		self.send_to_console(textinput.read())
		textinput.close()

	def clearButton(self, button):
		textbuffer = textarea.get_buffer()
		textbuffer.set_text("")

	def openLogButton(self, button):
		# windows
		#subprocess.Popen(r'explorer /select,"C:\path\of\folder\file"')
		# linux
		if platform.system() == 'Linux':
			os.system("xdg-open '%s'" % "ErrorLogTest.txt")

	# definitions of the handlers for the buttons
	def yawLeftButtonClick(self, button):
		if switch:
			state2 = "on"
		else:
			state2 = "off"

		print("The state is " + state2)

		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Yaw Left Clicked " + now.strftime("%Y-%m-%d %H:%M") + "\n")

	def send_to_console(self, label):
		label = label + "\n" # pass by reference ftw
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, label + str(now.strftime("%Y-%m-%d %H:%M:%S")) + "\n")

	def pitchUpButtonClick(self, button):
		self.send_to_console("> Pitch Up")

	def yawRightButtonClick(self, button):
		self.send_to_console("> Yaw Right")

	def rollLeftButtonClick(self, button):
		self.send_to_console("> Roll Left")

	def pitchDownButtonClick(self, button):
		self.send_to_console("> Pitch Down")

	def rollRightButtonClick(self, button):
		self.send_to_console("> Roll Right")

	def clawOpenButtonClick(self, button):
		self.send_to_console("> Claw Open")

	def clawCloseButtonClick(self, button):
		self.send_to_console("> Claw Close")

	def armUpButtonClick(self, button):
		self.send_to_console("> Arm Up")

	def armLeftButtonClick(self, button):
		self.send_to_console("> Arm Left")

	def armDownButtonClick(self, button):
		self.send_to_console("> Arm Down")

	def armRightButtonClick(self, button):
		self.send_to_console("> Arm Right")

	def armBackButtonClick(self, button):
		self.send_to_console("> Arm Back")

	def armFwdButtonClick(self, button):
		self.send_to_console("> Arm Fwd")

	def toggleSwitchClick(self, button):
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> onn")
#		global state

#		if state is "on":
#		self.send_to_console("> On")
#		else:
#			self.send_to_console("> Off")

	def homePostionBtnClicked(self, button):
		'''
		print("The text is: " + str(text))
		#text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		'''

		# For the the motor position the home value is set to 10 (is it done when in manual or auto)
		buffers = [smotor1.get_buffer(), smotor2.get_buffer(), smotor3.get_buffer(), smotor4.get_buffer(), asmotor1.get_buffer(), asmotor2.get_buffer()]
		for text_buffer in buffers:
			text_buffer.set_text("10")

	# Get value in motor postion text box, change it an then re-enter it
	def alter_angle(self, button, motor, delta):
		text_buffer = motor.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) + delta))

	def stm1ButtonLeftClicked(self, button):
		self.alter_angle(button, smotor1, -1)

	def stm1ButtonRightClicked(self, button):
		self.alter_angle(button, smotor1, +1)

	def stm2ButtonLeftClicked(self, button):
		self.alter_angle(button, smotor2, -1)

	def stm2ButtonRightClicked(self, button):
		self.alter_angle(button, smotor2, +1)

	def stm3ButtonLeftClicked(self, button):
		self.alter_angle(button, smotor3, -1)

	def stm3ButtonRightClicked(self, button):
		self.alter_angle(button, smotor3, +1)

	def stm4ButtonLeftClicked(self, button):
		self.alter_angle(button, smotor4, -1)

	def stm4ButtonRightClicked(self, button):
		self.alter_angle(button, smotor4, +1)

	def astm1ButtonLeftClicked(self, button):
		self.alter_angle(button, asmotor1, -1)

	def astm1ButtonRightClicked(self, button):
		self.alter_angle(button, asmotor1, +1)

	def astm2ButtonLeftClicked(self, button):
		self.alter_angle(button, asmotor2, -1)

	def astm2ButtonRightClicked(self, button):
		self.alter_angle(button, asmotor2, +1)

	def toggleBtnTest(self):
		# fill each of the displacement boxes with random numbers between and including 10 and 71
		buffers = [smotor1.get_buffer(), smotor2.get_buffer(), smotor3.get_buffer(), smotor4.get_buffer(), asmotor1.get_buffer(), asmotor2.get_buffer()]

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

		#Get references to buttons that are then changed on switch
		#yawLeftBtn = builder.get_object("Yaw Leftgit1")
		#yawRightBtn = builder.get_object("Yaw Right1")
		rollLeftBtn = builder.get_object("Roll Left1")
		rollRightBtn = builder.get_object("Roll Right1")
		clawOpenBtn = builder.get_object("Claw Open1")
		clawCloseBtn = builder.get_object("Claw Close1")
		pitchUpBtn = builder.get_object("Pitch Up1")
		pitchDownBtn = builder.get_object("Pitch Down1")
		armLeftBtn = builder.get_object("Arm Left1")
		armRightBtn = builder.get_object("Arm Right1")
		armBackBtn = builder.get_object("Arm Back1")
		armFwdBtn = builder.get_object("Arm Forward1")
		armUpBtn = builder.get_object("Arm Up1")
		armDownBtn = builder.get_object("Arm Down1")

		if sw:
			state = "on"
			Gtk.Button.set_label(rollLeftBtn, "SMT2\n <<")
			Gtk.Button.set_label(rollRightBtn, "SMT2\n >>")
			Gtk.Button.set_label(clawOpenBtn, "SMT3\n <<")
			Gtk.Button.set_label(clawCloseBtn, "SMT3\n >>")
			Gtk.Button.set_label(pitchUpBtn, "SMT1\n <<")
			Gtk.Button.set_label(pitchDownBtn, "SMT1\n >>")
			Gtk.Button.set_label(armLeftBtn, "ASM1\n <<")
			Gtk.Button.set_label(armRightBtn, "ASM1\n >>")
			Gtk.Button.set_label(armBackBtn, "ASM2\n <<")
			Gtk.Button.set_label(armFwdBtn, "ASM2\n >>")

			# Change the view of buttons that the manual control don't use
			#Gtk.Widget.set_name(armUpBtn, "Arm-Up-Hide")
			#Gtk.Widget.set_name(armDownBtn, "Arm-Down-Hide")

			Gtk.Button.set_label(armUpBtn, "SMT4\n <<")
			Gtk.Button.set_label(armDownBtn, "SMT4\n >>")

			#Gtk.Widget.set_sensitive(armUpBtn, False)
			#Gtk.Widget.set_sensitive(armDownBtn, False)

			# make manual buttons switch handler code
			asimov_op.switch_controls(rollLeftBtn, "clicked", "onRollLeftClicked", asimov_op.stm2ButtonLeftClicked)
			asimov_op.switch_controls(rollRightBtn, "clicked", "onRollRightClicked", asimov_op.stm2ButtonRightClicked)
			asimov_op.switch_controls(clawOpenBtn, "clicked", "onClawOpenClicked", asimov_op.stm3ButtonLeftClicked)
			asimov_op.switch_controls(clawCloseBtn, "clicked", "onClawCloseClicked", asimov_op.stm3ButtonRightClicked)
			asimov_op.switch_controls(pitchUpBtn, "clicked", "onPitchUpClicked", asimov_op.stm1ButtonLeftClicked)
			asimov_op.switch_controls(pitchDownBtn, "clicked", "onPitchDownClicked", asimov_op.stm1ButtonRightClicked)
			asimov_op.switch_controls(armUpBtn, "clicked", "onArmUpClicked", asimov_op.stm4ButtonLeftClicked)
			asimov_op.switch_controls(armDownBtn, "clicked", "onArmDownClicked", asimov_op.stm4ButtonRightClicked)
			asimov_op.switch_controls(armLeftBtn, "clicked", "onArmLeftClicked", asimov_op.astm1ButtonLeftClicked)
			asimov_op.switch_controls(armRightBtn, "clicked", "onArmRightClicked", asimov_op.astm1ButtonRightClicked)
			asimov_op.switch_controls(armBackBtn, "clicked", "onArmBackClicked", asimov_op.astm2ButtonLeftClicked)
			asimov_op.switch_controls(armFwdBtn, "clicked", "onArmFwdClicked", asimov_op.astm2ButtonRightClicked)

		else:
			state = "off"
			Gtk.Button.set_label(rollLeftBtn, "Roll\nLeft")
			Gtk.Button.set_label(rollRightBtn, "Roll\nRight")
			Gtk.Button.set_label(clawOpenBtn, "Claw\nOpen")
			Gtk.Button.set_label(clawCloseBtn, "Claw\nClose")
			Gtk.Button.set_label(pitchUpBtn, "Pitch\nUp")
			Gtk.Button.set_label(pitchDownBtn, "Pitch\nDown")
			Gtk.Button.set_label(armLeftBtn, "Arm\nLeft")
			Gtk.Button.set_label(armRightBtn, "Arm\nRight")
			Gtk.Button.set_label(armBackBtn, "Arm\nBack")
			Gtk.Button.set_label(armFwdBtn, "Arm\nFwd")
			Gtk.Button.set_label(armUpBtn, "Arm\nUp")
			Gtk.Button.set_label(armDownBtn, "Arm\nDown")

			# Remove manual buttons switch handler code
			asimov_op.switch_controls(rollLeftBtn, "clicked", "onRollLeftClicked", asimov_op.rollLeftButtonClick)
			asimov_op.switch_controls(rollRightBtn, "clicked", "onRollRightClicked", asimov_op.rollRightButtonClick)
			asimov_op.switch_controls(clawOpenBtn, "clicked", "onClawOpenClicked", asimov_op.clawOpenButtonClick)
			asimov_op.switch_controls(clawOpenBtn, "clicked", "onClawOpenClicked", asimov_op.clawOpenButtonClick)
			asimov_op.switch_controls(clawCloseBtn, "clicked", "onClawCloseClicked", asimov_op.clawCloseButtonClick)
			asimov_op.switch_controls(pitchUpBtn, "clicked", "onPitchUpClicked", asimov_op.pitchUpButtonClick)
			asimov_op.switch_controls(pitchDownBtn, "clicked", "onPitchDownClicked", asimov_op.pitchDownButtonClick)
			asimov_op.switch_controls(armUpBtn, "clicked", "onArmUpClicked", asimov_op.armUpButtonClick)
			asimov_op.switch_controls(armDownBtn, "clicked", "onArmDownClicked", asimov_op.armDownButtonClick)
			asimov_op.switch_controls(armLeftBtn, "clicked", "onArmLeftClicked", asimov_op.armLeftButtonClick)
			asimov_op.switch_controls(armRightBtn, "clicked", "onArmRightClicked", asimov_op.armRightButtonClick)
			asimov_op.switch_controls(armBackBtn, "clicked", "onArmBackClicked", asimov_op.armBackButtonClick)
			asimov_op.switch_controls(armFwdBtn, "clicked", "onArmFwdClicked", asimov_op.armFwdButtonClick)

		#print("Switch was turned", state)

if __name__ == "__main__":
	asimov_op = AsimovOperation()

	# CSS Styling part to make GUI look better
	screen = Gdk.Screen.get_default()
	css_provider = Gtk.CssProvider()
	css_provider.load_from_path('Styling.css')
	context = Gtk.StyleContext()
	context.add_provider_for_screen(screen, css_provider, Gtk.STYLE_PROVIDER_PRIORITY_USER)
	# End of CSS

	builder = Gtk.Builder()
	builder.add_from_file("ArmGuiLayout2.glade")

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
		"onTestClick": asimov_op.testButton,
		"onClearClick": asimov_op.clearButton,
		"onOpenLogClick": asimov_op.openLogButton,
		"onYawLeftClicked": asimov_op.yawLeftButtonClick,
		"onPitchUpClicked": asimov_op.pitchUpButtonClick,
		"onYawRightClicked": asimov_op.yawRightButtonClick,
		"onRollLeftClicked": asimov_op.rollLeftButtonClick,
		"onPitchDownClicked": asimov_op.pitchDownButtonClick,
		"onRollRightClicked": asimov_op.rollRightButtonClick,
		"onClawOpenClicked": asimov_op.clawOpenButtonClick,
		"onClawCloseClicked": asimov_op.clawCloseButtonClick,
		"onArmUpClicked": asimov_op.armUpButtonClick,
		"onArmLeftClicked": asimov_op.armLeftButtonClick,
		"onArmDownClicked": asimov_op.armDownButtonClick,
		"onArmRightClicked":asimov_op.armRightButtonClick,
		"onArmBackClicked": asimov_op.armBackButtonClick,
		"onArmFwdClicked": asimov_op.armFwdButtonClick,
		"onHomePostionClicked": asimov_op.homePostionBtnClicked
	}

	builder.connect_signals(handlers)

	armPos = ArmPosition(600) # 600 scatterpoints
	armPos.setVariables()
	armPos.positionArm()

	# add arm positon panel
	placeholder = builder.get_object("Arm Position Placeholder")
	placeholder.add_with_viewport(armPos.getCanvas())

	# boiler plate gtk stuff
	window = builder.get_object("ArmGuiLayoutWindow")
	# sw = Gtk.ScrolledWindow()
	# window.add(sw)
	# sw.add_with_viewport(canvas)
	window.set_title("Asimov Operation")
	window.connect("destroy", Gtk.main_quit)
	window.show_all()
	asimov_op.toggleBtnTest()
	Gtk.main()
