# This uses Gtk 3, so make sure that is installed before proceeding.
import gi
import datetime # time stamps for test console logs
import random # for generating random values in motor table
from ArmPosition import ArmPosition

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

		textbuffer = textarea.get_buffer()
		textbuffer.set_text(textinput.read())
		textinput.close()

	def clearButton(self, button):
		textbuffer = textarea.get_buffer()
		textbuffer.set_text("")

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

	def pitchUpButtonClick(self, button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Pitch Up Clicked " + str(now) + "\n")

	def yawRightButtonClick(self, button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Yaw Right Clicked " + str(now) + "\n")

	def rollLeftButtonClick(self, button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Roll Left Clicked " + str(now) + "\n")

	def pitchDownButtonClick(self, button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Pitch Down Clicked " + str(now) + "\n")

	def rollRightButtonClick(self, button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Roll Right Clicked " + str(now) + "\n")

	def clawOpenButtonClick(self, button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Claw Open Clicked " + str(now) + "\n")

	def clawCloseButtonClick(self, button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Claw Close Clicked " + str(now) + "\n")

	def armUpButtonClick(self, button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Arm Up Clicked " + str(now) + "\n")

	def armLeftButtonClick(self, button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Arm Left Clicked " + str(now) + "\n")

	def armDownButtonClick(self, button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Arm Down Clicked " + str(now) + "\n")

	def armRightButtonClick(self, button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Arm Right Clicked " + str(now) + "\n")

	def armBackButtonClick(self, button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Arm Back Clicked " + str(now) + "\n")

	def armFwdButtonClick(self, button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Arm Fwd Clicked " + str(now) + "\n")

	def toggleSwitchClick(self, button):
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> onn")

	def homePostionBtnClicked(self, button):
		'''
		print("The text is: " + str(text))
		#text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		'''

		# For the the motor position the home value is set to 10 (is it done when in manual or auto)
		text_buffer = smotor1.get_buffer()
		text_buffer.set_text("10")

		text_buffer = smotor2.get_buffer()
		text_buffer.set_text("10")

		text_buffer = smotor3.get_buffer()
		text_buffer.set_text("10")

		text_buffer = smotor4.get_buffer()
		text_buffer.set_text("10")

		text_buffer = asmotor1.get_buffer()
		text_buffer.set_text("10")

		text_buffer = asmotor2.get_buffer()
		text_buffer.set_text("10")

	def stm1ButtonLeftClicked(self, button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = smotor1.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) - 1))

	def stm1ButtonRightClicked(self, button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = smotor1.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) + 1))

	def stm2ButtonLeftClicked(self, button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = smotor2.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) - 1))

	def stm2ButtonRightClicked(self, button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = smotor2.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) + 1))

	def stm3ButtonLeftClicked(self, button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = smotor3.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) - 1))

	def stm3ButtonRightClicked(self, button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = smotor3.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) + 1))

	def stm4ButtonLeftClicked(self, button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = smotor4.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) - 1))

	def stm4ButtonRightClicked(self, button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = smotor4.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) + 1))

	def astm1ButtonLeftClicked(self, button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = asmotor1.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) - 1))

	def astm1ButtonRightClicked(self, button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = asmotor1.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) + 1))

	def astm2ButtonLeftClicked(self, button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = asmotor2.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) - 1))

	def astm2ButtonRightClicked(self, button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = asmotor2.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) + 1))

	def toggleBtnTest(self):
		# fill each of the displacement boxes with random numbers between 10 and 50
		rand = random.randint(10, 71)
		text_buffer = smotor1.get_buffer()
		text_buffer.set_text(str(rand))

		rand = random.randint(10, 71)
		text_buffer = smotor2.get_buffer()
		text_buffer.set_text(str(rand))

		rand = random.randint(10, 71)
		text_buffer = smotor3.get_buffer()
		text_buffer.set_text(str(rand))

		rand = random.randint(10, 71)
		text_buffer = smotor4.get_buffer()
		text_buffer.set_text(str(rand))

		rand = random.randint(10, 71)
		text_buffer = asmotor1.get_buffer()
		text_buffer.set_text(str(rand))

		rand = random.randint(10, 71)
		text_buffer = asmotor2.get_buffer()
		text_buffer.set_text(str(rand))

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
			rollLeftBtn.disconnect_by_func(handlers["onRollLeftClicked"])
			handlers["onRollLeftClicked"] = asimov_op.stm2ButtonLeftClicked
			rollLeftBtn.connect("clicked", handlers["onRollLeftClicked"])

			rollRightBtn.disconnect_by_func(handlers["onRollRightClicked"])
			handlers["onRollRightClicked"] = asimov_op.stm2ButtonRightClicked
			rollRightBtn.connect("clicked", handlers["onRollRightClicked"])

			clawOpenBtn.disconnect_by_func(handlers["onClawOpenClicked"])
			handlers["onClawOpenClicked"] = asimov_op.stm3ButtonLeftClicked
			clawOpenBtn.connect("clicked", handlers["onClawOpenClicked"])

			clawCloseBtn.disconnect_by_func(handlers["onClawCloseClicked"])
			handlers["onClawCloseClicked"] = asimov_op.stm3ButtonRightClicked
			clawCloseBtn.connect("clicked", handlers["onClawCloseClicked"])


			pitchUpBtn.disconnect_by_func(handlers["onPitchUpClicked"])
			handlers["onPitchUpClicked"] = asimov_op.stm1ButtonLeftClicked
			pitchUpBtn.connect("clicked", handlers["onPitchUpClicked"])

			pitchDownBtn.disconnect_by_func(handlers["onPitchDownClicked"])
			handlers["onPitchDownClicked"] = asimov_op.stm1ButtonRightClicked
			pitchDownBtn.connect("clicked", handlers["onPitchDownClicked"])
			# -----------------------------------
			armUpBtn.disconnect_by_func(handlers["onArmUpClicked"])
			handlers["onArmUpClicked"] = asimov_op.stm4ButtonLeftClicked
			armUpBtn.connect("clicked", handlers["onArmUpClicked"])

			armDownBtn.disconnect_by_func(handlers["onArmDownClicked"])
			handlers["onArmDownClicked"] = asimov_op.stm4ButtonRightClicked
			armDownBtn.connect("clicked", handlers["onArmDownClicked"])
			# -----------------------------------

			armLeftBtn.disconnect_by_func(handlers["onArmLeftClicked"])
			handlers["onArmLeftClicked"] = asimov_op.astm1ButtonLeftClicked
			armLeftBtn.connect("clicked", handlers["onArmLeftClicked"])

			armRightBtn.disconnect_by_func(handlers["onArmRightClicked"])
			handlers["onArmRightClicked"] = asimov_op.astm1ButtonRightClicked
			armRightBtn.connect("clicked", handlers["onArmRightClicked"])

			armBackBtn.disconnect_by_func(handlers["onArmBackClicked"])
			handlers["onArmBackClicked"] = asimov_op.astm2ButtonLeftClicked
			armBackBtn.connect("clicked", handlers["onArmBackClicked"])

			armFwdBtn.disconnect_by_func(handlers["onArmFwdClicked"])
			handlers["onArmFwdClicked"] = asimov_op.astm2ButtonRightClicked
			armFwdBtn.connect("clicked", handlers["onArmFwdClicked"])
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

			'''Gtk.Widget.set_name(armUpBtn, "Arm-Up")
			Gtk.Widget.set_name(armDownBtn, "Arm-Down")
			Gtk.Widget.set_sensitive(armUpBtn, True)
			Gtk.Widget.set_sensitive(armDownBtn, True)'''

			Gtk.Button.set_label(armUpBtn, "Arm\nUp")
			Gtk.Button.set_label(armDownBtn, "Arm\nDown")

			# Remove manual buttons switch handler code
			rollLeftBtn.disconnect_by_func(handlers["onRollLeftClicked"])
			handlers["onRollLeftClicked"] = asimov_op.rollLeftButtonClick
			rollLeftBtn.connect("clicked", handlers["onRollLeftClicked"])

			rollRightBtn.disconnect_by_func(handlers["onRollRightClicked"])
			handlers["onRollRightClicked"] = asimov_op.rollRightButtonClick
			rollRightBtn.connect("clicked", handlers["onRollRightClicked"])

			clawOpenBtn.disconnect_by_func(handlers["onClawOpenClicked"])
			handlers["onClawOpenClicked"] = asimov_op.clawOpenButtonClick
			clawOpenBtn.connect("clicked", handlers["onClawOpenClicked"])

			clawCloseBtn.disconnect_by_func(handlers["onClawCloseClicked"])
			handlers["onClawCloseClicked"] = asimov_op.clawCloseButtonClick
			clawCloseBtn.connect("clicked", handlers["onClawCloseClicked"])


			pitchUpBtn.disconnect_by_func(handlers["onPitchUpClicked"])
			handlers["onPitchUpClicked"] = asimov_op.pitchUpButtonClick
			pitchUpBtn.connect("clicked", handlers["onPitchUpClicked"])

			pitchDownBtn.disconnect_by_func(handlers["onPitchDownClicked"])
			handlers["onPitchDownClicked"] = asimov_op.pitchDownButtonClick
			pitchDownBtn.connect("clicked", handlers["onPitchDownClicked"])
			# -----------------------------------
			armUpBtn.disconnect_by_func(handlers["onArmUpClicked"])
			handlers["onArmUpClicked"] = asimov_op.armUpButtonClick
			armUpBtn.connect("clicked", handlers["onArmUpClicked"])

			armDownBtn.disconnect_by_func(handlers["onArmDownClicked"])
			handlers["onArmDownClicked"] = asimov_op.armDownButtonClick
			armDownBtn.connect("clicked", handlers["onArmDownClicked"])
			# -----------------------------------



			armLeftBtn.disconnect_by_func(handlers["onArmLeftClicked"])
			handlers["onArmLeftClicked"] = asimov_op.armLeftButtonClick
			armLeftBtn.connect("clicked", handlers["onArmLeftClicked"])

			armRightBtn.disconnect_by_func(handlers["onArmRightClicked"])
			handlers["onArmRightClicked"] = asimov_op.armRightButtonClick
			armRightBtn.connect("clicked", handlers["onArmRightClicked"])

			armBackBtn.disconnect_by_func(handlers["onArmBackClicked"])
			handlers["onArmBackClicked"] = asimov_op.armBackButtonClick
			armBackBtn.connect("clicked", handlers["onArmBackClicked"])

			armFwdBtn.disconnect_by_func(handlers["onArmFwdClicked"])
			handlers["onArmFwdClicked"] = asimov_op.armFwdButtonClick
			armFwdBtn.connect("clicked", handlers["onArmFwdClicked"])

		#print("Switch was turned", state)

# Need to close file, but program won't run if I include it
# textinput.close()
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


	#Get a reference to motor table text boxes
	smotor1 = builder.get_object("Stepper Motor 1 Angle")
	smotor2 = builder.get_object("Stepper Motor 2 Angle")
	smotor3 = builder.get_object("Stepper Motor 3 Angle")
	smotor4 = builder.get_object("Stepper Motor 4 Angle")
	asmotor1 = builder.get_object("Arm Servo Motor 1 Angle")
	asmotor2 = builder.get_object("Arm Servo Motor 2 Angle")


	#Get reference to button so we can update the label when switched to manual
	switch = builder.get_object("man-auto-switch")
	switch.connect("state-set", AsimovOperation.on_switch_activated)
	switch.set_active(False)

	# This is used to connect signals sent by the widgets in Glade
	# and assign them functionality using python code
	handlers = {
		"onTestClick": asimov_op.testButton,
		"onClearClick": asimov_op.clearButton,
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
	###########################End Matlab Stuff


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
