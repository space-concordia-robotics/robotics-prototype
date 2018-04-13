# This uses Gtk 3, so make sure that is installed before proceeding.
import gi
import datetime
import random

gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk

from math import sin, cos, pi
from matplotlib.figure import Figure
from matplotlib.backends.backend_gtk3cairo import FigureCanvasGTK3Cairo as FigureCanvas
'''
This will access and run the glade file
If your glade file is not in the same directory as this python file,
then you must specify the directory in the
builder.add_from_file("xxx"), where xxx is your directory.

The glade file is currently using textboxes as placeholders
for the windows that will be implemented in the future.
'''

class AsimovOperation(Gtk.Window):
	def __init__(self):
		Gtk.Window.__init__(self, title="Asimov Operation")

	# Test button for error log. Will remove once error log can
	# read errors from other windows
	def testButton(button):
		textinput = open("ErrorLogTest.txt", "r")

		textbuffer = textarea.get_buffer()
		textbuffer.set_text(textinput.read())
		textinput.close()

	def clearButton(button):
		textbuffer = textarea.get_buffer()
		textbuffer.set_text("")

	# definitions of the handlers for the buttons
	def yawLeftButtonClick(button):
		if switch:
			state2 = "on"
		else:
			state2 = "off"

		print("The state is " + state2)

		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Yaw Left Clicked " + now.strftime("%Y-%m-%d %H:%M") + "\n")

	def picthUpButtonClick(button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Pitch Up Clicked " + str(now) + "\n")

	def yawRightButtonClick(button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Yaw Right Clicked " + str(now) + "\n")

	def rollLeftButtonClick(button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Roll Left Clicked " + str(now) + "\n")

	def pitchDownButtonClick(button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Pitch Down Clicked " + str(now) + "\n")

	def rollRightButtonClick(button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Roll Right Clicked " + str(now) + "\n")

	def clawOpenButtonClick(button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Claw Open Clicked " + str(now) + "\n")

	def clawCloseButtonClick(button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Claw Close Clicked " + str(now) + "\n")

	def armUpButtonClick(button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Arm Up Clicked " + str(now) + "\n")

	def armLeftButtonClick(button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Arm Left Clicked " + str(now) + "\n")

	def armDownButtonClick(button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Arm Down Clicked " + str(now) + "\n")

	def armRightButtonClick(button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Arm Right Clicked " + str(now) + "\n")

	def armBackButtonClick(button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Arm Back Clicked " + str(now) + "\n")

	def armFwdButtonClick(button):
		now = datetime.datetime.now()
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> Arm Fwd Clicked " + str(now) + "\n")

	def toggleSwitchClick(button):
		text_buffer = textarea.get_buffer()
		end_iter = text_buffer.get_end_iter()
		text_buffer.insert(end_iter, "> onn")

	def homePostionBtnClicked(button):
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

	def stm1ButtonLeftClicked(button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = smotor1.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) - 1))

	def stm1ButtonRightClicked(button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = smotor1.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) + 1))

	def stm2ButtonLeftClicked(button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = smotor2.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) - 1))

	def stm2ButtonRightClicked(button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = smotor2.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) + 1))

	def stm3ButtonLeftClicked(button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = smotor3.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) - 1))

	def stm3ButtonRightClicked(button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = smotor3.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) + 1))

	def stm4ButtonLeftClicked(button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = smotor4.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) - 1))

	def stm4ButtonRightClicked(button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = smotor4.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) + 1))

	def astm1ButtonLeftClicked(button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = asmotor1.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) - 1))

	def astm1ButtonRightClicked(button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = asmotor1.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) + 1))

	def astm2ButtonLeftClicked(button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = asmotor2.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) - 1))

	def astm2ButtonRightClicked(button):
		# Get value in motor postion text box, change it an then re-enter it
		text_buffer = asmotor2.get_buffer()
		text = text_buffer.get_text(text_buffer.get_start_iter(), text_buffer.get_end_iter(), False)
		text_buffer.set_text(str(int(text) + 1))

	@staticmethod
	def toggleBtnTest():
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
			handlers["onRollLeftClicked"] = stm2ButtonLeftClicked
			rollLeftBtn.connect("clicked", handlers["onRollLeftClicked"])

			rollRightBtn.disconnect_by_func(handlers["onRollRightClicked"])
			handlers["onRollRightClicked"] = stm2ButtonRightClicked
			rollRightBtn.connect("clicked", handlers["onRollRightClicked"])

			clawOpenBtn.disconnect_by_func(handlers["onClawOpenClicked"])
			handlers["onClawOpenClicked"] = stm3ButtonLeftClicked
			clawOpenBtn.connect("clicked", handlers["onClawOpenClicked"])

			clawCloseBtn.disconnect_by_func(handlers["onClawCloseClicked"])
			handlers["onClawCloseClicked"] = stm3ButtonRightClicked
			clawCloseBtn.connect("clicked", handlers["onClawCloseClicked"])


			pitchUpBtn.disconnect_by_func(handlers["onPitchUpClicked"])
			handlers["onPitchUpClicked"] = stm1ButtonLeftClicked
			pitchUpBtn.connect("clicked", handlers["onPitchUpClicked"])

			pitchDownBtn.disconnect_by_func(handlers["onPitchDownClicked"])
			handlers["onPitchDownClicked"] = stm1ButtonRightClicked
			pitchDownBtn.connect("clicked", handlers["onPitchDownClicked"])
			# -----------------------------------
			armUpBtn.disconnect_by_func(handlers["onArmUpClicked"])
			handlers["onArmUpClicked"] = stm4ButtonLeftClicked
			armUpBtn.connect("clicked", handlers["onArmUpClicked"])

			armDownBtn.disconnect_by_func(handlers["onArmDownClicked"])
			handlers["onArmDownClicked"] = stm4ButtonRightClicked
			armDownBtn.connect("clicked", handlers["onArmDownClicked"])
			# -----------------------------------

			armLeftBtn.disconnect_by_func(handlers["onArmLeftClicked"])
			handlers["onArmLeftClicked"] = astm1ButtonLeftClicked
			armLeftBtn.connect("clicked", handlers["onArmLeftClicked"])

			armRightBtn.disconnect_by_func(handlers["onArmRightClicked"])
			handlers["onArmRightClicked"] = astm1ButtonRightClicked
			armRightBtn.connect("clicked", handlers["onArmRightClicked"])

			armBackBtn.disconnect_by_func(handlers["onArmBackClicked"])
			handlers["onArmBackClicked"] = astm2ButtonLeftClicked
			armBackBtn.connect("clicked", handlers["onArmBackClicked"])

			armFwdBtn.disconnect_by_func(handlers["onArmFwdClicked"])
			handlers["onArmFwdClicked"] = astm2ButtonRightClicked
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
			handlers["onRollLeftClicked"] = rollLeftButtonClick
			rollLeftBtn.connect("clicked", handlers["onRollLeftClicked"])

			rollRightBtn.disconnect_by_func(handlers["onRollRightClicked"])
			handlers["onRollRightClicked"] = rollRightButtonClick
			rollRightBtn.connect("clicked", handlers["onRollRightClicked"])

			clawOpenBtn.disconnect_by_func(handlers["onClawOpenClicked"])
			handlers["onClawOpenClicked"] = clawOpenButtonClick
			clawOpenBtn.connect("clicked", handlers["onClawOpenClicked"])

			clawCloseBtn.disconnect_by_func(handlers["onClawCloseClicked"])
			handlers["onClawCloseClicked"] = clawCloseButtonClick
			clawCloseBtn.connect("clicked", handlers["onClawCloseClicked"])


			pitchUpBtn.disconnect_by_func(handlers["onPitchUpClicked"])
			handlers["onPitchUpClicked"] = picthUpButtonClick
			pitchUpBtn.connect("clicked", handlers["onPitchUpClicked"])

			pitchDownBtn.disconnect_by_func(handlers["onPitchDownClicked"])
			handlers["onPitchDownClicked"] = pitchDownButtonClick
			pitchDownBtn.connect("clicked", handlers["onPitchDownClicked"])
			# -----------------------------------
			armUpBtn.disconnect_by_func(handlers["onArmUpClicked"])
			handlers["onArmUpClicked"] = armUpButtonClick
			armUpBtn.connect("clicked", handlers["onArmUpClicked"])

			armDownBtn.disconnect_by_func(handlers["onArmDownClicked"])
			handlers["onArmDownClicked"] = armDownButtonClick
			armDownBtn.connect("clicked", handlers["onArmDownClicked"])
			# -----------------------------------



			armLeftBtn.disconnect_by_func(handlers["onArmLeftClicked"])
			handlers["onArmLeftClicked"] = armLeftButtonClick
			armLeftBtn.connect("clicked", handlers["onArmLeftClicked"])

			armRightBtn.disconnect_by_func(handlers["onArmRightClicked"])
			handlers["onArmRightClicked"] = armRightButtonClick
			armRightBtn.connect("clicked", handlers["onArmRightClicked"])

			armBackBtn.disconnect_by_func(handlers["onArmBackClicked"])
			handlers["onArmBackClicked"] = armBackButtonClick
			armBackBtn.connect("clicked", handlers["onArmBackClicked"])

			armFwdBtn.disconnect_by_func(handlers["onArmFwdClicked"])
			handlers["onArmFwdClicked"] = armFwdButtonClick
			armFwdBtn.connect("clicked", handlers["onArmFwdClicked"])

		#print("Switch was turned", state)

	# Some helper functions
	@staticmethod
	def armpos(jointsX, jointsY, armsX, armsY):
		ax.plot( jointsX[0] , jointsY[0] , 'ko' , markersize=7) # the base & segment 1
		ax.plot( jointsX[1] , jointsY[1] , 'ko' , markersize=7) # segment 1 & 2 joint
		ax.plot( jointsX[2] , jointsY[2] , 'ko' , markersize=7) # segment 2 & 3 joint
		ax.plot( armsX[0] , armsY[0] ,'k' , linewidth=3) # the base
		ax.plot( armsX[1] , armsY[1] , 'k' , linewidth=3) # segement 1
		ax.plot( armsX[2] , armsY[2] , 'k' , linewidth= 3); # segement 2
		ax.plot( armsX[3] , armsY[3] , 'k' , linewidth=3); # segement 3
		return
	@staticmethod
	def scatterPoints(numberOfScatterPoints):
		x = [0] * numberOfScatterPoints # initialize some bigass arrays...slowly
		y = [0] * numberOfScatterPoints # initialize some bigass arrays...slowly
		for i in range(0,numberOfScatterPoints):
			t2 = random.uniform(0,adif) + amin
			t3 = random.uniform(0,adif2) + amin2
			t4 = random.uniform(0,adif3) + amin3
			x[i] = l1 * cos(t2) + l2 * cos(t2 + t3) + l3 * cos(t2 + t3 + t4)
			y[i] = d1 + l1 * sin(t2) + l2 * sin(t2 + t3) + l3 * sin(t2 + t3 + t4)
		ax.scatter(x,y,color='b',s=7,alpha=0.5)
		return

# Need to close file, but program won't run if I include it
# textinput.close()
if __name__ == "__main__":

	test = AsimovOperation()

	# Global Variables
	fig = Figure(dpi=50)
	ax = fig.add_subplot(111)
	canvas = FigureCanvas(fig)
	canvas.set_size_request(400,337)

	# CSS Styling part to make GUI look better
	screen = Gdk.Screen.get_default()
	css_provider = Gtk.CssProvider()
	css_provider.load_from_path('Styling.css')
	context = Gtk.StyleContext()
	context.add_provider_for_screen(screen, css_provider,
	Gtk.STYLE_PROVIDER_PRIORITY_USER)
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
	switch.connect("state-set", test.on_switch_activated)
	switch.set_active(False)

	# This is used to connect signals sent by the widgets in Glade
	# and assign them functionality using python code
	handlers = {
		"onTestClick": test.testButton,
		"onClearClick": test.clearButton,
		"onYawLeftClicked": test.yawLeftButtonClick,
		"onPitchUpClicked": test.picthUpButtonClick,
		"onYawRightClicked": test.yawRightButtonClick,
		"onRollLeftClicked": test.rollLeftButtonClick,
		"onPitchDownClicked": test.pitchDownButtonClick,
		"onRollRightClicked": test.rollRightButtonClick,
		"onClawOpenClicked": test.clawOpenButtonClick,
		"onClawCloseClicked": test.clawCloseButtonClick,
		"onArmUpClicked": test.armUpButtonClick,
		"onArmLeftClicked": test.armLeftButtonClick,
		"onArmDownClicked": test.armDownButtonClick,
		"onArmRightClicked": test.armRightButtonClick,
		"onArmBackClicked": test.armBackButtonClick,
		"onArmFwdClicked": test.armFwdButtonClick,
		"onHomePostionClicked": test.homePostionBtnClicked


	}
	builder.connect_signals(handlers)

		########################## Matlab ARM Pos ##########################
	# http://gtk3-matplotlib-cookbook.readthedocs.io/en/latest/hello-plot.html
	# The following code's comments have been removed for brevity
	# See armWorkSpace.py for detailed comments/instructions
	# Contstant Variables
	d1 = 0.5
	l1 = 1
	l2 = 1
	l3 = 0.25
	amax = (4 * pi) / 6
	amin = pi / 6
	amax2 = pi / 2
	amin2 = -(4 * pi / 6)
	amax3 = pi / 2
	amin3 = -2 * pi / 3
	aa = (amax + amin) / 2
	aa2 = (amax2 + amin2) / 2
	aa3 = (amax3 + amin3) / 2
	adif = amax - amin
	adif2 = amax2 - amin2
	adif3 = amax3 - amin3
	joint1x = 0
	joint1y = d1
	joint2x = l1*cos(aa)
	joint2y = d1+l1*sin(aa)
	joint3x = l1*cos(aa)+l2*cos(aa+aa2)
	joint3y = d1+l1*sin(aa)+l2*sin(aa+aa2)
	arm1x = [0 , 0];
	arm1y = [0 , d1];
	arm2x = [0 , l1*cos(aa)];
	arm2y = [d1 , d1+l1*sin(aa)];
	arm3x = [l1*cos(aa) , l1*cos(aa)+l2*cos(aa+aa2)]
	arm3y = [d1+l1*sin(aa) , d1+l1*sin(aa)+l2*sin(aa+aa2)]
	arm4x = [l1*cos(aa)+l2*cos(aa+aa2) , l1*cos(aa)+l2*cos(aa+aa2)+l3*cos(aa+aa2+aa3)]
	arm4y = [d1+l1*sin(aa)+l2*sin(aa+aa2) , d1+l1*sin(aa)+l2*sin(aa+aa2)+l3*sin(aa+aa2+aa3)]

	########################## Matlab ARM Pos

	jointsX = [joint1x, joint2x, joint3x]
	jointsY = [joint1y, joint2y, joint3y]
	armsX = [arm1x, arm2x, arm3x, arm4x]
	armsY = [arm1y, arm2y, arm3y, arm4y]

	AsimovOperation.scatterPoints(600)
	AsimovOperation.armpos(jointsX,jointsY,armsX,armsY)

	placeholder = builder.get_object("Arm Position Placeholder")
	placeholder.add_with_viewport(canvas)

	###########################End Matlab Stuff


	# boiler plate gtk stuff
	window = builder.get_object("ArmGuiLayoutWindow")
	# sw = Gtk.ScrolledWindow()
	# window.add(sw)
	# sw.add_with_viewport(canvas)
	window.set_title("Space Concordia Robtotics GUI")
	window.connect("destroy", Gtk.main_quit)
	window.show_all()
	AsimovOperation.toggleBtnTest()
	Gtk.main()
