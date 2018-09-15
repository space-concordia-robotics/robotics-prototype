# only needed for testing (see test at the end of this file)
#import random
#import gi
#gi.require_version('Gtk', '3.0')
#from gi.repository import Gtk, Gdk

# needed for Armposition
import random
from math import sin, cos, pi
from matplotlib.figure import Figure
from matplotlib.backends.backend_gtk3cairo import FigureCanvasGTK3Cairo as FigureCanvas

class ArmPosition:
	def __init__(self, scatterPoints):
		self.fig = Figure(dpi=50)
		self.ax = self.fig.add_subplot(111)
		self.canvas = FigureCanvas(self.fig)
		self.canvas.set_size_request(400,337)
		self.scatterPoints = scatterPoints

	def setVariables(self):
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

		self.jointsX = [joint1x, joint2x, joint3x]
		self.jointsY = [joint1y, joint2y, joint3y]
		self.armsX = [arm1x, arm2x, arm3x, arm4x]
		self.armsY = [arm1y, arm2y, arm3y, arm4y]

		x = [0] * self.scatterPoints # initialize some bigass arrays...slowly
		y = [0] * self.scatterPoints # initialize some bigass arrays...slowly
		for i in range(0, self.scatterPoints):
			t2 = random.uniform(0,adif) + amin
			t3 = random.uniform(0,adif2) + amin2
			t4 = random.uniform(0,adif3) + amin3
			x[i] = l1 * cos(t2) + l2 * cos(t2 + t3) + l3 * cos(t2 + t3 + t4)
			y[i] = d1 + l1 * sin(t2) + l2 * sin(t2 + t3) + l3 * sin(t2 + t3 + t4)
		self.ax.scatter(x, y , color='b', s=7, alpha=0.5)

	def positionArm(self):
		self.ax.plot(self.jointsX[0], self.jointsY[0], 'ko', markersize=7) # the base & segment 1
		self.ax.plot(self.jointsX[1], self.jointsY[1], 'ko', markersize=7) # segment 1 & 2 joint
		self.ax.plot(self.jointsX[2], self.jointsY[2], 'ko', markersize=7) # segment 2 & 3 joint
		self.ax.plot(self.armsX[0], self.armsY[0],'k', linewidth=3) # the base
		self.ax.plot(self.armsX[1], self.armsY[1], 'k', linewidth=3) # segement 1
		self.ax.plot(self.armsX[2], self.armsY[2], 'k', linewidth= 3); # segement 2
		self.ax.plot(self.armsX[3], self.armsY[3], 'k', linewidth=3); # segement 3
		return

	def getCanvas(self):
		return self.canvas

	def getScatterPoints(self):
		return self.scatterPoints

# test

#if __name__ == "__main__":
#	builder = Gtk.Builder()
#	builder.add_from_file("ArmGuiLayout2.glade")

#	armPos = ArmPosition(600) # set scatter points to 600
#	armPos.setVariables()
#	armPos.positionArm()

#	placeholder = builder.get_object("Arm Position Placeholder")
#	placeholder.add_with_viewport(armPos.getCanvas())

#	window = builder.get_object("ArmGuiLayoutWindow")
	# sw = Gtk.ScrolledWindow()
	# window.add(sw)
	# sw.add_with_viewport(canvas)
#	window.set_title("Asimov Operation")
#	window.connect("destroy", Gtk.main_quit)
#	window.show_all()
	#asimov_op.toggleBtnTest()
#	Gtk.main()
