class SensorDataReader:
	def __init__(self):
		self.motor_angles = [0, 0, 0, 0, 0, 0] # this actually only assumes for the 6 motors for the arm, will need to assess how many motors in total including the rover
		self.motor_currents = [0, 0, 0, 0, 0, 0] # same as motor_angles

	def read(self):
		# get all the motor angles and currents
		# consider implementing different versions of this method for different types of connections
		# such as i2c, serial, etc.
		return False # should return a string prepared in the expected format for the logger to send to the base station through the server connection
