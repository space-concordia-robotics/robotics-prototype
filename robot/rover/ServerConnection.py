class ServerConnection:
	def __init__(self):
		self.status = False
		# next four values TBD
		self.base_ip = ""
		self.base_port = ""
		self.rover_ip = ""
		self.rover_port = ""

	def ping_test(self):
		# ping the base station
		return False

	def send_motor_currents(self):
		# send all motor currents in expected format
		return False

	def send_motor_positions(self):
		# send all motor positions in expected format
		return False

	def send_logs(self):
		# send all logs to base station
		return False

	def drive_motors(self):
		# send angle/command received from base station to move respective motors
