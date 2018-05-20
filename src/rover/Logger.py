# all methods return false for now while they aren't fully implemented
class Logger:
	def __init__(self):
		self.log_file = "RoverLogs.txt" # consider renaming/path

	def info(self, message):
		# send message to base station of informational type
		return False

	def warn(self, message):
		# send message to base station of warning type
		return False

	def err(self, message):
		# send message to base station of error type
		return False

	def crit(self, message):
		# send message to base sation of critical type
		return False

	def shutdown(self):
		# shut down the logger (I forgot why we included this in design tbh)
		return False

	def run(self, connection):
		# pass a connection (other class/process) to listen to output from
		return False
