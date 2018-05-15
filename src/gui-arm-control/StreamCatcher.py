class StreamCatcher:
	def __init__(self):
		self.current_fps = 0
		self.fps_cap = 30 # initial value to be decided during testing
		self.status = True # ON == true, OFF == false
		self.brightness = 10 # initial value to be decided during testing
		self.color = False # initial value to be decided during testing (whatever our camera defaults to)
		self.buffer_container = 0 # place holder value for now, will be some kind of VideoCapture object, probably from opencv module

	def set_resolution(self):
		# set a new resolution for the camera feed
		# if new resolution is acceptable, change resolution
		# otherwise return false
		return False
	
	def enable_stream(self):
		# try to enable stream

		# if succeed return true
		# else return false
		# return bool
		return False

	def disable_stream(self):
		# try to disable stream

		# if succeed return true
		# else return false
		return False

	def enable_color(self):
		# try to enable color
		# if successfull then set self.color = True
		# and return self.color (if it fails then return False)
		return False

	def disable_color(self):
		# try to disable color
		# if successfull then set self.color = False
		# then return self.color (if it fails then return False)
		return False

	def update_fps(self, new_fps_rate):
		# try to set fps cap to new value
		if new_fps_rate <= self.fps_cap:
			self.fps_cap = new_fps_rate
			return True # :)
		else:
			return False # :(

	def toggle_save_feed():
		if status is True:
			# start saving that good stuff
			return True
		else:
			print("quick deuces")
			return False
