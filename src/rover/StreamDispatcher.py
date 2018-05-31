# for now methods that aren't fully implemented return False
class StreamDispatcher:
	def __init__(self):
		self.current_fps = 25 # set default value
		self.fps_cap = 30 # this should be a upper limit constant based of experimentation/known physical limits
		self.status = False # ON == true, OFF == false
		self.brightness = 10 # decide on default value via experimentation
		self.color = False # turn off color by default

	def start_udp_stream():
		return False

	def stop_udp_stream():
		return False
