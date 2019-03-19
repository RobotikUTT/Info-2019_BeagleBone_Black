from input_simulation.devices import DEVICES

class CanFrameHandler():
	def __init__(self, devicename):
		self.devinename = devicename
		self.device = DEVICES[devicename]
	
