from typing import List, Any

class Values:
	"""Definition of a range of possible value"""
	def __init__(self, *values: List[Any], cast: type = str):
		self.values = values