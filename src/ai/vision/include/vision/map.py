from . import Shape

from typing import Union, List

class Offset:
	def __init__(self, x = 0, y = 0):
		self.x: int = x
		self.y: int = y

class Symmetry:
	pass

class Zone:
	"""
		Define a zone in which some actions might take place.
		
		They are created in order to be provided as action args
	"""
	def __init__(self, shape: Shape, name: str):
		self.shape = shape
		self.name: str = name
		self.symmetry: Union[Symmetry, None] = None
		self.zones: List[Zone] = []
	
	def __str__(self):
		return  "\n\t".join(
			["<{}> {}".format(self.name, self.shape)] +
			("\n".join([z.__str__() for z in self.zones]).split("\n") if len(self.zones) > 0 else [])
		)

