from . import Shape

from typing import List, Dict

from action_manager import Argumentable
from copy import copy, deepcopy

class MapObjectAction(Argumentable):
	"""
		Action bound to an object on the field.
		The robot might choose to proceed with this action if
		it is a viable option.

		Some action bound to the same object can be incompatible
		(example: move an object to 2 different places)
	"""
	def __init__(self, file: str):
		super().__init__()

		self.file = file

	def __str__(self):
		return "Action ({}) {}".format(self.file, super().__str__())


class MapObjectArgument:
	"""
		Argument that can be given to an generic object.

		Example : a ball object is defined in objects.xml, but it might
		have different parameters depending on it's color, so an argument
		color is created.
	"""
	def __init__(self, name: str, type: str):
		self.name = name
		self.type = type
		self.bound: Dict[str, Argumentable] = {}
	
	def add_value(self, value: str) -> Argumentable:
		# Try to cast to handle errors
		if self.type == "int":
			int(value)
		elif self.type == "float":
			float(value)
		
		self.bound[value] = Argumentable()

		return self.bound[value]
	
	def get_value(self, name: str) -> Argumentable:
		return self.bound[name]

	def __str__(self):
		return "<{}: {}> {}" \
			.format(self.name, self.type,
			 	", ".join(["{}".format(a) for a in self.bound])
			 	#", ".join(["{} ({})".format(a, self.bound[a].__str__()) for a in self.bound])
			)


class MapObject(Argumentable):
	"""
		Define an object on the field, with associated action
		to deal with and some properties
	"""
	def __init__(self):
		super().__init__()

		self.name = ""
		self.shape: Shape = Shape()
		self.actions: List[MapObjectAction] = []
		self.args: Dict[str, MapObjectArgument] = {}
	
	def extends(self, obj: 'MapObject'):
		# Make a copy of the shape and args
		self.shape = copy(obj.shape)
		self.args = deepcopy(obj.args)
	
	def clone(self):
		return deepcopy(self)

	def __str__(self):
		return "\n\t".join(
			["[{}] {} {}".format(self.name, self.shape, super().__str__())] +
			[a.__str__() for a in self.actions] +
			[self.args[a].__str__() for a in self.args]
		)


