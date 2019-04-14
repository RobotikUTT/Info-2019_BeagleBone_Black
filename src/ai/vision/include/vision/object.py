from typing import List, Dict

from action_manager import Argumentable
from copy import copy

class MapObjectAction(Argumentable):
	def __init__(self, file: str):
		super().__init__()

		self.file = file

	def __str__(self):
		return "Action ({}) {}".format(self.file, super().__str__())


class MapObjectArgument:
	def __init__(self, name: str, type: str):
		super().__init__()

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
	
	def __str__(self):
		return "<{}: {}> {}" \
			.format(self.name, self.type,
			 	", ".join(["{} ({})".format(a, self.bound[a].__str__()) for a in self.bound])
			)


class MapObject(Argumentable):
	"""
		Define an object on the field, with associated action
		to deal with and some properties
	"""
	def __init__(self):
		self.name = ""
		self.shape = None
		self.actions: List[MapObjectAction] = []
		self.args: Dict[str, MapObjectArgument] = {}
	
	def extends(self, obj: 'MapObject'):
		# Make a copy of the shape
		self.shape = copy(obj.shape)
		
		# TODO clone args
		self.args = copy(obj.args)
	
	def __str__(self):
		return "\n\t".join(
				["[{}] {}".format(self.name, self.shape)] +
				[a.__str__() for a in self.actions] +
				[self.args[a].__str__() for a in self.args]
			)


