from .shape import Circle, Rect

from typing import List, Dict

from args_lib.argumentable import Argumentable
from copy import copy, deepcopy

from xml_class_parser import Parsable, Bind, BindDict, Context, Slice, ParsingException
from xml_class_parser.helper import AttrAttrTuple

@Parsable(
	name=Bind(to="name"),
	attributes = {
		"extends": str,
		"x": int,
		"y": int
	},
	children = [
		Bind(to="shape", type=Circle),
		Bind(to="shape", type=Rect),
		BindDict(to="values", type=AttrAttrTuple("arg", "name", "type"), key="name", post_cast=Slice("value"))
	]
)
class MapObject(Argumentable):
	"""
		Define an object on the field, with associated action
		to deal with and some properties
	"""
	def __init__(self):
		super().__init__()

		# Object's position
		self.x = 0
		self.y = 0

		self.name = ""
		self.shape: Union[Circle, Rect] = None
		self.extends = None
	
	def __before_children__(self, context: Context):
		# If this object extends another
		if self.extends is not None:
			if self.extends not in context.parent.objects:
				raise ParsingException(
					"object {} is not parsed yet and cannot be used for {}".format(self.extends, self.extends))

			obj = context.parent.objects[self.extends]

			# Make a copy of the shape and args
			self.x = obj.x
			self.y = obj.y
			self.shape = copy(obj.shape)
			self.values = deepcopy(obj.values) # argumentable values

	def clone(self):
		return deepcopy(self)

	def __str__(self):
		return "[{}] x={} y={} shape={} {{{}}}\n".format(self.name, self.x, self.y, self.shape, super().__str__())
		

@Parsable(
	name="objects",
	children = [
		BindDict(key="name", type=MapObject, to="objects")
	]
)
class MapObjectList:
	def __init__(self):
		self.objects = {}