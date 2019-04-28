#!/usr/bin/python3
from . import MapObject, Shape, RectShape, CircleShape, MapObjectAction, MapObjectArgument
from .parser import Parser, ParsingException, ElementParser, AttrParser, ListContainer
from xml.etree import ElementTree
import rospkg

from typing import Dict, List

from action_manager import Argumentable

class ObjectsParser(Parser):
	def __init__(self):
		# Argument
		self.argument_parser = ElementParser(MapObjectArgument, "argument", "arguments", True) \
			.bind_attr("name", "", mandatory=True) \
			.bind_attr("type", "", mandatory=True) \
			.bind_attr("default", "")

		# Argument possible value
		self.argument_value_parser = ElementParser(dict, "value", "values", True) \
			.bind_attr("content", "", mandatory=True)


		# Rect shape
		self.rect_shape_parser = ElementParser(RectShape, "rect", "shape") \
			.bind_attr("x", -1, cast=int) \
			.bind_attr("y", -1, cast=int) \
			.bind_attr("width", -1, cast=int) \
			.bind_attr("height", -1, cast=int) \
			.bind_attr("blocking", False, cast=bool)

		# Circle shape
		self.circle_shape_parser = ElementParser(CircleShape, "circle", "shape") \
			.bind_attr("x", -1, cast=int) \
			.bind_attr("y", -1, cast=int) \
			.bind_attr("radius", -1, cast=int) \
			.bind_attr("blocking", False, cast=bool)


		# Map special object
		self.map_object_parser = ElementParser(MapObject, "obj", "objects", shorthand=True) \
			.bind_attr("name", "", mandatory=True) \
			.bind_attr("extends", None)

		"""# Offset (TODO: move to map parser)
		self.offset_parser = ElementParser(Offset, "offset") \
			.bind_attr("x", -1, cast=int) \
			.bind_attr("y", -1, cast=int)"""

		''' Children definition '''

		# Argument value and argument both have each other as child
		self.argument_parser.add_child(self.argument_value_parser)
		self.argument_value_parser.add_child(self.argument_parser)

		# Object
		self.map_object_parser.add_child(self.argument_parser)
		self.map_object_parser.add_child(self.rect_shape_parser)
		self.map_object_parser.add_child(self.circle_shape_parser)

		# Root parser
		root_parser = ListContainer.get_parser("objects")
		root_parser.add_child(self.map_object_parser)

		super().__init__(root_parser)
