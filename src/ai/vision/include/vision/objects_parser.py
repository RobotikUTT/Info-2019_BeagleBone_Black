#!/usr/bin/python3
from . import MapObject, Shape, RectShape, CircleShape, MapObjectAction, MapObjectArgument
from .parser import Parser, ParsingException
from xml.etree import ElementTree
import rospkg

from typing import Dict, List

from action_manager import Argumentable

class ObjectsParser(Parser):
	def __init__(self):
		self.objects: Dict[str, MapObject] = {}

	def parse(self, root: ElementTree.Element):
		if root.tag != "objects":
			print("invalid file, must contains a <objects> root element")
			exit(0)

		# Objects to parse later
		later = []
		now = root
		changed = True

		# Parse various objects while there is something to parse
		while len(now) > 0 and changed:
			changed = False

			# Try to parse all objects
			for child in now:
				# If parsing not succesful
				if not self.parse_object(child):
					# Try again later
					later.append(child)
				else:
					changed = True			

			# Switch position
			now = later
			later = []

	def parse_object(self, element: ElementTree.Element) -> bool:
		"""
			Parse an object from its XML element.
			Returns whether the object has been able to be parsed
		"""
		obj: MapObject = MapObject()
		obj.name = element.tag

		# If this object inherit another
		if "extends" in element.attrib:
			# If parent already declared
			if element.attrib["extends"] in self.objects:
				# Extends that object
				obj.extends(self.objects[element.attrib["extends"]])
			else:
				# Unable to parse now
				return False
		
		# Then parse all children
		for child in element:
			if child.tag == "action":
				self.parse_action(child, obj)
			elif child.tag == "argument":
				self.parse_argument(child, obj)
			else:
				# Parse shape
				obj.shape = Shape.parse(child, obj.shape)

		# Append object to list
		self.objects[obj.name] = obj
		return True

	def parse_action(self, element: ElementTree.Element, obj: MapObject):
		"""
			Parse an action associated with an object.
		"""

		if "file" in element.attrib:
			action = MapObjectAction(element.attrib["file"])

			# Then args bound to this value
			for bound_arg in element:
				action.set(bound_arg.tag, bound_arg.text)

			obj.actions.append(action)
		else:
			raise ParsingException("no file attribute given to an {}'s action".format(obj.name))

	def parse_argument(self, element: ElementTree.Element, obj: MapObject):
		"""
			Parse an argument that have to be passed when creating the object.

			It might contains somes values for this argument, with attributes
			that will be passed upon calling an action.
		"""

		# Check that argument is valid
		if "name" not in element.attrib or "type" not in element.attrib:
			raise ParsingException("an object argument must have a given name and a type")
		elif element.attrib["type"] not in ["string", "int", "float"]:
			raise ParsingException("invalid argument type : {}".format(element.attrib["type"]))
		
		argument = MapObjectArgument(element.attrib["name"], element.attrib["type"])

		# Parse given arguments
		for argument_value in element:
			bound_arguments = argument.add_value(argument_value.tag)

			# Then args bound to this value
			for bound_arg in argument_value:
				bound_arguments.set(bound_arg.tag, bound_arg.text)
		
		obj.args[element.attrib["name"]] = argument
		

