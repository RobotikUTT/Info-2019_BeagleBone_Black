#!/usr/bin/python3
from . import MapObject, Shape, RectShape, CircleShape
from xml.etree import ElementTree
import rospkg

from typing import Dict, List

class ObjectsParser():
	def __init__(self):
		self.objects: Dict[str, MapObject] = {}

		# Find mapping file directory
		source_folder = rospkg.RosPack().get_path("ai_description")

		# Parsing mapping file
		root = ElementTree.parse(source_folder + "/objects/objects.xml").getroot()

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
				self.parse_shape(child, obj)

		# Append object to list
		self.objects[obj.name] = obj
		return True

	def parse_action(self, element: ElementTree.Element, obj: MapObject):
		pass

	def parse_shape(self, element: ElementTree.Element, obj: MapObject):
		"""
			Parse the shape of an object.

			Currently 2 shapes are supported : rects and circles. The
			function keep the curent shape (if defined, propbable shape
			from the parent obejct), and add properties given in XML
		"""
		# Make sure the right shape class is used
		if obj.shape == None:
			obj.shape = Shape()

		if element.tag == "rect" and type(obj.shape) is not RectShape:
			# Keep x, y
			obj.shape = RectShape(obj.shape.x, obj.shape.y)
		elif element.tag == "circle" and type(obj.shape) is not CircleShape:
			# Keep x, y
			obj.shape = CircleShape(obj.shape.x, obj.shape.y)

		elif element.tag != "circle" and element.tag != "rect":
			# No shape nor anything else
			raise Exception(
				"{} does not name an object parameter type"
					.format(element.tag)
			)

		# Then parse shape attributes
		for attr in element.attrib:
			if hasattr(obj.shape, attr):
				# Copy value as int (raise error if wrong)
				setattr(obj.shape, attr, int(element.attrib[attr]))
			else:
				raise Exception(
					"{} attribute not defined for a {} shape".format(attr, element.tag)
				)

	def parse_argument(self, element: ElementTree.Element, obj: MapObject):
		pass