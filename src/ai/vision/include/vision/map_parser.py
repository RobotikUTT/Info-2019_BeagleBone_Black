#!/usr/bin/python3
from . import MapObject, Shape, RectShape, CircleShape, MapObjectAction, MapObjectArgument
from .map import Zone, Symmetry, Offset
from xml.etree import ElementTree
import rospkg

from typing import Dict, List

from action_manager import Argumentable

class MapParser():
	def __init__(self, objects: Dict[str, MapObject]):
		self.objects: Dict[str, MapObject] = objects

		# Find mapping file directory
		source_folder = rospkg.RosPack().get_path("ai_description")

		# Parsing mapping file
		root = ElementTree.parse(source_folder + "/map/table.xml").getroot()

		# Init new map
		self.root = Zone(Shape.parse(root), "__root")
		self.parse_zone_content(root, self.root, Offset())


	def parse_zone_content(self, element: ElementTree.Element, parent_zone: Zone, offset: Offset):
		for child in element:
			if child.tag == "symmetry":
				parent_zone.symmetry = self.parse_symmetry(child)

			elif child.tag == "offset":
				self.parse_zone_content(child, parent_zone, self.parse_offset(child, offset))

			elif child.tag in self.objects:
				self.parse_object(child, parent_zone)

			else:
				# Parse sub-zone
				child_zone = Zone(Shape.parse(child, None, True), child.attrib["name"] if "name" in child.attrib else "")
				self.parse_zone_content(child, child_zone, offset)

				# Add to parent zone
				parent_zone.zones.append(child_zone)
	
	def parse_symmetry(self, child) -> Symmetry:
		return Symmetry()

	def parse_offset(self, child, offset) -> Offset:
		return offset

	def parse_object(self, child, parent_zone) -> MapObject:
		return MapObject()