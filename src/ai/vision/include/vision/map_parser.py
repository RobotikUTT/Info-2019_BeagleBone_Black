from . import MapObject, Shape, RectShape, CircleShape, MapObjectAction, MapObjectArgument
from .map import Zone, Symmetry, Offset
from .parser import Parser, ParsingException
from xml.etree import ElementTree
import rospkg

from typing import Dict, List

from args_lib.argumentable import Argumentable

class MapParser(Parser):
	"""
		XML parser utility for a map
	"""
	def __init__(self, objects: Dict[str, MapObject]):
		self.objects: Dict[str, MapObject] = objects

	def parse(self, root: ElementTree.Element):
		# Init new map
		self.root = Zone(Shape.parse(root, None, True), "__root")
		self.root.wall = bool(root.attrib["wall"]) if "wall" in root.attrib else False
		self.parse_zone_content(root, self.root, Offset())


	def parse_zone_content(self, element: ElementTree.Element, parent_zone: Zone, offset: Offset, symmetry_on_done: bool = True):
		"""Parse all sub-elements in a XML zone (circle or rect)"""

		for child in element:
			if child.tag == "symmetry":
				parent_zone.symmetry = self.parse_symmetry(child)

			elif child.tag == "offset":
				# Continue same zone with different offset
				self.parse_zone_content(child, parent_zone, self.parse_offset(child, offset), False)

			elif child.tag in self.objects:
				self.parse_object(child, parent_zone, offset)
			else:
				# Parse sub-zone
				child_zone = Zone(Shape.parse(child, None, True), child.attrib["name"] if "name" in child.attrib else "")
				self.parse_zone_content(child, child_zone, offset)

				if "wall" in child.attrib:
					child_zone.wall = bool(child.attrib["wall"])

				# Add to parent zone
				parent_zone.zones.append(child_zone)
		
		# Apply symmetry to all subzone
		if symmetry_on_done and parent_zone.symmetry != None:
			parent_zone.symmetry.apply(parent_zone)

		if parent_zone.name == "chaos":
			print(parent_zone.objects)
	
	def parse_symmetry(self, el: ElementTree.Element) -> Symmetry:
		sym = Symmetry()

		if "x" in el.attrib:
			sym.axis = "x"
		elif "y" in el.attrib:
			sym.axis = "y"
		else:
			raise ParsingException("symmetry with no x or y axis provided")
		
		sym.source = el.attrib["source"]
		sym.target = el.attrib["target"]

		sym.offset = int(el.attrib[sym.axis])
		return sym

	def parse_offset(self, el: ElementTree.Element, current_offset: Offset) -> Offset:
		new_offset = Offset(current_offset.x, current_offset.y)
		
		if "x" in el.attrib:
			new_offset.x += int(el.attrib["x"])

		if "y" in el.attrib:
			new_offset.y += int(el.attrib["y"])

		return new_offset

	def parse_object(self, el: ElementTree.Element, parent_zone: Zone, offset: Offset) -> None:
		map_object: MapObject = self.objects[el.tag].clone()

		# Parse XML attributes
		for attr in el.attrib:
			# Shape attribute
			if hasattr(map_object.shape, attr):
				setattr(map_object.shape, attr, int(el.attrib[attr]))
			
			# Object specific attribute
			if attr in map_object.args:
				# Set all bound properties to object
				map_object.from_list(
					map_object.args[attr].get_value(el.attrib[attr]).to_list()
				)

		# Apply offset to x and y
		map_object.shape.x += offset.x
		map_object.shape.y += offset.y

		# Add object to zone
		parent_zone.objects.append(map_object)