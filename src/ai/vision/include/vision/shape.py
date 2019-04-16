from xml.etree import ElementTree
from .parser import ParsingException

from typing import Dict

class Shape:
	"""
		Definition of a generic shape
	"""

	# Fifty shades of shapes
	types: Dict[str, type] = {}

	def __init__(self, x: int = -1, y: int = -1):
		self.x: int = x
		self.y: int = y

	def __str__(self):
		return "Shape ({}, {})".format(self.x, self.y)
	
	def __eq__(self, other):
		if isinstance(other, self.__class__):
			return self.__dict__ == other.__dict__

		return NotImplemented

	def __ne__(self, other):
		e = self.__eq__(other)
		if e is NotImplemented:
			return e
		return not e

	@staticmethod
	def declare(name: str, associated_class: type):
		Shape.types[name] = associated_class

	@staticmethod
	def parse(element: ElementTree.Element, current_shape: 'Shape' = None, allow_any: bool = False) -> 'Shape':
		"""
			Parse the shape of an object from it's XML element

			The function keep the curent shape (if defined, probably shape
			from the parent object), and add properties given in XML.

			If the shape type differ from current, change type but keep x and
			y properties (unless new x/y given)
		"""
		# Make sure the right shape class is used
		if current_shape == None:
			current_shape = Shape()

		if element.tag in Shape.types and type(current_shape) is not Shape.types[element.tag]:
			# Keep x, y from previous shape
			current_shape = Shape.types[element.tag](current_shape.x, current_shape.y)

		elif element.tag not in Shape.types:
			# No shape nor anything else
			raise ParsingException(
				"{} does not name an object parameter type"
					.format(element.tag)
			)

		# Then parse shape attributes
		for attr in element.attrib:
			if hasattr(current_shape, attr):
				# Copy value as int (raise error if wrong)
				setattr(current_shape, attr, int(element.attrib[attr]))
			elif not allow_any: # if it does not allow other parameters
				raise Exception(
					"{} attribute not defined for a {} shape".format(attr, element.tag)
				)
		
		return current_shape


"""
	Some simple shapes classes
"""
class CircleShape(Shape):
	def __init__(self, x: int = -1, y: int = -1, radius: int = -1):
		super().__init__(x, y)

		self.radius: int = radius

	def __str__(self):
		return "Circle ({}, {}, r={})".format(self.x, self.y, self.radius)

# Add shape to list
Shape.declare("circle", CircleShape)

class RectShape(Shape):
	def __init__(self, x: int = -1, y: int = -1, height: int = -1, width: int = -1):
		super().__init__(x, y)

		self.width: int = height
		self.height: int = width

	def __str__(self):
		return "Rect ({}, {}, w={}, h={})".format(self.x, self.y, self.width, self.height)

# Add shape to list
Shape.declare("rect", RectShape)
