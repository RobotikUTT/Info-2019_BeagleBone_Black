from xml.etree import ElementTree
from .parser import ParsingException

from typing import Dict, Union

from xml_class_parser import Parsable, Bind

class Point:
	def __init__(self, x: int, y: int):
		self.x = x
		self.y = y

	def add(self, p: 'Point'):
		self.x += p.x
		self.y += p.y
	
	def mult(self, sc: Union[int, 'Point']) -> Union[None, int]:
		"""
			Either return the scalar product of two points,
			or multiply all coordinates by given scalar factor
		"""
		if type(sc) == 'Point':
			return sc.x * self.x + sc.y * self.y
		else:
			self.x *= sc
			self.y *= sc
	
	def sym(self, axis: 'Axis'):
		"""Set itself to the opposite side of the given axis"""
		self.add(axis.relative_position(self).mult(2))

class Axis:
	def __init__(self, pos: Point, dir: Point):
		self.pos = pos
		self.dir = dir
	
	def relative_position(self, pos: Point) -> Point:
		# TODO
		return pos

@Parsable(
	name = Bind(to=type),
	attributes = {
		"x": int,
		"y": int,
		"blocking": bool
	}
)
class Shape:
	"""
		Definition of a generic shape
	"""

	# Fifty shades of shapes
	types: Dict[str, type] = {}

	def __init__(self, pos: Point = None):
		self.pos = Point(0, 0) if pos == None else pos
		self.blocking = False

	@property
	def x(self) -> int:
		return self.pos.x

	@x.setter
	def x(self, x: int):
		self.pos.x = x

	@property
	def y(self):
		return self.pos.y

	@y.setter
	def y(self, y: int):
		self.pos.y = y

	def symmetry(self, axis: Axis) -> None:
		"""
			Move object symmetrically from given axis
		"""

		# Add two time the distance from a point to another
		self.pos.sym(axis)

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
@Parsable(
	name="circle",
	extends=Shape,
	attributes={
		"radius": int
	}
)
class CircleShape(Shape):
	def __init__(self, pos: Point = None, radius: int = -1):
		super().__init__(pos)

		self.radius: int = radius

	def __str__(self):
		return "Circle ({}, {}, r={})".format(self.x, self.y, self.radius)

# Add shape to list
Shape.declare("circle", CircleShape)

@Parsable(
	name="rect",
	extends=Shape,
	attributes = {
		"width": int,
		"height": int
	}
)
class RectShape(Shape):
	"""
		Rectangle shape defined from it's upper left corner (pos)
	"""
	def __init__(self, pos: Point = None, height: int = -1, width: int = -1):
		super().__init__(pos)

		self.width: int = height
		self.height: int = width

	def symmetry(self, axis: Axis):
		super().symmetry(axis)

	def __str__(self):
		return "Rect ({}, {}, w={}, h={})".format(self.x, self.y, self.width, self.height)

# Add shape to list
Shape.declare("rect", RectShape)
