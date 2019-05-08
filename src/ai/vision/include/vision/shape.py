from typing import Dict, Union
from xml_class_parser import Parsable, Bind

@Parsable(
	name="circle",
	attributes={
		"radius": int,
		"blocking": bool
	}
)
class Circle:
	""" Circle defined by it's center """
	def __init__(self, radius: int = -1):
		self.blocking = False
		self.radius: int = radius

	def __str__(self):
		return "circle(r={})".format(self.radius)


@Parsable(
	name="rect",
	attributes = {
		"width": int,
		"height": int,
		"blocking": bool
	}
)
class Rect:
	"""
		Rectangle shape defined from it's center
	"""
	def __init__(self, height: int = -1, width: int = -1):
		self.blocking = False
		self.width: int = height
		self.height: int = width

	def __str__(self):
		return "rect(w={}, h={})".format(self.width, self.height)

