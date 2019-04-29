from . import Shape, MapObject

from typing import Union, List, Dict
from copy import copy, deepcopy

from xml_class_parser import Parsable, Bind, BindList

class Offset:
	def __init__(self, x = 0, y = 0):
		self.x: int = x
		self.y: int = y

@Parsable(
	name="zone",
	attributes={
		"name": Bind(type=str, mandatory=True)
	},
	extends = Shape,
	children = [
		Bind(to="shape", Shape),
		BindList(to="zones", Offset)
	]
)
class Zone:
	"""
		Define a zone in which some actions might take place.
		
		They are created in order to be provided as action args
	"""
	def __init__(self, shape: Shape, name: str):
		self.name: str = name

		self.shape = shape
		self.symmetry: Union[Symmetry, None] = None
		self.zones: List[Zone] = []
		self.objects: Dict[MapObject] = []
		self.wall: bool = False
	
	def __str__(self):
		return  "\n\t".join(
			["<{}> {}{}".format(self.name, self.shape, " wall" if self.wall else "")] +
			("\n".join([z.__str__() for z in (self.zones + self.objects)]).split("\n") if len(self.zones) + len(self.objects) > 0 else [])
		)

class Symmetry:
	"""
		Symmetry according to an axis (x or y).

		Having axis set to "x" means that objects on one side
		of the line of equation x = [offset] are cloned on the
		other side
	"""
	def __init__(self):
		self.axis: str = "x"
		self.offset: int = 0
		self.source = ""
		self.target = ""

	def change_shape_side(self, shape: Shape) -> None:
		if self.axis == "x":
			shape.x += (self.offset - shape.x) * 2
		elif self.axis == "y":
			shape.y += (self.offset - shape.y) * 2
		else:
			raise Exception("unhandled axis : {}".format(self.axis))

	def change_zone_content_side(self, zone: Zone) -> None:
		# Then on every items
		for obj in zone.objects:
			self.change_shape_side(obj.shape)
		
		# Then on every subzones
		for subzone in zone.zones:
			self.change_shape_side(subzone.shape)
			self.change_zone_content_side(subzone)
		

	def apply(self, zone: Zone):
		"""
			Duplicate zone content into it's symmetrical zone,
			then return
		"""
		# Make two copy of the shape
		original = copy(zone)
		duplicate = deepcopy(zone)
		self.change_zone_content_side(duplicate)

		# Reset original shape
		zone.__init__(original.shape, original.name)

		# Set new names
		original.name = self.source
		duplicate.name = self.target

		# Only original zone keep wall as it's shape is kept
		if original.wall:
			zone.wall = True

			original.wall = False
			duplicate.wall = False

		# Add subzones
		zone.zones = [original, duplicate]

