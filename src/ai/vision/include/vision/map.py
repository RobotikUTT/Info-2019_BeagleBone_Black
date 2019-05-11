from . import MapObject
from .shape import Circle, Rect

from typing import Union, List, Dict
from copy import copy, deepcopy

from xml_class_parser import Parsable, Bind, BindList

@Parsable(
	name="offset",
	attributes={
		"x": int,
		"y": int
	},
	children = [
		Bind(to="zones", RectZone),
		Bind(to="zones", CircleZone),
		BindList(to="zones", Offset)
	]
)
class Offset:
	def __init__(self, x = 0, y = 0):
		self.x: int = x
		self.y: int = y
		self.zones: List[Offset] = []
	
	def __apply_offset(self, obj: Offset):
		obj.x += self.x
		obj.y += self.y
		return obj

	def __parsed__(self, context):
		# flatten offsets
		max = len(self.zones)
		i = 0
		while i < max:
			zone = self.zones[i]

			if type(zone) == Offset:
				self.zones = self.zones[0:i] + \
					zone.zones.map(self.__apply_offset) + self.zones[i+1:]
			else:
				i += 1


class Zone(Offset):
	"""
		Define a zone in the map, that is either a "virtual" zone,
		either a physical blocking zone
	"""
	def __init__(self):
		super().__init__()
		self.name: str = ""
		self.symmetry: Union[Symmetry, None] = None
		self.blocking: bool = False
	
	def __parsed__(self, ctx):
		# first flatten offsets
		super().__parsed__(ctx)

		# apply symmetry
		if self.symmetry is not None:
			self.symmetry.apply(self)


	def __str__(self):
		return  "\n\t".join(
			["<{}> {}{}".format(self.name, self.shape, " wall" if self.wall else "")] +
			("\n".join([z.__str__() for z in (self.zones + self.objects)]).split("\n") if len(self.zones) + len(self.objects) > 0 else [])
		)
@Parsable(
	name="rect-zone",
	attributes = {
		"x": int,
		"y": int,
		"width": int,
		"height": int,
		"blocking": bool
		"name": Bind(type=str, mandatory=True)
	},
	children = [
		BindList(to="zones", RectZone),
		BindList(to="zones", CircleZone),
		BindList(to="zones", Offset)
	]
)
class RectZone:
	pass

@Parsable(
	name="circle-zone",
	attributes = {
		"x": int,
		"y": int,
		"radius": int,
		"blocking": bool
		"name": Bind(type=str, mandatory=True)
	},
	children = [
		BindList(to="zones", RectZone),
		BindList(to="zones", CircleZone),
		BindList(to="zones", Offset)
	]
)
class RectZone:
	pass

@Parsable(
	name="symmetry"
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

