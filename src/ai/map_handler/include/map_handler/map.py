from . import MapObject
from .shape import Circle, Rect

from typing import Union, List, Dict
from copy import copy, deepcopy

from xml_class_parser import Parsable, Bind, BindList


class Offset:
	def __init__(self, x = 0, y = 0):
		self.x: int = x
		self.y: int = y
		self.zones: List[Offset] = []
	
	def __apply_offset(self, obj: 'Offset'):
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

	id = 0

	def __init__(self):
		super().__init__()

		self.id = Zone.id
		Zone.id += 1

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
			["<{}> {},{}{}".format(self.name, self.x, self.y, " blocking" if self.blocking else "")] +
			("\n".join([z.__str__() for z in self.zones]).split("\n") if len(self.zones) > 0 else [])
		)

class RectZone(Zone):
	def __init__(self):
		super().__init__()
		self.width = 0
		self.height = 0

class CircleZone(Zone):
	def __init__(self):
		super().__init__()
		self.radius = 0

@Parsable(
	name="symmetry",
	attributes = {
		"offset": Bind(mandatory=True, type=int),
		"axis": Bind(mandatory=True),
		"source": str,
		"target": str
	}
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

	def change_zone_content_side(self, zone: Zone) -> None:
		# Then on every items
		if self.axis == "x":
			zone.x += (self.offset - zone.x) * 2
		elif self.axis == "y":
			zone.y += (self.offset - zone.y) * 2
		else:
			raise Exception("unhandled axis : {}".format(self.axis))
		
		# Then on every subzones
		for subzone in zone.zones:
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
		zone.__init__()
		zone.width = original.width
		zone.height = original.height
		zone.name = original.name
		zone.blocking = original.blocking

		# Set new names
		original.name = self.source
		original.blocking = False

		duplicate.name = self.target
		duplicate.blocking = False

		# Add subzones
		zone.zones = [original, duplicate]


# Make objet parsables
Offset = Parsable(
	name="offset",
	attributes={
		"x": int,
		"y": int
	},
	children = [
		BindList(to="zones", type=RectZone),
		BindList(to="zones", type=CircleZone),
		BindList(to="zones", type=Offset)
	]
)(Offset)

RectZone = Parsable(
	name="rect-zone",
	attributes = {
		"x": int,
		"y": int,
		"width": int,
		"height": int,
		"blocking": bool,
		"name": str
	},
	children = [
		BindList(to="zones", type=RectZone),
		BindList(to="zones", type=CircleZone),
		BindList(to="zones", type=Offset),
		Bind(to="symmetry", type=Symmetry)
	]
)(RectZone)

CircleZone = Parsable(
	name="circle-zone",
	attributes = {
		"x": int,
		"y": int,
		"radius": int,
		"blocking": bool,
		"name": str
	},
	children = [
		BindList(to="zones", type=RectZone),
		BindList(to="zones", type=CircleZone),
		BindList(to="zones", type=Offset),
		Bind(to="symmetry", type=Symmetry)
	]
)(CircleZone)