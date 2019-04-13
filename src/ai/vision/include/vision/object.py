from typing import List, Dict

from action_manager import Argumentable
from copy import copy

class ObjectAction:
	pass

class MapObject(Argumentable):
	"""
		Define an object on the field, with associated action
		to deal with and some properties
	"""
	def __init__(self):
		self.name = ""
		self.shape = None
		self.actions: List[ObjectAction] = []
		self.argsSets: Dict[str, Argumentable] = {}
	
	def extends(self, obj: 'MapObject'):
		# Make a copy of the shape
		self.shape = copy(obj.shape)
		
		# TODO clone argsSet if used


class Zone(Argumentable):
	"""
		Define a zone in which some actions might take place.
		
		They are created in order to be provided as action args
	"""
	def __init__(self):
		self.shape = None
		self.name = ""