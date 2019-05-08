from xml_class_parser import Parsable

from args_lib.argumentable import Argumentable

from typing import Union

@Parsable(name="need")
class ObjectRequirement(Argumentable):
	"""
		Define a need of objects by an action
	"""
	def __init__(self):
		self.object: str = ""
		self.filter: str = ""
		self.quantity = 1

		self.__alias: Union[str, None] = None
		self.__min: int = -1
		self.__max: int = -1

	#Properties min, max and alias that have default values depending on other parameters
	@property
	def max(self) -> int:
		return self.__max if self.__max != -1 else self.quantity

	@max.setter
	def max(self, max: int):
		self.__max = max

	@property
	def min(self) -> int:
		return self.__min if self.__min != -1 else self.quantity

	@min.setter
	def min(self, min: int):
		self.__min = min

	@property
	def alias(self) -> str:
		return self.__alias if self.__alias != None else self.object

	@alias.setter
	def alias(self, alias: str):
		self.__alias = alias
