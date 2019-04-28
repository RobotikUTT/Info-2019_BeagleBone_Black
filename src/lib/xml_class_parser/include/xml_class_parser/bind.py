from . import ParsingException
from typing import Union, Any

class Bind:
	'''
		Define a binding between XML element and properties
	'''
	def __init__(self, to: str = Union[None, str], type: type = str, mandatory=False):
		self.to = to
		self.type = type
		self.mandatory = mandatory

	def apply(self, obj: object, value: str):
		'''Apply value to object as defined in the binding'''
		if self.to == None:
			raise ParsingException("destination is not defined inside the binding")

		if hasattr(self.type, "parse"):
			self.__apply(obj, self.type.parse(value))
		else:
			self.__apply(obj, self.type(value))
		

	def __apply(self, obj: object, value: Any):
		# Parse object or just cast it
		if hasattr(self.type, "parse"):
			setattr(obj, self.to, self.type.parse(value))
		else:
			setattr(obj, self.to, self.type(value))

class BindList(Bind):
	def __apply(self, obj: object, value: Any):
		'''Apply value to object as defined in the binding'''

		# Check that the attribute is a list
		if not hasattr(obj, self.to) or not isinstance(getattr(obj, self.to), list):
			raise ParsingException("{} does not exists or is not a list".format(self.to))

		getattr(obj, self.to).append(value)

class BindDict(Bind):
	'''
		Define a binding between XML element and dict property
	'''
	def __init__(self, key: str, to: Union[None, str] = None, type: type = str, mandatory = False):
		super().__init__(to=to, type=type, mandatory=mandatory)

		self.key = key

	def __apply(self, obj: object, value: Any):
		'''Apply value to object as defined in the binding'''

		# Check that attribute is a dict
		if not hasattr(obj, self.to) or not isinstance(getattr(obj, self.to), dict):
			raise ParsingException("{} does not exists or is not a list".format(self.to))

		# Check for key values existence
		if not hasattr(value, self.key):
			raise ParsingException("{} does not have a parameter named {}".format(value, self.key))

		# Register to dict
		getattr(obj, self.to)[getattr(value, self.key)] = value