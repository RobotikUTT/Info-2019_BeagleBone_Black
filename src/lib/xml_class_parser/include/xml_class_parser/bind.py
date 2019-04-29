from . import ParsingException
from typing import Union, Any

class Bind:
	'''
		Define a binding between XML element and properties
	'''
	def __init__(self, to: str = Union[None, str], type: type = str, mandatory=False, xml_name: str = None):
		self.to = to
		self.type = type
		self.mandatory = mandatory
		self.xml_name = xml_name

	def apply(self, obj: object, value: str):
		'''Apply value to object as defined in the binding'''
		if self.to == None:
			raise ParsingException("destination is not defined inside the binding")

		if hasattr(self.type, "parse"):
			self.apply_casted(obj, self.type.parse(value))
		else:
			self.apply_casted(obj, self.type(value))
		

	@property
	def xml_name(self):
		if self.__xml_name != None:
			return self.__xml_name
		elif hasattr(self.type, "xml_name"):
			return self.type.xml_name
		else:
			raise ParsingException("{} does not have an XML name".format(self.type))

	@xml_name.setter
	def xml_name(self, xml_name):
		self.__xml_name = xml_name

	def apply_casted(self, obj: object, value: Any):
		setattr(obj, self.to, value)
		

class BindList(Bind):
	def apply_casted(self, obj: object, value: Any):
		'''Apply value to object as defined in the binding'''

		# Check that the attribute is a list
		if not hasattr(obj, self.to) or not isinstance(getattr(obj, self.to), list):
			raise ParsingException("{} does not exists or is not a list".format(self.to))
		
		getattr(obj, self.to).append(value)

class BindDict(Bind):
	'''
		Define a binding between XML element and dict property
	'''
	def __init__(self, key: str, to: Union[None, str] = None, type: type = str, mandatory = False, xml_name = None):
		super().__init__(to=to, type=type, mandatory=mandatory, xml_name=xml_name)

		self.key = key

	def apply_casted(self, obj: object, value: Any):
		'''Apply value to object as defined in the binding'''

		# Check that attribute is a dict
		if not hasattr(obj, self.to) or not isinstance(getattr(obj, self.to), dict):
			raise ParsingException("{} does not exists or is not a list".format(self.to))

		# Check for key values existence
		if not hasattr(value, self.key):
			raise ParsingException("{} does not have a parameter named {}".format(value, self.key))

		# Register to dict
		getattr(obj, self.to)[getattr(value, self.key)] = value