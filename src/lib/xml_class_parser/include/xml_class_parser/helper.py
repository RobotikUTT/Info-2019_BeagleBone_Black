from .parsable import Parsable
from .bind import Bind, BindList, BindDict

from typing import Type, Union

def ParsableList(name: str, type: Type) -> Type:
	class List(list):
		@property
		def array(self):
			return self
		
	return Parsable(
		name = name,
		children = [
			BindList(type=type, to="array")
		]
	)(List)

def ParsableDict(name: str, type: Type, key: str) -> Type:
	class Dict(dict):
		@property
		def values_dict(self):
			return self
		
	return Parsable(
		name = name,
		children = [
			BindDict(type=type, to="values_dict", key = key)
		]
	)(Dict)

def TagAttrTuple(attribute: str, value_type: Type = str) -> Type:
	"""
		Returns a class containing a name and attribute, and
		apply parsable decorator on it, binding xml tag to name and
		given attribute to value
	"""
	# Create simple name-value tuple class
	class Tuple:
		def __init__(self):
			self.name = ""
			self.value = ""
	
	# Apply parsable and returns
	return Parsable(name=Bind(to="name"), attributes={
		attribute: Bind(to="value", mandatory=True, type=value_type)
	})(Tuple)

def AttrAttrTuple(name: Union[str, Bind], name_attr: str, value_attr: str, value_type: Type = str) -> Type:
	"""
		Returns a class containing a name and attribute, and
		apply parsable decorator on it, binding xml attribute to name and
		given attribute to value
	"""
	# Create simple name-value tuple class
	class Tuple:
		def __init__(self):
			self.name = ""
			self.value = ""
	
	# Apply parsable and returns
	return Parsable(name=name, attributes={
		name_attr: Bind(mandatory=True, to="name"),
		value_attr: Bind(mandatory=True, type=value_type, to="value")
	})(Tuple)


def TagContentTuple(content_type: Type = str) -> Type:
	"""
		Returns a class containing a name and attribute, and
		apply parsable decorator on it, binding xml tag to name and
		content to value
	"""
	# Create simple name-value tuple class
	class Tuple:
		def __init__(self):
			self.name = ""
			self.value = ""
	
	# Apply parsable and returns
	return Parsable(
		name=Bind(to="name"),
		content=Bind(to="value",mandatory=True, type=content_type)
	)(Tuple)