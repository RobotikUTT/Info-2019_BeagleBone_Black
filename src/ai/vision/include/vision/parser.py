from xml.etree import ElementTree

import rospkg

from typing import List, Union

class ParsingException(Exception):
	pass


class AttrParser:
	def __init__(self, name: str, default_value: Union[None, str, int, float] = "", target: str = None, shorthand: bool = False, mandatory: str = False, cast: bool = None):
		self.name: str = name
		self.target: str = name if target == None else target
		self.shorthand: bool = shorthand
		self.default_value = default_value
		self.cast: type = cast
		self.mandatory: bool = False

	def parse(self, element: ElementTree.Element, obj: object):
		value = self.default_value

		# Check if attribute is inside
		if self.name in element.attrib:
			value = element.attrib[self.name]
		elif self.name[0] in element.attrib:
			value = element.attrib[self.name[0]]
		elif self.mandatory: # not found + mandatory : gotcha !
			raise ParsingException("attribute {} not provided for {}".format(self.name, element.tag))
		
		# Cast if necessary
		if self.cast:
			value  = self.cast(value)

		# Display warning for wrong setattr
		if not hasattr(obj, self.target) or type(getattr(obj, self.target)) != type(value):
			print("warning : {} may not have an attribute {} of type {}".format(obj, self.target, type(value)))

		# Apply attribute
		setattr(obj, self.target, value)


class ElementParser:
	def __init__(self, type: type, tag: str, bindto: str = None, shorthand: bool = False, allowany = False):
		self.tag = tag
		self.type = type
		self.attributes: List[AttrParser] = []
		self.children: List[ElementParser] = []
		self.shorthand = shorthand
		self.allowany = allowany # allow any children
		self.bindto = bindto if bindto != None else tag # variable name to bind with
	
	def add_child(self, parser: 'ElementParser'):
		self.children.append(parser)

	def bind_attr(self, name: str, default_value: Union[None, str, int, float] = "", target: str = None,\
		shorthand: bool = True, mandatory: str = False, cast: bool = None):
		
		self.attributes.append(
			AttrParser(name, default_value, target, shorthand, mandatory, cast)
		)
		return self

	def parse(self, element: ElementTree.Element, parent: object) -> bool:
		# Not this element :)
		if element.tag != self.tag and element.tag != self.tag[0]:
			return
		
		# Instanciate element
		content = self.type()

		# Add all attributes
		for attr in self.attributes:
			attr.parse(element, content)

		# Then parse childrens
		for child in element:
			parsed = False
			for child_type in self.children:
				if child_type.parse(child, content):
					parsed = True
					break
			
			# Check it has been parsed
			if not parsed:
				raise ParsingException("{} does not belong inside a {} element".format(child.tag, element.tag))
		
		# TODO check types and waning
		# Then add element to parent (to list or as object)
		if type(parent, self.bindto) == list:
			getattr(parent, self.bindto).append(content)
		else:
			setattr(parent, self.bindto, content)

class ListContainer():
	"""Basic container for a list"""
	def __init__(self):
		self.values = []
	
	@staticmethod
	def get_parser(tag: str):
		return ElementParser(ListContainer, tag, "values")

class Parser:
	"""
		Generic XML parser for ROS description files
	"""

	def __init__(self, root_parser: ElementParser):
		self.root_parser: ElementParser = root_parser

	def parse_string(self, data: str, element_parser: ElementParser = None):
		root = ElementTree.fromstring(data)
		self.parse(root, element_parser)

	def parse_description_file(self, path: str, element_parser: ElementParser = None):
		# Find mapping file directory
		source_folder = rospkg.RosPack().get_path("ai_description")

		if path[0] != "/":
			path = "/" + path

		# Parsing mapping file
		root = ElementTree.parse(source_folder + path).getroot()

		self.parse(root, element_parser)

	def parse(self, root: ElementTree.Element, element_parser: ElementParser = None):
		parent = {}
		parser = element_parser if element_parser != None else self.root_parser
		
		if parser.parse(root, parent):
			return parent[parser.bindto]