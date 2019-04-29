from xml.etree import ElementTree
from typing import List, Union
import rospkg

from .bind import Bind, BindDict, BindList
from . import ParsingException

from typing import Union, Dict

AnyBinding = Union[Bind, BindDict, BindList]

class Parsable:
	SELF = "SELF"

	def __init__(self,
			name: Union[str, Bind],
			attributes: Dict[str, Union[type, Bind]] = {},
			children: Dict[str, Union[AnyBinding, type]] = {},
			content: Union[str, Bind, None] = None ):
		'''Initialize parsable and fill missing informations'''
		
		self.name: Union[str, Bind] = name
		self.attributes: Dict[str, Bind] = {}
		self.children: List[AnyBinding] = []
		self.content: Union[Bind, None] = Bind(to=content) if type(content) == str else content
	
		# Get attributes
		for key, value in attributes.items():
			if isinstance(value, type):
				value = Bind(to=key, type=value)
			elif value.to == None:
				value.to = key # fill optional {to} with key value

			self.attributes[key] = value
		
		# Then children
		for value in children:
			# In case only type given
			if isinstance(value, type):
				if not hasattr(value, "parse"):
					raise ParsingException("a children type must be parsable")

				# Consider it to be a list
				value = BindList(type=value)
			
			# Fill missing [to]
			if value.to == None:
				value.to = value.xml_name

			self.children.append(value)

	def __call__(self, class_type):
		'''
			Create class with parsing functions
		'''
		self.generated = class_type
		
		class_type.parse_string = self.parse_string
		class_type.parse = self.parse
		class_type.parse_file = self.parse_file

		class_type.xml_name = self.name

		# Apply SELF type on children
		for child in self.children:
			if child.type == Parsable.SELF:
				child.type = class_type

		return class_type


	def parse_string(self, data: str):
		root = ElementTree.fromstring(data)
		return self.parse(root)

	def parse_file(self, path: str):
		# Find mapping file directory
		source_folder = rospkg.RosPack().get_path("ai_description")

		if path[0] != "/":
			path = "/" + path

		# Parsing mapping file
		root = ElementTree.parse(source_folder + path).getroot()

		return self.parse(root)

	def parse(self, root: ElementTree.Element):
		"""
			Parse a XML element to create an object of decorated type
		"""
		obj = self.generated()

		self.parse_name(root, obj)
		self.parse_attributes(root, obj)
		self.parse_children(root, obj)
		self.parse_content(root, obj)

		return obj

	def parse_name(self, root: ElementTree.Element, obj):
		# Handle element tag
		if type(self.name) == str:
			# Check for invalid name
			if root.tag != self.name:
				raise ParsingException("invalid element {}, expected {}".format(root.tag, self.name))
		else:
			# Apply name property
			self.name.apply(obj, root.tag)

	def parse_attributes(self, root: ElementTree.Element, obj):
		# Handle attributes
		for attr in self.attributes:
			# If attribute is registered
			if attr in root.attrib:
				self.attributes[attr].apply(obj, root.attrib[attr])
			elif self.attributes[attr].mandatory:
				# Otherwise throw if mandatory
				raise ParsingException("missing attribute {} in {}".format(attr, root.tag))

	def parse_children(self, root: ElementTree.Element, obj):
		# Handle children
		for child in root:
			generic: Union[AnyBinding, None] = None
			handled = False

			for available in self.children:
				# If it is a generic binding
				if isinstance(available.xml_name, Bind):
					# Only one allowed
					if generic != None:
						raise ParsingException(
							"cannot have 2 children with generic names : {} and {}"
								.format(generic, available)
						)
					generic = available

				# Otherwise check that it match
				elif available.xml_name == child.tag:
					if handled:
						raise ParsingException("{} have two bound values".format(child.tag))

					available.apply(obj, child)
					handled = True

			# Apply generic element if any and not handled yet
			if not handled:
				if generic != None:
					generic.apply(obj, child)
				else:
					raise ParsingException("unable to handle {} element in {}".format(child.tag, root.tag))

	def parse_content(self, root: ElementTree.Element, obj):
		# Handle content
		if self.content != None:
			self.content.apply(obj, root.text)

