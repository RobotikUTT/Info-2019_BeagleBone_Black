from xml.etree import ElementTree
from typing import List, Union
import rospkg

from .bind import Bind, BindDict, BindList
from .values import Values
from . import ParsingException

from typing import Union, Dict

AnyBinding = Union[Bind, BindDict, BindList]

class Parsable:
	def __init__(self,
			name: Union[str, Bind],
			attributes: Dict[str, Union[type, Bind]] = {},
			children: Dict[str, Union[AnyBinding, type]] = {},
			generic_children: Union[AnyBinding, None] = None,
			content: Union[str, Bind, None] = None ):
		'''Initialize parsable and fill missing informations'''
		
		self.name: Union[str, Bind] = name
		self.attributes: Dict[str, Bind] = {}
		self.children: Dict[str, Bindings] = {}
		self.generic_children = generic_children
		self.content: Union[Bind, None] = Bind(to=content) if type(content) == str else content
	
		# Get attributes
		for key, value in attributes.items():
			if isinstance(value, type):
				value = Bind(to=key, type=value)
			elif value.to == None:
				value.to = key # fill optional {to} with key value

			self.attributes[key] = value
		
		# Then children
		for key, value in children.items():
			# In case only type given
			if isinstance(value, type):
				if not hasattr(value, "parse"):
					raise ParsingException("a children type must be parsable")

				# Consider it to be a list
				value = BindList(to=key, type=value)
			
			# Fill missing to
			if value.to == None:
				value.to = key

			# Handle automatic parsable key value
			if key == "@":
				key = value.type.xml_name

			self.attributes[key] = value

	def __call__(self, class_type):
		'''
			Create class with parsing functions
		'''
		self.generated = class_type

		class WrappedClass(class_type):
			xml_name = self.name

			@staticmethod
			def parse_string(content: str) -> class_type:
				return self.parse_string(content)
			
			@staticmethod
			def parse_file(filename: str) -> class_type:
				return self.parse_file(filename)
			
			@staticmethod
			def parse(root: ElementTree.Element) -> class_type:
				return self.parse(root)
		
		return WrappedClass


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

		# Handle element tag
		if type(self.name) == str:
			# Check for invalid name
			if root.tag != self.name:
				raise ParsingException("invalid element {}, expected {}".format(root.tag, self.name))
		else:
			# Apply name property
			self.name.apply(obj, root.tag)

		# Handle attributes
		for attr in self.attributes:
			# If attribute is registered
			if attr in root.attrib:
				self.attributes[attr].apply(obj, root.attrib[attr])
			elif self.attributes[attr].mandatory:
				# Otherwise throw if mandatory
				raise ParsingException("missing attribute {} in {}".format(attr, root.tag))

		# Handle children
		for child in root:
			if child.tag in self.children:
				self.children[child.tag].apply(obj, child)
			elif self.generic_children != None:
				self.generic_children.apply(obj, child)
			else:
				raise ParsingException("unable to handle {} element in {}".format(child.tag, root.tag))

		# Handle content
		if self.content != None:
			self.content.apply(obj, root.text)

		return obj
