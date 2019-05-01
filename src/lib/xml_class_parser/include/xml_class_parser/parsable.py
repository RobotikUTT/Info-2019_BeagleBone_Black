from xml.etree import ElementTree
from typing import List, Union
import rospkg
import inspect

from .bind import Bind, BindDict, BindList
from .context import Context
from . import ParsingException

from typing import Union, Dict, Type, List

AnyBinding = Union[Bind, BindDict, BindList]

class Parsable:
	SELF = "SELF"

	def __init__(self,
			name: Union[str, Bind],
			extends: Union[None, Type] = None,
			attributes: Dict[str, Union[Type, Bind]] = {},
			children: List[Union[AnyBinding, Type]] = [],
			ignored_children: List[str] = [],
			content: Union[str, Bind, None] = None ):
		'''Initialize parsable and fill missing informations'''
		
		self.name: Union[str, Bind] = name
		self.attributes: Dict[str, Bind] = {}
		self.children: List[AnyBinding] = []
		self.content: Union[Bind, None] = Bind(to=content) if isinstance(content, str) else content
		self.extends = extends
		self.ignored_children = ignored_children

		# Get attributes
		for key, value in attributes.items():
			if not isinstance(value, Bind):
				value = Bind(to=key, type=value)
			elif value.to == None:
				value.to = key # fill optional {to} with key value

			self.attributes[key] = value
		
		# Then children
		for child in children:
			# In case only type given
			if not isinstance(child, Bind):
				if not hasattr(child, "parse") and child is not Parsable.SELF:
					raise ParsingException("a children type must be parsable")

				# Consider it to be a list
				child = BindList(type=child)

			# Fill missing [to]
			if child.to is None:
				child.to = child.xml_name

			self.children.append(child)

	def __call__(self, class_type):
		'''
			Create class with parsing functions
		'''
		self.generated = class_type

		# Add parsing function
		class_type.parse_string = self.parse_string
		class_type.parse = self.parse
		class_type.parse_file = self.parse_file

		class_type.xml_name = self.name

		# Apply SELF type on children
		for child in self.children:
			if child.type == Parsable.SELF:
				child.type = class_type

		return class_type


	def parse_string(self, data: str, obj = None, context = {}):
		root = ElementTree.fromstring(data)
		return self.parse(root, obj, context)

	def parse_file(self, path: str, package="ai_description", obj = None, context = {}):
		# Find mapping file directory
		source_folder = rospkg.RosPack().get_path(package)

		if path[0] != "/":
			path = "/" + path

		# Parsing mapping file
		root = ElementTree.parse(source_folder + path).getroot()

		return self.parse(root, obj, context)

	def parse(self, root: ElementTree.Element, obj: Union[type, None] = None, context: Union[dict, Context] = {}, inherited=False, xml_name=None):
		"""
			Parse a XML element to create an object of decorated type
		"""
		if isinstance(context, dict):
			context = Context(**context)

		if obj == None:
			obj = self.generated()

		# Manage inheritance
		if self.extends is not None:
			if hasattr(self.extends, "parse"):
				obj = getattr(self.extends, "parse")(root, obj, context, True)
			else:
				raise ParsingException("extends is not a parsable object")

		self.__parse_attributes(root, obj)
		self.__parse_content(root, obj)

		if not inherited:
			# Parse name only in bare class
			self.__parse_name(root, obj, xml_name)

			# Call parsed callback
			if hasattr(obj, "__parsed__"):
				getattr(obj, "__parsed__")(context)

		# Then parse children
		self.__parse_children(root, obj, context)

		# Call parsed callback
		if hasattr(obj, "__parsed_children__"):
			getattr(obj, "__parsed_children__")(context)

		return obj

	def __parse_name(self, root: ElementTree.Element, obj, possible_xml_name: str = None):
		# Handle element tag
		if isinstance(self.name, str):
			# Check for invalid name
			if root.tag != self.name and root.tag != possible_xml_name:
				raise ParsingException("invalid element {}, expected {}".format(root.tag, self.name))
		else:
			# Apply name property
			self.name.apply(obj, root.tag)

	def __parse_attributes(self, root: ElementTree.Element, obj):
		# Handle attributes
		for attr in self.attributes:
			# If attribute is registered
			if attr in root.attrib:
				self.attributes[attr].apply(obj, root.attrib[attr])
			elif self.attributes[attr].mandatory:
				# Otherwise throw if mandatory
				raise ParsingException("missing attribute {} in {}".format(attr, root.tag))

	def __parse_children(self, root: ElementTree.Element, obj, context: Context):
		# Copy context with new parent
		context = Context(**context.params)
		context.parent = obj

		# Handle children
		for child in root:
			generic: Union[AnyBinding, None] = None
			handled = False

			# Ignore children that are supposed to be ignored
			if child.tag in self.ignored_children:
				continue 

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

					available.apply(obj, child, context)
					handled = True

			# Apply generic element if any and not handled yet
			if not handled:
				if generic is not None:
					generic.apply(obj, child, context)
				else:
					raise ParsingException("unable to handle {} element in {}".format(child.tag, root.tag))

	def __parse_content(self, root: ElementTree.Element, obj):
		# Handle content
		if self.content is not None:
			self.content.apply(obj, root.text if root.text is not None else "")

