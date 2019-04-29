#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

from xml_class_parser import Parsable, Bind, BindList, BindDict, Enum, ParsingException, Slice

from typing import Dict, List

class SomeClass:
	def __init__(self):
		self.str_attr = "default"
		self.int_attr = 36
		self.array_attr = []
		self.dict_attr = {}

class SomeOtherClass:
	def __init__(self):
		self.name = ""
		self.value = 0
		self.contained: SomeClass = None
		self.children: List[SomeClass] = []

class TestElementParser(unittest.TestCase):

	def test_invalid_name_exception(self):
		global SomeClass
		SomeClass = Parsable(
			name="yes"
		)(SomeClass)

		with self.assertRaises(ParsingException):
			SomeClass.parse_string("<no />")

	def test_attribute_parsing(self):
		global SomeClass
		SomeClass = Parsable(
			name="one", attributes={ "name": Bind(to="str_attr", type=str) }
		)(SomeClass)

		parsed = SomeClass.parse_string("<one name='bob' />")

		self.assertIsInstance(parsed, SomeClass)
		self.assertEqual(parsed.str_attr, "bob")

	def test_content_parsing(self):
		global SomeClass
		SomeClass = Parsable(name="two", content=Bind(type=int, to="int_attr"))(SomeClass)

		parsed = SomeClass.parse_string("<two>84</two>")

		self.assertIsInstance(parsed, SomeClass)
		self.assertEqual(parsed.int_attr, 84)

	def test_enum_binding_parsing(self):
		global SomeClass
		enum = Enum(binding={"one": 1, "two": 2, "three": 3, "four": 4, "five": 5})
		SomeClass = Parsable(name="two", content=Bind(type=enum, to="int_attr"))(SomeClass)

		# Parsing success
		parsed = SomeClass.parse_string("<two>one</two>")

		self.assertIsInstance(parsed, SomeClass)
		self.assertEqual(parsed.int_attr, 1)

		# Parsing error
		with self.assertRaises(ParsingException):
			SomeClass.parse_string("<two>zero</two>")

	def test_enum_values_parsing(self):
		global SomeClass
		enum = Enum(values=[5, 6, 7, 8, 9], cast=int)
		SomeClass = Parsable(name="three", attributes={"value": Bind(type=enum, to="int_attr") })(SomeClass)

		# Parsing success
		parsed = SomeClass.parse_string("<three value='7' />")

		self.assertIsInstance(parsed, SomeClass)
		self.assertEqual(parsed.int_attr, 7)

		# Parsing error
		with self.assertRaises(ParsingException):
			SomeClass.parse_string("<three value='3' />")

	def test_recursive_parsing(self):
		global SomeClass
		SomeClass = Parsable(name="four", children=[ BindList(type=Parsable.SELF, to="array_attr") ])(SomeClass)

		# Parsing success
		parsed = SomeClass.parse_string("<four><four></four></four>")

		self.assertIsInstance(parsed, SomeClass)
		self.assertEqual(len(parsed.array_attr), 1)
		self.assertIsInstance(parsed.array_attr[0], SomeClass)

		# Parsing error
		with self.assertRaises(ParsingException):
			SomeClass.parse_string("<three value='3' />")
	
	def test_list_parsing(self):
		global SomeClass
		global SomeOtherClass

		SomeClass = Parsable(name="in", attributes={ "v": Bind(to="int_value", type=int) })(SomeClass)
		SomeOtherClass = Parsable(name="box", children = [ BindList(to="children", type=SomeClass) ])(SomeOtherClass)
		
		parsed: SomeOtherClass = SomeOtherClass.parse_string("""
			<box>
				<in v="1" />
				<in v="2" />
				<in v="3" />
			</box>
		""")

		self.assertIsInstance(parsed, SomeOtherClass, "class parsed")
		self.assertEqual(len(parsed.children), 3, "right number of children")

		for i in range(3):
			child = parsed.children[i]

			self.assertIsInstance(child, SomeClass, "children has right class")
			self.assertEqual(child.int_value, i + 1, "values are parsed in order")

	def test_dict_parsing(self):
		global SomeClass
		global SomeOtherClass

		SomeOtherClass = Parsable(name=Bind(to="name"), content=Bind(to="value", type=int))(SomeOtherClass)
		SomeClass = Parsable(name="box", children=[ BindDict(to="dict_attr", key="name", type=SomeOtherClass) ])(SomeClass)
		
		parsed: SomeClass = SomeClass.parse_string("""
			<box>
				<oui>5</oui>
				<non>21</non>
				<maybe>37</maybe>
			</box>
		""")

		self.assertIsInstance(parsed, SomeClass, "right class parsed")
		self.assertEqual(len(parsed.dict_attr.keys()), 3, "right number of keys")

		for key in parsed.dict_attr:
			if key == "oui":
				self.assertEqual(parsed.dict_attr[key].value, 5)
			elif key == "non":
				self.assertEqual(parsed.dict_attr[key].value, 21)
			elif key == "maybe":
				self.assertEqual(parsed.dict_attr[key].value, 37)

	def test_slicing(self):
		global SomeClass
		global SomeOtherClass

		SomeOtherClass = Parsable(name=Bind(to="name"), content=Bind(to="value", type=int))(SomeOtherClass)
		SomeClass = Parsable(name="box", children=[ BindDict(to="dict_attr", key="name", type=SomeOtherClass, post_cast=Slice("value")) ])(SomeClass)
		
		parsed: SomeClass = SomeClass.parse_string("""
			<box>
				<oui>5</oui>
				<non>21</non>
				<maybe>37</maybe>
			</box>
		""")

		self.assertIsInstance(parsed, SomeClass, "right class parsed")
		self.assertEqual(len(parsed.dict_attr.keys()), 3, "right number of keys")

		for key in parsed.dict_attr:
			if key == "oui":
				self.assertEqual(parsed.dict_attr[key], 5)
			elif key == "non":
				self.assertEqual(parsed.dict_attr[key], 21)
			elif key == "maybe":
				self.assertEqual(parsed.dict_attr[key], 37)
	
	def test_loop_parsing(self):
		global SomeClass
		global SomeOtherClass

		SomeOtherClass = Parsable(name="babana", children=[ BindList(to="children", type=SomeClass) ])(SomeOtherClass)
		SomeClass = Parsable(name="appel", children=[ BindList(to="array_attr", type=SomeOtherClass) ])(SomeClass)
		
		parsed: SomeClass = SomeClass.parse_string("""
			<appel>
				<babana>
					<appel>
						<babana />
					</appel>
				</babana>
				<babana/>
			</appel>
		""")

		self.assertIsInstance(parsed, SomeClass)
		self.assertEqual(len(parsed.array_attr), 2)

		# First depth
		self.assertEqual(len(parsed.array_attr[0].children), 1)
		self.assertEqual(len(parsed.array_attr[1].children), 0)

		# Second depth
		self.assertEqual(len(parsed.array_attr[0].children[0].array_attr), 1)

		self.assertEqual(len(parsed.array_attr[0].children[0].array_attr[0].children), 0)


if __name__ == '__main__':
	rostest.rosrun('xml_class_parser', 'test_parser', TestElementParser, sys.argv)
