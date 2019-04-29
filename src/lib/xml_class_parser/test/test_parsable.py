#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

from xml_class_parser import Parsable, Bind, BindList, BindDict, Enum, ParsingException

from typing import Dict, List

class SomeClass:
	def __init__(self):
		self.str_attr = "default"
		self.int_attr = 36
		self.array_attr = []
		self.dict_attr = {}

class TestElementParser(unittest.TestCase):

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
		enum = Enum(values=[5, 6, 7, 8, 9], cast=int)
		SomeClass = Parsable(name="four", children=[ BindList(type=Parsable.SELF, to="array_attr") ])(SomeClass)

		# Parsing success
		parsed = SomeClass.parse_string("<four><four></four></four>")

		self.assertIsInstance(parsed, SomeClass)
		self.assertEqual(len(parsed.array_attr), 1)
		self.assertIsInstance(parsed.array_attr[0], SomeClass)

		# Parsing error
		with self.assertRaises(ParsingException):
			SomeClass.parse_string("<three value='3' />")
	
	def test_container_parsing(self):
		self.assertTrue(False, "parse parsable objects in object")
if __name__ == '__main__':
	rostest.rosrun('xml_class_parser', 'test_parser', TestElementParser, sys.argv)
