#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

from xml_class_parser import Parsable, Bind, BindList, BindDict

from typing import Dict, List

# Some class supposed to be contained in Container
@Parsable(
	name = Bind(to="name"),
	attributes = {
		"al": Bind(type=str, to="alias", mandatory=True)
	},
	content = Bind(to="value", type=int)
)
class Contained:
	def __init__(self):
		self.name = ""
		self.alias = "lol"
		self.value = 3

# Some container
@Parsable(
	name = "container",
	attributes = { "number": int, "string": str },
	children = {
		"@": BindList(type=Contained, to="array"),
		"lol": BindDict(type=Contained, to="chidren", key="alias")
	}
)
class Container:
	def __init__(self):
		self.number: int = 3
		self.string: str = ""

		self.children: Dict[str, float] = {}
		self.array: List[Contained] = []

class TestElementParser(unittest.TestCase):

	def test_parsing(self):
		parsed: Contained = getattr(Contained, "parse_string")("<el_name al='oui'>45</el_name>")
		
		self.assertEqual(parsed.name, "el_name", "parse element tag inside object")
		self.assertEqual(parsed.alias, "oui", "parse element attribute inside object")
		self.assertEqual(parsed.value, 45, "parse element content in object")
	
	def test_recursive_parsing(self):
		self.assertTrue(False, "parse itself recursively")
	
	def test_container_parsing(self):
		self.assertTrue(False, "parse parsable objects in object")
if __name__ == '__main__':
	rostest.rosrun('xml_class_parser', 'test_parser', TestElementParser, sys.argv)
