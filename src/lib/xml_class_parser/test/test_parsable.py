#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

from xml_class_parser import Parsable, Bind, BindList, BindDict, Enum, ParsingException, Slice, Context

from typing import Dict, List

############################################################################
############################ TESTED CLASSES ################################
############################################################################

class SomeClass:
	def __init__(self):
		self.str_attr = "default"
		self.int_attr = 36
		self.array_attr = []
		self.dict_attr = {}

	def __parsed__(self, context: Context):
		if "name" in context.params:
			self.str_attr = context.get("name")

	def __before_children__(self, context: Context):
		context.set("bob", 76)

class YetAnotherClass(SomeClass):
	def __init__(self, oui=False, wow=0):
		super().__init__()
		self.oui = oui
		self.wow = wow
		
		self.get_called = False
		self.set_called = False

	def __parsed__(self, context: Context):
		self.oui = True
		
		if "bob" in context.params:
			self.wow = context.get("bob")

	def __before_children__(self, ctx):
		pass

	@property
	def x(self):
		self.get_called = True
		return 5
	
	@x.setter
	def x(self, x):
		self.set_called = True

class SomeOtherClass:
	def __init__(self):
		self.name = ""
		self.value = 0
		self.contained: SomeClass = None
		self.children: List[SomeClass] = []

############################################################################
############################## TEST SUITE ##################################
############################################################################

class TestElementParser(unittest.TestCase):
	def setUp(self):
		rospy.init_node('test_parsable', anonymous=True)
		
	def test_invalid_name_exception(self):
		global SomeClass
		SomeClass = Parsable(
			name="yes"
		)(SomeClass)

		with self.assertRaises(ParsingException):
			SomeClass.parse_string("<no />")

	def test_simple_binding(self):
		global SomeClass
		SomeClass = Parsable(name="i", attributes={"int_attr": int})(SomeClass)

		self.assertEqual(SomeClass.parse_string("<i int_attr='4' />").int_attr, 4)

	def test_ignoring_children(self):
		global YetAnotherClass
		YetAnotherClass = Parsable(name="o", ignored_children=["a"])(YetAnotherClass)

		YetAnotherClass.parse_string("<o><a></a><a></a></o>")

	def test_context_parent(self):
		raise NotImplementedError()

	def test_property_call(self):
		"""Test that a class property is not override by parsed values"""
		global YetAnotherClass
		YetAnotherClass = Parsable(name=Bind(to="x", type=Enum(binding = {"zero": 0})))(YetAnotherClass)

		parsed = YetAnotherClass.parse_string('<zero />')

		self.assertIsInstance(parsed, YetAnotherClass)
		self.assertEqual(parsed.set_called, True, "setter called on parsing")
		self.assertEqual(parsed.x, 5, "property response unchanged")
		self.assertEqual(parsed.get_called, True, "getter called")

	def test_takes_external_object(self):
		global YetAnotherClass
		YetAnotherClass = Parsable(name="minus_one")(YetAnotherClass)

		# Try to set wow to 4 and oui (first arg) to True
		parsed = YetAnotherClass.parse_string("<minus_one />", obj=YetAnotherClass(True, wow=4))

		self.assertEqual(parsed.oui, True)
		self.assertEqual(parsed.wow, 4)

	def test_before_children_callback(self):
		global YetAnotherClass
		YetAnotherClass = Parsable(name="minus_two")(YetAnotherClass)

		# Try to set wow to 4 and oui (first arg) to True
		parsed = YetAnotherClass.parse_string("<minus_two />", YetAnotherClass(oui=False))

		self.assertEqual(parsed.oui, True, "call __before_children__ after parsing properties and before children")

	def test_context_passing(self):
		global SomeClass
		global YetAnotherClass

		YetAnotherClass = Parsable(name="array_attr")(YetAnotherClass)
		SomeClass = Parsable(name="o", children = [ BindList(type=YetAnotherClass, to="array_attr") ])(SomeClass)

		parsed = SomeClass.parse_string("<o><array_attr /></o>", context=Context(name="hello"))

		self.assertEqual(parsed.str_attr, "hello", "original context is passed")
		self.assertEqual(len(parsed.array_attr), 1)
		self.assertEqual(parsed.array_attr[0].wow, 76, "context is passed with additional args")

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

	def test_mandatory(self):
		global SomeClass

		SomeClass = Parsable(name="lowl", attributes={"name": Bind(mandatory=True, to="str_attr")})(SomeClass)
		
		parsed = SomeClass.parse_string("<lowl name='bob' />")

		self.assertIsInstance(parsed, SomeClass)
		self.assertEqual(parsed.str_attr, "bob")

		with self.assertRaises(ParsingException):
			SomeClass.parse_string("<lowl />")

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

	def test_alias_parsing(self):
		raise NotImplementedError()

	def test_call_parsed_callback(self):
		global YetAnotherClass

		YetAnotherClass = Parsable(name="a")(YetAnotherClass)

		self.assertTrue(YetAnotherClass.parse_string("<a />").oui)

if __name__ == '__main__':
	rostest.rosrun('xml_class_parser', 'test_parsable', TestElementParser, sys.argv)
