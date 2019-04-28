#!/usr/bin/python3
import unittest

from vision import ObjectsParser, MapParser, MapObject,\
	Shape, RectShape, CircleShape, MapObjectAction, MapObjectArgument
from vision.shape import Point
from xml.etree import ElementTree as ET

class TestShapeParser(unittest.TestCase):
	def test_circle_shape(self):
		el = ET.fromstring('<circle x="30" y="40" radius="20" />')
		shape = Shape.parse(el, None, False)

		self.assertEqual(type(shape), CircleShape)
		self.assertEqual(shape.x, 30)
		self.assertEqual(shape.y, 40)
		self.assertEqual(shape.radius, 20)
	
	def test_rect_shape(self):
		el = ET.fromstring('<rect x="30" y="40" width="20" height="80" />')
		shape = Shape.parse(el, None, False)

		self.assertEqual(type(shape), RectShape)
		self.assertEqual(shape.x, 30)
		self.assertEqual(shape.y, 40)
		self.assertEqual(shape.width, 20)
		self.assertEqual(shape.height, 80)

class TestObjectParser(unittest.TestCase):
	def setUp(self):
		self.parser = ObjectsParser()

		# Init thing object
		self.thing = MapObject()
		self.thing.name = "thing"
		self.thing.shape = RectShape(Point(35, -1), 30, 20)
		self.thing.actions.append(
			MapObjectAction("yes")
		)
		self.thing.args["size"] = MapObjectArgument("size", "int")

		# Amazing-thing object
		self.amazing_thing = MapObject()
		self.amazing_thing.name = "amazing-thing"
		self.amazing_thing.shape = CircleShape(Point(35, -1), 50)
		self.amazing_thing.args["size"] = MapObjectArgument("size", "int")

	def test_object_parsing(self):
		el = ET.fromstring("""
			<thing>
				<rect x="35" width="30" height="20" />
				<argument name="size" type="int" />
				<action file="yes" />
			</thing>
		""")

		self.assertTrue(self.parser.parse_object(el))
		self.assertEqual(self.parser.objects["thing"], self.thing)
	
	def test_object_inherited_parsing(self):
		# Load thing into parser
		self.parser.objects["thing"] = self.thing

		el = ET.fromstring("""
			<amazing-thing extends="thing">
				<circle radius="50" />
			</amazing-thing>
		""")
		self.assertTrue(self.parser.parse_object(el))
		self.assertEqual(self.parser.objects["amazing-thing"], self.amazing_thing)

		# Unload thing
		self.parser.objects = {}

	def test_object_inherited_parsing_reject(self):
		self.assertFalse(
			self.parser.parse_object(ET.fromstring('<lol extends="nothing" />'))
		)

if __name__ == "__main__":
	unittest.main()