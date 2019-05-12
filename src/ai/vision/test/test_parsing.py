#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

from vision import MapObject, Rect, Circle, MapObjectList
from vision.map import RectZone, CircleZone, Offset, Symmetry

from xml_class_parser import ParsingException

class TestParsing(unittest.TestCase):
	def setUp(self):
		# Init thing object
		self.thing = MapObject()
		self.thing.x = 35
		self.thing.name = "thing"
		self.thing.shape = Rect(30, 20)
		self.thing.set("size", "int")

	def test_circle_shape(self):
		shape = Circle.parse_string('<circle radius="20" />')

		self.assertEqual(type(shape), Circle)
		self.assertEqual(shape.radius, 20)
	
	def test_rect_shape(self):
		shape = Rect.parse_string('<rect width="20" height="80" />')

		self.assertEqual(type(shape), Rect)
		self.assertEqual(shape.width, 20)
		self.assertEqual(shape.height, 80)

	def test_object_parsing(self):
		obj = MapObject.parse_string("""
			<thing x="35">
				<rect width="30" height="20" />

				<arg name="size" type="int" />
			</thing>
		""")

		self.assertIsInstance(obj, MapObject)

		self.assertEqual(obj.x, self.thing.x)
		self.assertEqual(obj.y, self.thing.y)

		self.assertEqual(obj.name, self.thing.name)
		self.assertEqual(obj.shape.width, self.thing.shape.width)
		self.assertEqual(obj.shape.height, self.thing.shape.height)

		self.assertEqual(obj.get("size"), "int")
	
	def test_object_inherited_parsing(self):
		# Parse without context
		obj_list = MapObjectList.parse_string("""
			<objects>
				<thing x="35">
					<rect width="30" height="20" />
					<arg name="size" type="int" />
				</thing>
				<amazing-thing extends="thing">
					<circle radius="50" />
				</amazing-thing>
			</objects>
		""")

		self.assertIn("thing", obj_list.objects)
		self.assertIn("amazing-thing", obj_list.objects)

		obj = obj_list.objects["amazing-thing"]

		self.assertIsInstance(obj, MapObject)
		
		# Inherit x and y too
		self.assertEqual(obj.x, self.thing.x, "inherit x")
		self.assertEqual(obj.y, self.thing.y, "inherit y")

		self.assertEqual(obj.name, "amazing-thing")
		self.assertEqual(obj.shape.radius, 50)

		self.assertEqual(obj.get("size"), "int")

	def test_object_inherited_parsing_reject(self):
		with self.assertRaises(ParsingException):
			MapObjectList.parse_string('<objects><lol extends="nothing" /></objects>')

	def test_map_parsing(self):
		result = RectZone.parse_string('''
			<rect x="0" y="0" height="3000" width="2000" blocking="true">
				<symmetry axis="y" offset="1500" source="up" target="down" />

				<rect x="1400" y="1500" width="200" height="20" blocking="true" />
				<rect x="1543" y="450" width="57" height="778" />
			</rect>
		''')

		print(str(result))


if __name__ == '__main__':
	rospy.init_node('test_vision_parsing', anonymous=True)
	rostest.rosrun('action_manager', 'test_vision_parsing', TestParsing, sys.argv)
