from xml.etree import ElementTree

import rospkg

class ParsingException(Exception):
	pass

class Parser:
	"""
		Generic XML parser for ROS description files
	"""

	def parse_string(self, data: str):
		root = ElementTree.fromstring(data)
		self.parse(root)

	def parse_description_file(self, path):
		# Find mapping file directory
		source_folder = rospkg.RosPack().get_path("ai_description")

		if path[0] != "/":
			path = "/" + path

		# Parsing mapping file
		root = ElementTree.parse(source_folder + path).getroot()

		self.parse(root)

	def parse(self, root: ElementTree.Element):
		pass
		