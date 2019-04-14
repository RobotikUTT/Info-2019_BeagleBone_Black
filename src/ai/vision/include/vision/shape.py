from xml.etree import ElementTree


class Shape:
	def __init__(self, x: int = -1, y: int = -1):
		self.x: int = x
		self.y: int = y

	@staticmethod
	def parse(element: ElementTree.Element, current_shape: 'Shape' = None, allow_any: bool = False) -> 'Shape':
		"""
			Parse the shape of an object from it's XML element

			Currently 2 shapes are supported : rects and circles. The
			function keep the curent shape (if defined, propbable shape
			from the parent obejct), and add properties given in XML
		"""
		# Make sure the right shape class is used
		if current_shape == None:
			current_shape = Shape()

		if element.tag == "rect" and type(current_shape) is not RectShape:
			# Keep x, y
			current_shape = RectShape(current_shape.x, current_shape.y)
		elif element.tag == "circle" and type(current_shape) is not CircleShape:
			# Keep x, y
			current_shape = CircleShape(current_shape.x, current_shape.y)

		elif element.tag != "circle" and element.tag != "rect":
			# No shape nor anything else
			raise Exception(
				"{} does not name an object parameter type"
					.format(element.tag)
			)

		# Then parse shape attributes
		for attr in element.attrib:
			if hasattr(current_shape, attr):
				# Copy value as int (raise error if wrong)
				setattr(current_shape, attr, int(element.attrib[attr]))
			elif not allow_any: # if it does not allow other parameters
				raise Exception(
					"{} attribute not defined for a {} shape".format(attr, element.tag)
				)
		
		return current_shape

	def __str__(self):
		return "Shape ({}, {})".format(self.x, self.y)

class CircleShape(Shape):
	def __init__(self, x: int = -1, y: int = -1):
		super().__init__(x, y)

		self.radius: int = -1

	def __str__(self):
		return "Circle ({}, {}, r={})".format(self.x, self.y, self.radius)

class RectShape(Shape):
	def __init__(self, x: int = -1, y: int = -1):
		super().__init__(x, y)

		self.width: int = -1
		self.height: int = -1

	def __str__(self):
		return "Rect ({}, {}, w={}, h={})".format(self.x, self.y, self.width, self.height)