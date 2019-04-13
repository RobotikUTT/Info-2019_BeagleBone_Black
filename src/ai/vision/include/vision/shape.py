
class Shape:
	def __init__(self, x: int = -1, y: int = -1):
		self.x: int = x
		self.y: int = y

class CircleShape(Shape):
	def __init__(self, x: int = -1, y: int = -1):
		super().__init__(x, y)

		self.radius: int = -1

class RectShape(Shape):
	def __init__(self, x: int = -1, y: int = -1):
		super().__init__(x, y)

		self.width: int = -1
		self.height: int = -1