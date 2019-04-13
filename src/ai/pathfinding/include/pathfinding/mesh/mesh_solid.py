from typing import List
from .vertex import Vertex

class MeshSolid():
	"""
		?
	"""
	def __init__(self, vertices: List[Vertex]):
		self._hull = vertices

	def get_hull(self) -> List[Vertex]:
		return self._hull

	def set_hull(self, vertices: List[Vertex]):
		self._hull = vertices