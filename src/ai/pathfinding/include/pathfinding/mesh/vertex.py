from . import Edge

from typing import List

class Vertex:
	pass

class GhostVertex(Vertex):
	"""
		Representation of a point placed at infinity; used to define
		ghost faces on the outter perimeter of any mesh/hull
	"""
	pass

class MeshVertex(Vertex):
	"""
		Main representation of a point; holds info on x,y position and
		points to one edge connected to it.

		If the vertex is on the border of a convex hull, it must point 
		to the interiorboundry edge (turning clockwise)
	"""
	def __init__(self, x: float, y: float, edge: Edge = None):
		self.__pos = [x,y]
		self.edge = edge

	@property
	def pos(self) -> List[float]:
		return self.__pos

	@property
	def edge(self) -> Edge:
		return self.__edge
	
	@edge.setter
	def edge(self, edge: Edge):
		self.__edge = edge