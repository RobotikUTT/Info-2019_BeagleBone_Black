from . import Edge

class Face:
	def __init__(self, edge=None):
		self.edge = edge

	@property
	def edge(self) -> Edge:
		return self.__edge

	@edge.setter
	def edge(self, edge: Edge):
		self.__edge = edge

class GhostFace(Face):
	"""
		Representation of a triangle with a vertex at infinity.
		Those tiangles are attached on the exterior of a mesh/hull.

		and allow to circulate around in the anti-clockwise direction
	"""

	# def __del__(self):
	#	 print('ghost face deleted')
	pass

class MeshFace(Face):
	"""
		Main representation of a triangle. Counter-intuitively,
		only holds a reference to one of its edges.
		
		with that edge as an entry point in the graph from where
		to retreive the other edges of the triangle it is enough
	"""
	pass