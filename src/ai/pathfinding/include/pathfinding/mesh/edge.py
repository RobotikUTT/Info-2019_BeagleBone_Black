from . import Face, Vertex

class Edge:
	def __init__(self, face: Face=None, next_edge: Edge=None, opp_edge: Edge=None):
		self.opp_edge = opp_edge
		self.face = face
		self.next_edge = next_edge

	# Opposite edge
	@property
	def opp(self) -> Edge:
		return self.__opp

	@opp.setter
	def opp(self, opp: Edge):
		self.__opp = opp

	# Face
	@property
	def face(self) -> Face:
		return self.__face
	
	@face.setter
	def face(self, face: Face):
		self.__face = face

	# Next edge
	@property
	def next(self) -> Edge:
		return self.__next
	
	@next.setter
	def next(self, next: Edge):
		self.__next = next


class GhostEdge(Edge):
	"""
		Representation of an externat triangle pointing to a vertex at infinity.
		A ghost edge should always be opposed to a ghost edge in such a way that it allows turning anti-clockwise arond a mesh/hull
	"""
	def __init__(self, vertex: Vertex, face: Face = None, next_edge: GhostEdge = None, opp_edge: Edge = None):
		super().__init__(face, next_edge, opp_edge)

		self.vertex = vertex

	# def __del__(self):
	#	 print('ghost edge deleted')

	@property
	def vertex(self) -> Vertex:
		return self.__vertex
	
	@vertex.setter
	def vertex(self, vertex: Vertex):
		self.__vertex = vertex

class MeshEdge(Edge):
	"""
		Main representation of a segment in the mesh.
		
		The edges are the core component of triangles and thus of the graph.
		They are defined by an opposite edge, a next edge, a destination vertex and an associated faces(triangle).
		They have to be defined in a geometricaly coherent process for the overwhole mesh to mesh sense.
		Defining triangles means that the next,next,next edge should be the same edge, and edges of  triangle should always
		be defined clockwise.

		Also, a mesh edge should always oppose a mesh edge

		Edges can represent obstacles or not. If so the pathfinding algorithm will be forbiden to cross though it.
	"""

	def __init__(self, vertex_dest, face=None, opp_edge: MeshEdge = None, next_edge=None, is_obstacle=False):
		super().__init__(face, next_edge, opp_edge)

		self.dest = vertex_dest
		self.is_obstacle = is_obstacle

	@property
	def dest(self) -> Vertex:
		return self.__dest
	
	@dest.setter
	def dest(self, dest: Vertex):
		self.__dest = dest

	
	@property
	def obstacle(self) -> bool:
		return self.__obstacle
	
	@obstacle.setter
	def obstacle(self, obstacle: bool):
		self.__obstacle = obstacle

