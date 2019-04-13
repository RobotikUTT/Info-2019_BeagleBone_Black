
from typing import List

from . import Face, Vertex, Edge, \
	MeshVertex, MeshFace, MeshEdge, \
	GhostFace, GhostVertex, GhostEdge, MeshSolid

from .collision import circumcircle_check, clockwise_check, colinear_check, cone_check,\
	inside_triangle_check, intersect_check, on_segment_check

from .util import merge_meshes, edge_flip, dist_point_segment
from .create import create_isolated_edge, create_isolated_face

class MeshMap():
	def __init__(self, mesh={'faces': [],'verties': [],'grid': []}, grid_len=None):

		if type(mesh) is list:
			self._vertices: List[Vertex] = []
			self._faces: List[Face] = []
			self._grid = []
			self._grid_len = grid_len
			self.divide_and_conquer(sorted(mesh, key=lambda k: [k[0], k[1]]))
			self._solids = []
		elif type(mesh) is dict and 'faces' in list(mesh.keys()) and 'vertices' in list(mesh.keys()):
			self._vertices = mesh['faces']
			self._faces = mesh['vertices']
			if 'grid' in list(mesh.keys()):
				self._grid = mesh['grid']
			self._solids = []

		if self._vertices != [] and self._grid_len is not None:
			self.grid_sampling()
			self.fill_grid()

	def get_vertices(self):
		return self._vertices

	def get_faces(self):
		return self._faces




	def grid_sampling(self):
		#generates a square grid that holds references to vertices as entry points in the graph
		#it is usefull to quistart the search to find in wich triangle lies a point of coordinates x,y
		#the resolution depends on the square_len and the functions associate each square with any vertex it contains
		#the final grid has one vertex by square, zero if there is none
		sampled_vertices = [[None]]
		for vertex in self._vertices:
			if type(vertex) is MeshVertex:
				i = int(vertex.pos[0]//self._grid_len)
				j = int(vertex.pos[1]//self._grid_len)
				if i >= len(sampled_vertices):
					x_extend = []
					for k in list(range(len(sampled_vertices[-1]))):
						x_extend.append(None)
					for extend_len in list(range(i-len(sampled_vertices)+1)):
						sampled_vertices.append(x_extend)
				if j >= len(sampled_vertices[-1]):
					for l in list(range(len(sampled_vertices))):
						for extend_len in list(range(j- len(sampled_vertices[l])+1)):
							sampled_vertices[l].append(None)
				if sampled_vertices[i][j] is None:
					sampled_vertices[i][j] = vertex
		self._grid = sampled_vertices

	def fill_grid(self):
		#an expansion function that fills the grid whenever the value of a square isn't set
		#it will then copy the vertex reference of its closest neighbour until all the grid is filled
		grid_filled = False
		while not grid_filled:
			print('grid fill')
			none_remaining = False
			for i in list(range(len(self._grid))):
				for j in list(range(len(self._grid[i]))):
					if self._grid[i][j] is None:
						closest_sample = None
						if i == 0:
							ibound = [+1]
						elif  i == len(self._grid) -1:
							ibound = [-1]
						else:
							ibound = [+1, -1]
						if j == 0:
							jbound = [+1]
						elif  j == len(self._grid[i]) -1:
							jbound = [-1]
						else:
							jbound = [+1, -1]

						for di in ibound:
							for dj in jbound:
								if self._grid[i+di][j+dj] is not None:
									if closest_sample is None:
										closest_sample = self._grid[i+di][j+dj]
									else:
										if pow(self._grid[i+di][j+dj].pos[0]-self._grid_len*(i+1/2), 2) + pow(self._grid[i+di][j+dj].pos[1]-self._grid_len*(j+1/2), 2) \
												< pow(closest_sample.pos[0]-self._grid_len*(i+1/2),2)+pow(closest_sample.pos[1]-self._grid_len*(j+1/2),2):
											closest_sample = self._grid[i+di][j+dj]
						if closest_sample is None:
							none_remaining = True
						else:
							self._grid[i][j] = closest_sample
			if not none_remaining:
				grid_filled = True

	def find_face(self, x_pos, start_vertex=None, face=None):
		#given a point's coordinates x,y, find the face in the mesh that point lies in
		#must be given a start vertex as an entry point in the graph
		# (the closer the entry point, thefaster the face will be found)
		if start_vertex is None:
			if self._grid != [] and self._grid_len is not None:
				start_vertex = [int(x_pos[0]//self._grid_len)][int(x_pos[1]//self._grid_len)]
			else:
				start_vertex = self._vertices[0]

		if face is None:
			#if face is not set, pick any face around the starting vertex
			face = start_vertex.edge.get_face()
			return self.find_face(x_pos, start_vertex, face)
		else:
			#TODO: check if the x_pos is a vertex position or on an edge!!
			if type(face) is GhostFace:
				border_edge = face.edge
				if border_edge.opp.dest == start_vertex:
					if not cone_check(start_vertex.pos, border_edge.dest.pos,
									  border_edge.next.next.opp.next.next.opp.dest.pos , x_pos):
						return self.find_face(x_pos, start_vertex, border_edge.next.next.opp.next.next.opp.get_face())
					else:
						return face
				elif border_edge.dest == start_vertex:
					if not cone_check(start_vertex.pos, border_edge.next.opp.next.opp.dest.pos,
									  border_edge.opp.dest.pos , x_pos):
						return self.find_face(x_pos, start_vertex, border_edge.opp.get_face())
					else:
						return face
				else:
					print('oopsys')
			else:
				#identify the edge of the current face at the opposite of the starting vertex
				crossing_edge = face.edge
				if crossing_edge.dest == start_vertex:
					crossing_edge = crossing_edge.next.next
				elif crossing_edge.next.next.dest == start_vertex:
					crossing_edge = crossing_edge.next
				#verify if the point to localize is in the cone defined by the current face, with the starting vertex as summit
				if cone_check(start_vertex.pos, crossing_edge.next.next.dest.pos, crossing_edge.dest.pos, x_pos):

					#if the point to localize is in the direction of the cone, it can then be either
					#inside the triangle or outside and the line start/end will intersect the crossing edge
					if intersect_check(start_vertex.pos, x_pos, crossing_edge.opp.dest.pos, crossing_edge.dest.pos):

						#if the edge is crossed, then the function falls itself
						# after changing the face to be the one on the other side of the crossed edge,
						#thus getting the start vertex closer to the end vertex
						if type(crossing_edge.opp.get_face()) is MeshFace:
							return self.find_face(x_pos, crossing_edge.opp.next.dest, None)
						else:
							return crossing_edge.opp.get_face()
					else:
						#if the point to localize is in the cone and no interection happens,
						#it means that it should be inside the face, wich will be the reslut to return
						if inside_triangle_check(face.edge.dest.pos,
												 face.edge.next.dest.pos,
												 face.edge.next.next.dest.pos, x_pos):
							return face
						else:
							print('oops')
				else:
					#if the point to localize is not in the cone as defined, it means that the start/end line
					#will go though another face around the start vertex, so the function
					#is called again with a face that is the next face by rotation around the starting vertex
					return self.find_face(x_pos, start_vertex, crossing_edge.next.opp.get_face())

	def vertex_insertion(self, vertex, face, epsilon=0.001):
		#modifies the graph to insert a vertex in the mesh, given the face it will be located in

		superposition = False
		e = face.edge
		i = 0
		while not superposition and i < 3:
			if pow(e.dest.pos[0] - vertex.pos[0],2) + pow(e.dest.pos[1] - vertex.pos[1],2) < pow(epsilon,2):
				superposition = True
				vertex.edge = e.dest.edge
				e.dest.edge = None
				current_edge = e.opp.next.next
				while current_edge != e:
					if type(current_edge) is MeshEdge:
						current_edge.dest = vertex
					current_edge = current_edge.opp.next.next
				self._vertices.insert(self._vertices.index(e.dest), vertex)
				del self._vertices[self._vertices.index(e.dest)]
				e.dest = vertex
			e = e.next
			i+=1

		if not superposition:
			e = face.edge
			i = 0
			while i < 3:
				if on_segment_check(e.opp.dest.pos, e.dest.pos, vertex.pos) \
						or dist_point_segment(e.opp.dest.pos, e.dest.pos, vertex.pos) < pow(epsilon,2):
					# split the edge in two and the two opposing faces in four (each face is split in two)
					superposition = True
					face.edge = e.next.next
					new_face = MeshFace(e.next)
					e.next.face = new_face
					split_new_edge = MeshEdge(e.dest, new_face, opp_edge=e.opp, next_edge=e.next)
					vertex.edge = split_new_edge
					outward_new_edge = MeshEdge(e.next.dest, face, next_edge=e.next.next)
					inward_new_edge = MeshEdge(vertex, new_face, opp_edge=outward_new_edge, next_edge=split_new_edge)
					outward_new_edge.opp = inward_new_edge
					e.dest = vertex
					e.next = outward_new_edge

					e.opp.get_face().edge = e.opp.next.next
					opp_face = MeshFace(e.opp.next)
					e.opp.next.face = opp_face
					outward_opp_edge = MeshEdge(e.opp.next.dest, e.opp.get_face(), next_edge=e.opp.next.next)
					inward_opp_edge = MeshEdge(vertex, opp_face, opp_edge= outward_opp_edge)
					outward_opp_edge.opp = inward_opp_edge
					split_opp_edge = MeshEdge(e.opp.dest, opp_face, opp_edge=e, next_edge=e.opp.next)
					inward_opp_edge.next = split_opp_edge
					e.opp.dest = vertex
					e.opp.next = outward_opp_edge
					e.opp.opp = split_new_edge
					e.opp = split_opp_edge


					self._faces.insert(self._faces.index(face), new_face)
					self._faces.insert(self._faces.index(face), opp_face)

					vertex_index = 0
					index_found = False
					while vertex_index <= len(self._vertices) and not index_found:
						if vertex.pos[0] < self._vertices[vertex_index].pos[0]:
							vertex_index += 1
						else:
							if vertex.pos[1] < self._vertices[vertex_index].pos[1]:
								vertex_index += 1
							else:
								index_found = True
					self._vertices.insert(vertex_index, vertex)

					#after point insertion, check if the delaunay criterion is preserved in those three faces;
					# if not, flip the external edge
					ext_edge = vertex.edge.next
					for fnum in [0,1,2,3]:
						if type(ext_edge.opp.get_face()) is MeshFace:
							if not circumcircle_check(ext_edge.dest, ext_edge.next.dest, ext_edge.opp.dest, ext_edge.opp.next.dest) \
									and not ext_edge.obstacle:
								edge_flip(ext_edge)
								ext_edge = ext_edge.opp.next.next.opp.next
							else:
								ext_edge = ext_edge.next.opp.next

				e = e.next
				i+=1

		if not superposition:

			#assuming the point is indeed located in the given face, then that face should be split in three faces
			#so there will be two new faces and three new edges and opposites
			# linking the new inserted vertex with every vertex summit of the face it is inserted in

			#the entry point will be any edge of the face that face is pointing to
			#the other two edges are associted with newly created faces
			first_new_face = MeshFace(face.edge.next.next)
			face.edge.next.next.face = first_new_face
			second_new_face = MeshFace(face.edge.next)
			face.edge.next.face = second_new_face

			#add the first new and opposite edge after the third edge of the face
			first_new_edge = MeshEdge(face.edge.opp.dest,face, next_edge=face.edge)
			first_opp_edge = MeshEdge(vertex, first_new_face, opp_edge=first_new_edge)
			first_new_edge.opp = first_opp_edge
			vertex.edge = first_new_edge
			face.edge.next.next.next = first_opp_edge

			#add the second new edge and opposite after the second edge of the face, then close the first new face
			second_new_edge = MeshEdge(face.edge.next.dest, first_new_face,  next_edge=face.edge.next.next)
			first_opp_edge.next = second_new_edge
			second_opp_edge = MeshEdge(vertex, second_new_face, opp_edge=second_new_edge)
			second_new_edge.opp = second_opp_edge
			face.edge.next.next = second_opp_edge

			#add the third new edge and opposite after the first edge of the face, then close the second and third new faces
			third_new_edge = MeshEdge(face.edge.dest, second_new_face, next_edge=face.edge.next)
			second_opp_edge.next = third_new_edge
			third_opp_edge = MeshEdge(vertex, face, opp_edge=third_new_edge, next_edge= first_new_edge)
			face.edge.next = third_opp_edge
			third_new_edge.opp = third_opp_edge

			#insert the new faces and the new vertex in the mesh where they should be
			vertex_index = 0
			index_found = False
			while vertex_index <= len(self._vertices) and not index_found:
				if vertex.pos[0] < self._vertices[vertex_index].pos[0]:
					vertex_index += 1
				else:
					if vertex.pos[1] < self._vertices[vertex_index].pos[1]:
						vertex_index += 1
					else:
						index_found = True
			self._vertices.insert(vertex_index, vertex)
			self._faces.insert(self._faces.index(face), first_new_face)
			self._faces.insert(self._faces.index(face), second_new_face)

			#after point insertion, check if the delaunay criterion is preserved in those three faces;
			# if not, flip the external edge
			ext_edge = vertex.edge.next
			for fnum in [0,1,2]:
				if type(ext_edge.opp.get_face()) is MeshFace:
					if not circumcircle_check(ext_edge.dest, ext_edge.next.dest, ext_edge.opp.dest, ext_edge.opp.next.dest) \
							and not ext_edge.obstacle:
						edge_flip(ext_edge)
						ext_edge = ext_edge.opp.next.next.opp.next
					else:
						ext_edge = ext_edge.next.opp.next

	def edge_slice(self, start_vertex, end_vertex, face=None, epsilon=0.001, obstacle=True, added_vertices=[]):
		#modifies the graph to create a straight line between two vertices
		#it will also add a vertex on each crossed edge, effectively slicing each crossed face in three
		#the obstacle parameter can be set to give the obstacle property to ever created edge
		#the obstacle property is directionnal/oriened so any object should be a hull of edge slices
		# between vertices given in clockwise order

		#the fonction's structure resembles the one of find_face() expect it is not as passive
		#it will go though the graph the same way but will edit it on the way
		if face is None:
			face = start_vertex.edge.get_face()
			return self.edge_slice(start_vertex, end_vertex, face, epsilon, obstacle, added_vertices)

		else:
			if type(face) is GhostFace:
				border_edge = face.edge
				if border_edge.opp.dest == start_vertex:
					if not cone_check(start_vertex.pos, border_edge.dest.pos,
									  border_edge.next.next.opp.next.next.opp.dest.pos , end_vertex.pos):
						return self.edge_slice(start_vertex, end_vertex, border_edge.next.next.opp.next.next.opp.get_face(), epsilon, obstacle, added_vertices)
				elif border_edge.dest == start_vertex:
					if not cone_check(start_vertex.pos, border_edge.next.opp.next.opp.dest.pos,
									  border_edge.opp.dest.pos , end_vertex.pos):
						return self.edge_slice(start_vertex, end_vertex, border_edge.opp.get_face(), epsilon, obstacle, added_vertices)
				else:
					print('oopsys')
			else:
				#if the current face contains the end vertex, one of its edges should be the last edge of the slice
				end_edge = face.edge
				end_found = False
				i = 0
				while i < 2 and not end_found:
					if end_edge.dest.pos == end_vertex.pos:
						if end_edge.opp.dest == start_vertex:
							end_found = True
						elif end_edge.next.dest == start_vertex:
							end_edge = end_edge.next.opp
							end_found = True
					else:
						end_edge = end_edge.next
					i += 1

				if end_found:
					start_vertex.edge = end_edge
					if obstacle:
						end_edge.obstacle = True

					return added_vertices

				else:
					#TODO: check if the target line is on an edge
					#identify the external edge opposed to the current start vertex
					crossing_edge = face.edge
					if crossing_edge.dest == start_vertex:
						crossing_edge = crossing_edge.next.next
					elif crossing_edge.next.next.dest == start_vertex:
						crossing_edge = crossing_edge.next
					#check if the diretion of the end vertex is in the cone defined by the current face
					if cone_check(start_vertex.pos, crossing_edge.next.next.dest.pos, crossing_edge.dest.pos, end_vertex.pos):

						intersection = intersect_check(
							start_vertex.pos, end_vertex.pos,
							crossing_edge.opp.dest.pos, crossing_edge.dest.pos
						)

						#if so, evaluate the position of the intersection point between the crossing edge and the direction line
						#if that point lies too close to an existing vertex (distance epsilon) then that vertex sould be used instead
						if intersection != False:

							if (abs(crossing_edge.dest.pos[0] - intersection[0]) < epsilon) and (abs(crossing_edge.dest.pos[1] - intersection[1]) < epsilon):
								start_vertex.edge = crossing_edge.next.opp
								if obstacle:
									crossing_edge.next.opp.obstacle = True
									# crossing_edge.next.obstacle = True
								return self.edge_slice(crossing_edge.dest, end_vertex, None, epsilon, obstacle, added_vertices)

							elif (abs(crossing_edge.opp.dest.pos[0] - intersection[0]) < epsilon) and (abs(crossing_edge.opp.dest.pos[1] - intersection[1]) < epsilon):
								start_vertex.edge = crossing_edge.next.next
								if obstacle:
									crossing_edge.next.next.obstacle = True
									# crossing_edge.next.next.opp.obstacle = True
								return self.edge_slice(crossing_edge.opp.dest, end_vertex, None, epsilon, obstacle, added_vertices)

							#if a vertex needs to be inserted in the crossing edge then the crossed face will be split in two
							#so must be the next face, opposing the crossing edge.
							#since the next face will be split along the start/end line at the next round, when the next crossing vertex is found
							#it will now be split simply by edges between the crossing vertex and the next face summit vertex
							if type(crossing_edge.opp.get_face()) is MeshFace:
								#TODO:  add a vetex at crossing point, split both triangles on the side of the crossing edge
								#by adding edges and faces linking opping vertices, then iterate from the crossing point

								#add the crossing vertex from intersection position
								crossing_vertex = MeshVertex(intersection[0], intersection[1])
								# added_vertices.append(crossing_vertex)

								#add edges to link the starting vertex and the crossing vertexx
								inward_new_edge = MeshEdge(crossing_edge.next.dest, face= face, next_edge=crossing_edge.next.next)
								face.edge = inward_new_edge
								inward_opp_edge = MeshEdge(crossing_vertex, opp_edge=inward_new_edge)
								crossing_edge.next.dest.edge = inward_opp_edge
								#create a new face to split the current one at the newly created edges
								inward_new_face = MeshFace(inward_opp_edge)
								inward_opp_edge.face = inward_new_face
								crossing_edge.next.face = inward_new_face
								#close the start_vertex side of both faces around the new edges
								crossing_edge.next.next = inward_opp_edge
								inward_new_edge.opp = inward_opp_edge
								#split the crossing edge at the crossing vertex, then assign each edge to its face
								#(the the face that has be split for the split crossing edge and the newly created one for the new split edge)
								split_new_edge = MeshEdge(crossing_edge.dest, inward_new_face, next_edge=crossing_edge.next)
								#then close both face on that side as well
								inward_opp_edge.next = split_new_edge
								crossing_edge.next = inward_new_edge
								crossing_edge.dest = crossing_vertex

								#for the face on the opposite edge of the crossing edge to be split,
								#the split segment is the one between the crossing vertex and the summit vertex of that face
								#that edge and its opposite are created
								outward_new_edge = MeshEdge(crossing_edge.opp.next.dest, crossing_edge.opp.get_face(), next_edge=crossing_edge.opp.next.next)
								crossing_edge.opp.get_face().edge = outward_new_edge
								outward_opp_edge = MeshEdge(crossing_vertex, opp_edge=outward_new_edge)
								outward_new_edge.opp = outward_opp_edge
								#as the previous face, this one will aslo be split in two, so a new face is created
								outward_new_face = MeshFace(outward_opp_edge)
								outward_opp_edge.face = outward_new_face
								#the second new face is close at its summit end
								crossing_edge.opp.next.next = outward_opp_edge
								crossing_edge.opp.next.face = outward_new_face
								#the the oppoing edge of the crossing edge is also split in two
								split_opp_edge = MeshEdge(crossing_edge.opp.dest, outward_new_face, crossing_edge, crossing_edge.opp.next)
								#the second createdface is closed on its opposed crossing edge side
								outward_opp_edge.next = split_opp_edge
								#the two split opposing edges need to be set as opposites of the two split crossig edges
								crossing_edge.opp.opp = split_new_edge
								split_new_edge.opp = crossing_edge.opp
								#the opposing face now split can be closed on the opposed crossing side, completing te process
								crossing_edge.opp.dest = crossing_vertex
								crossing_edge.opp.next = outward_new_edge
								crossing_edge.opp = split_opp_edge

								#TODO might not stop (check stop condition) about the last jump => no edge crossing, only a link to the opposite vertex

								crossing_vertex.edge = outward_new_edge

								if obstacle:
									inward_opp_edge.obstacle = True

								#the disposition of the edges once they get split may produce fces that don't
								#comply to the delaunay criterion, so we check them along their creation
								#and proceed to edge flips if necessary
								if type(split_new_edge.next.opp.get_face()) is MeshFace:
									if clockwise_check(start_vertex, crossing_vertex, split_new_edge.next.opp.next.dest):
										if not circumcircle_check(start_vertex, crossing_vertex, split_new_edge.dest, split_new_edge.next.opp.next.dest) \
												and not split_new_edge.next.obstacle and not split_new_edge.next.opp.obstacle:

											edge_flip(split_new_edge.next)

									if (not circumcircle_check(split_new_edge.dest, split_new_edge.next.dest, split_new_edge.next.next.dest, split_new_edge.opp.next.dest) \
												or not circumcircle_check(split_new_edge.opp.dest,split_new_edge.opp.next.dest, split_new_edge.opp.next.next.dest, split_new_edge.next.dest)) \
											and not split_new_edge.obstacle and not split_new_edge.opp.obstacle:

										edge_flip(split_new_edge)

								#add the new vertices and faces to the mesh
								vertex_index = 0
								index_found = False
								while vertex_index <= len(self._vertices) and not index_found:
									if crossing_vertex.pos[0] < self._vertices[vertex_index].pos[0]:
										vertex_index += 1
									else:
										if crossing_vertex.pos[1] < self._vertices[vertex_index].pos[1]:
											vertex_index += 1
										else:
											index_found = True
								self._vertices.insert(vertex_index, crossing_vertex)
								self._faces.insert(0, inward_new_face)
								self._faces.insert(0, outward_new_face)

								# TODO: add the crossing vertex to an object representation in the map

								#the function calls itself with the new crossing edge as starting point
								#if the next vertex to be identified on the path is not the end vertex,
								#then there should be a face around the crossing vertex to split in the direction of the end vertex
								return self.edge_slice(crossing_vertex, end_vertex, None, epsilon, obstacle, added_vertices)
							else:
								return added_vertices
						else:
							return added_vertices

					else:
						#if the direction of the end vertex is not contained in the cone of this face,
						#the the function is called with the next face in a rotation around the crossing vertex
						return self.edge_slice(start_vertex, end_vertex, crossing_edge.next.opp.get_face(), epsilon, obstacle, added_vertices)

	def add_solid(self, hull_points, epsilon = 0.001):
		#adds the reprensentation of a solid to the mesh
		#it takes a list of points that should define a non-null-area hull with its points given in clockwise order
		#it will then slice the mesh along the segments between each points
		#this will create new edges with the obstacle property on the inner edge of the hull
		#new vertices, faces will be added wherever an edge of the mesh is crossed by those new edges
		#the new vertices are returned to keep track of what represents the solid in the mesh (care for vertex supperposition)

		init_hull_vertices = []
		for hull_point in hull_points:
			hull_vertex = MeshVertex(hull_point[0], hull_point[1])
			init_hull_vertices.append(hull_vertex)
			#for each segment to become an edge, each x,y coordinate must be associated
			# with the face it should lie in in the mesh graph
			if self._grid !=[] and self._grid_len is not None:
				insert_face = self.find_face(hull_point, self._grid[int(hull_point[0]//self._grid_len)][int(hull_point[1]//self._grid_len)])
			else:
				insert_face = self.find_face(hull_point, self._vertices[0])

			#once each face is found from the point cordinate, it can be inserted in the mesh, splitting that face in three
			self.vertex_insertion(hull_vertex, insert_face, epsilon)

		hull_vertices = []
		hull_vertices.extend(init_hull_vertices)

		for hull_index in list(range(len(init_hull_vertices))):
			#for every pair of newly added vertices, a slice of the mesh between those vertices is done
			#it will create all edges and vertices on a straight line between them and give those edges the obstacle property
			next_index = hull_index+1
			if hull_index == len(init_hull_vertices) -1:
				next_index = 0
			hull_edges = self.edge_slice(init_hull_vertices[hull_index], init_hull_vertices[next_index], epsilon=epsilon)

			hull_vertices.extend(hull_edges)

			#TODO: sort the added vertices by x then y pos to keep coherence
		self._solids.append(MeshSolid(hull_vertices))

