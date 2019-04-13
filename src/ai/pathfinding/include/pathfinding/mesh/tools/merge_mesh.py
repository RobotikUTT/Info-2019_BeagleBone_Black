from typing import List

from .. import Vertex, MeshVertex, MeshFace, GhostFace, GhostVertex, MeshEdge, Face
from ..util import edge_flip, merge_edges
from ..create import add_ghost_vertex, create_isolated_edge
from ..collision import half_plane_check, circumcircle_check, colinear_check

def merge_meshes(leftMesh: (List[Vertex], List[Face]), rightMesh: (List[Vertex], List[Face])) -> (List[Vertex], List[Face]):
	#Main function of the mesh generation that takes two valid delaunay meshes/convex hulls
	#then adds triangles untill both are merged in a single valid delauna mesh/hull

	leftVertices, leftFaces = leftMesh
	rightVertices, rightFaces = rightMesh

	vertices: List[Vertex] = []
	for v in leftVertices:
		if type(v) is MeshVertex:
			vertices.append(v)

	for v in rightVertices:
		if type(v) is MeshVertex:
			vertices.append(v)
	for v in leftVertices:
		if type(v) is GhostVertex:
			vertices.append(v)
	for v in rightVertices:
		if type(v) is GhostVertex:
			vertices.append(v)

	faces = []
	for f in leftFaces:
		if type(f) is MeshFace:
			faces.append(f)
	for f in rightFaces:
		if type(f) is MeshFace:
			faces.append(f)
	for f in leftFaces:
		if type(f) is GhostFace:
			faces.append(f)
	for f in rightFaces:
		if type(f) is GhostFace:
			faces.append(f)

	L_flat = len([f for f in leftFaces if type(f) is MeshFace]) == 0
	R_flat = len([f for f in rightFaces if type(f) is MeshFace]) == 0

	#Find the upper and lower boundary vertices of each convex hull
	#TODO: check for occlusion between each upper or lower vertices (if there is, select another starting or exit vertex!)
	upper_L_vertex = leftVertices[0]
	upper_L_height = leftVertices[0].pos[1]
	for vertex in leftVertices:
		if type(vertex) is MeshVertex:
			if vertex.pos[1] > upper_L_height:
				upper_L_vertex = vertex
				upper_L_height = vertex.pos[1]
			elif vertex.pos[1] == upper_L_height and vertex.pos[0] < upper_L_vertex.pos[0]:
				upper_L_vertex = vertex

	lower_L_vertex = leftVertices[0]
	lower_L_height = leftVertices[0].pos[1]
	for vertex in leftVertices:
		if type(vertex) is MeshVertex:
			if vertex.pos[1] < lower_L_height:
				lower_L_vertex = vertex
				lower_L_height = vertex.pos[1]
			elif vertex.pos[1] == lower_L_height and vertex.pos[0] < lower_L_vertex.pos[0]:
				lower_L_vertex = vertex

	upper_R_vertex = rightVertices[0]
	upper_R_height = rightVertices[0].pos[1]
	for vertex in rightVertices:
		if type(vertex) is MeshVertex:
			if vertex.pos[1] > upper_R_height:
				upper_R_vertex = vertex
				upper_R_height = vertex.pos[1]
			elif vertex.pos[1] == upper_R_height and vertex.pos[0] > upper_R_vertex.pos[0]:
				upper_R_vertex = vertex

	lower_R_vertex = rightVertices[0]
	lower_R_height = rightVertices[0].pos[1]
	for vertex in rightVertices:
		if type(vertex) is MeshVertex:
			if vertex.pos[1] < lower_R_height:
				lower_R_vertex = vertex
				lower_R_height = vertex.pos[1]
			elif vertex.pos[1] == lower_R_height and vertex.pos[0] > lower_R_vertex.pos[0]:
				lower_R_vertex = vertex

	#case of to flat and vertical edges that are colinear, then no face is added; only a linking edge and its ghost faces
	if L_flat and R_flat and lower_L_vertex.pos[0] == upper_L_vertex.pos[0] \
			and lower_R_vertex.pos[0] == upper_R_vertex.pos[0] \
			and upper_L_vertex.pos[0] == lower_R_vertex.pos[0]:

		upper_L_edge = upper_L_vertex.edge
		lower_R_edge = lower_R_vertex.edge
		ghosts = create_isolated_edge(upper_L_vertex, lower_R_vertex)

		#after the edge and opposite are created, along with thir ghost faces, they are linked to the next ghosts
		#thatgo around the colinear edges

		upper_L_vertex.edge.next.opp = lower_R_edge.next.next
		lower_R_edge.next.next.opp = upper_L_vertex.edge.next
		lower_R_vertex.edge.next.next.opp = lower_R_edge.opp.next
		lower_R_edge.opp.next.opp = lower_R_vertex.edge.next.next

		upper_L_vertex.edge.next.next.opp = upper_L_edge.opp.next
		upper_L_edge.opp.next.opp = upper_L_vertex.edge.next.next
		lower_R_vertex.edge.next.opp = upper_L_edge.next.next
		upper_L_edge.next.next.opp = lower_R_vertex.edge.next

		lower_R_vertex.edge = lower_R_edge

		#fill up the mesh with the new vertices and faces
		vertices.append(ghosts['vertices'][0])
		vertices.append(ghosts['vertices'][1])

		faces.append(ghosts['faces'][0])
		faces.append(ghosts['faces'][1])

		return (vertices, faces)
	else:
		#otherwise, at least one side of the merge has an area, or not vertically colinear,
		#meaning that new triangles/faces will have to be created
		start_R = lower_R_vertex

		start_L = lower_L_vertex
		stop_R = upper_R_vertex
		stop_L = upper_L_vertex
		#start by setting the start and stop vertices to the upper and lower bound vertices of each mesh/hull

		#check the case of a flat mesh that needs to be merged with a non flat mesh.
		# If the starting point on the flat mesh is on the same vertical line as a point on the other mesh,
		#then that oint on the other mesh should be the starting point (or ending point depending if left or right)
		if R_flat and not L_flat:
			aligned_L = None
			for vertex in leftVertices:
				if type(vertex) is MeshVertex:
					if vertex.pos[0] == lower_R_vertex.pos[0]:
						if aligned_L is None:
							aligned_L = vertex
						elif vertex.pos[1] > aligned_L.pos[1]:
							aligned_L = vertex
			if aligned_L is not None:
				start_L = aligned_L

		if L_flat and not R_flat:
			aligned_R = None
			for vertex in rightVertices:
				if type(vertex) is MeshVertex:
					if vertex.pos[0] == upper_L_vertex.pos[0]:
						if aligned_R is None:
							aligned_R = vertex
						elif vertex.pos[1] < aligned_R.pos[1]:
							aligned_R = vertex
			if aligned_R is not None:
				stop_R = aligned_R

		#then check for occlusion or extension of the field of view
		#an occlusion occurs if the segment between the end or start vertices crosses any mesh
		#some half plane checks are performed.
		#One way is the occlusion, then the start or end vertexmust be displaced
		# towards the center to narrow the field of view
		#The other way is to extnd the field of view if there is no occusion
		# while picking the ext vertex as start or end
		no_occlusion = False
		#check and resolution of the occlusion
		while not no_occlusion:

			no_occlusion = True
			if not R_flat:
				if start_L.pos[1] >= start_R.pos[1]:
					if half_plane_check(start_R.edge.dest.pos,start_R.pos, start_L.pos) \
							and half_plane_check(start_R.pos, start_R.edge.opp.next.opp.next.dest.pos, start_L.pos):
						no_occlusion = False
						start_R = start_R.edge.dest

				if stop_L.pos[1] <= stop_R.pos[1]:
					if half_plane_check(stop_R.edge.dest.pos, stop_R.pos,stop_L.pos) \
							and half_plane_check(stop_R.pos, stop_R.edge.opp.next.opp.next.dest.pos,stop_L.pos):
						no_occlusion = False
						stop_R = stop_R.edge.opp.next.opp.next.dest
			if not L_flat:
				if start_L.pos[1] <= start_R.pos[1]:
					if half_plane_check(start_L.edge.dest.pos, start_L.pos, start_R.pos) \
							and half_plane_check(start_L.pos, start_L.edge.opp.next.opp.next.dest.pos,
												 start_R.pos):
						no_occlusion = False
						start_L = start_L.edge.opp.next.opp.next.dest
				if stop_L.pos[1] >= stop_R.pos[1]:
					if half_plane_check(stop_L.pos, stop_L.edge.opp.next.opp.next.dest.pos, stop_R.pos) \
							and half_plane_check(stop_L.edge.dest.pos, stop_L.pos, stop_R.pos):
						no_occlusion = False
						stop_L = stop_L.edge.dest

		while no_occlusion:
			#check and resolution of the extenion of the field of view, just before any occlusion happens

			no_occlusion = False
			if not R_flat:
				if start_L.pos[1] < start_R.pos[1]:
					if not half_plane_check(start_R.pos, start_R.edge.opp.next.opp.next.dest.pos, start_L.pos):
						no_occlusion = True
						start_R = start_R.edge.opp.next.opp.next.dest
				if stop_L.pos[1] > stop_R.pos[1]:
					if not half_plane_check(stop_R.edge.dest.pos, stop_R.pos,stop_L.pos):
						no_occlusion = True
						stop_R = stop_R.edge.dest
			if not L_flat:
				if start_L.pos[1] > start_R.pos[1]:
					if not half_plane_check(start_L.edge.dest.pos, start_L.pos, start_R.pos):
						no_occlusion = True
						start_L = start_L.edge.dest
				if stop_L.pos[1] < stop_R.pos[1]:
					if not half_plane_check(stop_L.pos, stop_L.edge.opp.next.opp.next.dest.pos, stop_R.pos):
						no_occlusion = True
						stop_L = stop_L.edge.opp.next.opp.next.dest

		#Once the start and end vertices are selected, the merge process cn begin
		#create an edge between both lower points and its opposite
		merge_edge_start = MeshEdge(start_L)
		merge_edge_opp = MeshEdge(start_R, opp_edge=merge_edge_start)

		merge_edge_start.opp = merge_edge_opp

		#add the ghost faces under and over them
		over_ghost_face = add_ghost_vertex(merge_edge_start)
		under_ghost_face = add_ghost_vertex(merge_edge_opp)

		faces.append(over_ghost_face['face'])
		faces.append(under_ghost_face['face'])
		vertices.append(over_ghost_face['vertex'])
		vertices.append(under_ghost_face['vertex'])

		#link the opposites edges of the ghost faces bellow and over the starting edge
		if start_L != stop_L:
			merge_edge_start.next.opp = start_L.edge.opp.next.opp
			start_L.edge.opp.next.opp.opp = merge_edge_start.next
			merge_edge_opp.next.next.opp = start_L.edge.opp.next
			start_L.edge.opp.next.opp = merge_edge_opp.next.next
		else:
			merge_edge_start.next.opp = merge_edge_start.opp.next.next
			merge_edge_start.opp.next.next.opp = merge_edge_start.next

		if start_R != stop_R:
			merge_edge_opp.next.opp = start_R.edge.opp.next.opp
			start_R.edge.opp.next.opp.opp = merge_edge_opp.next
			merge_edge_start.next.next.opp = start_R.edge.opp.next
			start_R.edge.opp.next.opp = merge_edge_start.next.next
		else:
			merge_edge_start.next.next.opp = merge_edge_start.opp.next
			merge_edge_start.opp.next.opp = merge_edge_start.next.next

		current_edge = merge_edge_start

		#after the starting edge is initialized, new edges and faces will be created until the end vertices get linked
		while not (current_edge.dest == stop_L and current_edge.opp.dest == stop_R):
			merged = merge_edges(current_edge, faces, vertices)

			faces = merged['faces']
			vertices = merged['vertices']

			#checks the delaunay property and performs edge flips if necessary
			if type(current_edge.opp.face) is MeshFace:
				if not circumcircle_check(current_edge.dest, current_edge.next.dest, current_edge.opp.dest, current_edge.opp.next.dest):
					if not colinear_check(current_edge.opp.next.dest, current_edge.dest, current_edge.next.dest) \
							and not colinear_check(current_edge.opp.next.dest, current_edge.opp.dest, current_edge.next.dest):
						edge_flip(current_edge)

			if type(current_edge.next.oppface) is MeshFace:
				if not circumcircle_check(current_edge.dest, current_edge.next.dest, current_edge.opp.dest, current_edge.next.opp.next.dest):
					if not colinear_check(current_edge.next.opp.next.dest, current_edge.dest, current_edge.opp.dest) \
							and not colinear_check(current_edge.next.opp.next.dest, current_edge.next.dest, current_edge.opp.dest):
						edge_flip(current_edge.next)

			if type(current_edge.next.next.oppface) is MeshFace:
				if not circumcircle_check(current_edge.dest, current_edge.next.dest, current_edge.opp.dest, current_edge.next.next.opp.next.dest):
					if not colinear_check(current_edge.next.next.opp.next.dest, current_edge.dest, current_edge.next.dest) \
							and not colinear_check(current_edge.next.next.opp.next.dest, current_edge.dest, current_edge.opp.dest):
						edge_flip(current_edge.next.next)

			current_edge = merged['edge'].opp

		current_edge.dest.edge = current_edge.opp
		start_L.edge = merge_edge_start.opp.next.next.opp.next.next.opp
		start_R.edge = merge_edge_start

		return (vertices, faces)