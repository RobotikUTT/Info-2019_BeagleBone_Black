from typing import List

from . import merge_meshes

from .. import Vertex, Face, MeshVertex
from ..create import create_isolated_edge, create_isolated_face
from ..collision import clockwise_check, colinear_check

from ..mesh_map import MeshMap

Point = List[float]

def divide_and_conquer(ordered_points: List[Point]) -> (List[Vertex], List[Face]):
	#dichotomic function that takes any number of points ordered by x then y,
	#and applies itself to half those points until there are only less than three points
	#in that case, it creates a single face (3 points) or edge (2 points) with their ghost faces
	#add finlly merges the result of this dichotomic process

	#Case of isolated segment
	if len(ordered_points) == 2:
		vertices = []
		faces = []

		#Create 2 mesh vertices from points coordinates
		a = MeshVertex(ordered_points[0][0], ordered_points[0][1])
		b = MeshVertex(ordered_points[1][0], ordered_points[1][1])

		ghosts = create_isolated_edge(a, b)

		#fill up the resulting lists
		vertices.append(a)
		vertices.append(b)
		vertices.append(ghosts['vertices'][0])
		vertices.append(ghosts['vertices'][1])

		faces.append(ghosts['faces'][0])
		faces.append(ghosts['faces'][1])

		return (vertices, faces)

	#case of triangulation
	elif len(ordered_points) == 3:

		vertices = []
		faces = []

		#Create 3 mesh vertices from points coordinates
		a = MeshVertex(ordered_points[0][0],ordered_points[0][1])
		b = MeshVertex(ordered_points[2][0],ordered_points[2][1])
		c = MeshVertex(ordered_points[1][0],ordered_points[1][1])

		if not clockwise_check(a,b,c):

			#TODO: check coherence of the middle point placment and order of segments

			if colinear_check(a,b,c):

				if not (b.pos[0]<= max(a.pos[0], c.pos[0]) \
								and b.pos[0] >= min(a.pos[0], c.pos[0]) \
								and b.pos[1]<= max(a.pos[1], c.pos[1]) \
								and b.pos[1] >= min(a.pos[1], c.pos[1])):

					b,c = c,b

				first_ghosts = create_isolated_edge(a,b)
				other_ghosts = create_isolated_edge(b,c)


				first_ghosts['faces'][0].edge.next.opp = other_ghosts['faces'][0].edge.next.next
				other_ghosts['faces'][0].edge.next.next.opp = first_ghosts['faces'][0].edge.next

				first_ghosts['faces'][1].edge.next.next.opp = other_ghosts['faces'][1].edge.next
				other_ghosts['faces'][1].edge.next.opp = first_ghosts['faces'][1].edge.next.next

				#fill up the resulting lists
				vertices.append(a)
				vertices.append(b)
				vertices.append(c)


				vertices.append(first_ghosts['vertices'][0])
				vertices.append(first_ghosts['vertices'][1])
				vertices.append(other_ghosts['vertices'][0])
				vertices.append(other_ghosts['vertices'][1])

				faces.append(first_ghosts['faces'][0])
				faces.append(first_ghosts['faces'][1])
				faces.append(other_ghosts['faces'][0])
				faces.append(other_ghosts['faces'][1])

				return (vertices, faces)
			else:
				b,c = c,b

				tri = create_isolated_face(a,b,c)

				vertices.append(a)
				vertices.append(b)
				vertices.append(c)
				vertices.append(tri['vertices'][0])
				vertices.append(tri['vertices'][1])
				vertices.append(tri['vertices'][2])

				faces.append(tri['faces'][0])
				faces.append(tri['faces'][1])
				faces.append(tri['faces'][2])
				faces.append(tri['faces'][3])

				return (vertices, faces)

		else:
			tri = create_isolated_face(a,b,c)

			vertices.append(a)
			vertices.append(b)
			vertices.append(c)
			vertices.append(tri['vertices'][0])
			vertices.append(tri['vertices'][1])
			vertices.append(tri['vertices'][2])

			faces.append(tri['faces'][0])
			faces.append(tri['faces'][1])
			faces.append(tri['faces'][2])
			faces.append(tri['faces'][3])

			return (vertices, faces)

	#case of overpopulation of points; halfen the list then call recursively
	else:
		return merge_meshes(
			divide_and_conquer(ordered_points[:len(ordered_points)//2]),
			divide_and_conquer(ordered_points[len(ordered_points)//2:])
		)