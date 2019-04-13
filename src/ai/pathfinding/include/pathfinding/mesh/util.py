#!/usr/bin/python3

from . import MeshFace, MeshVertex, GhostVertex, GhostFace, MeshEdge, Edge, Vertex, Face
from .create import add_ghost_vertex, create_isolated_edge
from .collision import half_plane_check, circumcircle_check, clockwise_check, colinear_check, intersect_check

from typing import List

Point = List[float]

def dist_point_segment(a_pos: Point, b_pos: Point, x_pos: Point) -> float:
	v = [b_pos[0]-a_pos[0], b_pos[1]-a_pos[1]]
	w = [x_pos[0]-a_pos[0], x_pos[1]-a_pos[1]]

	c1 = v[0]*w[0] + v[1]*w[1]
	c2 = v[0]*v[0] + v[1]*v[1]
	if c1 <= 0:
		return pow(x_pos[0]-a_pos[0],2) + pow(x_pos[1]-a_pos[1],2)


	elif c2 <= c1:
		return pow(x_pos[0]-b_pos[0],2) + pow(x_pos[1]-b_pos[1],2)
	else:
		b = c1/c2
		np = [a_pos[0]+b*v[0], a_pos[1]+b*v[1]]
		return pow(x_pos[0]-np[0],2) + pow(x_pos[1]-np[1],2)

def edge_flip(edge: MeshEdge):
	#Flips an edge by modifying the graph locally, in the edge's face and it's opposing face,
	#by assigning that edge and opposite to point toward the prior external summit vertices
	#This results in the edge 'rotating' clockwise, reordering the edges sequences that define both faces

	#This must be used in the context of an edge between two mesh faces, to change the area defined by both faces
	#often used after a negative circumcircle check of the three vertices of one face against the summit of the opposite one

	if type(edge.face) is MeshFace and type(edge.opp.face) is MeshFace:

		#set the faces's edges to point toward the two external edges that will remain the same
		edge.next.next.face.edge = edge.next.next
		edge.opp.next.next.face.edge = edge.opp.next.next

		#slip the faces pointers of the other two external edges,
		edge.next.face = edge.opp.next.next.face
		edge.opp.next.face = edge.next.next.face

		#set the destination of the vertices pointed by the central edge and opposite to exterior edges
		#in case they were pointing to any central edge that will flip
		if edge.opp.dest.edge == edge:
			edge.opp.dest.edge = edge.opp.next
		if edge.dest.edge == edge.opp:
			edge.dest.edge = edge.next

		#open one side of both triangles by making the last outter edge clockwise
		# point as next to the first outter edge clockwise of the opposite triangle
		# this is because the vertices around the centrad edge will be the new exterior summmit of the flipped faces
		edge.next.next.next = edge.opp.next
		edge.opp.next.next.next = edge.next

		#first flip the opposite center edge by reassigning destination ertex and next edge
		#it can be used as an entry point in the graph to one of the flipped faces
		edge.opp.dest = edge.next.next.next.dest
		edge.opp.next = edge.next.next.next.next

		#then close the other new face by setting it's second edge next edge to be the central edge
		#at this point all edges are accessible by starting at the opposite central edge, then circulation around
		#all four external edges, and finally the central edge.
		edge.next.next.next.next = edge

		#close the first flipped face completely by  flipping the central edge destination and next edge
		edge.dest = edge.opp.next.next.dest
		edge.next = edge.opp.next.next.next

		#close the second flipped face by setting its second ecternal edge's next edge to the fliped oppposite edge
		edge.opp.next.next.next = edge.opp




def merge_edges(start_edge: Edge, faces: List[Face], vertices: Vertex):
	#######
	#One step of the recursive process that generates the mesh
	#Finds and evaluates suitting (clockwise order) candidates amongst the starting edge neighbours
	#elects the vertex that will maximise the minimal angle and creates a new face with it
	#merges the ghost edges of the starting ege and the elected edge where the face is created
	# and associates it to the newly created outter edge
	#checks for collision/crossing with pre-existing edges and proceeds to remove them

	#Identify vertices around the starting edge
	startL = start_edge.dest
	startR = start_edge.opp.dest

	#initiate the duo of vertices candidates, starting with the most exterior edge (minimal angle)
	#can be found from the ghost faces of the starting edge
	edgeL = start_edge.next.opp.next.opp

	if edgeL.opp.dest == startR:
		#Left hand side one point case; startL is the only left point, so it is also the left candidate
		#then startL == Lcandidate can identify this case for handling
		Lcandidate = startL
		Lfound = True
	else:
		#otherwise, Lcandidate is set to the outter neighbouring vertex
		# and the next edge is set by rotation around the startL vertex
		Lcandidate = edgeL.opp.dest
		next_edgeL = edgeL.next.opp
		Lfound = False

	#a counter that holds the number of crossed neighbouring edges that will need to be removed
	Ledges_crossed = 0


	while clockwise_check(startR, startL, Lcandidate) and Lfound==False:
		#the search for a better candidate should stop after the pi angle threshold is crossed
		# or a ghost face signifies the border of the mesh/hull
		if type(next_edgeL) is MeshEdge:
			next_Lcandidate = next_edgeL.opp.dest
			#check if the candidate vertex complies to the delaunay criterion
			if not circumcircle_check(startR, startL, Lcandidate, next_Lcandidate):
				#if not, change the candidate couple with the next vertex connected to startL by rotation
				edgeL = next_edgeL
				Lcandidate = next_Lcandidate
				next_edgeL = edgeL.next.opp
				Ledges_crossed += 1
			else:
				#if the delaunay criterion is valid, register a left candidate
				Lfound = True
		else:
			Lfound = True

	#Same sogic applies to the Right hand side
	#only differences comes from the clockwise roation of edges around a mesh/hull and inside faces themselves
	edgeR = start_edge.next.next.opp.next.next.opp

	Rfound = False

	if edgeR.dest == startL:
		Rcandidate = startR
		Rfound = True
	else:
		Rcandidate = edgeR.dest
		next_edgeR = edgeR.next.next.opp
		Rfound = False

	Redges_crossed = 0

	while clockwise_check(startR, startL, Rcandidate) and Rfound==False:
		if type(next_edgeR) is MeshEdge:
			next_Rcandidate = next_edgeR.dest
			if not circumcircle_check(startR, startL, Rcandidate, next_Rcandidate):
				edgeR = next_edgeR
				Rcandidate = next_Rcandidate
				next_edgeR = edgeR.next.next.opp
				Redges_crossed += 1
			else:
				Rfound = True
		else:
			Rfound = True

	#at the point there should be at least one candidate found either on the left or right hand side
	#if both sides have candidates, the better suitted one should be elected
	elected_edge = None
	elected_vertex = None
	if Rfound or Lfound:
		if Rfound and Lfound:
			#the choice of candidate vertex is done with the delaunay criterion using a circumcircle check
			if circumcircle_check(startR, startL, Lcandidate, Rcandidate):
				elected_vertex = Lcandidate
				elected_edge = edgeL
				Rfound = False
			else:
				elected_vertex = Rcandidate
				elected_edge = edgeR
				Lfound = False
		else:
			if Lfound:
				elected_vertex = Lcandidate
				elected_edge = edgeL
			if Rfound:
				elected_vertex = Rcandidate
				elected_edge = edgeR

	if elected_vertex is None:
		#pyplot.show()
		print('aie')

	if elected_vertex is not None and elected_edge is not None:


		#finally, connect the elected vertex with edges to form the new face of the mesh,
		# delete (unreference) the edges that were crossed by the new edge,
		#and the new edge's opposite edge should belong to an external ghost edge
		# that is a merge of pre-existing external edges of the start and elected edges because
		#they should be already properly linked with the other external ghost faces around the mesh/hull
		if Lfound:
			if Ledges_crossed > 0:
				#loop that will erase/unreference edges that will be crossed by the new edge
				current_edge = start_edge.next.opp.next.opp
				a,b = elected_vertex, startR
				for edge_num in list(range(Ledges_crossed)):
					k,l = current_edge.dest, current_edge.opp.dest
					#the next edge is defined by turning around the assiociated start vertex to find neighbouring vertices
					next_edge = current_edge.next.opp
					#for every face not elected as candidate, the non elected edge should be shecked
					#for collision with the new edge and if there is, removed
					if intersect_check(a.pos, b.pos, k.pos, l.pos):

						#the ghost face of the edge and opposite about to be erased is assigned
						#to the next neighbouring edge to be evaluated

						#vertices or faces that could point to the edge to be destroyed are reassigned to point to external edges

						current_edge.opp.next.next.next = current_edge.next.next
						current_edge.opp.dest.edge = current_edge.next.next.opp

						if current_edge.dest.edge == current_edge.opp:
							current_edge.dest.edge = current_edge.next

						if type(current_edge.next.next.opp.face) is MeshFace or current_edge.opp.dest.edge ==  current_edge:
							current_edge.opp.dest.edge = current_edge.next.next.opp

						if type(current_edge.next.opp) is MeshFace():
							current_edge.next.dest.edge = current_edge.next.opp
						else:
							current_edge.next.dest.edge = current_edge.next.next
						current_edge.next.next.next = current_edge.opp.next
						# current_edge.next.next.dest.edge = current_edge.next.next.opp




						#the mesh face is unreferenced/removed from any list composing the graph
						current_edge.face.edge = None
						current_edge.next.next.face = current_edge.opp.face
						current_edge.next.next = None
						current_edge.next.face = None
						del faces[faces.index(current_edge.face)]
						current_edge.face = None

						#add ghost vertex and face to the second resulting external edge efter the removal
						#each new external edge inherists half the ghost edges of the ghost face
						# that was on the opposite side of the edge to be removed because they hold links
						#to the rest of the external mesh/hull
						# both ghost faces will then be completed with opposing sides
						new_ghost = add_ghost_vertex(current_edge.next)
						faces.append(new_ghost['face'])
						vertices.append(new_ghost['vertex'])
						current_edge.next.next.opp = current_edge.opp.next.next
						current_edge.next.next.next.opp = start_edge.next

						#finally, the edge and it's opposite can be removed frm any list in the mesh, removing any reference
						current_edge.opp.next = None
						current_edge.opp.face = None
						current_edge.opp.opp = None
						current_edge.opp.set_dest(None)


						current_edge.opp = None
						current_edge.next = None
						current_edge.set_dest(None)
						current_edge.face = None
						del current_edge


					current_edge = next_edge

			#create the new edge and face
			new_edge = MeshEdge(startR, next_edge=start_edge)
			new_face = MeshFace(start_edge)

			#add an opposite edge
			new_opp_edge = MeshEdge(elected_vertex, opp_edge=new_edge)
			new_edge.opp = new_opp_edge


			#unset ghost face reference to prepare the merging of the ghost faces
			elected_edge.opp.face.edge = None
			del faces[faces.index(elected_edge.opp.face)]
			#set the face pointer of the previously exterior elected edge to the new face
			elected_edge.opp.face = new_face
			#associates the external ghost edge to the ghost face of the starting edge, to merge both ghost faces
			elected_edge.opp.next.face = start_edge.face
			#delete refernces of the second ghost edge
			elected_edge.opp.next.next.next = None
			elected_edge.opp.next.next.opp = None
			elected_edge.opp.next.next.face = None
			elected_edge.opp.next.next.set_vertex(None)
			#set the next attribute to link both gohst triangles
			elected_edge.opp.next.next = start_edge.next.next

			# if start_edge.next.vertex() in vertices:
			#delete a ghost infinity vertex
			del vertices[vertices.index(elected_edge.opp.next.vertex())]
			elected_edge.opp.next.set_vertex(start_edge.next.vertex())

			#close the new merged ghost triangle
			elected_edge.opp.next.next.next = new_opp_edge
			new_opp_edge.next = elected_edge.opp.next
			new_opp_edge.face = start_edge.face
			# new_opp_edge.next.set_vertex(start_edge.next.vertex())
			new_opp_edge.face.edge = new_opp_edge

			#delete all references to the first ghost edge of the starting face to complete the merge
			start_edge.next.face = None
			start_edge.next.next = None
			start_edge.next.opp = None
			start_edge.next.set_vertex(None)

			#finally, close the the new face by linking edges clockwise
			start_edge.next = elected_edge.opp
			elected_edge.opp.next = new_edge

			#associate the new face
			start_edge.face = new_face
			new_edge.face = new_face

			#set the vertices so that they point along the edges of the created hull (clockwise)
			if type(elected_edge.face) is MeshFace:
				elected_vertex.edge = new_edge
			else:
				startL.edge = elected_edge.opp

			#add the new face to the mesh
			faces.insert(0,new_face)

			merged = {'faces': faces, 'edge': new_edge, 'vertices':vertices}
			return merged

		if Rfound:

			if Redges_crossed > 0:
				current_edge = start_edge.next.next.opp.next.next.opp
				a,b = elected_vertex, startL
				for edge_num in list(range(Redges_crossed)):
					k,l = current_edge.dest, current_edge.opp.dest
					next_edge = current_edge.next.next.opp
					if intersect_check(a.pos, b.pos, k.pos, l.pos):

						current_edge.next.next.next = None
						current_edge.next.next.face = None

						if current_edge.dest.edge == current_edge.opp:
							current_edge.dest.edge = current_edge.next

						if type(current_edge.next.opp.face) is MeshFace:
							current_edge.next.dest.edge = current_edge.next.opp
						else:
							current_edge.next.dest.edge = current_edge.next.next

						if type(current_edge.next.next.opp.face) is MeshFace or current_edge.next.next.dest.edge == current_edge:

							# startR.edge = current_edge.next.next.opp
							current_edge.next.next.dest.edge = current_edge.next.next.opp
						else:
							current_edge.next.next.dest.edge = current_edge.next.next.opp.next.next.opp.next.next.opp

						new_ghost = add_ghost_vertex(current_edge.next.next)
						faces.append(new_ghost['face'])
						vertices.append(new_ghost['vertex'])
						current_edge.next.next.next.next.opp = current_edge.opp.next
						current_edge.next.next.next.opp = start_edge.next.next
						current_edge.next.next = current_edge.opp.next
						current_edge.next.face = current_edge.opp.face
						current_edge.opp.next.next.next = current_edge.next

						current_edge.face.edge = None
						del faces[faces.index(current_edge.face)]

						current_edge.opp.next = None
						current_edge.opp.opp = None
						current_edge.opp.face = None
						current_edge.opp.set_dest(None)

						current_edge.face = None
						current_edge.opp = None
						current_edge.next = None
						current_edge.set_dest(None)
						del current_edge

					current_edge = next_edge

			#create new edge
			new_edge = MeshEdge(elected_vertex, next_edge=elected_edge.opp)
			new_face = MeshFace(start_edge)

			new_opp_edge = MeshEdge(startL, opp_edge=new_edge)
			new_edge.opp = new_opp_edge

			#delete references to left ghost face
			start_edge.face.edge = None
			del faces[faces.index(start_edge.face)]

			start_edge.next.face = elected_edge.opp.face

			#delete references to left side of linking ghost face

			start_edge.next.next.face = None
			start_edge.next.next.opp = None
			start_edge.next.next.next = None
			start_edge.next.next.set_vertex(None)

			#link both ghost by a next pointer
			start_edge.next.next = elected_edge.opp.next.next
			del vertices[vertices.index(elected_edge.opp.next.vertex())]

			elected_edge.opp.next.set_vertex(None)

			#then delete right hand side of the ghost edge to remove
			elected_edge.opp.next.face = None
			elected_edge.opp.next.opp = None
			elected_edge.opp.next.next = None
			elected_edge.opp.next.set_vertex(None)

			#redefine the last edge of the ghost triangle
			start_edge.next.next.next = new_opp_edge
			new_opp_edge.next = start_edge.next

			new_opp_edge.face = elected_edge.opp.face
			new_opp_edge.face.edge = new_opp_edge

			start_edge.face = new_face
			start_edge.next = new_edge
			elected_edge.opp.next = start_edge
			elected_edge.opp.face = new_face
			new_edge.face = new_face

			if type(elected_edge.face) is not MeshFace:
				elected_vertex.edge = elected_edge.opp
			faces.insert(0,new_face)

			merged = {'faces': faces, 'edge': new_edge, 'vertices':vertices}
			return merged
