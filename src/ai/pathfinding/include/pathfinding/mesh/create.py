from . import Edge, GhostEdge, GhostFace, GhostVertex, MeshEdge, MeshFace


def add_ghost_vertex(edge: Edge):
	#Completes an existing mesh edge with a ghost vertex, creating 2 ghost edges defined clockwise and a ghost face
	#should be used with an external edge of the mesh/hull and then the opposites edges of the ghost edges should be
	#linked to other outter ghost edges in a ay that allows to turn anti-clockwise around the mesh/hull

	facex = GhostFace(edge)
	edge.face = facex

	a = edge.opp.dest
	x = GhostVertex()

	edge_bx = GhostEdge(x, face=facex)
	edge.next = edge_bx

	edge_xa = GhostEdge(a, face=facex, next_edge=edge)
	edge_bx.next = edge_xa

	return {'vertex': x, 'face':facex}

def create_isolated_edge(a,b):
	#Creates the opposite edges connecting two vertices a and b,
	#then adds gosh vertices on both sides and links their opposite sides
	edge_ab = MeshEdge(b)
	edge_ba = MeshEdge(a, opp_edge=edge_ab)
	edge_ab.opp = edge_ba
	a.edge = edge_ab
	b.edge = edge_ba

	# add ghost vertices, edges, and faces to obtain a quad-faced polygon with two ooposite ghost vertices at infinity
	up_ghost = add_ghost_vertex(edge_ab)

	down_ghost = add_ghost_vertex(edge_ba)

	#link the lateral edges of the ghost faces as opposite edges
	edge_ab.next.opp = edge_ba.next.next
	edge_ba.next.next.opp = edge_ab.next
	edge_ab.next.next.opp = edge_ba.next
	edge_ba.next.opp = edge_ab.next.next

	return {'vertices':[up_ghost['vertex'],down_ghost['vertex']],
			'faces':[up_ghost['face'],down_ghost['face']]}

def create_isolated_face(a,b,c):
	#Creates a triangle graph from three vertices a, b and c by adding edges inbetween in a clockwise manner,
	#then creating opposing edges with ghost faces at infinity that allows to turn around the triangle anti-clockwise

	#add a first edge and its associated opposite edge
	edge_ab = MeshEdge(b)
	edge_ba = MeshEdge(a, opp_edge=edge_ab)
	edge_ab.opp = edge_ba

	#define the current face
	face = MeshFace(edge_ab)
	edge_ab.face = face

	#add second and third edges similarely, linking them by their next and opposite attributes
	edge_bc = MeshEdge(c, face=face)
	edge_cb = MeshEdge(b, opp_edge=edge_bc)
	edge_bc.opp = edge_cb
	edge_ab.next = edge_bc

	edge_ca = MeshEdge(a, face=face)
	edge_ac = MeshEdge(c, opp_edge=edge_ca)
	edge_ca.opp = edge_ac
	edge_bc.next = edge_ca
	edge_ca.next = edge_ab

	a.edge = edge_ab
	b.edge = edge_bc
	c.edge = edge_ca

	#add  3 ghost faces all around
	first_ghost = add_ghost_vertex(edge_ba)
	second_ghost = add_ghost_vertex(edge_cb)
	third_ghost = add_ghost_vertex(edge_ac)

	#then associate the ghosts faces lateral edges by their opposite attribute

	edge_ba.next.next.opp = edge_cb.next
	edge_cb.next.opp = edge_ba.next.next
	edge_cb.next.next.opp = edge_ac.next
	edge_ac.next.opp = edge_cb.next.next
	edge_ac.next.next.opp = edge_ba.next
	edge_ba.next.opp = edge_ac.next.next

	return {'vertices':[first_ghost['vertex'],second_ghost['vertex'],third_ghost['vertex']],
			'faces':[face,first_ghost['face'],second_ghost['face'],third_ghost['face']]}
