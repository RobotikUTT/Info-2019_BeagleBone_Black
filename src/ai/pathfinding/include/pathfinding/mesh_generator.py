from matplotlib import pyplot

from numpy import array, concatenate, append
from math import pow
from datetime import datetime
import json

from .mesh import Face, Vertex, Edge, \
	MeshVertex, MeshFace, MeshEdge, \
	GhostFace, GhostVertex, GhostEdge

from .mesh.mesh_map import MeshMap

from typing import List

def json_load(filename, grid_len=None):
	points_list =  json.loads(open(filename))
	return MeshMap(points_list, grid_len)


if __name__ == '__main__':

	#an utility display function
	def display_faces(faces, color=None):
		fig = pyplot.figure()
		ar = fig.add_subplot(1,1,1)
		for face in faces:
			if type(face) is MeshFace:
				edge = face.edge
				if type(edge) is MeshFace:
					print('nimp!')
					print(edge.next.dest.pos,edge.next.opp.dest.pos)
				else:
					tx = []
					ty = []
					for iter in [0,1,2,3]:
						tx.append(edge.dest.pos[0])
						ty.append(edge.dest.pos[1])

						edge = edge.next
			if color is not None:
				ar.plot(tx,ty,color)
			else:
				ar.plot(tx,ty)
		return ar

	def display_map(meshmap: MeshMap):
		segz = []
		wegz = []
		for f in meshmap.get_faces():
			if type(f) is MeshFace:
				e = f.edge
				for i in [0,1,2,3]:
					if e.is_obstacle():
						wegz.append([e.opp.dest.pos, e.dest.pos])
					else:
						segz.append([e.opp.dest.pos, e.dest.pos])
					e = e.next
		fig = pyplot.figure()
		ar = fig.add_subplot(1,1,1)
		sar = array(segz)
		for h in sar:
			tx = h[:,0]
			ty = h[:,1]
			ar.plot(tx,ty, color='r')
		war = array(wegz)
		for w in war:
			wx = w[:,0]
			wy = w[:,1]
			ar.plot(wx,wy, color='k')
		return ar

	#List of points that will be the base vertices of the triangle mesh
	points_list: List[float]=[[1.,2.],[1.6,2.3],[2.,1.],[2.2,8.],[2.7,1.8],[3.5,5.],[4.2,8.],[4.6,3.6],[5.,3.],[5.4,2.],[5.6,5.],[6.,4.2], [6.2, 5.8]]
	map_patch = []
	for i in list(range(3)):
		for j in list(range(3)):
			for pnum in list(range(len(points_list))):
				map_patch.append([points_list[pnum][0]+i*8,points_list[pnum][1]+j*8])
	map_points = []
	for i in list(range(3)):
		for j in list(range(3)):
			for pnum in list(range(len(map_patch))):
				map_points.append([map_patch[pnum][0]+(i)*20,map_patch[pnum][1]+(j)*20])

	print(len(map_points))

	print(datetime.now())
	grid_len = 10

	#To generate the Mesh, the points need to be sorted by x then y
	sorted_map = sorted(map_points, key=lambda k: [k[0], k[1]])

	map=MeshMap(sorted_map,grid_len)

	# display_faces(map.get_faces())
	# pyplot.show()

	#Solids are defined by a list of points given in CLOCKORK ORDER
	hull_points = [[7.2,5.4],[7.2,18.4],[9.2,18.4],[9.2,5.4]]
	map.add_solid(hull_points)

	# display_map(map)
	# pyplot.show()

	print(datetime.now())
