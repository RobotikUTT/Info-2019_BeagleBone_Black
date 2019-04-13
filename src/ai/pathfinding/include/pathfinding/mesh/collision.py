from numpy.linalg import det

from . import Vertex, MeshVertex

from typing import List, Union

Point = List[float]

def intersect_check(a: Point, b: Point, k: Point, l: Point) -> Union[Point, bool]:
	"""
		This function checks for an intersection point beteen two segments
		It takes the coordinates of the (a,b) points bounding the first segments,
		then the coordinates of (k,l) of the second one and returns either the x,y coordinates of the intersection point
		or False if no such point was found
		It does not handle parallelism and will return False because in that case
		there would be either no intersection point or an infinity of them
	"""

	xa, ya = a
	xb, yb = b
	xk, yk = k
	xl, yl = l

	#parallel vertical segments
	if xa == xb and xk == xl:
		return False

	#intersection of a vertical segments with a non_vertical segment
	#an vertical segment is a special case because the linear factor a of ax+b is then infinite
	#in that case y values should be checked at the vertical x position
	elif xa != xb and xk == xl:
		if min(xa,xb) > xk or max(xa,xb) < xk:
			return False
		else:
			Aa = (ya-yb)/(xa-xb)
			ba = ya-Aa*xa
			Yx = Aa*xk + ba
			if Yx < max(yk,yl) and Yx > min(yk,yl):
				return [xk, Yx]
			else:
				return False
	elif xk != xl and xa == xb:
		if min(xk,xl) > xa or max(xk,xl) < xa:
			return False
		else:
			Ak = (yk-yl)/(xk-xl)
			bk = yk-Ak*xk
			Yx = Ak*xa + bk
			if Yx < max(ya,yb) and Yx > min(ya,yb):
				return [xa, Yx]
			else:
				return False

	#otherwise, both linear parameters are calculated, then an intersection point is found
	#then returned only if it's defined on both segments

	if max(xa,xb) < min(xk,xl) or max(xk, xl) < min(xa,xb):
		return False
	I1 = [min(xa,xb), max(xa,xb)]
	I2 = [min(xk,xl), max(xk,xl)]

	Aa = (ya-yb)/(xa-xb)
	Ak = (yk-yl)/(xk-xl)
	ba = ya-Aa*xa
	bk = yk-Ak*xk

	if Aa == Ak:
		return False

	Xx = (bk-ba) / (Aa-Ak)
	Yx = Aa*Xx + ba

	if (Xx < max(I1[0],I2[0])) or (Xx > min(I1[1], I2[1])):
		return False
	else:
		return[Xx, Yx]

def clockwise_check(a: MeshVertex, b: MeshVertex, c: MeshVertex) -> bool:
	"""This is a geometric test that checks if the angle between vertices a,b and c in that order is defined clockwise
	It is used to ensure the coherence of the mesh because any triangle in it must be defined clockwise
	It calculated the area inside the triangle and returns True if positive (points listed clockwise) and False otherwise"""
	surface_sum = sum([(b.pos[0]-a.pos[0])*(b.pos[1]+a.pos[1]),
					   (c.pos[0]-b.pos[0])*(c.pos[1]+b.pos[1]),
					   (a.pos[0]-c.pos[0])*(a.pos[1]+c.pos[1])])
	return surface_sum > 0

def colinear_check(a: MeshVertex, b: MeshVertex, c: MeshVertex):
	"""This is a geometric test that checks weather the surface area defined by three vertices a,b and c is null
	If so then the three points are colinear and it is impossible to define anthing else than thwo segments"""
	surface_sum = sum([(b.pos[0]-a.pos[0])*(b.pos[1]+a.pos[1]),
					   (c.pos[0]-b.pos[0])*(c.pos[1]+b.pos[1]),
					   (a.pos[0]-c.pos[0])*(a.pos[1]+c.pos[1])])
	return surface_sum == 0

def circumcircle_check(v1: MeshVertex, v2: MeshVertex, v3: MeshVertex, x: MeshVertex):
	"""This function checks if a the evaluated vertex x lies inside the circumcircle defined by the vertices a,b,c
	This problem is solvable by resolving 4/4 matrix determinant"""
	M = []
	for v in [v1,v2,v3,x]:
		M.append([v.pos[0],v.pos[1],pow(v.pos[0],2)+pow(v.pos[1],2),1])

	return det(M) > 0

def cone_check(center_pos: Point, b1_pos: Point, b2_pos: Point, x_pos: Point):
	"""This is a geometric test that checks weather or not the evaluated point at given x coordinates fits inside
	a planar cone defined by its center position and two positions of any point on each outline of the cone
	this function is often used as a directionnality check with"""

	if (b1_pos[0]-center_pos[0])*(x_pos[1]-center_pos[1]) - (b1_pos[1]-center_pos[1])*(x_pos[0]-center_pos[0]) >= 0:
		return False
	if (b2_pos[0]-center_pos[0])*(x_pos[1]-center_pos[1]) - (b2_pos[1]-center_pos[1])*(x_pos[0]-center_pos[0])  <= 0:
		return False
	else:
		return True

def half_plane_check(a_pos: Point, b_pos: Point, x_pos: Point):
	#This is a geometric test that check if a point at the x coordinated lies on the upper half plane
	#defined by the directionnal line a,b
	if (b_pos[0]-a_pos[0])*(x_pos[1]-a_pos[1]) - (b_pos[1]-a_pos[1])*(x_pos[0]-a_pos[0]) >= 0:
		return True
	else:
		return False

def inside_triangle_check(a_pos: Point, b_pos: Point, c_pos: Point, x_pos: Point) -> bool:
	"""This is a geometric thest that checks if a point at the x coordinates lies inside a triangle
	defined by three points a,b,c given in a clockwise order.
	It is essensially three consecutive half plane checks"""
	if (b_pos[0]-a_pos[0])*(x_pos[1]-a_pos[1]) - (b_pos[1]-a_pos[1])*(x_pos[0]-a_pos[0]) > 0:
		return False
	elif (c_pos[0]-a_pos[0])*(x_pos[1]-a_pos[1]) - (c_pos[1]-a_pos[1])*(x_pos[0]-a_pos[0]) < 0:
		return  False
	elif (c_pos[0]-b_pos[0])*(x_pos[1]-b_pos[1]) - (c_pos[1]-b_pos[1])*(x_pos[0]-b_pos[0]) > 0:
		return  False
	else:
		return  True

def on_segment_check(a_pos: Point, b_pos: Point, x_pos: Point) -> bool:
	"""This is a geometric test that checks if a point at the x_pos coordinates lies on the segment defined by a and b"""
	
	if a_pos[0] == b_pos[0]:
		if x_pos[0] == a_pos[0]:
			if x_pos[1] < max(a_pos[1], b_pos[1]) and x_pos[1] < min(a_pos[1], b_pos[1]):
				return True
			else:
				return False
		else:
			return False
	else:
		if x_pos[0] < max(a_pos[0], b_pos[0]) and x_pos[0] < min(a_pos[0], b_pos[0]):
			Aa = (a_pos[1]-b_pos[1])/(a_pos[0]-b_pos[0])
			ba = a_pos[1]-Aa*a_pos[0]
			Yx = Aa*x_pos[0] + ba
			if Yx == x_pos[1]:
				return True
			else:
				return False
		else:
			return False
