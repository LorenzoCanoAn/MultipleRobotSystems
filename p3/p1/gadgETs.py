## Module gadgETs: Collection of convenience low level functions and the like
## Version: 20210403
## Author: Enrique Teruel (ET) eteruel@unizar.es
## License: CC-BY-SA

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.affinity import translate, rotate, scale
from point2d import Point2D 

def pipi(angle):
    """ Put angle (in rad) in [-pi:pi] """
    while angle>np.pi:
        angle=angle-2*np.pi
    while angle<-np.pi:
        angle=angle+2*np.pi
    return angle

def ellipse(n,c0,A,th0=0,ar=1):
    """ Return a 4n-vertices polygon inscribed in ellipse centered at Point c0, oriented towards th0, with area A and aspect ratio ar """
    vertices=[Point2D(r=np.sqrt(1/np.pi),a=x*np.pi/2/n) for x in range(4*n)]
    e=Polygon([(vertices[i].x,vertices[i].y) for i in range(len(vertices))])
    a=np.sqrt(A/np.pi/ar)
    e=scale(e,a,ar*a)
    e=rotate(e,th0,use_radians=True)
    e=translate(e,c0.x,c0.y)
    return e

def voronoi(me,neigh,r):
    """ Return a Polygon with the Voronoi cell <r with the neigh around me, neigh is a list of (x,y) """
    W=Point(me).buffer(r)
    pm=Point2D(me)
    for n in neigh:
        pn=Point2D(n)
        v=pn-pm
        pc=pm+0.5*v
        v.r=r
        q0=pm+2*v
        v.a += np.pi/2
        q1=pc+2*v
        q2=pc-2*v
        W -= Polygon([(q0.x,q0.y),(q1.x,q1.y),(q2.x,q2.y)])
    return W

def get_all_cc(graph):
    already_seen = set()
    result = []
    for node in graph:
        if node not in already_seen:
            cc, already_seen = get_cc(graph, node, already_seen)
            result.append(cc)
    return result

def get_cc(graph, node, already_seen):
        result = set([node])
        nodes2see = set([node])
        while nodes2see:
            node = nodes2see.pop()
            already_seen.add(node)
            nodes2see = nodes2see | graph[node] - already_seen
            result = result | graph[node]
        return result, already_seen

""" test get_all_cc
graph = { # a directed graph; when undirected it is necessary to have (i,j) and (j,i)
     0: {0, 1, 2, 3},
     1: set(),
     2: {1, 2},
     3: {3, 4, 5},
     4: {3, 4, 5},
     5: {3, 4, 5, 7}, 
     6: {6, 8},
     7: set(),
     8: {8, 9},
     9: set()}
components = get_all_cc(graph)
print(components)
max([len(cc) for cc in components])
"""

## Pattern code for debugging with temporal drawings:
# p=patches.Arrow(corridoRT.coords[0][0],corridoRT.coords[0][1],corridoRT.coords[1][0]-corridoRT.coords[0][0],corridoRT.coords[1][1]-corridoRT.coords[0][1],0.1,fc=b.fc)
# self.ax.add_patch(p)
# plt.draw()
# plt.pause(0.5)   
# p.remove() 