## Module small_worl2d
## Version: 1.0.20210517
## Author: Enrique Teruel (ET) eteruel@unizar.es
## License: CC-BY-SA

## A typical small_worl2d is a rectangular Space shown in a matplotlib window, with or without limits
## populated by a number of Body's like Obstacle's, Nest's or Food's,
## MoBody's (a Moving subclass of Body's) like MObstacle's, and
## AniBody's (an Animated subclass of MoBody's), which are the interesting Mobot's, Killer's, Shepherd's, etc
## AniBody's are animated by Soul's (there can be several in one Body), which are the (most) interesting control codes

from time import time, localtime, strftime
import numpy as np
from point2d import Point2D 
from random import uniform, randint, choice
from shapely.geometry import Point, Polygon, LineString, MultiPoint
from shapely.affinity import translate, rotate, scale
from shapely.ops import nearest_points, unary_union
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FFMpegWriter # requires having ffmpeg installed, from https://ffmpeg.org/

from gadgETs import pipi, ellipse, voronoi
from small_worl2d_config import loginfo, logerror, visual, shoul, showconn, SS, W, H, room, TS, RT, vN, wN # configuration parameters in a separate file

class Space: # a rectangle populated by Body's
    
    def __init__(self,name,T=RT,R=1,limits=''):
        self.name=name
        if visual:
            self.fig=plt.figure(figsize=(SS*W/np.sqrt(W**2+H**2), SS*H/np.sqrt(W**2+H**2)),frameon=False) # W:H proportion, SS inches diag (resizeable)
            self.fig.canvas.set_window_title('small_worl2d '+self.name+' ('+str(W)+':'+str(H)+')')
            self.ax=self.fig.add_axes([0,0,1,1]) # full window
            self.ax.set_facecolor('w') # white background
            self.ax.set_xlim(-W, W) # note that x coords, of everything visible in the space, range from -W to W
            self.ax.set_ylim(-H, H) # and y coords from -H to H
            self.movie_writer = FFMpegWriter(fps=10)
            self.movie_writer.setup(self.fig, self.name+'.mp4', self.fig.dpi)
        self.bodies=[] # list ob Body's in this Space
        self.dist=np.zeros((0,0)) # distances between self.bodies centroids in a symmetric np.matrix
        self.R=R # communication radius (by now, communication is omnidirectional and with same R for every type of (ani)Body)
        self.conn={} # a dict where conn[i] is the set of j's such that (i,j) is edge; i, j are AniBody's of the same type in self.bodies
        self.conngraph=[] # a list of edges (i,j) for graphical representation
        self.time=time() # time (s) last updated
        self.t0=self.time
        self.T=T # update cycle time (s)
            # the "right" value depends on the number of moving bodies, the window size and events,
            # and the performance of the Hw, Sw, and code (it is developped for readability first, profiling yet TBD)
        self.updates=-1
        self.avgT=0 # for timing statistics
        self.limits=limits # str possibly containing v or/and h
            # Limits are implemented as Obstacles, with a near-infinity enclosing radio
            # Without limits, moving bodies that cross the top appear at the bottom, etc
            # In such case, avoid placing obstacles, etc near the no-limit
        # Creation and drawing of the borders:
        if 'v' in limits:
            self.bodies.append(Obstacle('top',pos=-1,area=-1,fc='k',vertices=[(-W,H-0.1),(W,H-0.1),(W,1e12),(-W,1e12)]))
            self.bodies.append(Obstacle('bottom',pos=-1,area=-1,fc='k',vertices=[(-W,-H+0.1),(W,-H+0.1),(W,-1e12),(-W,-1e12)]))
        if 'h' in limits:
            self.bodies.append(Obstacle('left',pos=-1,area=-1,fc='k',vertices=[(-W+0.1,-H),(-W+0.1,H),(-1e12,H),(-1e12,-H)]))
            self.bodies.append(Obstacle('right',pos=-1,area=-1,fc='k',vertices=[(W-0.1,-H),(W-0.1,H),(1e12,H),(1e12,-H)]))
        if visual:
            self.redraw() # required to init axes
            plt.pause(0.001)
            if loginfo:
                print('Init '+name+' with mpl version '+mpl.__version__)
                for b in self.bodies:
                    print(repr(b))

    ## Closing functions

    def has_been_closed(self):
        """ Returns True when the figure where self is drawn is not active """
        if visual:
            fig=self.ax.figure.canvas.manager
            active_figs=plt._pylab_helpers.Gcf.figs.values()
            return fig not in active_figs
        else:
            return False

    def close(self):
        print(self.name+' closed')
        if loginfo:
            for b in self.bodies:
                print('Body '+b.name+' updated each avg {0:1.2f} s'.format(b.avgT))
                if isinstance(b,AniBody):
                    for x in b.souls:
                        print('Soul '+str(x)+' updated each avg {0:1.2f} s (T = {1:1.2f} s)'.format(x.avgT,x.T))
            if visual: print('Space '+self.name+' redrawn each avg {0:1.2f} s (T ={1:1.2f} s)'.format(self.avgT,self.T))
        if visual:
            self.movie_writer.finish()
        del self

    ## Drawing functions

    def redraw(self):
        """ Draws all the self.bodies whose time is more recent than self.time

        The patch to draw each Body and Soul is stored within it, so it's a matter of removing one and adding another.
        In ROS, this is called in the space node, just frequently enough.
        """
        t=time()
        for b in self.bodies:
            if b.time>self.time:
                if not b.pp==None:
                    b.pp.remove()
                b.pp=patches.Polygon(b.vertices,fc=b.fc) # the Body is dense
                self.ax.add_patch(b.pp)
                if isinstance(b,AniBody) and shoul:
                    for s in b.souls:
                        if s.time>self.time:
                            if not s.pp==None:
                                s.pp.remove()
                            if s.vertices==None:
                                s.pp=None
                            else:
                                s.pp=patches.Polygon(s.vertices,fill=False,ec=b.fc,lw=0.5,ls=':') # the Soul is ethereal
                                self.ax.add_patch(s.pp)   
        if showconn:
            while len(self.conngraph)>0:
                trash=self.conngraph.pop()
                trash[0].remove()
            for i in list(self.conn):
                (xi,yi)=(self.bodies[i].pos.x,self.bodies[i].pos.y)
                for j in list(self.conn[i]):
                    (xj,yj)=((self.bodies[j].pos.x,self.bodies[j].pos.y))
                    self.conngraph.append(self.ax.plot([xi,xj],[yi,yj],color=[0.7,0.7,0.7],lw=0.3,ls=':'))       
        plt.draw()
        if not self.has_been_closed():
            self.movie_writer.grab_frame()
            plt.pause(0.001)
        self.updates += 1
        if self.updates>0:
            self.avgT = ((self.updates-1)*self.avgT+(t-self.time))/self.updates
        self.time=t      

    def flash(self,bodies):
        """ Useful for debugging """
        i=0
        while i < 10:
            i += 1
            for b in bodies:
                if not b.pp==None:
                    b.pp.remove()
            plt.draw()
            plt.pause(0.02)    
            for b in bodies:
                if not b.pp==None:
                    self.ax.add_patch(b.pp)
            plt.draw()
            plt.pause(0.02)

    ## Body's management functions

    def bodindex(self,name):
        """ Returns the index in self.bodies of the Body named so """
        names=[self.bodies[i].name for i in range(len(self.bodies))]
        return names.index(name)

    def typindices(self,type):
        """ Returns the indices in self.bodies of the Body's of the type """
        indices=[]
        for i in range(len(self.bodies)):
            if isinstance(self.bodies[i],type):
                indices.append(i)
        return indices

    def update_dist(self):
        """ Updates the matrix of dist between all Body's in self.bodies """
        for i in range(len(self.bodies)):
            bi=self.bodies[i]
            for j in range(i+1,len(self.bodies)):
                bj=self.bodies[j]
                self.dist[i,j]=self.dist[j,i]=bi.pos.distance(bj.pos)

    def update_conn(self):
        """ Updates the list of conn pairs between AniBody's of the same type in self.bodies """
        self.conn={}
        for i in range(len(self.bodies)):
            if isinstance(self.bodies[i],AniBody):
                self.conn[i]=set()
        for i in list(self.conn):        
            type_i=type(self.bodies[i])
            for j in range(i+1,len(self.bodies)):
                if isinstance(self.bodies[j],type_i) and self.dist[i,j]<self.R: # i and j are kin and near
                    ray=LineString([self.bodies[i].pos,self.bodies[j].pos])
                    for k in range(len(self.bodies)):
                        bk=self.bodies[k]
                        if not k in (i,j) and self.dist[i,k]<self.dist[i,j]+bk.r_encl:
                            if isinstance(bk,Obstacle) and Polygon(bk.vertices).intersects(ray):
                                break
                    else:
                        self.conn[i] |= {j}
                        self.conn[j] |= {i}

    def graph(self,type):
        """ Returns the graph (dictionary of connections) formed by the AniBody's of type """
        result={}
        for i in range(len(self.bodies)):
            if isinstance(self.bodies[i],type):
                result[i]=self.conn[i]
        return result
    
    def remobodies(self,ko):
        """ Removes all the self.bodies in list ko """
        if len(ko)>0:
            ko=sorted(set(ko)) ## needs to be ordered
            ok=[]
            for i in range(len(self.bodies)):
                if not(i in ko):
                    ok.append(i)
            for i in ko:
                if visual:
                    self.bodies[i].pp.remove() # remove the Body patch in the plot
                    if isinstance(self.bodies[i],AniBody) and shoul:
                        for s in self.bodies[i].souls:
                            if not s.pp==None:
                                s.pp.remove() # remove the Soul patch in the plot
            ko.reverse() # after removing (poping) i, the j>i would advance one position
            for i in ko: # so remove in reverse order
                if logerror: print('Removed '+self.bodies.pop(i).name)
            self.dist=self.dist[ok,:][:,ok] # remove row and column from dist matrix
    
    def spawn_bodies(self,nm=50,nk=0,ns=0,nf=0,nn=0,nO=0,nMO=0,room=room):
        """ A convenience SCRIPT for spawning many self.bodies randomly, and initializing self.dist and the plot
        
        Typically, obstacles and nests won't be random, they will be a part of the definition of a case;
        Their creation should be done BEFORE calling this funcion, so they're taken into account for the initialization.
        Some details, e.g., initial and maximum velocities, might be changed, in their new=Body(...) call
        """
        s=self
        left=-1e9
        right=1e9
        bottom=-1e9
        top=1e9
        for vertice in room:
            x=vertice[0]
            y=vertice[1]
            if x>left:
                left=x
            elif x<right:
                right=x
            if y>bottom:
                bottom=y
            elif y<top:
                top=y
        # Obstacle's and MObstacle's can overlap
        i=0
        while i<nO:
            new=Obstacle('O'+str(i),(uniform(left,right),uniform(bottom,top)),uniform(-np.pi,np.pi))
            if s.fits(new,room,True):
                s.bodies.append(new)
                i += 1
        i=0
        while i<nMO:
            new=MObstacle('MO'+str(i),(uniform(left,right),uniform(bottom,top)),uniform(-np.pi,np.pi),v=vN/20,v_max=vN/10,w_max=wN/10)
            if s.fits(new,room,True):
                s.bodies.append(new)
                i += 1
        # But not the other Body's
        i=0
        while i<nn:
            new=Nest('n'+str(i),(uniform(left,right),uniform(bottom,top)),uniform(-np.pi,np.pi))
            if s.fits(new,room):
                s.bodies.append(new)
                i += 1
        i=0
        while i<nf:
            new=Food('f'+str(i),(uniform(left,right),uniform(bottom,top)),uniform(-np.pi,np.pi))
            if s.fits(new,room):
                s.bodies.append(new)
                i += 1
        i=0
        while i<nm:
            new=Mobot('m'+str(i),(uniform(left,right),uniform(bottom,top)),uniform(-np.pi,np.pi),v_max=vN/4,w_max=wN)
            if s.fits(new,room):
                s.bodies.append(new)
                i += 1
        i=0
        while i<nk:
            new=Killer('k'+str(i),(uniform(left,right),uniform(bottom,top)),uniform(-np.pi,np.pi),v_max=vN/2,w_max=wN/4)
            if s.fits(new,room):
                s.bodies.append(new)
                i += 1
        i=0
        while i<ns:
            new=Shepherd('s'+str(i),(uniform(left,right),uniform(bottom,top)),uniform(-np.pi,np.pi),v_max=vN/3,w_max=wN/2)
            if s.fits(new,room):
                s.bodies.append(new)
                i += 1
        s.dist=np.zeros((len(s.bodies),len(s.bodies))) # distances between centroids in a np.matrix
        s.update_dist()
        s.update_conn()
        if visual:
            if loginfo:
                for b in s.bodies:
                    if b.time>s.time: print(repr(b))
            s.redraw()
            print('Ready to start. You have a few secs to resize window, do it now or leave it.')
            plt.pause(2) # some pause is required by the GUI to show the effects o a draw, and cath events
                     # this first one is long to allow some time to resize window before starting
            s.time=time() # reset initial time of space
            s.t0=s.time
            s.updates=0
            for b in s.bodies:
                b.time=s.time # reset initial time of every body

    ## Collision detection functions

    def fits(self,new,where=room,noverlap=True):
        """ Returns True if the new Body fits in where (list of vertices of a polygon) """
        newPolygon=Polygon(new.vertices)
        newPolygon=newPolygon.buffer(new.r_encl)
        if noverlap:
            for old in self.bodies:
                if newPolygon.intersects(Polygon(old.vertices)):
                    return False
        if Polygon(where).contains(Polygon(new.vertices)):
            return True
        else:
            return False

    def collisions(self):
        """ Detect collisions between self.bodies and return list of ko AniBody's (only AniBody's die, they must avoid collisions) """
        """ TBD: return list of collisions instead, of any kind of Body's, to be filtered out outside of this function, more flexible use """
        ko=[]
        for i in range(len(self.bodies)):
            bi=self.bodies[i]
            if not(i in ko) and isinstance(bi,(Obstacle,AniBody)):
                for j in range(i+1,len(self.bodies)):
                    bj=self.bodies[j]
                    if not(j in ko) and isinstance(bj,(Obstacle,AniBody)) and self.dist[i,j]<(bi.r_encl+bj.r_encl) and Polygon(bi.vertices).intersects(Polygon(bj.vertices)):
                        if isinstance(bi,Mobot) and isinstance(bj,(Obstacle,Mobot,Killer)) or isinstance(bi,Killer) and isinstance(bj,(Obstacle,Killer,Shepherd)) or isinstance(bi,Shepherd) and isinstance(bj,(Obstacle,Shepherd)):
                            ko.append(i)
                        if isinstance(bj,Mobot) and isinstance(bj,(Obstacle,Mobot,Killer)) or isinstance(bj,Killer) and isinstance(bi,(Obstacle,Killer,Shepherd)) or isinstance(bj,Shepherd) and isinstance(bi,(Obstacle,Shepherd)):
                            ko.append(j)
        return ko
    
    ## Perception functions: perception functions must be defined in the Body's Space, not in the Body itself

    def nearby(self,i,r,rng,type):
        """ Returns a list with the Body's of type "visible" from Body i """
        pos=self.bodies[i].pos
        th=self.bodies[i].th
        nearby=[]
        a=np.linspace(-rng,rng,60) 
        vpa=[(r*np.cos(x),r*np.sin(x)) for x in a] # vertices in perception area, relative to pos and th
        if rng<np.pi: # when not full range, the body (its centroid) is another vertex
            vpa.append((0,0))
        pa=translate(rotate(Polygon(vpa),th,(0,0),True),pos.x,pos.y) # rotate th and translate to pos the perception area
        for j in range(len(self.bodies)):
            bj=self.bodies[j]
            if isinstance(bj,type) and j != i and self.dist[i,j]<(r+bj.r_encl) and pa.intersects(Polygon(bj.vertices)):
                ray=LineString([pos,bj.pos])
                for k in range(len(self.bodies)):
                    bk=self.bodies[k]
                    if not k in (i,j) and self.dist[i,k]<self.dist[i,j]+bk.r_encl:
                        if isinstance(bk,Obstacle) and Polygon(bk.vertices).intersects(ray):
                            break
                else:
                    nearby.append(bj)
        return nearby

    def nearest(self,i,r,rng,type):
        """ Returns the nearest to Body i of the nearby Body's of type """
        pos=self.bodies[i].pos
        nearby=self.nearby(i,r,rng,type)
        nearest=None
        mindist=r
        while len(nearby)>0:
            dist=pos.distance(Polygon(nearby[-1].vertices))
            if dist<mindist:
                nearest=nearby.pop()
                mindist=dist
            else:
                nearby.pop()
        return nearest

    def nearestpoint(self,i,b):
        """ Returns the nearest to Body i point of b """
        bi=Polygon(self.bodies[i].vertices)
        bj=Polygon(b.vertices)
        np=nearest_points(bi,bj)
        return np[1]

    def incontact(self,i,type):
        """ Returns a list with the Body's of type in contact with Body i """
        bi=self.bodies[i]
        pbi=Polygon(bi.vertices)
        incontact=[]
        for j in range(len(self.bodies)):
            bj=self.bodies[j]
            if isinstance(bj,type) and j != i and self.dist[i,j]<(bi.r_encl+bj.r_encl) and pbi.intersects(Polygon(bj.vertices)):
                incontact.append(bj)
        return incontact

class KPIdata: # the key performance indices 
    
    def __init__(self,name,n,T=RT):
        self.name=name
        self.T=T # s
        self.time=time()
        self.t0=self.time
        self.updates=-1
        self.avgT=0 # for timing statistics
        self.KPI=np.zeros(min(n,6)) # initial value of the n KPIs
        if visual:
            self.fig=plt.figure(figsize=(SS*16/np.sqrt(16**2+9**2), SS*9/2/np.sqrt(16**2+9**2))) # 16:9/2 proportion, SS inches diag (resizeable)
            self.fig.canvas.set_window_title('KPI '+self.name)
            self.ax=self.fig.add_axes([0.1,0.1,0.8,0.8]) # 10% margin
            self.ax.set_facecolor('w') # white background
            self.ax.set_xlim(0, self.T/TS) # initial time scale (will expand from 0 to "now" as time passes)
            self.ax.set_ylim(0, 1) # all KPIs are normalized
            self.color=['k','r','g','b','m','c'] # ready for n up to 6
            self.update(self.KPI) # required to init axes and time
            plt.pause(0.001)      

    def has_been_closed(self):
        """ Returns True when the figure where self is drawn is not active """
        fig=self.ax.figure.canvas.manager
        active_figs=plt._pylab_helpers.Gcf.figs.values()
        return fig not in active_figs 

    def close(self):
        if visual:
            self.fig.savefig(self.name+'.pdf', orientation='landscape', format='pdf')
        del self

    def update(self,KPI):
        """ Writes/Plots the last KPI values """
        t=time()
        with open(self.name+'.dat','a') as f:
            f.write('{0:1.2f}'.format((t-self.t0)/TS))
            for i in range(len(self.KPI)):
                f.write(' {0:1.2f}'.format(KPI[i]))
            f.write('\n')
        if visual:
            self.ax.set_xlim(0, (t-self.t0)/TS)
            for i in range(len(self.KPI)):
                plt.plot([(self.time-self.t0)/TS,(t-self.t0)/TS],[self.KPI[i],KPI[i]],self.color[i])
            plt.draw()
            if not self.has_been_closed():
                plt.pause(0.001) # some pause is required by the GUI to show the effects o a draw, and cath events
            for i in range(len(self.KPI)):
                self.KPI[i]=KPI[i]
        else:
            print('{0:1.2f}'.format((t-self.t0)/TS))
            for i in range(len(self.KPI)):
                print(' {0:1.2f}'.format(KPI[i]))
            print('\n')
        self.updates += 1
        if self.updates>0: self.avgT = ((self.updates-1)*self.avgT+(t-self.time))/self.updates
        self.time=t

class Body: # Something in a Space
    
    def __init__(self,name,pos=-1,th=0,area=-1,vertices=[],fc='k'):
        """ name should be unique, it is for info but also used to find a Body
            pos is (x,y), within [-W:W,-H:H] save for borders; -1 for absolute vertices, typically for obstacles
            th in rad, 0 is pointing right
            area is -1 for absolute vertices; constant values below have been chosen to be nice with W=16 H=9 (total area=576)
            vertices are relative (save otherwise indicated by -1's), they are affected by area scale, pos translation and th rotation
            fc='k' by default; the default for subclasses are different (grey, blue, yellow, green, red, cyan), and typically objects are fc-ish
        """
        self.name=name # str
        self.fc=fc #(R,G,B)
        if len(vertices)>0:
            self.vertices=vertices # list of (x,y) defining a polygon, for graphical representation
        else:
            self.vertices=[(0,0),(0.01,0.0025),(0,0.005)] # good for a MoBody
        polygon=Polygon(self.vertices)
        if area==-1: # do not scale
            self.area=polygon.area
        else:
            self.area=area
            fact=np.sqrt(self.area/polygon.area)
            polygon=scale(polygon,fact,fact,origin='centroid')
        centroid=polygon.centroid
        if pos==-1: # vertices positions are absolute
            self.pos=centroid
            self.th=0
        else:
            self.pos=Point(pos) # unconstrained, but usually within [-W:W,-H:H] -- except borders
            self.th=pipi(th) # rad in [-pi:pi]
            polygon=translate(polygon,self.pos.x-centroid.x,self.pos.y-centroid.y)
            polygon=rotate(polygon,self.th,origin='centroid',use_radians=True)
        self.vertices=list(polygon.exterior.coords)
        self.r_encl=0 # radius of enclosing circle centered at c
        for vertex in self.vertices:
            Pv=Point(vertex)
            d=self.pos.distance(Pv)
            if d>self.r_encl:
                self.r_encl=d            
        if visual:
            self.pp=None # to store the current plot patch
        self.time=time()
        self.updates=-1
        self.avgT=0

    def __repr__(self):
        return self.name+' is a '+str(self.__class__)[17:-2]+' at ({0:1.2f},{1:1.2f})'.format(self.pos.x,self.pos.y)+' with area {0:1.2e}'.format(self.area)+' of color '+str(self.fc)

    def update(self):
        dt=time()-self.time
        self.updates += 1
        if self.updates>0:
            self.avgT = ((self.updates-1)*self.avgT+dt)/self.updates
        self.time += dt
        return dt

class Obstacle(Body):
    
    def __init__(self,name,pos=0,th=0,area=0,vertices=[],fc=0):
        """ pos==0 for random pose, area==0 for random size """
        if area==0: # random size
            area=uniform(0.5,5)
        if len(vertices)==0:
            vertices=[(0,0),(1,0),(1,1),(0,1)]
        if fc==0:
            R=G=B=uniform(0.2,0.4) # dark grey
            fc=(R,G,B)
        super().__init__(name,pos,th,area,vertices,fc)

class Nest(Body):
    def __init__(self,name,pos=0,th=0,area=0,vertices=[],fc=0):
        """ pos==0 for random pose, area==0 for random size """
        if pos==0: # random pose
            pos=(uniform(-W,W),uniform(-H,H))
            th=uniform(-np.pi,np.pi)
        if area==0: # random size
            area=uniform(1,10)
        if len(vertices)==0:
            for i in range(36):
                vertices.append((np.cos(i/18*np.pi),np.sin(i/18*np.pi)))
        if fc==0:
            R=G=0
            B=uniform(0.5,1) # blue
            fc=(R,G,B)
        super().__init__(name,pos,th,area,vertices,fc)

class Food(Body):
    def __init__(self,name,pos=0,th=0,area=0,vertices=[],fc=0):
        """ pos==0 for random pose, area==0 for random size """
        if pos==0: # random pose
            pos=(uniform(-W,W),uniform(-H,H))
            th=uniform(-np.pi,np.pi)
        if area==0: # random size
            area=uniform(0.005,0.02)
        if len(vertices)==0:
            for i in range(12):
                vertices.append((np.cos(i/6*np.pi),np.sin(i/6*np.pi)))
        if fc==0:
            R=G=uniform(0.7,1)
            B=0 # yellow-ish
            fc=(R,G,B)
        super().__init__(name,pos,th,area,vertices,fc)

class MoBody(Body): # Moving Body

    def __init__(self,name,pos=-1,th=0,area=-1,vertices=[],fc='k',v=0,w=0,vth=0,v_max=0,w_max=0):
        super().__init__(name,pos,th,area,vertices,fc)
        self.v=v
        self.w=w
        self.vth=vth
        self.set_vel_max(v_max,w_max)

    def __repr__(self):
        return super().__repr__()+' with velocity '+str((self.v,self.w,self.vth))

    def teleport(self,pos=None,th=None):
        """ Teleport to pos,th """
        if len(self.vertices)>0:
            polygon=Polygon(self.vertices)
            if pos!=None:
                pos=Point(pos)
                polygon=translate(polygon,pos-self.pos)
            if th!=None:
                th=pipi(th)
                polygon=rotate(polygon,pipi(th-self.th),origin='centroid',use_radians=True)
            self.vertices=list(polygon.exterior.coords)
        if pos!=None: self.pos=pos
        if th!=None: self.th=pipi(th)
        self.time=time() # something changed

    def set_vel_max(self,v_max=0,w_max=0):
        """ Change maximum velocities """
        self.v_max=max(0,v_max) # dist/s
        self.w_max=max(0,w_max) # rad/s
        self.cmd_vel() # apply the new max

    def cmd_vel(self,v='=',w='=',vth='='):
        """ Change v, w, vth respecting their limits. '=' means don't change. v and w can be increased or decreased with '+' or '-'."""
        if v=='=':
            v=self.v
        elif v=='+':
            v=self.v+self.v_max/10
        elif v=='-':
            v=self.v-self.v_max/10
        self.v=max(0,min(v,self.v_max)) # dist/s
        if w=='=':
            w=self.w
        elif w=='+':
            w=self.w+self.w_max/10
        elif w=='-':
            w=self.w-self.w_max/10
        self.w=max(-self.w_max,min(w,self.w_max)) # rad/s
        if vth=='=':
            vth=self.vth
        self.vth=max(-np.pi,min(vth,np.pi)) # rad (angle of velocity vector)
        self.time=time() # something changed

    def update(self):
        """ Update pose of self and time-stamp. Typically called (frequently) when v,w>0
        
        Assumed dtime short wrt. velocity values: linear approx
        """
        dt=super().update()
        if self.v+abs(self.w)>0:
            dth=self.w*dt
            dx=self.v*dt*np.cos(self.th+dth/2+self.vth) # vth is the rel angle of velocity vector
            dy=self.v*dt*np.sin(self.th+dth/2+self.vth)
            # Going beyond one limit teleports to the contrary
            if self.pos.x+dx>W:
                dx=dx-2*W
            elif self.pos.x+dx<-W:
                dx=dx+2*W
            if self.pos.y+dy>H:
                dy=dy-2*H
            elif self.pos.y+dy<-H:
                dy=dy+2*H
            self.pos=translate(self.pos,dx,dy)
            self.th=pipi(self.th+dth)
            if len(self.vertices)>0:
                polygon=Polygon(self.vertices)
                polygon=translate(polygon,dx,dy)
                polygon=rotate(polygon,dth,origin='centroid',use_radians=True)
                self.vertices=list(polygon.exterior.coords)
        return dt

class MObstacle(Obstacle,MoBody): # A Moving Obstacle
    def __init__(self,name,pos=0,th=0,area=0,vertices=[],fc=0,v=0,w=0,vth=0,v_max=0,w_max=0):
        if fc==0:
            R=G=B=uniform(0.6,0.8) # light grey
            fc=(R,G,B)
        if len(vertices)==0:
            for i in range(30):
                vertices.append((np.cos(i/15*np.pi),np.sin(i/15*np.pi)))        
        super().__init__(name,pos,th,area,vertices,fc)
        self.set_vel_max(v_max,w_max)
        self.cmd_vel(v,w,vth)

    def update(self): # random velocity changes in the central area of the room
        super().update()
        if randint(0,10)>9:
            self.cmd_vel(choice(('+','-','=')),choice(('+','-','=')),'=')
        if self.pos.x>W/2 and abs(self.th)<np.pi/2 or self.pos.x<-W/2 and abs(self.th)>np.pi/2 or self.pos.y>H/2 and self.th>0 or self.pos.y<-H/2 and self.th<0:
            self.th = pipi(self.th+np.pi)

class AniBody(MoBody): # An Animated (Moving) Body
    def __init__(self,name,pos=-1,th=0,area=-1,vertices=[],fc='k',v=0,w=0,vth=0,v_max=0,w_max=0):
        super().__init__(name,pos,th,area,vertices,fc,v,w,vth,v_max,w_max)
        self.souls=[] # They must be given later by calling for a Soul(body,T)
        self.knows=None # It must be given later by calling for a Knows(body)

    def update(self): # AniBodies specialize the MoBody.update method by updating their souls that correspond
        dt=super().update()
        for s in self.souls:
            if (self.time-dt)>s.t0+(s.updates+1)*s.T:
                s.update()

class Mobot(AniBody): # A member of the swarm of mobile robots; there should be many
    def __init__(self,name,pos=0,th=0,area=0,vertices=[],fc=0,v=0,w=0,vth=0,v_max=0,w_max=0):
        if area==0:
            area=uniform(0.005,0.01)
        if fc==0:
            R=0
            G=uniform(0.7,1)
            B=uniform(0,0.5*G) # green-ish
            fc=(R,G,B)
        super().__init__(name,pos,th,area,vertices,fc,v,w,vth,v_max,w_max)

class Killer(AniBody): # Chases Mobots; might be more than one, collaborating or not
    def __init__(self,name,pos=0,th=0,area=-1,vertices=[],fc=0,v=0,w=0,vth=0,v_max=0,w_max=0):
        if area==0:
            area=uniform(0.01,0.02)
        if fc==0:
            R=uniform(0.7,1)
            G=0
            B=uniform(0,0.5*R) # red-ish
            fc=(R,G,B)
            vertices=[(4,0),(0,0.5),(-2,2),(-1,0),(-2,-2),(0,-0.5)]
        super().__init__(name,pos,th,area,vertices,fc,v,w,vth,v_max,w_max)

class Shepherd(AniBody): # Takes care of the Mobots; might be more than one, collaborating or not
    def __init__(self,name,pos=0,th=0,area=0,vertices=[],fc=0,v=0,w=0,vth=0,v_max=0,w_max=0):
        if area==0:
            area=uniform(0.01,0.02)
        if fc==0:
            R=0
            G=B=uniform(0.7,1) # cyan-ish
            fc=(R,G,B)
            vertices=[(4,0),(0,1),(-2,0),(0,-1)]
        super().__init__(name,pos,th,area,vertices,fc,v,w,vth,v_max,w_max)

class Soul: # Something within a AniBody that controls it: manipulates its behavior in response to its environment, possibly using what it knows
    """ Souls often access to the (ani)body variables, including what it knows     
        They are called by their body periodically, each self.T (or asap if self.T==0)
        a body might have several souls, typically for low-level and high-level loops
    """

    def __init__(self,body,T=0):
        self.body=body
        self.T=T # s
        self.time=time()
        self.t0=self.time
        self.body.time=self.time
        self.updates=-1
        self.avgT=0
        body.souls.append(self)
        self.vertices=None # vertices of a polygon for graphical representation
        if visual:
            self.pp=None #  # to store the current plot patch

    def update(self):
        """ this update is called in the (ani)body update when the (body.time-soul.time)>soul.T """
        pass # the update of a basic Soul does nothing at all --- if it did, it would come here (don't think so)
        # subclasses of Soul specialize this update function their own way
        # and they end with a call to this super().update() --- or (if this update is finally so empty) they could do directly:
        dt=time()-self.time
        self.updates += 1
        if self.updates>0:
            self.avgT = ((self.updates-1)*self.avgT+dt)/self.updates
        self.time += dt
        return dt

class GoTo(Soul):
    """ Low-level Soul to go somewhere by several step-by-step cmd_vel's (rather than one teleport)

        When no where to go keepgoing, stop or wander. TBD: add obstacle avoidance
    """

    def __init__(self,body,T=0,Kp=1,tol=0.01,nw='keepgoing',p=0.1):
        self.Kp=Kp
        self.tol=tol
        self.nw=nw # ''keepgoing', stop', 'wander', 'zigzag' when no where to go
        self.p=p
        self.where=None
        self.when=0
        super().__init__(body,T)
    
    def cmd_set(self,where='=',after='=',Kp='=',tol='=',nw='=',p='='):
        """ Set destination, arrival time after secs, Kp, tol, and p; '=' to keep current values  """
        if where != '=':
            if where==None:
                self.where=None
            else:
                self.where=Point(where) # (x,y) or Point destination
        if after != '=': self.when=time()+after # time arrival time
        if Kp != '=': self.Kp=Kp # Kp for the P ctrl of w wrt th error
        if tol != '=': self.tol=tol # tolerance distance from destination
        if nw != '=': self.nw=nw # 'stop', 'keepgoing', 'wander' when no where to go
        if p != '=': self.p=p # probability [0,1] to change vel when wandering

    def update(self):
        super().update()
        if self.where != None:
            arrow=Point2D(self.where.x-self.body.pos.x,self.where.y-self.body.pos.y)
            self.vertices=[(self.body.pos.x,self.body.pos.y),(self.where.x,self.where.y)]
            if arrow.r<self.tol:
                self.where=None
            else:
                if self.when>self.time:
                    v=arrow.r/(self.when-self.time)
                else: v=np.Infinity
                w=self.Kp*((pipi(arrow.a-self.body.th)))
                self.body.cmd_vel(v,w)
        else:
            self.vertices=None
            if self.nw=='stop': # reset v and w
                self.body.cmd_vel(v=0,w=0)
            elif self.nw=='wander': # random changes of w
                if uniform(0,1)<self.p:
                    self.body.cmd_vel(w=choice(('+','-','=')))
            elif self.nw=='zigzag': # random changes of direction
                if uniform(0,1)<self.p:
                    self.body.teleport(th=self.body.th+uniform(-1,1)*self.body.w_max*self.T)
            elif self.nw=='keepgoing': # keep v and w
                pass

class Voronoid(Soul):
    """ Soul of a Voronoi's-based mobile sensor in a network covering a static region, Cortes2004 like """
    
    def __init__(self,body,space,r=1,rng=np.pi,T=TS,Q=None):
        self.space=space
        self.r=r
        self.rng=rng # might be smaller than pi, but not too much, or it would work badly, too many collisions
        self.Q=Polygon(Q) # polygon region to be covered
        GoTo(body,T/10,Kp=0.2,tol=0.01,nw='stop') # requires a GoTo low-level soul executing faster in the same body
        self.GoTo=body.souls[-1] # this way it knows how to call it
        self.d=0 # distance of last GoTo /r (a KPI)
        super().__init__(body,T)

    def update(self):
        super().update()
        i=self.space.bodindex(self.body.name)
        bpos=(self.body.pos.x,self.body.pos.y)
        neigh=self.space.nearby(i,self.r,self.rng,type(self.body))
        npos=[]
        for n in neigh:
            npos.append((n.pos.x,n.pos.y))
        self.W = self.Q & voronoi(bpos,npos,self.r)
        self.vertices=list(self.W.exterior.coords) # soul represented by the destination Voronoi cell
        self.GoTo.cmd_set(self.W.centroid,self.T)
        self.d=Point(bpos).distance(self.W.centroid)/self.r

class Knowledge: # What a (Ani)Body knows
    """ A class to contain what a (Ani)Body knows, and to communicate it to others
        The basic version contains only a state variable, that can be set and tell
    """

    def __init__(self,body,state='idle'):
        self.body=body
        self.state=state
        body.knows=self

    def set_state(self,state):
        self.state=state

    def tell_state(self):
        return self.state

if __name__ == '__main__': # a meaningless demo of almost everything (and a typical pattern for the main code)
    name='SW2D'+strftime("%Y%m%d%H%M", localtime())
    s=Space(name,T=RT,limits='hv')
    p=KPIdata(name,2,TS)
    N=50
    s.spawn_bodies(N,3,3,100,2,10,5)
    for b in s.bodies:
        if isinstance(b,MoBody):
            b.cmd_vel(v=b.v_max/2)
        if isinstance(b,AniBody):
            GoTo(b,0.1*TS,nw='wander') # they wander to no where
    KPI=[1,0]
    end=False
    while not end:
        for b in s.bodies:                        
            if isinstance(b,MoBody):
                b.update()
        s.update_dist()
        s.remobodies(s.collisions())
        s.update_conn()
        if visual and time()>s.t0+(s.updates+1)*s.T:
            s.redraw()
        if time()>p.t0+(p.updates+1)*p.T:
            KPI=[0,0]
            for b in s.bodies:
                if isinstance(b,Mobot):
                    KPI[0] += 1
                    KPI[1] += b.avgT
            if KPI[0]>0: KPI[1] /= KPI[0]
            KPI[0] /= N
            p.update(KPI)
        end = s.has_been_closed() or KPI[0]==0 or p.updates>300
    else:
        s.close()
        p.close()           
