## A Demo of SmallWorl2D
## Version: 1.0.20210514
## Author: Enrique Teruel (ET) eteruel@unizar.es
## License: CC-BY-SA

from time import time, localtime, strftime
from random import uniform
import numpy as np
from matplotlib.animation import FFMpegWriter # requires having ffmpeg installed, from https://ffmpeg.org/

from small_worl2d_config import visual, W, H, TS, RT # configuration parameters in a separate file
from small_worl2d import Space, KPIdata, Nest, Mobot, Soul, GoTo, Knowledge

class Nestxists(Knowledge):
    """ A silly specialization of Knowledge, actually it is just the basic Knowledge
        
        Typically, here would be defined the specific variables and functions required
    """

    def __init__(self,body,state):
        super().__init__(body,state)

class Demo(Soul):
    """ A Soul that wanders differently if it knows ThereAreNests or Nothing """

    def __init__(self,body,space,T=TS,r=1,rng=np.pi):
        self.space=space
        self.r=r
        self.rng=rng # might be smaller than pi, but not too much
        GoTo(body,0,Kp=0.2,tol=0.01,nw='zigzag',p=0.1) # requires a GoTo soul in the same body (to go nowhere)
        self.GoTo=body.souls[-1] # this way it knows how to call it
        Nestxists(body,'KnowNothing') # this Soul needs a Mind in its Body to work
        super().__init__(body,T)

    def update(self):
        if self.body.knows.tell_state()=='KnowNothing':
            i=self.space.bodindex(self.body.name)
            nest=self.space.nearest(i,self.r,self.rng,Nest)
            if nest!=None:
                self.GoTo.cmd_set(self.space.nearestpoint(i,nest)) # let's pay it a visit
            if len(self.space.incontact(i,Nest))>0:
                self.body.knows.set_state('ThereAreNests') # I've been in one!
            neigh=self.space.nearby(i,self.r,self.rng,type(self.body))
            for n in neigh:
                if n.knows.tell_state()=='ThereAreNests':
                    self.body.knows.set_state('ThereAreNests') # If some neigh is aware of Nests, then so I am
                    break
            if self.body.knows.tell_state()=='ThereAreNests': # changes color and starts moving faster and more straight
                self.body.fc='m' # this is NOT the usual way to show a soul, see Voronoid or GoTo for examples
                self.body.set_vel_max(self.body.v_max*2,self.body.w_max/4)
                self.body.cmd_vel(self.body.v_max,0)
                self.GoTo.cmd_set(None)
        super().update()

## MAIN

name='Demo'+strftime("%Y%m%d%H%M", localtime())
s=Space(name,T=RT,limits='')
p=KPIdata(name,2,TS)
# a big nest in the second quadrant:
bignest=Nest('BigNest',pos=(uniform(-0.8*W,-0.2*W),uniform(0.4*H,0.6*H)),area=0.04) # 
s.bodies.append(bignest)
N=100
s.spawn_bodies(nm=N)
for b in s.bodies:
    if isinstance(b,Mobot):
        b.cmd_vel(v=b.v_max/2)
        Demo(b,s,TS/10,r=2,rng=np.pi/2) # a new Soul for b (who assigns new Knowledge too)
KPI=[0,1] # [ fraction of live mobots who know ThereAreNests , fraction of initial robots still alive ]
end=False
while not end:
    for b in s.bodies:
        if isinstance(b,Mobot):
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
                KPI[1] += 1
                if b.knows.tell_state()=='ThereAreNests':
                    KPI[0] += 1
        KPI[0] /= KPI[1]
        KPI[1] /= N
        p.update(KPI)
    end=s.has_been_closed() or KPI[0]>0.9 or KPI[1]<0.1 
else:
    s.close()
    p.close()
