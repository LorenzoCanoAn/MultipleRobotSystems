## A solution to the Boids exercise
## Version: 20210517
## Author: Enrique Teruel (ET) eteruel@unizar.es
## License: CC-BY-SA

from time import time, localtime, strftime
import numpy as np
# YOU MIGHT FIND USEFUL FUNCTIONS IN shapely, Point2D, AND gadgETs
from small_worl2d_config import visual, TS, RT
from small_worl2d import Space, KPIdata, MoBody, Mobot, Soul

class Boid(Soul):

    def __init__(self,body,space,goodistance=1,T=TS): # ADD YOUR ARGUMENTS
        self.space=space
        self.goodistance=goodistance
        # YOUR BOID INIT CODE
        super().__init__(body,T)

    def update(self):
        b=self.body
        i=self.space.bodindex(b.name)
        # YOUR BOID UDATE CODE
        super().update()

## MAIN

R=1.8 # so goodistance=1 is little less than half R
name='Boids_R'+str(R)+'_'+strftime("%Y%m%d%H%M", localtime())
s=Space(name,T=RT,R=R,limits='')
N=50
s.spawn_bodies(N,room=[(-4,-4),(4,-4),(4,4),(-4,4)])
for b in s.bodies:
    if isinstance(b,Mobot):
        Boid(b,s,R,0.1*TS) # ADD YOUR ARGUMENTS
p=KPIdata(name,5,TS)
KPI=[1,1,np.sqrt(N),1,0]
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
        # YOUR KPI CALCULATIONS
        p.update(KPI)
    end= s.has_been_closed() or KPI[1]==0 or p.updates>300 # 5 realtime min
else:
    s.close()
    p.close()
