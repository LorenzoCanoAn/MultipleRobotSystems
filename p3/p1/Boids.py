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

    def __init__(self,body,space,goodistance,T=TS): # ADD YOUR ARGUMENTS
        self.space=space
        self.goodistance=goodistance
        # YOUR BOID INIT CODE
        super().__init__(body,T)


    def update(self):

        b=self.body

        i=self.space.bodindex(b.name)

        p=self.proximal()


        a=self.allignment()

        f=[p[0]+a[0],p[1]+a[1]]

        b.cmd_vel(v=f[0],w=0.45*f[1])

        # YOUR BOID UDATE CODE
        super().update()

    def allignment(self):
        i = self.space.bodindex(b.name)
        neigh = self.space.nearby(i, self.space.R, np.pi, type(self.body))
        avarage=0
        for n in neigh:
            avarage=avarage+n.th
        if len(neigh)>0:
            avarage=avarage/len(neigh)
        th=avarage-b.th

        return [np.cos(th),np.sin(th)]

    def proximal(self):
        i = self.space.bodindex(b.name)
        neigh = self.space.nearby(i, self.space.R, np.pi, type(self.body))
        result=[0,0]
        for n in neigh:
            result[0]=result[0]+self.contribution_proximal(n)[0]
            result[1] = result[1] + self.contribution_proximal(n)[1]

        return result


    def contribution_proximal(self, n):
        alpha=2
        epsilon=1.5
        sigma=0.71

        rng=self.rang(n)
        bear=self.bearing(n)

        return [-(4*alpha*epsilon/rng)*(2*np.power((sigma/rng),2*alpha)-np.power((sigma/rng),alpha))*np.cos(bear),-(4*alpha*epsilon/rng)*(2*np.power((sigma/rng),2*alpha)-np.power((sigma/rng),alpha))*np.sin(bear)]




    def bearing(self,n):
        return np.arctan2((n.pos.y - self.body.pos.y),(n.pos.x-self.body.pos.x))-b.th

    def  rang(self,n):
        return np.linalg.norm([(n.pos.y - self.body.pos.y),(n.pos.x-self.body.pos.x)])





        print('avarage: ', avarage)
        print(len(neigh))



## MAIN
if __name__ == '__main__':
    R=1.8 # so goodistance=1 is little less than half R
    name='Boids_R'+str(R)+'_'+strftime("%Y%m%d%H%M", localtime())
    s=Space(name,T=RT,R=R,limits='')
    N=50
    s.spawn_bodies(N,room=[(-4,-4),(4,-4),(4,4),(-4,4)])
    for b in s.bodies:
        if isinstance(b,Mobot):
            Boid(b,s,1,0.1*TS) # ADD YOUR ARGUMENTS
    p=KPIdata(name,5,TS)
    KPI=[1,1,np.sqrt(N),1,0]
    end=False
    while not end:
        for b in s.bodies:
            if isinstance(b,MoBody):
                b.update()
        s.update_dist()
        #s.remobodies(s.collisions())
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
