## A Demo of SmallWorl2D
## Version: 1.0.20210514
## Author: Enrique Teruel (ET) eteruel@unizar.es
## License: CC-BY-SA

from time import time_ns, time, localtime, strftime
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
        self.info_nest=[0,0]
        self.follow=False
        # self.agrees=0
        # self.disagrees=0
        # self.people=0

        self.previous_time=time()
        super().__init__(body,state)
    def check_quadrant(self,body):
        if body.pos.x>0 and body.pos.y>0:
            return 1
        elif body.pos.x<=0 and body.pos.y>0:
            return 2
        elif body.pos.x<=0 and body.pos.y<=0:
            return 3
        elif body.pos.x>0 and body.pos.y<=0:
            return 4
    def change_belief(self,body):
        if self.disagrees>self.agrees:
            a=self.agrees
            self.agrees=self.disagrees
            self.disagrees=a
            if self.belief==2:
                self.belief=4
                body.fc=(1,0,0)
            elif self.belief==4:
                self.belief=2
                body.fc=(0,0,1)
    def restart(self,body):
        self.agree=0
        self.disagree=0
        self.belief=0
        self.body.fc==(0,1,0)

class Demo(Soul):
    """ A Soul that wanders differently if it knows ThereAreNests or Nothing """

    def __init__(self,body,space,T=TS,r=1,rng=np.pi):
        self.space=space
        self.r=r
        self.rng=rng # might be smaller than pi, but not too much
        GoTo(body,0,Kp=0.2,tol=0.01,nw='zigzag',p=0.1) # requires a GoTo soul in the same body (to go nowhere)
        self.GoTo=body.souls[-1] # this way it knows how to call it
        Nestxists(body,'Explore') # this Soul needs a Mind in its Body to work
        super().__init__(body,T)
        self.num=0


    def change_color(self,word):
        if word=='To stay':
            if self.body.knows.check_quadrant(self.body)==2:
                self.body.fc = (0, 0, 1)
            elif self.body.knows.check_quadrant(self.body)==4:
                self.body.fc = (1, 0, 0)

        elif word=='To leave':
            self.body.fc = (0, 0, 0)

        elif word=='To walk':
            self.body.fc=(0,1,0)
            if self.body.knows.info_nest[1]==2:
                self.body.fc = (0.678, 0.847, 0.902)

            elif  self.body.knows.info_nest[1]==4:
                self.body.fc = (255/255, 182/255, 193/255)

            else:
                self.body.fc = (0, 1, 0)

    def set_velocity_random_walk(self):
        if self.body.knows.previous_time + 10 < time():
            self.body.knows.previous_time = time()
            w_set = self.body.w_max * np.random.normal(0, 1 / 5)
            self.body.cmd_vel(v=self.body.v_max, w=w_set)
        elif time() > self.body.knows.previous_time + 1:
            self.body.cmd_vel(v=self.body.v_max, w=0)
    def set_velocity_stay(self):
        i = self.space.bodindex(self.body.name)
        nest = self.space.nearest(i, self.r, self.rng, Nest)
        if nest != None:
            if self.body.knows.info_nest[1]==self.body.knows.check_quadrant(self.body):
                self.GoTo.cmd_set(self.space.nearestpoint(i, nest))
            else:
                self.body.knows.set_state('Leave')

    def set_velocity_leave(self):
        self.GoTo.cmd_set(None)
        self.body.cmd_vel(v=self.body.v_max, w=0)

    def algorithm_0(self):
        if self.body.knows.tell_state()=='Explore':
            i = self.space.bodindex(self.body.name)
            nest=self.space.nearest(i,self.r,self.rng,Nest)
            if nest!=None:

                if self.body.knows.belief==0:
                    self.body.knows.belief=self.body.knows.check_quadrant(self.body)
                    self.body.knows.agrees = self.body.knows.agrees + 1
                    if self.body.knows.belief==2:
                        self.body.fc=(0,0,1)
                    else:
                        self.body.fc=(1,0,0)
            neigh = self.space.nearby(i, self.r, self.rng, type(self.body))
            for n in neigh:
                if n.knows.belief>0:
                    if n.knows.belief==self.body.knows.belief:
                        self.body.knows.agrees=self.body.knows.agrees+1
                    if self.body.knows.belief==0:
                        self.body.knows.belief = n.knows.belief
                        if self.body.knows.belief == 2:
                            self.body.fc = (0, 0, 1)
                        else:
                            self.body.fc = (1, 0, 0)
                    else:
                        self.body.knows.disagrees=self.body.knows.disagrees+1
                        self.body.knows.change_belief(self.body)
            if uniform(0,1)<0.01:
                if self.body.knows.belief>0:
                    if self.body.knows.belief==2:
                        self.body.knows.set_state('GoToNest')
                        self.body.fc = (0, 0, 0.6)
                    else:
                        self.body.knows.set_state('GoToNest')
                        self.body.fc = (0.6, 0, 0)


        if self.body.knows.tell_state() == 'GoToNest':
            i = self.space.bodindex(self.body.name)
            neigh = self.space.nearby(i, self.r, self.rng, type(self.body))
            for n in neigh:
                if n.knows.belief>0:
                    if n.knows.belief==self.body.knows.belief:
                        self.body.knows.agrees=self.body.knows.agrees+1
                        if n.knows.state == 'Nest':
                            self.GoTo.cmd_set(n.souls[-2].where)  # let's pay it a visit
                            self.body.knows.set_state('Nest')

                    else:
                        self.body.knows.disagrees=self.body.knows.disagrees+1
                        self.body.knows.change_belief(self.body)

            nest=self.space.nearest(i,self.r,self.rng,Nest)
            if nest!=None and self.body.knows.belief==self.body.knows.check_quadrant(self.body):
                self.GoTo.cmd_set(self.space.nearestpoint(i,nest)) # let's pay it a visit
                self.body.knows.set_state('Nest')

            if uniform(0,1)<0.001:
                self.body.knows.set_state('Explore')
                self.body.knows.restart(self.body)
                self.GoTo.cmd_set(None)

        if self.body.knows.tell_state() == 'Nest':
            i = self.space.bodindex(self.body.name)
            neigh = self.space.nearby(i, self.r, self.rng, type(self.body))
            self.people=len(neigh)
            for n in neigh:
                if n.knows.tell_state()=='Comunicate' and self.body.knows.check_quadrant(self.body)!=n.knows.belief:
                   if n.knows.people>self.body.knows.people:
                       self.GoTo.cmd_set(None)
                       self.body.knows.belief=n.knows.belief
                       if self.body.knows.belief==2:
                           self.body.fc=(0,0,1)
                       else:
                            self.body.fc=(1,0,0)
                       self.body.knows.set_state('Comunicate')


            if uniform(0,1)<0.001:
                self.body.knows.set_state('Comunicate')
                self.GoTo.cmd_set(None)


        if self.body.knows.tell_state() == 'Comunicate':
            i = self.space.bodindex(self.body.name)
            nest = self.space.nearest(i, self.r, self.rng, Nest)
            if nest != None and self.body.knows.belief!=self.body.knows.check_quadrant(self.body):
                self.GoTo.cmd_set(self.space.nearestpoint(i,nest)) # let's pay it a visit
                self.body.knows.set_state('Comunicating')

        if self.body.knows.tell_state() == 'Comunicating':
            i = self.space.bodindex(self.body.name)
            neigh = self.space.nearby(i, self.r, self.rng, type(self.body))
            for n in neigh:
                    number=0
                    sum=0
                    if n.knows.tell_state=='Nest':
                        number=number+1
                        sum=sum+n.knows.people

            if number>0:
                    if sum/(number)>self.body.knows.people:
                        if uniform(0,1)>0.01:
                            self.body.knows.set_state('GoToNest')
                            self.GoTo.cmd_set(None)
            elif number==0:
                if uniform(0, 1) > 0.01:
                    self.body.knows.set_state('GoToNest')
                    self.GoTo.cmd_set(None)
            else:
                    self.body.knows.agree=0
                    self.body.knows.disagree=0
                    self.body.knows.people=0
                    self.body.knows.belief=self.body.knows.check_quadrant(self.body)
                    self.body.knows.set_state('Nest')


    def algorithm_1(self):
        a=3.5 #The higher, the more likely to stay
        b=10  #The higher, the less likely to leave a nest

        if self.body.knows.tell_state() == 'Random_Walk':
            self.set_velocity_random_walk()
            self.change_color('To walk')

            i = self.space.bodindex(self.body.name)
            nest = self.space.nearest(i, self.r, self.rng, Nest)
            neigh = self.space.nearby(i, self.r, self.rng, type(self.body))
            if nest != None:
                num_neigh=len(neigh)
                print('Robot ',i,' has found a nest, and it has ', num_neigh, 'neighbours',' its radius is ', nest.r_encl)
                if self.body.knows.info_nest[0]<nest.r_encl:
                    self.body.knows.info_nest=[nest.r_encl,self.body.knows.check_quadrant(self.body)]



                if uniform(0,1)<0.2+0.6*(1-np.exp(-a*num_neigh)) and self.body.knows.check_quadrant(self.body)==self.body.knows.info_nest[1]:
                    self.body.knows.set_state('Stay')
                    print('Robot ', i, 'Stays')
                else:
                    print('Robot ', i, 'Leaves')
                    self.body.knows.set_state('Leave')
                    self.body.knows.previous_time = time()
                    self.body.cmd_vel(v=self.body.v_max, w=0)

            for n in neigh:
                if n.knows.info_nest[0] > self.body.knows.info_nest[0]:
                    self.body.knows.info_nest=n.knows.info_nest
                    if (n.knows.tell_state=='Stay' or n.knows.follow == True) and n.knows.info_nest[1]==self.body.knows.info_nest[1]:
                        self.GoTo.cmd_set(self.space.nearestpoint(i, n))
                        self.body.knows.follow = True



        if self.body.knows.tell_state() == 'Stay':
            self.body.knows.follow=True
            self.set_velocity_stay()
            self.change_color('To stay')
            i = self.space.bodindex(self.body.name)
            neigh = self.space.nearby(i, self.r, self.rng, type(self.body))
            neigh=len(neigh)
            if uniform(0,1)<np.exp(-b*neigh):
                print('Im ',i,' and I left from the nest with ',neigh,' neighbours')
                self.body.knows.set_state('Leave')





        if  self.body.knows.tell_state() == 'Leave':
            self.body.knows.follow = False
            self.set_velocity_leave()
            self.change_color('To leave')
            i = self.space.bodindex(self.body.name)

            nest = self.space.nearest(i, self.r, self.rng, Nest)
            if nest==None:
                self.body.knows.set_state('Random_Walk')
                self.body.knows.previous_time = time()








    def update(self):
        self.algorithm_0()
        super().update()

## MAIN
if __name__ == '__main__':
    name='Demo'+strftime("%Y%m%d%H%M", localtime())
    s=Space(name,T=RT,limits='')
    p=KPIdata(name,2,TS)
    # a big nest in the second quadrant:
    bignest=Nest('BigNest',pos=(uniform(-0.8*W,-0.2*W),uniform(0.4*H,0.6*H)),area=0.02,fc=(0,0,1)) #
    smallnest = Nest('SmallNest', pos=(uniform(0.8*W,0.2*W),uniform(-0.4*H,-0.6*H)), area=0.04, fc=(1,0,0))  #
    s.bodies.append(bignest)
    s.bodies.append(smallnest)
    N=100
    s.spawn_bodies(nm=N)
    for b in s.bodies:
        if isinstance(b,Mobot):
            b.cmd_vel(v=b.v_max/2)
            Demo(b,s,TS/10,r=2,rng=np.pi/2) # a new Soul for b (who assigns new Knowledge too)
            b.knows.set_state('Random_Walk')
    KPI=[0,1] # [ fraction of live mobots who know ThereAreNests , fraction of initial robots still alive ]
    end=False
    numero=0
    while not end:
        numero=numero+1

        for b in s.bodies:
            if isinstance(b,Mobot):
                b.update()
        s.update_dist()
        #s.remobodies(s.collisions())
        s.update_conn()
        if visual and time()>s.t0+(s.updates+1)*s.T:
            s.redraw()
        if time()>p.t0+(p.updates+1)*p.T:
            KPI=[0,0]

            for b in s.bodies:

                if isinstance(b,Mobot):
                    if b.knows.tell_state()=='Stay' and b.knows.check_quadrant(b)==2:

                            KPI[0]=KPI[0]+1

                    if b.knows.tell_state()=='Stay' and b.knows.check_quadrant(b)==4:

                            KPI[1] = KPI[1]+1


            KPI[0] /= N

            KPI[1] /= N
            p.update(KPI)
        end=s.has_been_closed() or KPI[0]>0.95
    else:
        s.close()
        p.close()
