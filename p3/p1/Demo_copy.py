## A Demo of SmallWorl2D
## Version: 1.0.20210514
## Author: Enrique Teruel (ET) eteruel@unizar.es
## License: CC-BY-SA

from time import time, localtime, strftime
from random import uniform, randint
import numpy as np
from matplotlib.animation import FFMpegWriter
from shapely.geometry import point # requires having ffmpeg installed, from https://ffmpeg.org/
from point2d import Point2D

from small_worl2d_config import visual, W, H, TS, RT # configuration parameters in a separate file
from small_worl2d import Space, KPIdata, Nest, Mobot, Soul, GoTo, Knowledge

quad_coord = {
    1: (W/2, H/2),
    2: (-W/2, H/2),
    3: (-W/2, -H/2),
    4: (W/2, -H/2)
}


class ant_knowledge(Knowledge):
    """ A silly specialization of Knowledge, actually it is just the basic Knowledge
        
        Typically, here would be defined the specific variables and functions required
    """

    def __init__(self,body, state):
        self.quadrant = 0
        self.ticks = 0
        super().__init__(body, state)
    
    def check_quadrant(self,body):
        if body.pos.x>0 and body.pos.y>0:
            return 1
        elif body.pos.x<=0 and body.pos.y>0:
            return 2
        elif body.pos.x<=0 and body.pos.y<=0:
            return 3
        elif body.pos.x>0 and body.pos.y<=0:
            return 4

    def restart(self,body):
        self.quadrant = 0
        self.ticks = 0






class go_to_biggest_nest(Soul):
    def __init__(self,body,space,T=TS,r=1,rng=np.pi):
        self.space=space
        self.r=r
        self.rng=rng
        GoTo(body,0,Kp=0.2,tol=0.01,nw='zigzag',p=0.1)
        self.GoTo=body.souls[-1]
        ant_knowledge(body,'random_walk')
        super().__init__(body,T)
        self.index = self.space.bodindex(self.body.name)
        self.states = {"random_walk":self.random_walk,
                        "measure_nest":self.measure_nest,
                        "go_and_spread_word":self.go_and_spread_word,
                        "go_to_nest":self.go_to_nest,
                        "stay_in_nest":self.stay_in_nest}
        
        self.color_code = { "random_walk": (0,1,0),
                            "measure_nest":(1,0,1), 
                            "go_and_spread_word":self.get_nest_color,
                            "go_to_nest":self.get_nest_color,
                            "stay_in_nest":self.get_nest_color}

    def change_color(self,word):
        if word in ["go_and_spread_word","go_to_nest", "stay_in_nest"]:
            self.body.fc = self.color_code[word]()
        else:
            self.body.fc = self.color_code[word]

    def get_nest_color(self):
        if self.body.knows.quadrant == 2:
            return (0,0,1)
        elif self.body.knows.quadrant == 4:
            return (1,0,0)

    def change_state(self, new_state, t_data=1):
        self.body.knows.set_state(new_state)
        self.change_color(new_state)
        self.states[new_state](t_data=t_data)

    def set_velocity_random_walk(self):
        random_int = randint(1,100)
        if random_int < 20:
            self.body.knows.previous_time = time()
            w_set = self.body.w_max * np.random.normal(0, 1 / 5)
            self.body.cmd_vel(v=self.body.v_max, w=w_set)
        elif random_int < 80:
            pass
        else:
            self.body.cmd_vel(v=self.body.v_max, w=0)

    def compare_neighbors(self):
        neighs = self.space.nearby(self.index,self.r,self.rng,Mobot)
        quad = 0
        max_ticks = 0
        for neigh in neighs:
            if neigh.knows.ticks > max_ticks:
                quad = neigh.knows.quadrant
                max_ticks = neigh.knows.ticks
        if max_ticks > self.body.knows.ticks:
            if self.body.knows.quadrant != quad:
                self.body.knows.ticks = max_ticks
                self.body.knows.quadrant = quad
                self.change_state("go_and_spread_word")
            else:
                self.body.knows.ticks = max_ticks
                self.body.knows.quadrant = quad

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # STATES
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # random_walk
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def random_walk(self,t_data=None):
        # Wanders around. If it finds a nest
        self.set_velocity_random_walk()

        if self.body.knows.quadrant == 0:
            nest = self.space.nearest(self.index, self.r, self.rng, Nest)
            if type(nest) != type(None):
                self.body.knows.quadrant = self.body.knows.check_quadrant(nest)
                self.change_state("measure_nest", t_data=nest)
            else:
                self.compare_neighbors()

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # measure_nest
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def measure_nest(self, t_data=None):
        if type(t_data) != type(None):
            self.GoTo.cmd_set(self.space.nearestpoint(self.index, t_data), nw="keepgoing", Kp=2, tol = 0.3)
            self.body.cmd_vel(v=self.body.v_max/2)
            self.nest = t_data
        else:
            self.body.cmd_vel(v=self.body.v_max/2)
            if self.nest in self.space.incontact(self.index, Nest):
                self.body.knows.ticks += 1
            else:
                if self.body.knows.ticks > 0:
                    # we have now exited the nest and know the size
                    self.change_state("go_and_spread_word")

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # go_and_spread_word
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def go_and_spread_word(self, t_data=None):
        if type(t_data) != type(None):
            pass

        self.set_velocity_random_walk()
        self.compare_neighbors()

        if randint(1,5000) < 10:
            self.change_state("go_to_nest")

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # go_to_nest
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~           

    def go_to_nest(self,t_data=None):
        if type(t_data) != type(None):
            self.GoTo.cmd_set( quad_coord[self.body.knows.quadrant],nw="spiral", Kp=1, tol=0.5)

        nests = self.space.nearby(self.index,self.r,self.rng,Nest)
        if type(nests) != type(None):
            for nest in nests:               
                if self.body.knows.check_quadrant(nest) == self.body.knows.quadrant:
                    t_data = self.space.nearestpoint(self.index, nest)
                    self.change_state("stay_in_nest",t_data=t_data)
                    break
        if randint(1,100) < 5:
            self.compare_neighbors()

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # stay_in_nest
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~     

    def stay_in_nest(self, t_data=None):
        if type(t_data) != type(None):
            self.GoTo.cmd_set(t_data, nw="stop", Kp=5, tol = 0.5)

        if randint(1,100) < 5:
            self.compare_neighbors()

    def update(self):
        self.states[self.body.knows.tell_state()]()
        super().update()

## MAIN
if __name__ == '__main__':
    name='Demo'+strftime("%Y%m%d%H%M", localtime())
    s=Space(name,T=RT,limits='')
    p=KPIdata(name,2,TS)

    # SPAWN NESTS
    bignest=Nest('BigNest',pos=(uniform(-0.8*W,-0.2*W),uniform(0.4*H,0.6*H)),area=0.8,fc=(0,0,1)) #
    smallnest = Nest('SmallNest', pos=(uniform(0.8*W,0.2*W),uniform(-0.4*H,-0.6*H)), area=0.02, fc=(1,0,0))  #
    s.bodies.append(bignest)
    s.bodies.append(smallnest)

    # SPAWN MOBOTS
    N=25
    param_R = 2
    param_rng = np.pi/2
    s.spawn_bodies(nm=N)
    for b in s.bodies:
        if isinstance(b,Mobot):
            b.cmd_vel(v=b.v_max/2)
            go_to_biggest_nest(b,s,TS/10,r=param_R,rng=param_rng)

    # RUN SIMULATION
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
                    if b.knows.tell_state()=='stay_in_nest' and b.knows.check_quadrant(b)==2:

                            KPI[0]=KPI[0]+1

                    if b.knows.tell_state()=='stay_in_nest' and b.knows.check_quadrant(b)==4:

                            KPI[1] = KPI[1]+1


            KPI[0] /= N

            KPI[1] /= N
            p.update(KPI)
        end=s.has_been_closed() or KPI[0]>0.95
    else:
        s.close()
        p.close()
