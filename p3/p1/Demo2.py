# A Demo of SmallWorl2D
# Version: 1.0.20210514
# Author: Enrique Teruel (ET) eteruel@unizar.es
## License: CC-BY-SA

from time import time, localtime, strftime
from random import uniform
import numpy as np
# requires having ffmpeg installed, from https://ffmpeg.org/
from matplotlib.animation import FFMpegWriter

# configuration parameters in a separate file
from small_worl2d_config import visual, W, H, TS, RT
from small_worl2d import Space, KPIdata, Nest, Mobot, Soul, GoTo, Knowledge


class Nestxists(Knowledge):
    def __init__(self, body, state):
        self.info_nest = [0, 0]
        self.follow = False
        self.previous_time = time()
        super().__init__(body, state)

    def check_quadrant(self, body):
        if body.pos.x > 0 and body.pos.y > 0:
            return 1
        elif body.pos.x <= 0 and body.pos.y > 0:
            return 2
        elif body.pos.x <= 0 and body.pos.y <= 0:
            return 3
        elif body.pos.x > 0 and body.pos.y <= 0:
            return 4

    def change_belief(self, body):
        if self.disagrees > self.agrees:
            a = self.agrees
            self.agrees = self.disagrees
            self.disagrees = a
            if self.belief == 2:
                self.belief = 4
                body.fc = (1, 0, 0)
            elif self.belief == 4:
                self.belief = 2
                body.fc = (0, 0, 1)

    def restart(self, body):
        self.agree = 0
        self.disagree = 0
        self.belief = 0
        self.body.fc == (0, 1, 0)


class Demo(Soul):
    """ A Soul that wanders differently if it knows ThereAreNests or Nothing """

    def __init__(self, body, space, T=TS, r=1, rng=np.pi):
        self.space = space
        self.r = r
        self.rng = rng  # might be smaller than pi, but not too much
        # requires a GoTo soul in the same body (to go nowhere)
        GoTo(body, 0, Kp=0.2, tol=0.01, nw='zigzag', p=0.1)
        self.GoTo = body.souls[-1]  # this way it knows how to call it
        # this Soul needs a Mind in its Body to work
        Nestxists(body, 'Explore')
        super().__init__(body, T)

    def change_color(self, word):
        if word == 'To stay':
            if self.body.knows.check_quadrant(self.body) == 2:
                self.body.fc = (0, 0, 1)
            elif self.body.knows.check_quadrant(self.body) == 4:
                self.body.fc = (1, 0, 0)

        elif word == 'To leave':
            self.body.fc = (0, 0, 0)

        elif word == 'To walk':
            self.body.fc = (0, 1, 0)
            if self.body.knows.info_nest[1] == 2:
                self.body.fc = (0.678, 0.847, 0.902)

            elif self.body.knows.info_nest[1] == 4:
                self.body.fc = (255/255, 182/255, 193/255)

            else:
                self.body.fc = (0, 1, 0)

    def algorithm_0(self):
        self.body.cmd_vel(v=0)


    def update(self):
        self.algorithm_0()
        super().update()


# MAIN
if __name__ == '__main__':
    name = 'Demo'+strftime("%Y%m%d%H%M", localtime())
    s = Space(name, T=RT, limits='')
    p = KPIdata(name, 2, TS)
    # a big nest in the second quadrant:
    bignest = Nest('BigNest', pos=(uniform(-0.8*W, -0.2*W),
                   uniform(0.4*H, 0.6*H)), area=0.02, fc=(0, 0, 1))
    smallnest = Nest('SmallNest', pos=(uniform(0.8*W, 0.2*W),
                     uniform(-0.4*H, -0.6*H)), area=0.04, fc=(1, 0, 0))  #
    s.bodies.append(bignest)
    s.bodies.append(smallnest)
    N = 100
    s.spawn_bodies(nm=N)
    for b in s.bodies:
        if isinstance(b, Mobot):
            b.cmd_vel(v=b.v_max/2)
            # a new Soul for b (who assigns new Knowledge too)
            Demo(b, s, TS/10, r=2, rng=np.pi/2)
            b.knows.set_state('Random_Walk')
    # [ fraction of live mobots who know ThereAreNests , fraction of initial robots still alive ]
    KPI = [0, 1]
    end = False
    numero = 0
    while not end:
        numero = numero+1

        for b in s.bodies:
            if isinstance(b, Mobot):
                b.update()
        s.update_dist()
        # s.remobodies(s.collisions())
        s.update_conn()
        if visual and time() > s.t0+(s.updates+1)*s.T:
            s.redraw()
        if time() > p.t0+(p.updates+1)*p.T:
            KPI = [0, 0]

            for b in s.bodies:

                if isinstance(b, Mobot):
                    if b.knows.tell_state() == 'Stay' and b.knows.check_quadrant(b) == 2:

                        KPI[0] = KPI[0]+1

                    if b.knows.tell_state() == 'Stay' and b.knows.check_quadrant(b) == 4:

                        KPI[1] = KPI[1]+1

            KPI[0] /= N

            KPI[1] /= N
            p.update(KPI)
        end = s.has_been_closed() or KPI[0] > 0.95
    else:
        s.close()
        p.close()
