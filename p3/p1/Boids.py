## A solution to the Boids exercise
## Version: 20210517
## Author: Enrique Teruel (ET) eteruel@unizar.es
## License: CC-BY-SA

from time import time, localtime, strftime
import numpy as np
# YOU MIGHT FIND USEFUL FUNCTIONS IN shapely, Point2D, AND gadgETs
from small_worl2d_config import visual, TS, RT
from small_worl2d import Space, KPIdata, MoBody, Mobot, Soul, Killer
from random import uniform
from small_worl2d_config import loginfo, logerror, visual, shoul, showconn, SS, W, H, room, TS, RT, vN, wN
import matplotlib.pyplot as plt

class Boid(Soul):

    def __init__(self,body,space,goodistance,T=TS): # ADD YOUR ARGUMENTS
        self.space=space
        self.goodistance=goodistance
        # YOUR BOID INIT CODE
        super().__init__(body,T)

        self.boid_integrants=[]
    def update(self):

        b=self.body

        i=self.space.bodindex(b.name)

        p=self.proximal()


        a=self.allignment()

        f=[p[0]+a[0],p[1]+a[1]]

        b.cmd_vel(v=f[0],w=0.25*f[1])

        self.update_size_boid()

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

    def update_size_boid(self):
        i = self.space.bodindex(b.name)
        neigh = self.space.nearby(i, self.space.R, np.pi, type(self.body))
        for n in neigh:
            j=self.space.bodindex(n.name)
            self.boid_integrants=self.boid_integrants+[j]
            self.boid_integrants=self.boid_integrants+n.souls[-1].boid_integrants
        self.boid_integrants=list(set(self.boid_integrants))



    def contribution_proximal(self, n):
        alpha=2
        epsilon=1.5
        sigma=self.goodistance/(np.power(2,1/alpha))

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
    R=1 # so goodistance=1 is little less than half R
    name='Boids_R'+str(R)+'_'+strftime("%Y%m%d%H%M", localtime())
    s=Space(name,T=RT,R=R,limits='')
    N=50
    s.spawn_bodies(nm=N,room=[(-4,-4),(4,-4),(4,4),(-4,4)])

    good_distance=0.7


    for b in s.bodies:
        if isinstance(b,Mobot):
            Boid(b,s,good_distance,0.1*TS) # ADD YOUR ARGUMENTS
    p=KPIdata(name,5,TS)


    KPI=[0,1,np.sqrt(N),1,0]

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
            graph = s.graph(Mobot)
            alive=0
            centroid_x = 0
            centroid_y = 0
            heading_avg = [0, 0]
            avg_error_to_good = 0
            n_total_neighbours = 0
            cum=0
            for b in s.bodies:
                if isinstance(b,Mobot):
                    alive=alive+1
                    # Compute centroid
                    centroid_x += b.pos.x
                    centroid_y += b.pos.y

                    # Compute average heading for order metric
                    heading_avg[0] += np.cos(b.th)
                    heading_avg[1] += np.sin(b.th)

                    # Compute average error to good distance
                    neighbours = graph[s.bodindex(b.name)]
                    n_total_neighbours += len(neighbours)
                    for neigh_index in neighbours:
                        neigh = s.bodies[neigh_index]
                        distance_to_neighbour = np.sqrt((neigh.pos.x - b.pos.x) ** 2 + (neigh.pos.y - b.pos.y) ** 2)
                        avg_error_to_good += ((distance_to_neighbour - good_distance) / good_distance) ** 2

                    if len(b.souls[-1].boid_integrants)/N>cum:
                         cum=len(b.souls[-1].boid_integrants)/N
            KPI[0]=alive/N   #Esto es el numero de robots que estan vivod
            centroid_x /= alive
            centroid_y /= alive


            max_dist = 0
            for b in s.bodies:
                distance_to_centroid = np.sqrt((centroid_y - b.pos.y) ** 2 + (centroid_x - b.pos.x) ** 2)
                if distance_to_centroid > max_dist:
                    max_dist = distance_to_centroid

            KPI[1]=cum  #Esto es el porcentaje de robots que estan en el boid mas grande

            KPI[2] = max_dist / (np.sqrt(N) * good_distance)  #Esto es el ratio entre maxima distancia al centroide de todos los robots, y el valor que deberia tener
                                                            #si todos los robots estuvieran en un boid. Esto siempre es mayor que uno, idealmente seróia 1.

            KPI[3] = avg_error_to_good / n_total_neighbours  #Esto tiene que ser cercano a 0 para estar bien, porque 0 error es que todos los robots estan juntos

            KPI[4] = np.linalg.norm(heading_avg) / N  #Esto tiene que ser cercano a 1 para estar bien. es 1 cuando todos los robots miran en la misma direccion

            # YOUR KPI CALCULATIONS
            print(KPI)
            p.update(KPI)



        end= s.has_been_closed() or KPI[1]==0 or p.updates>300 # 5 realtime min
    else:
        s.close()
        p.close()
        a=np.genfromtxt(name+'.dat')
        x=a.T[0]
        for i in range(a.shape[1]-1):
            plt.figure(i)
            plt.plot(x,a.T[i+1])
            plt.show()

