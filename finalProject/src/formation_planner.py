import cv2
import numpy as np
import matplotlib.pyplot as plt
from numpy.lib.histograms import histogram_bin_edges
from environment import *
import operator



n_bots_formation = 10

class formation_planner:
    def __init__(self,env, n_bots_formation = n_bots_formation):
        self.center=np.array([0,0]) # Center of the formation
        self.enviroment=env         
        self.number_actives=0       # Number of active robots
        self.active_index=[]
        self.direction=np.array([1,0]) # Orientation of the formation
        self.gap=9
        self.n_bots_formation = n_bots_formation
        for i in range(len(env.robots)):
            if self.enviroment.robots[i].state in ['active', "going_formation", "waiting_active"]:
                self.number_actives+=1
                self.active_index.append(env.robots[i].index)
            else:
                self.enviroment.robots[i].consensus.set_off()

    def update_actives(self):
        self.number_actives = 0
        self.active_index=[]
        order=0
        for i in range(len(env.robots)):
            if self.enviroment.robots[i].state in ['active', "going_formation", "waiting_active"] :
                self.number_actives += 1
                self.active_index.append(i)
                self.enviroment.robots[i].consensus.num_order=order
                order+=1

    def update_actives__(self):
        self.number_actives = 0
        self.active_index=[]
        for i in range(len(env.robots)):
            if self.enviroment.robots[i].state in ['active', "going_formation", "waiting_active"]:
                self.number_actives += 1
                self.active_index.append(i)

    def update(self):
        self.enviroment.update_env()
        self.update_center()


    def update_center(self):
        if self.active_index.__len__() == 0:
            self.center = np.array([0,0])
        else:
            old_center=self.center

            numero=0
            self.center=np.array([0,0])
            cent=np.array([0,0])
            for i in range(len(self.active_index)):
                    cent=cent+self.enviroment.robots[self.active_index[i]].position
            cent=cent/len(self.active_index)

            comp=(old_center==np.array([0,0]))
            if comp.all==True:
                cent=old_center
            for i in range(len(self.active_index)):
                if dist(cent, self.enviroment.robots[self.active_index[i]].position)<40 :
                    self.center=self.center+self.enviroment.robots[self.active_index[i]].position
                    numero+=1
            if numero>0:
                self.center=self.center/numero
            else:
                self.center=old_center

    def create_formation(self):
        self.update_actives__()
        direction = self.direction
        gap = self.gap

        for i in range(len(self.active_index)):
            print(i,end="------")
            print(gap*direction*(self.enviroment.robots[self.active_index[i]].consensus.num_order+1-(len(self.active_index)+1)/2))
            self.enviroment.robots[self.active_index[i]].consensus.set_displacement(gap*direction*(self.enviroment.robots[self.active_index[i]].consensus.num_order+1-(len(self.active_index)+1)/2))

    def move_formation(self,direction,intensity):
        self.create_formation()
        for i in range(len(self.active_index)):
            self.enviroment.robots[self.active_index[i]].consensus.displacement+=direction*intensity


    def set_robots_active(self,number):
        n_actives=0
        n_waiting=0

        active_index=[]
        waiting_index=[]

        for i in range(len(self.enviroment.robots)):
            if self.enviroment.robots[i].state in ['active', "going_formation", "waiting_active"]:
                n_actives+=1
                active_index.append(self.enviroment.robots[i].index)
            if self.enviroment.robots[i].state in ['waiting_base']:
                n_waiting+=1
                waiting_index.append(self.enviroment.robots[i].index)
        if n_actives>=number:
            return

        if n_actives<number:
            i=0
            while number>n_actives:
                self.enviroment.robots[waiting_index[i]].change_state('going_formation',transition=self.enviroment.leaving_info[1])
                self.enviroment.robots[waiting_index[i]].consensus.num_order = self.enviroment.leaving_info[0]
                print("I go to formation, i inherit: {}".format(self.enviroment.leaving_info))
                
                active_index.append(waiting_index[i])
                n_actives+=1
                i+=1
        


    def move_formation_goal(self,goal):
            self.update_center()
            dist=np.linalg.norm(goal-self.center)
            vel=goal-self.center
            vel=vel/np.linalg.norm(vel)
            pathplanner.move_formation(vel, min(dist,50))

    def next_goal(self,current_goal):
        new_goal=[0,0]

        h_desp = (self.number_actives-0.5) * self.gap

        check=((current_goal[0]+(self.
        n_bots_formation-1)*self.gap/2)/h_desp)%2==0
        if current_goal[1]>=399.0 and  check:
            new_goal[1]=0
            new_goal[0]=current_goal[0]
        elif current_goal[1]<=0.01 and not check:
            new_goal[1]=400
            new_goal[0] = current_goal[0]
        else:
            new_goal[1]=current_goal[1] 
            new_goal[0] = current_goal[0]+h_desp

        new_goal=np.array(new_goal)
        return new_goal

def dist(a,b):
    distan=np.sqrt(np.power(a[0]-b[0],2)+np.power(a[1]-b[1],2))
    return distan



def listas_iguales(lista, lista2):
    if len(lista) == len(lista2):
        for i in range(len(lista)):
                if lista[i] != lista2[i]:

                    return False
        return True
    else:
        return False


if __name__ == "__main__":
    env_params = {
        "base_pose": [10, 10]
    }


    env = environment(env_params)
    pathplanner=formation_planner(env)

    pathplanner.set_robots_active(n_bots_formation)
    pathplanner.update_actives()



    gap = env.robots[0].radius - 2
    goal=np.array([(gap * n_bots_formation)/2,0])
    print('New goal:', goal)

    pathplanner.create_formation()
    pathplanner.move_formation_goal(np.array([20,20]))
    count=0
    set_goal=0
    same_robots=True
    vary=False
    i=0
    old_active = pathplanner.active_index
    hola = 0

    out = cv2.VideoWriter('output.avi', -1, 20.0, (400, 600))

    while goal[0]<600:
        # ROBOT CHANGE HANDLING
        ###############################################################3

        if np.linalg.norm(goal-pathplanner.center)<1:
            set_goal = set_goal + 1
            goal = pathplanner.next_goal(goal)


        ###############################################################

        pathplanner.move_formation_goal(goal)
        pathplanner.update()
        pathplanner.set_robots_active(n_bots_formation)
        if i%10==0:
            pathplanner.enviroment.draw_env()
            out.write(env.dispImage)


        i=i+1

        cv2.waitKey(1)
        potential_goal=pathplanner.center

        #pathplanner.update_actives()
        same_robots = listas_iguales(old_active, pathplanner.active_index)
        old_active = pathplanner.active_index
    out.release()
    plt.plot(pathplanner.enviroment.T_list , pathplanner.enviroment.percentage_covered)
    plt.show()



    #
    #
    #
