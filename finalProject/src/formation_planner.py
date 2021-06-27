import cv2
import numpy as np
import matplotlib.pyplot as plt
from environment import *
import operator

class formation_planner:
    def __init__(self,env):
        self.center=np.array([0,0]) # Center of the formation
        self.enviroment=env         
        self.number_actives=0       # Number of active robots
        self.active_index=[]
        self.direction=np.array([0,0]) # Orientation of the formation
        self.gap=0

        for i in range(len(env.robots)):
            if self.enviroment.robots[i].state=='Active':
                self.number_actives+=1
                self.active_index.append(env.robots[i].index)
            else:
                self.enviroment.robots[i].consensus.set_off()

    def update_actives(self):
        self.number_actives = 0
        self.active_index=[]
        order=0
        for i in range(len(env.robots)):
            if self.enviroment.robots[i].state == 'Active':
                self.number_actives += 1
                self.active_index.append(i)
                self.enviroment.robots[i].consensus.num_order=order
                order+=1

    def update(self,env):
        self.enviroment.update_env()
        self.update_actives()
        self.update_center()


    def update_center(self):
        old_center=self.center

        numero=0
        self.center=np.array([0,0])
        cent=np.array([0,0])
        for i in range(len(self.active_index)):
                cent=cent+self.enviroment.robots[self.active_index[i]].position[-1]
        cent=cent/len(self.active_index)

        comp=(old_center==np.array([0,0]))
        if comp.all==True:
            cent=old_center
        for i in range(len(self.active_index)):
            if dist(cent, self.enviroment.robots[self.active_index[i]].position[-1])<40 :
                self.center=self.center+self.enviroment.robots[self.active_index[i]].position[-1]
                numero+=1
        if numero>0:
            self.center=self.center/numero
        else:
            self.center=old_center

    def create_formation(self,direction,gap):
        self.direction=direction
        self.gap=gap
        for i in range(len(self.active_index)):
            self.enviroment.robots[self.active_index[i]].consensus.set_displacement(gap*direction*(self.enviroment.robots[self.active_index[i]].consensus.num_order+1-(len(self.active_index)+1)/2))

    def move_formation(self,direction,intensity):
        self.create_formation(self.direction, self.gap)
        for i in range(len(self.active_index)):
            self.enviroment.robots[self.active_index[i]].consensus.displacement+=direction*intensity


    def set_robots_active(self,number):
        active={}
        n_actives=0
        n_waiting=0

        active_index=[]
        waiting_index=[]

        for i in range(len(self.enviroment.robots)):
            if self.enviroment.robots[i].state=='Active':
                n_actives+=1
                active_index.append(self.enviroment.robots[i].index)
            if self.enviroment.robots[i].state=='Waiting':
                n_waiting+=1
                waiting_index.append(self.enviroment.robots[i].index)

        if n_actives>=number:
            return

        if n_actives<number:
            i=0
            while number>n_actives:
                self.enviroment.robots[waiting_index[i]].state='Active'
                active_index.append(waiting_index[i])
                n_actives+=1
                i+=1
        self.update_actives()


    def move_formation_goal(self,goal):
            self.update_center()
            dist=np.linalg.norm(goal-self.center)
            vel=goal-self.center
            vel=vel/np.linalg.norm(vel)
            pathplanner.move_formation(vel, min(dist,50))

def dist(a,b):
    distan=np.sqrt(np.power(a[0]-b[0],2)+np.power(a[1]-b[1],2))
    return distan

def next_goal(current_goal):
    new_goal=[0,0]

    check=((current_goal[0]+18)/36)%2==0
    if current_goal[1]==400 and  check:
        new_goal[1]=0
        new_goal[0]=current_goal[0]
    elif current_goal[1]==0 and not check:
        new_goal[1]=400
        new_goal[0] = current_goal[0]
    else:
        new_goal[1]=current_goal[1]
        new_goal[0] = current_goal[0]+36

    new_goal=np.array(new_goal)
    return new_goal

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

    pathplanner.set_robots_active(5)
    pathplanner.update_actives()



    direction = np.array([1, 0])
    gap = 9
    goal=np.array([18,0])
    print('New goal:', goal)

    pathplanner.create_formation(direction, gap)
    pathplanner.move_formation_goal(np.array([20,20]))
    count=0
    set_goal=0
    same_robots=True
    vary=False
    i=0
    old_active = pathplanner.active_index
    hola = 0
    while goal[0]<600:
        cv2.waitKey(10)

        if not same_robots and vary==False:
            previous_goal=goal
            goal=potential_goal+np.array([1,1])
            print('New goal: ', goal)
            vary=True

        if np.linalg.norm(goal-pathplanner.center)<1 and vary==False:
            if not vary:
                set_goal = set_goal + 1
                goal = next_goal(goal)
                print('New goal:', goal)

        if np.linalg.norm(goal-pathplanner.center)<1 and vary==True:
            count += 1
            set_goal = set_goal + 1
            if count > 10:
                vary = False
                count = 0
                goal = previous_goal
                print('New goal:', goal)


        pathplanner.move_formation_goal(goal)
        pathplanner.update(pathplanner.enviroment)
        pathplanner.set_robots_active(5)
        if i%10==0:
            pathplanner.enviroment.draw_env()

        i=i+1

        cv2.waitKey(1)
        potential_goal=pathplanner.center

        pathplanner.update_actives()
        same_robots = listas_iguales(old_active, pathplanner.active_index)
        old_active = pathplanner.active_index

    plt.plot(pathplanner.enviroment.T_list , pathplanner.enviroment.percentage_covered)
    plt.show()



    #
    #
    #
