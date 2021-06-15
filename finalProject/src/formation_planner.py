import numpy as np
import matplotlib.pyplot as plt
from environment import *
import operator
class formation_planner:
    def __init__(self,env):
        self.enviroment=env
        self.number_actives=0
        self.active_index=[]
        self.direction=np.array([0,0])
        self.gap=0

        for i in range(len(env.robots)):
            if self.enviroment.robots[i].state=='Active' and self.enviroment.robots[i].consensus.active:
                self.number_actives+=1
                self.active_index.append(i)

            else:
                self.enviroment.robots[i].consensus.set_off()
        print(self.active_index)
    def update_actives(self):
        self.number_actives = 0
        self.actives_index=[]
        for i in range(len(env.robots)):
            if self.enviroment.robots[i].state == 'Active' and self.enviroment.robots[i].consensus.active:
                self.number_actives += 1
                self.active_index.append(i)
            else:
                self.enviroment.robots[i].consensus.set_off()


    def update(self,env):
        self.enviroment.update_env()
        self.update_actives()
        self.update_center()

    def update_center(self):
        self.center=np.array([0,0])
        for i in range(len(self.active_index)):
            self.center=self.center+self.enviroment.robots[self.active_index[i]].position[-1]
        self.center=self.center/len(self.active_index)

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

        for i in range(len(self.enviroment.robots)):
            if self.enviroment.robots[i].state=='Active':
                active[i]=self.enviroment.robots[i].battery.battery

        sort = sorted(active.items(), key=operator.itemgetter(1), reverse=True)
        if len(sort)>=number:
            self.number_actives=number

        else:
            self.number_actives=len(sort)
            number=self.number_actives

        for name in enumerate(sort):
            if number > 0:
                number = number - 1
                self.enviroment.robots[name[1][0]].consensus.set_active(number)

        self.update_actives()

    def move_formation_goal(self,goal):
            self.update_center()
            dist=np.linalg.norm(goal-self.center)
            vel=goal-self.center
            vel=vel/np.linalg.norm(vel)
            pathplanner.move_formation(vel, min(dist,50))



if __name__ == "__main__":
    env_params = {
        "base_pose": [10, 10]
    }
    env = environment(env_params)
    pathplanner=formation_planner(env)

    pathplanner.set_robots_active(5)

    print(pathplanner.active_index)
    direction = np.array([1, 0])
    gap = 4
    goal_array=[np.array([8,0]),np.array([8,400]),np.array([24,400]),np.array([24,0])]
    pathplanner.create_formation(direction, gap)
    pathplanner.move_formation_goal(np.array([20,20]))

    set_goal=0

    i=0
    while True:

        if np.linalg.norm(goal_array[set_goal]-pathplanner.center)<1:
            set_goal=set_goal+1
            print('New goal:', goal_array[set_goal])
        pathplanner.move_formation_goal(goal_array[set_goal])
        pathplanner.enviroment.update_env()
        if i%30==0:
            pathplanner.enviroment.draw_env()
        #print('neighbours:', env.neighbours_information)

        i=i+1
        print(i)
        pathplanner.update_center()
        print(pathplanner.center)
        if i==800:
            pathplanner.move_formation(np.array([0, 1]), 0)
        cv2.waitKey(1)






