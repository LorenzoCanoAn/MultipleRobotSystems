import numpy as np
import matplotlib.pyplot as plt

class battery:
    def __init__(self, robot, C=0.05, K=0.005,init_battery=100):

        self.robot=robot

        self.charging=False

        self.battery = init_battery  # Battery model:  battery[i+1]=battery[i]-K * v[i+1]- C * abs(v[i+1]-v[i]), con i+1 un segundo despues de i
        self.K = K  # percentage/((m/s))
        self.C = C  # percentage/((m/s))

        self.charging_rate=0.15 #percentage/second
        self.remaining=(self.battery>0)

    def update(self):
        if self.charging:
            self.battery=self.battery+self.charging_rate
        else:
            self.battery=self.battery-self.K*np.linalg.norm(self.robot.velocity[-1])-self.C*np.linalg.norm(self.robot.velocity[-1]-self.robot.velocity[-2])
        if self.battery<0:
            self.battery=0
            self.remaining=False
        if self.battery>100:
            self.battery=100



class robot:
    def __init__(self,index, init_battery=100, init_position=np.array([0,0]), radius=5, dT=1):
        self.index=index

        self.position=[init_position]
        self.radius=radius
        self.velocity=[np.array([0,0])]

        self.input=np.array([0,0])

        self.v_max=7  #(m/s)


        self.battery=battery(self)

        self.state='Active' #possible states: Active, Returning, Charging

        self.neighbours=[]

        self.dT=dT

    def update(self):

        self.velocity.append(self.velocity[-1]+self.input*self.dT)
        self.position.append(self.position[-1]+self.velocity*self.dT)
        self.battery.update()








