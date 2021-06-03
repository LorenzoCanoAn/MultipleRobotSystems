import numpy as np
import matplotlib.pyplot as plt

class battery:
    def __init__(self, robot, dT,C=0.05, K=0.007,init_battery=100):

        self.robot=robot
        self.dT=dT
        self.charging=False

        self.battery = init_battery  # Battery model:  battery[i+1]=battery[i]-K * v[i+1]- C * abs(v[i+1]-v[i]), con i+1 un segundo despues de i
        self.K = K  # percentage/((m/s))
        self.C = C  # percentage/((m/s))

        self.charging_rate=0.15 #percentage/second
        self.remaining=(self.battery>0)

    def update(self):
        if self.charging:
            self.battery=self.battery+self.charging_rate*self.dT
        else:
            self.battery=self.battery-self.K*np.linalg.norm(self.robot.velocity[-1])*self.dT-self.C*np.linalg.norm(self.robot.velocity[-1]-self.robot.velocity[-2])*self.dT
        if self.battery<0:
            self.battery=0
            self.remaining=False
        if self.battery>100:
            self.battery=100


class GoTo:
    def __init__(self,v_max,a_max,dT):
        self.active=False
        self.goal=None
        self.tolerance=0.5
        self.v_max=v_max
        self.a_max=a_max
        self.dT=dT
    def set_goal(self,goal):
        self.active=True
        self.goal=goal
    def stop(self):
        self.active=False
        self.goal=None

    def check_arrive(self,position):
        if np.linalg.norm(position-self.goal)<self.tolerance:
            self.stop()

    def compute_vel(self,position,velocity):
        v = 0.1 * (self.goal - position)
        if np.linalg.norm(v) > self.v_max:
            v = v / np.linalg.norm(v)
            v = v * self.v_max

        a = (v - velocity) / self.dT
        if np.linalg.norm(a) > self.a_max:
            a = a / np.linalg.norm(a)
            a = a * self.a_max
            v = velocity + a * self.dT
        return v


class Consensus:
    def __init__(self,v_max,a_max,dT,index):
        self.index=index
        self.active = False
        self.tolerance = 0.5
        self.v_max = v_max
        self.a_max = a_max
        self.dT = dT
        self.num_order=0
        self.displacement=np.array([0,0])
    def set_active(self,num_order):
        self.num_order=num_order
        self.active=True

    def set_off(self):
        self.num_order = 0
        self.active = False

    def compute_goal(self,environment,position):

        nei=environment.neighbours_information[self.index]
        sum=position
        num=1
        for i in range(len(nei)):
            if environment.robots[nei[i]].state=='Active' and environment.robots[nei[i]].consensus.active:
                sum=sum+environment.robots[nei[i]].position[-1]
                num=num+1
        self.goal=sum/(num)+self.displacement


    def set_displacement(self,displacement):
        self.displacement=displacement
    def compute_vel(self,position,velocity,environment):
        self.compute_goal(environment,position)
        v = 0.1 * (self.goal - position)
        if np.linalg.norm(v) > self.v_max:
            v = v / np.linalg.norm(v)
            v = v * self.v_max

        a = (v - velocity) / self.dT
        if np.linalg.norm(a) > self.a_max:
            a = a / np.linalg.norm(a)
            a = a * self.a_max
            v = velocity + a * self.dT
        return v



class robot:
    def __init__(self,index, base_pose,init_battery=100, init_position=np.array([0,0]), radius=5, dT=1,connection_radius=800):
        self.base_pose=base_pose
        self.index=index
        self.connection_radius=connection_radius
        self.position=[init_position]
        self.radius=radius
        self.velocity=[np.array([0,0])]

        self.input=np.array([0,0])

        self.v_max=7  #(m/s)
        self.a_max=1 #(m/s^2)




        self.state='Active' #possible states: Active, Returning, Charging

        self.neighbours=[]

        self.dT=dT

        self.battery=battery(self,self.dT)

        self.GoTo=GoTo(self.v_max,self.a_max,self.dT)
        self.consensus=Consensus(self.v_max,self.a_max,self.dT,self.index)

    def update(self,environment):
        new_velocity=self.compute_vel(environment)

        self.velocity.append(np.array(new_velocity))


        self.position.append(np.array(self.position[-1]+self.velocity[-1]*self.dT))

        self.battery.update()

        if self.battery.battery<10 and self.state=='Active':
            self.state='Returning'
            self.GoTo.set_goal(self.base_pose)

        if self.battery.battery==100 and self.state=='Charging':
             self.state=='Active'
             self.battery.charging=False

    def compute_vel(self,enviroment):
        "Aquí debería ir el cálculo de la velocidad basado en la posición de los neighbours"
        if self.battery.remaining==False:
            return np.array([0,0])

        if self.GoTo.active:
          v=self.GoTo.compute_vel(self.position[-1],self.velocity[-1])
          self.GoTo.check_arrive(self.position[-1])
          if self.GoTo.active==False and self.state=='Returning':
             self.state='Charging'
             self.battery.charging=True

        elif self.consensus.active:
            v=self.consensus.compute_vel(self.position[-1],self.velocity[-1],enviroment)

        else:
            v=np.array([0,0])



        return v
    def update_neighbours(self,environment):
        self.neighbours=environment.neighbours_information[self.index]









