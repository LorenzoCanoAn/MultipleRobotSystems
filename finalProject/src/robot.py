import numpy as np
import matplotlib.pyplot as plt
from random import randint


# Mensaje entre robots que tnga un timestamp y contenga la direccion d la formacion, el gap  y el centro

class battery:
    def __init__(self, robot, C=0.2, K=0.028, init_battery=100):

        self.robot = robot
        self.dT = robot.dT
        self.charging = False

        # Battery model:  battery[i+1]=battery[i]-K * v[i+1]- C * abs(v[i+1]-v[i]), con i+1 un segundo despues de i
        self.battery = init_battery + -randint(0,60)
        self.K = K  # percentage/((m/s))
        self.C = C  # percentage/((m/s))

        self.charging_rate=30 #percentage/second
        self.remaining=(self.battery>0)

    def update(self,state,robot):
        self.robot=robot
        if state=='Charging':
            self.charging==True
            self.battery = self.battery + self.charging_rate
        else:
            self.charging==False
            self.battery = self.battery - self.K * np.linalg.norm(
                self.robot.velocity[-1]) * self.dT - self.C * np.linalg.norm(
                self.robot.velocity[-1] - self.robot.velocity[-2]) * self.dT



        if self.battery<0:
            self.battery=0
            self.remaining=False
        if self.battery>100:
            self.battery=100


class GoTo:
    def __init__(self, robot):
        self.robot = robot
        self.active = False
        self.goal = None
        self.tolerance = 0.5
        self.v_max = robot.v_max
        self.a_max = robot.a_max
        self.dT = robot.dT

    def set_goal(self, goal):
        self.active = True
        self.goal = goal

    def stop(self):
        self.active = False
        self.goal = None

    def check_arrive(self, position):
        if np.linalg.norm(position-self.goal) < self.tolerance:
            self.stop()
            return True
        else:
            return False

    def compute_vel(self, position, velocity):
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
    def __init__(self, robot):
        
        self.robot = robot
        self.environment = self.robot.environment
        
        self.index = self.robot.index
        self.active = False
        self.tolerance = 0.5
        self.v_max = self.robot.v_max
        self.a_max = self.robot.a_max
        self.dT = self.robot.dT
        self.num_order=0
        self.displacement=np.array([0,0])


    def set_active(self,num_order):
        self.num_order=num_order
        self.active=True

    def set_off(self):
        self.num_order = 0
        self.active = False

    def compute_goal(self, position):

        nei=self.environment.neighbours_information[self.index]
        sum=0
        num=0
        for i in range(len(nei)):
            if self.environment.robots[nei[i]].state=='Active':
                sum=sum+self.environment.robots[nei[i]].position[-1]
                num=num+1
        self.goal=sum/(num)+self.displacement

    def set_displacement(self, displacement):
        self.displacement = displacement

    def compute_vel(self, position, velocity):
        self.compute_goal(position)
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
    def __init__(self,environment, index, base_pose,init_battery=100, init_position=np.array([0,0]), radius=11, dT=1,connection_radius=800):
        self.base_pose=base_pose
        self.index=index
        self.connection_radius=connection_radius
        self.position=[init_position]
        self.radius=radius
        self.velocity=[np.array([0,0])]
        self.environment = environment
        self.v_max = 7  # (m/s)
        self.a_max = 1  # (m/s^2)

        self.state = 'Active'  # possible states: Active, Returning, Charging

        self.neighbours = []
        

        self.dT = dT
        self.consensus = Consensus(self)
        self.battery = battery(self, init_battery = init_battery)
        self.GoTo = GoTo(self)


    ###########################################
    # UTILITY FUNCTIONS
    ###########################################

    def update(self):
        new_velocity = self.compute_vel()

        self.velocity.append(np.array(new_velocity))

        self.position.append(
            np.array(self.position[-1]+self.velocity[-1]*self.dT))

        self.battery.update(self.state,self)

        if self.battery.battery<20 and self.state=='Active':
            self.state='Returning'
            self.GoTo.set_goal(self.base_pose)

        if self.battery.battery==100 and self.state=='Charging':
             self.state='Waiting'
             self.battery.charging=False

    def compute_vel(self):
        #print('Soy el robot', self.index, 'y mi estado es:  ', self.state)
        "Aquí debería ir el cálculo de la velocidad basado en la posición de los neighbours"
        if self.battery.remaining==False or self.state=='Waiting' or self.state=='Charging':
            return np.array([0,0])

        if self.state=='Returning':
          self.consensus.active==False
          self.GoTo.active==True
          self.GoTo.set_goal(self.base_pose)
          v=self.GoTo.compute_vel(self.position[-1],self.velocity[-1])
          arrive=self.GoTo.check_arrive(self.position[-1])
          if arrive:
              self.state='Charging'

          return v

        if self.state=='Active':
            self.consensus.active==True
            v=self.consensus.compute_vel(self.position[-1],self.velocity[-1])

            return v

    def update_neighbours(self):
        self.neighbours = self.environment.neighbours_information[self.index]

    ###########################################
    # STATE FUNCTIONS
    ###########################################
    